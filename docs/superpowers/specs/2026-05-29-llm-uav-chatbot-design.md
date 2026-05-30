# LLM-Piloted UAV Chatbot — Design

**Date:** 2026-05-29
**Status:** Approved (pending spec review)
**Author:** brainstormed with Claude

## 1. Summary

An experiment: a UAV you talk to in natural language ("take off, fly 50m
north, what do you see?, come back, land"). An LLM agent translates speech into
discrete high-level flight commands; the flight controller does the actual
flying. The drone is **aware of its surroundings** via onboard sensors. Runs
entirely in simulation now, with an explicit goal of flying a real drone later,
so the flight-command code must transfer to hardware unchanged.

## 2. Goals & non-goals

**Goals (v1)**
- Natural-language chat → discrete flight commands, executed in sim.
- Core flight vocabulary: arm/disarm, takeoff, land, return-to-home (RTL),
  go-to-relative-position (e.g. "50m north"), change altitude, hold/hover,
  report status.
- **Modular onboard perception**: the drone is aware of its surroundings
  (proximity/obstacles) and can answer visual questions ("what do you see?").
  Easy sensors now; the implementation is swappable so it can be refined later.
- Terminal chat interface.
- Full 3D simulation.
- Code below the agent transfers to real PX4 hardware.

**Non-goals (deferred, but the architecture must not preclude them)**
- Autonomous obstacle avoidance / SLAM / mapping ("spatial autonomy" tier).
  v1 is *aware and reports/reasons*; the autopilot provides the safety reflex.
- Object detection / tracking pipelines (YOLO, etc.).
- ROS2 — deferred until perception is refined toward spatial autonomy; the
  perception layer is built behind an interface so a ROS2 backend slots in later.
- Web / voice interface (terminal now, web later).
- Mission patterns (orbit, waypoint missions, survey, follow-me).
- A second LLM provider — v1 is honestly Claude-coupled (see §6).

## 3. Stack

Confirmed as current best-practice (validated against 2025/2026 prior art):

| Concern | Choice | Notes |
|---|---|---|
| Flight stack | **PX4 SITL** | Industry standard; runs on real hardware later. |
| Simulator | **Gazebo Harmonic** on **Ubuntu 22.04** | `make px4_sitl gz_x500*`. Avoid Garden (EOL Nov 2024) / Classic. |
| Sim model | camera + depth model (start `gz_x500_depth`) | Ships a depth camera; enables PX4 collision prevention. |
| Control library | **MAVSDK-Python** (async) | PX4-recommended; clean high-level calls; runs against real PX4. Not DroneKit (dead). |
| Agent | **Claude Agent SDK** (`claude-agent-sdk`) | In-process tools via `@tool` + `create_sdk_mcp_server`; `ClaudeSDKClient` for persistent multi-turn chat + interrupts. Options class is `ClaudeAgentOptions`. |

**Prior art we lean on (don't reinvent):**
- **droneserver** (arXiv 2601.15486) & **EchoPilot** — command taxonomy and the
  "verify the maneuver actually happened via telemetry before reporting success"
  loop. (Confirm droneserver license before lifting code.)
- **Geofencing** — PX4 firmware geofence (`GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`,
  `GF_ACTION`) as the real enforcer + MAVSDK Geofence plugin to upload an
  inclusion fence at startup; thin app-side clamp as defense-in-depth.
- **Collision prevention** — PX4 `CP_DIST` + the sim distance/depth sensor as a
  firmware-level "don't hit the wall" backstop.

Simplification over prior art: droneserver/EchoPilot run MAVSDK behind an
*external* MCP server; the Claude Agent SDK runs our tools **in-process**, so
there is no separate MCP transport.

## 4. Architecture

Layers, each independently understandable and testable. The dominant shared
resource is the **single asyncio event loop** (see §5.1) — it is an explicit
architectural constraint, not an emergent one.

```
              ┌─ Chat REPL (terminal) ────────────────┐  async stdin; renders from
              │   live status; DIRECT (non-LLM) abort  │  StateStore/PerceptionStore
              └──────┬─────────────────────────────────┘
              ┌─ Agent (Claude Agent SDK) ─────────────┐  ClaudeSDKClient + system
              │   @tool adapters → CommandExecutor      │  prompt + conversation loop
              └──────┬──────────────────────┬──────────┘
                     │ command tools        │ look()/scan_surroundings()
       ┌─ Control ───┴──────────┐  ┌─ Perception ┴───────────────────┐
       │ CommandExecutor        │  │ PerceptionProvider (interface)   │ ← swap-seam
       │  (portable command     │  │   v1: GazeboPerception           │
       │   boundary, see §6)    │  │   later: ROS2 / real sensors     │
       │ DroneController(MAVSDK) │  │ PerceptionStore (latest snapshot)│
       │ StateStore (authority)  │  └──────┬───────────────────────────┘
       │ SafetyGuard (invariants)│         │ sim sensor topics
       └──────┬──────────────────┘         │
              │ MAVSDK (gRPC/MAVLink)       │
       ┌──────┴─────────────────────────────┴──────┐
       │ PX4 SITL + Gazebo (separate process)        │
       └─────────────────────────────────────────────┘
```

**Boundary claims (corrected from review):**
- Everything at/below the tool layer knows nothing about LLMs. ✓ achievable.
- The portability seam is the **command boundary (`CommandExecutor`)**, NOT an
  `LLMAgent` interface. The agent layer knows nothing about MAVLink. ✓
- The agent/control layers never know whether perception is Gazebo, ROS2, or
  real sensors — only `PerceptionSnapshot`. ✓

## 5. Critical design decisions (from architecture review)

### 5.1 Single asyncio event loop — owned by `app.py`
MAVSDK-Python and the Claude Agent SDK are both asyncio-native, and the SDK's
in-process `@tool` callbacks run inside the SDK's loop. The `DroneController`
(MAVSDK `System`) must be created and connected **on that same loop** and
injected into the tool layer — otherwise: cross-loop "Future attached to a
different loop" errors. **One process, one loop, owned by `app.py`.**

> **Build order: spike this first.** Before building anything else, prove that a
> MAVSDK coroutine can be `await`ed from inside a Claude Agent SDK in-process
> tool on one loop, against PX4 SITL. If that holds, the whole in-process design
> holds. This is the highest-risk unknown.

### 5.2 Fire-and-monitor commands + non-LLM abort path
A `takeoff`/`goto` maneuver takes tens of seconds. Tools must **issue** the
action and return quickly with an acknowledgment ("climbing to 10m, in
progress"); a background task tracks completion against telemetry. Blocking a
tool to completion would freeze the chat and make abort impossible.

**Abort bypasses the agent entirely.** "stop"/abort from the REPL calls the
controller's hold/emergency-land **directly** (and uses `ClaudeSDKClient`'s
interrupt), because the LLM may be mid-tool-call and unable to respond. This is
both a UX and a safety requirement.

### 5.3 Authoritative `StateStore`
Three parties hold a notion of drone state: PX4 (ground truth), the
controller's telemetry cache, and the LLM's narrative. They diverge, and a
diverged LLM issues dangerous commands. A single `StateStore` in the control
layer is authoritative (armed, flight mode, position, battery, link health),
fed by a background telemetry task. **The LLM is never the source of truth**;
every safety precondition and status report reads the StateStore, and tool
results re-assert ground-truth state so the model is continuously corrected.

### 5.4 SafetyGuard = runtime supervisor below the LLM
Placed in the control layer so it is **non-bypassable** regardless of what the
model emits. The prompt is never a safety boundary (prompt injection,
hallucinated args, drift are in scope). Documented invariants, unit-tested
independently of the LLM:
- **Preconditions**: reject takeoff if not armed; reject goto if not flying or
  no position fix; reject commands during failsafe/RTL.
- **Argument clamping**: altitude cap, geofence radius from home, max distance
  per goto, sane climb rate — all config-driven with **fail-closed** defaults.
- **Non-LLM abort/kill path** (see §5.2).
- **Connection-loss policy** (see §5.5).
- Invariants to test: "never command altitude > cap", "never command outside
  geofence", "never command without a position fix".

Tools **return `{is_error: True, ...}` rather than raising** — an uncaught
exception in a tool handler kills the SDK query loop; a structured error lets
Claude react (e.g., hold/RTL).

### 5.5 Failure model (which layer owns what)
- **Link loss to mavsdk_server/PX4** → detected via telemetry heartbeat
  staleness in the StateStore; surfaced as a user-visible state; commands
  rejected while disconnected.
- **PX4-initiated mode changes / failsafes** (low battery→RTL, geofence breach,
  RC loss) → the StateStore observes `flight_mode` transitions and surfaces them
  to the chat ("PX4 triggered Return-to-Launch"), so user and LLM aren't
  commanding a vehicle doing something else.
- **Command timeouts / acks** → every controller action has a timeout and
  returns a structured failure the tool layer hands back to the LLM.

### 5.6 Structured flight-record log (from day one)
Record: user utterance → LLM tool call (with args) → safety decision →
controller call → result → resulting telemetry. This is the experiment's audit
trail and the only way to debug a bad maneuver. Spans tool + control layers.

## 6. Swappability — the real seam

The Claude Agent SDK entangles the LLM, the agent loop, and tool dispatch
(in-process MCP). A bare `LLMAgent` interface gives **false** portability, so it
is **cut for v1** (YAGNI). The genuinely portable boundary is the
**`CommandExecutor`**: plain Python, typed args, structured results, independent
of MCP and any SDK. Claude `@tool` functions are *thin adapters* that translate
MCP calls into `CommandExecutor` calls. Any future agent (different SDK,
provider, or a scripted client) targets `CommandExecutor`. v1 is openly
Claude-coupled; the portability guarantee lives at the command layer.

The perception equivalent is `PerceptionProvider` (§7).

## 7. Perception layer (modular, easy-sensors-first)

The modularity contract is `PerceptionProvider` → produces `PerceptionSnapshot`:
```
PerceptionSnapshot = {
  timestamp,
  camera_frame(s),            # RGB image(s) for the agent's own vision
  obstacles: [ {direction, distance}, ... ],   # proximity / surroundings awareness
  # later: detections[], depth map, occupancy/3D, ...
}
```
- **v1 impl `GazeboPerception`**: reads the sim's sensors directly (Gazebo
  topics). Easy starting set: a **forward RGB camera** + a **depth camera /
  rangefinder** for proximity. Start with PX4's turnkey camera+depth model.
- **`PerceptionStore`**: authoritative latest snapshot, updated by a background
  task (mirrors the StateStore pattern). REPL/status read from it; the agent
  pulls frames on demand. No blocking on the main loop (offload any sync sensor
  reads with `asyncio.to_thread`).
- **Agent tools**: `look()` (returns latest camera frame as an image for
  Claude's vision + a one-line scene summary) and `scan_surroundings()`
  (structured nearest-obstacle list). A compact surroundings line ("nearest
  obstacle: 4m, ahead-left") is surfaced in `report status` and the agent's
  context so it stays aware while commanding.
- **Safety reflex**: PX4 collision prevention (`CP_DIST` + sim distance sensor)
  is the firmware backstop. v1 builds **no** autonomous avoidance.
- **Refinement path (deferred, same interface)**: object detection/tracking →
  depth→3D mapping → SLAM → ROS2 backend + autonomous avoidance.

**Open choice (noted, not blocking):** depth camera (turnkey, forward-facing,
enables PX4 collision prevention) vs. 2D lidar ring (true 360° surroundings,
more setup). Lean depth-camera-first.

## 8. Geo math (correctness/safety hotspot)

`relative N/E/up → target lat/lon/abs-alt` is a pure, side-effect-free, heavily
unit-tested module with **explicit frame conventions documented at the boundary**
(PX4/MAVLink is NED; many libs are ENU — sign errors send the drone the wrong
way; altitude-frame errors fly it into the ground). It also enforces the
per-command max-distance clamp before SafetyGuard sees the absolute target.

## 9. Configuration

`config.py` is the **only sim-vs-real seam**. Holds: geofence radius, altitude
cap, max goto distance, home position, mavsdk_server address/port, connection
URL, telemetry rates, model selection, perception sensor selection. Safety
limits are config-driven with **conservative fail-closed defaults**. The
sim→hardware transition should be (ideally) a config + connection-string change,
not a code change.

## 10. Repo layout

```
drone/
  README.md            .env.example      pyproject.toml
  src/dronebot/
    config.py          # fail-closed limits, geofence, conn URLs — sim-vs-real seam
    flight_log.py      # structured flight-record
    app.py             # owns the single event loop; wires layers
    control/
      executor.py      # CommandExecutor — portable command boundary (typed args/results)
      controller.py    # DroneController (MAVSDK), fire-and-monitor actions
      state.py         # StateStore — authoritative drone state
      telemetry.py     # background task: MAVSDK streams -> StateStore
      safety.py        # SafetyGuard — invariants, preconditions, clamps
      geo.py           # pure NED offset -> lat/lon/abs-alt, frame-explicit, tested
    perception/
      provider.py      # PerceptionProvider interface + PerceptionSnapshot
      gazebo_perception.py  # v1 sensor reader
      store.py         # PerceptionStore — latest snapshot
    agent/
      claude_agent.py  # ClaudeSDKClient + system prompt wiring
      tools.py         # @tool adapters -> CommandExecutor / PerceptionStore; return is_error
      prompts.py       # system prompt
    chat/
      repl.py          # async stdin; renders from stores; direct (non-LLM) abort
  scripts/run_sim.sh   # make px4_sitl gz_x500_depth (Harmonic, Ubuntu 22.04)
  tests/               # geo + safety (no sim), executor (vs SITL), loop-spike
```

## 11. Testing strategy

- **No-sim unit tests**: `geo.py` (frame fixtures, sign/altitude correctness),
  `safety.py` (each invariant independently of the LLM).
- **Loop spike** (§5.1): MAVSDK coroutine awaited from a Claude SDK in-process
  tool, against PX4 SITL.
- **Integration (vs PX4 SITL, headless)**: `CommandExecutor` round-trips —
  arm → takeoff → goto → land; safety rejections; abort path; telemetry-verified
  completion.
- **Agent**: dry-run mode where tools log instead of fly; structured-error
  handling.

## 12. Build order

1. **Spike the event loop** (§5.1) — go/no-go for the in-process design.
2. Sim bring-up script + PX4 geofence/collision-prevention params.
3. `geo.py` + `safety.py` with unit tests (no sim).
4. `DroneController` + `StateStore` + `telemetry.py` against SITL; fire-and-
   monitor actions; `CommandExecutor`.
5. `PerceptionProvider`/`GazeboPerception`/`PerceptionStore` (easy sensors).
6. Claude agent: tools (commands + look/scan), prompts, `ClaudeSDKClient`.
7. REPL: async stdin, status rendering, direct abort.
8. `flight_log.py` wired through (start in step 4, complete here).
9. End-to-end: chat → fly → perceive → report, in 3D sim.

## 13. Decision Log

### D1 (2026-05-30) — Agent layer = Claude Agent SDK; ROS tools attach to it, not ROSA
Evaluated nasa-jpl/rosa (★1528, mature) as an alternative agent layer. Decision:
**keep the Claude Agent SDK** as the agent runtime. Reasoning:
- ROSA's value is its ROS tools; it **requires ROS** (out of scope until the
  perception/spatial-autonomy phase) and is a thin LangChain ReAct wrapper — the
  *reasoning* is whatever LLM you plug in, and the *harness* is generic LangChain,
  not a purpose-built agentic loop. The Claude Agent SDK is a purpose-built engine
  tuned for Claude's native tool-use (parallel tools, prompt caching, context
  management, interrupts) and is already validated end-to-end here on OAuth.
- ROSA has **no safety layer** (ours, `SafetyGuard`, is the field differentiator),
  and no UAV/MAVSDK/PX4 awareness.
- **Implication for the future ROS phase:** we do NOT need to adopt ROSA even then —
  ROS tools can be exposed to our existing Claude Agent SDK agent as in-process
  `@tool`s or a ROS MCP server, keeping the better harness. ROSA stays a
  **LEARN-FROM tool catalog**, not an adopted framework.
- Portability of the agent layer remains contained at the `CommandExecutor` seam.

### D2 (2026-05-30) — Don't depend on immature LLM-drone repos
Live GitHub stats showed the command-layer candidates (droneserver ★4, EchoPilot
★17 [no license], MAVLinkMCP ★16, embodied-drone-agents ★22 [no license],
ros2-px4-agent-ws ★25) are all single-author / immature, two unlicensed. For a
safety-critical, hardware-bound project we **reuse mature libraries** (PX4,
Gazebo, MAVSDK-Python ★434, Claude Agent SDK, noVNC, PX4 geofence) and **own the
thin glue + safety layer**, borrowing ideas (e.g. droneserver's command interlock)
rather than taking code dependencies. bluerobotics/cockpit (★179, active) is the
only credible *UI* reuse candidate, deferred for separate evaluation.

### D3 (2026-05-30) — Positioning-command interlock (from droneserver's crash)
Implemented in `CommandExecutor`: at most one positioning maneuver
(`takeoff`/`goto_relative`/`return_to_launch`) in flight at a time. A new
positioning command is refused while the active one hasn't reached its target
(distance/altitude check against the authoritative `StateStore`). Completion is
re-evaluated on each new command, so the flag never sticks; `hold`/`land` are
terminators that clear it (the "say 'stop' to override" path). This closes the
command-stacking crash class droneserver documented, using our own tested layer
rather than a dependency. Covered by `tests/test_executor.py`.
