# Dronebot Swarm — Mission-Generic LLM Agent Swarm — Design

**Date:** 2026-06-14
**Status:** Approved (pending spec review)
**Author:** brainstormed with Claude
**Supersedes direction of:** `2026-05-29-llm-uav-chatbot-design.md` (single-drone v1). This is a
**fresh start** — same core idea (talk to UAVs in natural language; they are aware of their
surroundings), now scaled to a **hierarchical swarm of LLM sub-agents** on ROS2. We salvage useful
pieces from v1 but rebuild the system around ROS2.

## 1. Summary

A **mission-generic agent swarm**: you give a natural-language mission to a **Commander** agent; it
decomposes the mission and delegates to N **Drone** agents, each its own Claude process flying its own
PX4 drone, running its own SLAM, and **communicating with the other drone agents in free natural
language**. Each drone's local map is fused into a single **global map** in the shared GPS frame. Runs
entirely in simulation (PX4 SITL + Gazebo). This is an **experiment in emergent multi-agent
coordination**, not a safety-critical or hardware-bound system (see §6).

## 2. Goals & non-goals

**Goals (v1 of the swarm)**
- A **Commander** Claude agent takes a generic NL mission and decomposes it across drones.
- N **Drone** Claude agents (own process each), each: flies a PX4 drone, runs its own SLAM, perceives
  its surroundings, and talks to peer drone agents.
- **Per-drone local SLAM** fused into **one global map** in the shared GPS frame (map stitching, not
  collaborative SLAM — see §4).
- **Free natural-language peer-to-peer chat** between drone agents as the coordination medium.
- **Mission genericity**: the commander composes a small, generic task vocabulary; no hardcoded
  per-mission-type logic. The LLM does the mission-specific reasoning.
- Runs in full 3D simulation with 2 drones (3 = stretch).

**Non-goals (explicitly cut or deferred)**
- **Safety / hardware transfer.** Sim-only. No safety supervisor, no geofence, no altitude caps, no
  fail-closed clamping, no "transfers to real hardware unchanged" constraint. (See §6.)
- **Collaborative SLAM** (cross-robot loop closure, GPS-denied). The GPS frame gives a shared
  reference for free; fusion is map stitching. The hard wheel (Swarm-SLAM/Kimera-Multi) is deferred
  behind the fusion-server seam for a future GPS-denied mission.
- **Deterministic coordination / deconfliction / leases.** Coordination is pure chat; double-booking
  of regions is possible and is part of what we are observing.
- **Decentralized topology.** Commander is a real hierarchical brain (not peer self-organization).
- Web/voice UI, object-detection pipelines beyond what SLAM/perception needs, mission-pattern
  libraries.

## 3. Key decisions (from brainstorming)

- **D1 — Fresh start, reuse-maximizing.** Inverts v1's "own thin glue, avoid immature repos" lean.
  We reuse mature infrastructure wheels (PX4, ROS2, Nav2-ecosystem SLAM, map_merge/OctoMap, Claude
  Agent SDK) and salvage useful v1 code (`geo.py`, patterns from control/perception). Hybrid reuse:
  adopt mature *infrastructure*, own the orchestration + agent behavior.
- **D2 — Hierarchical topology.** One Commander agent + N Drone sub-agents. Single place for mission
  logic; fits the "commander decomposes & delegates" model. Decentralization deferred.
- **D3 — One fused global map.** The shared picture lives in a fused map, not just in conversation.
- **D4 — Outdoor / GPS available.** Shared reference frame is free (lat/lon/alt), so fusion = map
  stitching (`map_merge` / OctoMap in common frame), **not** collaborative SLAM. Backend kept
  swappable so a GPS-denied mission can later pull in the hard wheel.
- **D5 — Process-per-agent over the bus.** Commander is one Claude process; each drone is its own
  Claude process with its own SLAM node and tools. "Each drone has its SLAM" and "sub-agents talk to
  each other" are literally true, mapping 1:1 onto ROS2 per-drone namespaces. Not SDK subagents in one
  process.
- **D6 — Pure free natural-language peer chat.** Drone agents coordinate only by talking in English on
  `/swarm/chat`. No arbiter. Emergent division of labor; we watch what happens.
- **Consequence — ROS2 is the substrate.** Inverts v1's D1/D4 (which avoided ROS). The swarm + SLAM +
  comms wheels live in the ROS2 ecosystem.

## 4. Architecture

```
                          ┌────────────── COMMANDER HOST ──────────────┐
   you ──NL mission──▶    │  Commander Agent  (Claude Agent SDK)        │
                          │    tools: decompose, delegate, query_world, │
                          │           monitor, message, replan          │
                          │  Global World Model (positions, findings)   │
                          │  Map Fusion Server (OctoMap, GPS frame)     │  ← stitches submaps
                          └───────▲────────────────────────┬───────────┘
                       status / submaps / findings    delegation (NL + task msgs)
        ══════════════════════ ROS2 / DDS bus ══════════════════════════
              │  /drone_1                     │  /drone_2        (… /drone_N)
   ┌──────────┴───────────┐         ┌─────────┴────────────┐
   │ Drone Agent (Claude) │◀─ free NL peer chat (/swarm/chat) ─▶          "I'll take the
   │  tools: fly, look,   │         │                      │               north strip"
   │  query_local_map,    │         │                      │
   │  message_peer, report│         └──────────────────────┘
   │ Control (MAVSDK)     │  ── MAVSDK ──▶ PX4 SITL_i ──▶ Gazebo
   │ SLAM node (local map)│  ◀── depth cam / sensors ──
   └──────────────────────┘
```

**Coordination vs. data split (important):**
- **Coordination = free NL chat** on `/swarm/chat`. How drones *decide*.
- **Data = structured typed topics** (`/drone_i/status`, `/drone_i/map`, `/swarm/findings`). Facts
  that feed the fused map and world model. These stay typed because they are sensor/state data, not
  negotiation.

**Component → wheel ledger (the "don't reinvent" map):**

| Box | Wheel (reuse) | We own |
|---|---|---|
| Flight + multi-vehicle sim | **PX4 SITL** (multi-vehicle) + **Gazebo** | launch config |
| PX4 ↔ ROS2 | **micro-XRCE-DDS** (PX4-native) or **MAVROS** | — |
| Per-drone SLAM | **RTAB-Map** (depth cam) or **SLAM Toolbox** (lidar) | sensor wiring; default depth cam (x500_depth) |
| Map fusion (global) | **`map_merge`** / **OctoMap server** in GPS frame | fusion node config |
| Autonomous exploration | **`explore_lite`** (frontier) as a drone tool | when to invoke |
| Agents (commander + drones) | **Claude Agent SDK** | prompts, tool sets, orchestration |
| Inter-agent comms | **ROS2 / DDS** (topics, services, actions) | message schema (`swarm_msgs`) |
| Geo / frame math | salvage v1 **`geo.py`** | — (already tested) |

## 5. Mission lifecycle & comms

```
1. You → Commander (NL):  "search the 200×200m area north of home for vehicles"
2. Commander reasons → carves area into sub-regions, forms a task per drone
3. Commander → Drone_i:  delegation (NL message + optional typed task)
4. Drone_i agent executes via its tools (fly / explore / look / query_local_map)
5. Drone_i → bus:  status, local submap (→ fusion), findings (in GPS frame)
6. Map Fusion Server stitches submaps → global OctoMap;  World Model updates
7. Commander monitors growing global picture → re-delegates / replans
8. Drones coordinate laterally via free NL chat (no arbiter)
9. Mission-complete, or you interrupt
```

**Generic task vocabulary** (small on purpose — the genericity seam): `goto`, `explore_region`,
`search_region_for(target)`, `inspect(points)`, `hold`, `return`. Any phrased mission is expressed as
a sequence of these across drones.

**Comms channels:**

| Channel | ROS2 mechanism | Purpose |
|---|---|---|
| Commander → Drone: delegate task | **Action** (`/drone_i/task`) | long-running, feedback + cancel |
| Drone → all: status/pose/battery | Topic (`/drone_i/status`) | world model, peer awareness |
| Drone → fusion: local submap | Topic (`/drone_i/map`) | feeds global OctoMap |
| Drone → all: findings/detections | Topic (`/swarm/findings`) | semantic results in GPS frame |
| Commander → all: fused snapshot | Topic (`/swarm/world`) | shared picture |
| Peer ↔ peer: agent chat | Topic (`/swarm/chat`) | free NL coordination (D6) |

## 6. What's cut, and why it's safe to cut

Sim-only experiment → the entire safety apparatus from v1 is removed: per-drone safety supervisor,
geofence, altitude caps, fail-closed argument clamping, reflex separation-hold, and the
hardware-transfer constraint. No deconfliction arbiter or airspace leases either (D6). The one piece
of v1 "safety" code that survives is **`geo.py`** — kept purely for **correctness** (wrong NED↔lat/lon
frame math sends drones to the wrong place and corrupts the fused map), not safety.

Accepted consequence: drones may double-book regions or collide in sim. That is an observable outcome
of pure-chat coordination, not a defect to prevent in this version.

## 7. Highest-risk unknown — spike first

**Bridging `rclpy` (ROS2, executor/callback-based) with the Claude Agent SDK + MAVSDK
(asyncio-native).** ROS2's Python client is not asyncio-native; each drone process must marry a ROS2
executor with an asyncio loop (e.g. run the rclpy executor in a thread and hand messages to the async
loop, or use an async rclpy pattern). This is the analog of v1's event-loop spike. **If the bridge
holds, the design holds.** Spike it before building anything else.

## 8. Build order

1. **Bridge spike** — one Claude agent process ↔ `rclpy` ↔ PX4 SITL: fly via MAVSDK and read one SLAM
   map topic on one process. Go/no-go for the whole design.
2. **Single-drone slice** — NL → drone agent → fly + local SLAM + `look()`. (v1, re-based on ROS2.)
3. **Map fusion** — add OctoMap/`map_merge`; one drone produces a global map in the GPS frame.
4. **Second drone + namespacing** — 2× PX4 SITL under `/drone_1`, `/drone_2`; fused map from both.
5. **Commander agent** — NL mission → decompose → delegate to the two drones.
6. **Free chat** — `/swarm/chat`; drones coordinate emergently.
7. **End-to-end** — "search this area" with 2–3 drones, fused map, emergent division of labor.

## 9. Repo layout (fresh)

```
dronebot-swarm/
  docker/            ROS2 Humble + PX4 + Gazebo image
  sim/launch/        swarm.launch.py — N× PX4 SITL + Gazebo + per-drone SLAM + map_merge
  sim/worlds/
  ros2_ws/src/
    drone_bringup/   per-drone: micro-XRCE-DDS bridge, SLAM node wiring, control
    map_fusion/      OctoMap / map_merge config (global GPS frame)
    swarm_msgs/      typed msgs: Status, Finding, ChatMsg, Task
  agents/
    common/
      bus.py         rclpy ↔ asyncio bridge (the §7 risk lives here)
      llm.py         Claude Agent SDK setup
      geo.py         salvaged from v1 (frame-correct NED↔lat/lon)
    commander/
      agent.py       NL mission → decompose → delegate → monitor
      tools.py       decompose, delegate, query_world, monitor, message
      prompts.py
    drone/
      agent.py       per-drone Claude agent
      tools.py       fly (MAVSDK), look, query_local_map, message_peer, report
      prompts.py
  config.py          sim params, N drones, model selection, sensor selection
  tests/
```

## 10. Testing

- **`geo.py` unit tests** — salvaged from v1 (frame/sign/altitude correctness).
- **Bridge spike** (§7) — the go/no-go integration test.
- **Single-drone integration** vs PX4 SITL — fly + SLAM map present + `look()`.
- **Swarm scenarios** — *qualitative*. With pure-chat coordination there is no deterministic
  allocation to assert; we log the chat transcript + the fused map and observe emergent behavior
  (coverage, overlap, double-booking, collisions).

## 11. Reuse-from-v1 ledger

- **Salvage:** `geo.py` (verbatim); patterns/ideas from `control/executor.py`, `control/state.py`,
  `control/telemetry.py`, `perception/` (provider→snapshot pattern), `agent/` (Claude SDK wiring),
  `chat/repl.py` (async stdin), and the Docker/Gazebo/PX4 bring-up learnings (llvmpipe, standalone
  mavsdk_server, x500_depth camera topics) from recent commits.
- **Drop:** `safety.py` (no safety in sim), the single-loop single-drone `app.py` wiring (replaced by
  per-process agents on a bus), the v1 web cockpit (deferred).

## 12. Open choices (noted, not blocking)

- **Per-drone SLAM stack:** depth-camera + RTAB-Map (turnkey, matches x500_depth from v1) vs 2D/3D
  lidar + SLAM Toolbox/Cartographer. Lean depth-camera-first.
- **PX4 ↔ ROS2 bridge:** micro-XRCE-DDS (PX4-native, modern) vs MAVROS (mature, MAVLink). Lean
  micro-XRCE-DDS; revisit if SLAM/sensor packages expect MAVROS.
- **Scale:** start N=2; N=3 stretch. Watch compute *and* token cost (N continuously-reasoning Claude
  processes).
- **Commander↔drone delegation medium:** ROS2 action (typed, cancelable) vs NL message. Lean typed
  action for delegation + monitoring, NL reserved for `/swarm/chat` peer coordination.
