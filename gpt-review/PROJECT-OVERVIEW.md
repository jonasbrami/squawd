# Project overview

## What the project does

Squawd is a research system for letting a large language model operate a
simulated UAV through high-level tools. In the current rebuild, an operator
sends natural-language instructions to one pilot agent. The pilot uses
classical flight and perception primitives rather than emitting motor commands
or high-rate setpoints directly.

The runtime brings together:

- Gazebo Harmonic for the physical world and camera/range sensors;
- PX4 SITL for the flight controller;
- uXRCE-DDS and ROS 2 topics for PX4 telemetry and local application messages;
- MAVSDK for arm, takeoff, navigation, mission, orbit, hold, land, and offboard
  pursuit commands;
- a Claude Agent SDK client, optionally pointed at the Kimi Code endpoint, for
  the pilot reasoning loop;
- an ONNX, Ultralytics, or color-blob detector;
- a constant-velocity EKF contact tracker with camera geometry and optional
  time-of-flight range fusion;
- a browser cockpit for camera video, telemetry, detections, chat, commands,
  and emergency hold/land;
- declarative task evaluations and deterministic, simulator-truth grading.

The LLM is deliberately kept out of fast control loops. It chooses tools and
parameters. MAVSDK/PX4 executes ordinary maneuvers, while `track()` uses a 10 Hz
classical controller for moving-target shadow or intercept behavior.

## Current scope versus historical scope

The active source tree and the living project-state document describe a
single-drone rebuild:

- the active assembler is [`agents/pilot/run.py`](../agents/pilot/run.py);
- the active agent is [`agents/pilot/agent.py`](../agents/pilot/agent.py);
- the active operator topics are `/pilot/user_input`, `/pilot/chat`, and
  `/pilot/estop`;
- the current eval harness rejects multi-drone use in its active client path;
- the Commander implementation and swarm assembler were deliberately removed
  by the rebuild design.

The repository still contains historical swarm material:

- [`README.md`](../README.md) and [`docs/architecture.md`](../docs/architecture.md)
  present the Commander plus N autonomous drones as the current product;
- [`scripts/run_swarm_demo.sh`](../scripts/run_swarm_demo.sh) invokes the absent
  `agents/swarm/run.py`;
- [`agents/swarm/drone.py`](../agents/swarm/drone.py) remains as a legacy class;
- `bench/`, older task YAML, evaluation outputs, and benchmark reports preserve
  prior swarm experiments.

Accordingly, “LLM-piloted UAV swarm” is the project history and longer-term
direction; “single LLM-piloted simulated UAV with perception and evals” is the
implemented product at this snapshot.

## User-visible capabilities

### Natural-language piloting

The operator submits a command. `PilotAgent` forwards it to the backend client
with instructions to use tools and report a concise result. The tool surface is
assembled in [`agents/flight/tools.py`](../agents/flight/tools.py):

- `take_off`
- `fly`
- `goto`
- `orbit`
- `hover`
- `set_speed`
- `face`
- `land`
- `scan`
- `run_mission`
- `track`
- `report`
- `detect` when the perception pipeline is available

### Perception and target tracking

Frames arrive directly from Gazebo transport. A detector runs independently of
the LLM and emits immutable results. The vision pipeline projects detections
through recorded vehicle pose and attitude, creates or updates contact tracks,
optionally associates the forward ToF beam, and publishes a compact perception
snapshot. The pilot sees that state through textual `detect` and `scan` tools.

### Flight primitives

[`FlightOps`](../agents/flight/ops.py) provides blocking or explicitly
non-blocking movement, symbolic target resolution, obstacle-aware target
rejection, orbiting, heading changes, landing, uploaded missions, and moving
contact pursuit. World coordinates are east/north/up; MAVSDK/PX4 uses GPS and
NED internally, so the world and geo helpers own those conversions.

### Emergency action

An independent estop supervisor watches `/pilot/estop`. It cancels the active
tool task, waits for cleanup, and then commands hold or land using the same
`FlightOps` instance as the pilot tools. This is a sensible single-owner design
for avoiding a stale controller resuming after an estop.

### Cockpit

[`agents/observatory/server.py`](../agents/observatory/server.py) defines a
single-drone Starlette application with telemetry state, H.264 camera streaming,
detection streaming, chat polling, command submission, and hold/land endpoints.
It is a separate process from the pilot.

### Evaluation and research outputs

`evals/` expands declarative YAML tasks into model/repeat cells, resets the
simulator between cells, records tool-level transcripts and usage, samples world
state, and grades it with pure oracle functions. It supports explicit
truth-fed and vision-fed control lanes while reserving Gazebo truth for sampling
and grading in the production-like lane. `bench/` is the older infrastructure
capacity benchmark for render backend, camera rate, real-time factor, and drone
count.

## How it is intended to run

The intended single-drone flow is:

1. Build `squawd:dev` from `docker/Dockerfile.swarm`.
2. Have a compatible, already-built `PX4-Autopilot` tree at the repository root.
3. Provide Claude OAuth credentials, or set `SQUAWD_BACKEND=kimi` and a
   `KIMI_API_KEY`.
4. Run `scripts/run_single_demo.sh [world]` with an Intel or NVIDIA render
   backend for the camera-dependent path.
5. Start `agents/observatory/server.py` separately if the browser cockpit is
   desired; the current single-drone launcher starts the simulator and pilot but
   not the cockpit.
6. Send commands through the cockpit or publish `std_msgs/String` messages to
   `/pilot/user_input`.

This is the intended path inferred from the current scripts and entrypoints,
not a claim that a clean-clone quickstart presently works. The blockers are
listed in [ISSUES.md](ISSUES.md).

## Repository map

| Path | Current role |
|---|---|
| `agents/core/` | ROS bridge, stores, camera frames, telemetry history, Gazebo truth reader, range samples, shared DTOs |
| `agents/world/` | World configuration, ENU/NED mapping, target resolution, scripted trajectories |
| `agents/perception/` | Pure geometry and human-readable spatial descriptions |
| `agents/vision/` | Detection backends, detector worker, pipeline, trackers, CV-EKF contacts, ToF association |
| `agents/flight/` | Safety envelope, flight primitives, pursuit controller, MCP tools, backend event seam |
| `agents/pilot/` | Single-drone assembly, LLM loop, detect formatter, estop supervisor |
| `agents/observatory/` | Browser cockpit server, telemetry shaping, overlays, H.264 streaming, static UI |
| `agents/swarm/` | Legacy residue; no runnable Commander/swarm assembly remains |
| `sim/` | Gazebo/PX4 launch, custom model, procedural worlds, mover plugin |
| `evals/` | Task schema, matrix runner, simulator sampler/reset, deterministic oracle, reports, data |
| `bench/` | Historical swarm/render capacity benchmark |
| `tests/` | Unit and contract tests, including eval and benchmark helpers |
| `models/` | Trained mover segmentation ONNX model and manifest |
| `docs/` | Historical architecture, design specifications, living state, and benchmark evidence |

