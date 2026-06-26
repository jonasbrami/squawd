# squawd — LLM-piloted UAV swarm

A swarm of drones you command in **plain language**. You are the *Commander*: you
type into a browser ("everyone take off and spread out", "drone_1 climb to 20m",
"all return and land") and an LLM **Commander agent** decomposes your intent into
**directed per-drone tasks**. Each drone is its own LLM agent — its own onboard
thinking, able to run on its own hardware — that carries out the task it is given
and **reports back** to the Commander.

Everything runs self-contained in one Docker container: **Gazebo Harmonic** +
**PX4 SITL** (N flight controllers) + **ROS 2 Jazzy** + per-drone **onboard
cameras** + a web **Observatory**.

![Swarm Observatory](docs/img/observatory.png)

The Observatory (above) shows, live: each drone's **onboard camera POV** (top),
a **top-down map** of the swarm, and the **swarm feed** where the Commander's
dispatches and the drones' reports scroll by. Type a command at the bottom and
watch the Commander delegate.

---

## What you get

- **Natural-language command** of an N-drone swarm — no waypoints, no scripting.
- **Hierarchical agents**: one Commander that dispatches directed tasks to N
  autonomous drone agents — each with its own thinking — which report back.
- **Per-drone onboard cameras** rendered on the GPU, streamed to the browser.
- **A realistic "baylands" world** (PX4's coastal scene — road, grass, water,
  trees) by default, so the drones and their cameras have something real to see.
  `WORLD=city` swaps in a procedural building world (and is the only world with
  building obstacle-`scan`). *(Baylands' tree foliage renders as dark silhouettes
  under headless EGL — a known artifact; the rest of the scene is unaffected.)*
- **Scales with N** — `./scripts/run_swarm_demo.sh 5` just works; ports, namespaces,
  camera tiles, and agents are all derived from the drone index.

---

## Architecture

The code is **six small Python packages** with a one-directional dependency graph,
so each layer reads — and unit-tests — on its own.

### Code modules

```mermaid
flowchart TD
    classDef orch fill:#1e3a8a,stroke:#93c5fd,color:#eff6ff;
    classDef heavy fill:#374151,stroke:#9ca3af,color:#f3f4f6;
    classDef pure fill:#065f46,stroke:#6ee7b7,color:#ecfdf5;

    SWARM["<b>agents.swarm</b> · run.py + commander.py + drone.py<br/>CommanderAgent + N DroneAgent objects · wiring"]:::orch
    OBS["<b>agents.observatory</b> · server.py<br/>web UI — camera tiles · map · chat"]:::orch
    FLIGHT["<b>agents.flight</b> · ops.py + tools.py<br/>FlightOps (take_off·goto·orbit·…) + MCP tool bindings"]:::heavy
    CORE["<b>agents.core</b> · bus · store · geo · camera<br/>RosBridge · TopicLog · GeoPoint · GzCameras"]:::heavy
    PERC["<b>agents.perception</b><br/>scan / situation text · bearings — pure trig"]:::pure
    WORLD["<b>agents.world</b> · World<br/>city_boxes.json · NED→ENU · resolve targets — pure"]:::pure

    SWARM --> FLIGHT
    SWARM --> CORE
    SWARM --> PERC
    SWARM --> WORLD
    FLIGHT --> CORE
    FLIGHT --> PERC
    OBS --> CORE
    FLIGHT -. operates on .-> WORLD
    PERC -. operates on .-> WORLD
```

*Solid arrows = imports; dotted = used at runtime. The **green** packages are pure
(no sim dependencies) and are covered by host-side unit tests.*

- **`agents.core`** — primitives only: the ROS bridge + QoS (`bus`), thread-safe
  holders (`store`: `LatestStore`, `TopicLog`), GPS offset math (`geo`), and the one
  camera reader (`camera`: `GzCameras`).
- **`agents.world` + `agents.perception`** — ground truth + telemetry → the drone's
  sense of place (positions, bearings, "what's in view", scan/situation text).
- **`agents.flight`** — `FlightOps` is the flight logic; `tools.py` binds it to
  Claude-Agent-SDK `@tool`s. Logic and SDK plumbing are separable.
- **`agents.swarm`** — the agent layer as objects: `CommanderAgent` (`commander.py`)
  and `DroneAgent` (`drone.py`), each owning its ROS channels, its Claude client, and
  an `async run()` loop; `run.py` is a thin assembler that constructs them and
  `gather`s their loops. **`agents.observatory`** is the web UI and depends only on
  `core`.

### Runtime & data flow

One container, several cooperating processes. The simulator and flight stack are
classic PX4/Gazebo; the novel part is the **agent layer** and how it reads/acts on
the sim.

```mermaid
flowchart TB
    subgraph Browser["🌐 Browser — you are the Commander"]
        UI["Observatory UI<br/>cameras · map · chat · command box"]
    end

    subgraph Container["🐳 swarm-multi container"]
        subgraph Agents["Agent layer — agents/swarm/run.py (one asyncio process)"]
            CMD["🧠 Commander agent<br/>decomposes your intent → per-drone orders"]
            D0["🤖 drone_0 agent"]
            D1["🤖 drone_1 agent"]
            D2["🤖 drone_N agent"]
        end

        OBS["📡 Observatory<br/>agents/observatory/server.py (Starlette)"]

        subgraph Sim["Simulation + flight stack"]
            GZ["Gazebo Harmonic<br/>physics + GPU camera render"]
            PX4["PX4 SITL ×N<br/>px4 -i 0..N-1"]
            XRCE["uXRCE-DDS Agent<br/>:8888"]
            MAV["mavsdk_server ×N<br/>:50051+i"]
        end
    end

    UI -- "POST /command" --> OBS
    OBS -- "/swarm/user_input (ROS2)" --> CMD
    CMD == "/swarm/cmd/drone_i (directed task)" ==> D0 & D1 & D2
    D0 & D1 & D2 == "/swarm/report/drone_i (result)" ==> CMD
    CMD -. "/swarm/chat (mirror for UI)" .-> OBS
    D0 & D1 & D2 -- "arm/takeoff/goto (MAVSDK gRPC)" --> MAV
    MAV -- "MAVLink udp 14540+i" --> PX4
    PX4 <--> GZ
    PX4 -- "telemetry" --> XRCE
    XRCE -- "/px4_i/fmu/out/* (ROS2)" --> OBS & D0 & D1 & D2
    GZ -- "camera topics (gz-transport)" --> OBS
    OBS -- "camera tiles (/ws) + /state" --> UI
```

### Data buses

| Bus | Carries | Transport / QoS |
|-----|---------|-----------------|
| `/swarm/user_input` | **You → Commander** (your typed commands) | ROS 2, RELIABLE + TRANSIENT_LOCAL |
| `/swarm/cmd/drone_<i>` | **Commander → drone_i** — a directed task | ROS 2, RELIABLE + TRANSIENT_LOCAL |
| `/swarm/report/drone_<i>` | **drone_i → Commander** — its result | ROS 2, RELIABLE + TRANSIENT_LOCAL |
| `/swarm/chat` | Read-only mirror (dispatches + reports) for the UI feed | ROS 2, RELIABLE + TRANSIENT_LOCAL |
| `/px4_<i>/fmu/out/*` | PX4 telemetry (position, status…) | ROS 2, BEST_EFFORT (via uXRCE-DDS) |
| gz camera topic | Per-drone camera frames | gz-transport13 (read directly, no ros_gz) |
| MAVSDK gRPC / MAVLink | Flight commands (arm, takeoff, goto) | gRPC `:50051+i` ⇄ MAVLink `udp:14540+i` |

### Why these choices

- **uXRCE-DDS** bridges PX4 ⇄ ROS 2 so agents and the Observatory read telemetry as
  normal ROS topics, namespaced per drone (`/px4_0/...`, `/px4_1/...`).
- **MAVSDK** (one `mavsdk_server` per drone) gives the drone agents clean
  arm/takeoff/`goto_location` calls. The server is **version-matched** to the pip
  client (a mismatch silently hangs `connect()`).
- **gz-transport read directly** for cameras — `ros_gz` was dropped because its
  vendored Gazebo broke the system `gz sim`; the Observatory subscribes to the gz
  image topics itself and re-encodes to MJPEG.
- **One world named `city`** generated from PX4's `default.sdf` with injected
  building boxes. The world name, the file name, and `PX4_GZ_WORLD` are kept
  identical — otherwise the gz-launching PX4 instance calls a `/world/<name>/create`
  service that doesn't exist and dies on a spawn timeout.

---

## Agent organization

Each agent is a self-contained object — **`CommanderAgent`** (`agents/swarm/commander.py`)
and **`DroneAgent`** (`agents/swarm/drone.py`) — that owns its ROS channels, its
persistent **Claude Agent SDK** client, and an `async run()` loop;
**`agents/swarm/run.py`** is a thin assembler that builds them and `gather`s their
loops. They run as coroutines in one asyncio process today, but talk **only over ROS
topics** — so a drone can be split onto its own onboard computer with no code
change. The model is a **distributed hub**: the Commander is the one node that
talks to the human and tasks drones; the drones never hear each other.

```mermaid
flowchart TD
    Human([You / Observatory])
    CMD{{Commander agent}}
    D0[drone_0 agent]
    D1[drone_1 agent]
    D2[drone_N agent]
    Human -- "/swarm/user_input" --> CMD
    CMD == "/swarm/cmd/drone_0 (task)" ==> D0
    CMD == "/swarm/cmd/drone_1" ==> D1
    CMD == "/swarm/cmd/drone_N" ==> D2
    D0 == "/swarm/report/drone_0 (result)" ==> CMD
    D1 == "/swarm/report/drone_1" ==> CMD
    D2 == "/swarm/report/drone_N" ==> CMD
    CMD -. "/swarm/chat (mirror)" .-> Human
```

**`CommanderAgent`** (`agents/swarm/commander.py`)
- Owns the `/swarm/user_input` inbox **and** every `/swarm/report/drone_<i>` channel;
  `run()` polls them both.
- For each user command, builds a prompt with the live **situation map** (positions,
  facing, nearest buildings — from `agents.perception`) and **dispatches a directed
  task to each drone** that should act.
- For each drone **report**, decides whether a follow-up is needed (otherwise just
  summarizes for you), keeping the loop from re-tasking drones that are already done.
- `dispatch(drone_id, task)` (the Claude tool, backed by the method of the same name)
  → publishes the task to `/swarm/cmd/drone_<i>` and mirrors
  `commander→drone_<i>: <task>` to `/swarm/chat` for the UI.

**`DroneAgent`** (`agents/swarm/drone.py`, one instance per drone, `drone_0 … drone_<N-1>`)
- Owns its MAVSDK link (`System` on `:50051+i`), its PX4 telemetry subscription, and
  **its own** `/swarm/cmd/drone_<i>` inbox only — no shared chat, no message filtering.
  `connect()` brings up the link + geofence; `run()` acts only when the Commander
  tasks it.
- Tools = `agents.flight.FlightOps` bound as in-process MCP tools (`tools.py`),
  MAVSDK underneath:
  - **move** — `take_off`, `goto` (absolute world point or a named target like
    `bldg_7`/`drone_1`), `orbit` (circle a target, camera on it), `fly` (relative),
    `face`, `hover`, `set_speed`, `land`
  - **sense** — `scan` (nearby buildings + drones with bearing, `agents.perception`),
    `look` (live camera frame **fed to Claude's vision** — see below)
  - **report** — `report(message)` (the `DroneAgent.report` method, exposed as a tool)
    → publishes the result to `/swarm/report/drone_<i>` (mirrored to `/swarm/chat`)
- System prompt: carry out the task with your tools, then report back; be terse.

#### Claude is the vision model too (multimodal, not just text)

The drones don't just reason over text — they **see**. The same Claude that plans
and calls tools also processes images natively, so `look` turns the onboard camera
into genuine perception:

1. `agents.core.GzCameras` grabs the drone's RGB frame off gz-transport and
   `jpeg_b64()` encodes it (JPEG → base64 — base64 is just transport, so the bytes
   survive JSON/HTTP; it is **not** how Claude "reads" the image).
2. The `look` tool returns it as a typed **image** content block —
   `{"type": "image", "data": <b64>, "mimeType": "image/jpeg"}` (`agents/flight/tools.py`).
3. The Claude Agent SDK forwards that block to the API, where the `type: "image"`
   tag routes it into Claude's **vision encoder** (base64 → pixels → image tokens) —
   the same model, attending to pixels and text together.

So a drone can `look` and, in its very next thought, write *"open parkland with
paved paths and scattered trees, a parking lot to the south"* — real visual
understanding, not OCR or a bolt-on detector. (It is point-in-time and qualitative:
distances/coordinates still come from the `scan`/ground-truth channel, and each
`look` costs vision tokens, so drones use it deliberately, not continuously.)

**Observatory** (`agents/observatory/server.py`, Starlette + uvicorn)
- Pure consumer of the sim plus the one thing it publishes: your commands.
- Routes: `/` (UI), `/state` (JSON: positions + chat), `/ws` (one WebSocket of all
  camera tiles), `/cam/{i}` + `/frame/{i}` (MJPEG / single JPEG),
  `POST /command` (→ `/swarm/user_input`, and echoes `you: …` into the chat view).

### Wake dynamics — who wakes whom

Each agent is its own persistent Claude client, but it only spends tokens when
something lands on a channel it listens to. In the diagram below an **activation
bar means that Claude is awake** (running one LLM turn); no bar means idle
(subprocess alive, zero tokens). The `(wakes …)` labels are the trigger.

```mermaid
sequenceDiagram
    actor You
    participant OBS as Observatory
    participant CMD as Commander (Claude)
    participant D0 as drone_0 (Claude)
    participant D1 as drone_1 (Claude)
    participant D2 as drone_2 (Claude)
    participant SIM as PX4 / MAVSDK

    You->>OBS: type "drone_0 to 12m, drone_1 to 7m, leave drone_2"
    OBS->>CMD: /swarm/user_input  (wakes Commander)
    activate CMD
    Note over CMD: LLM turn - decompose intent into per-drone tasks
    CMD->>D0: /swarm/cmd/drone_0  (wakes drone_0)
    CMD->>D1: /swarm/cmd/drone_1  (wakes drone_1)
    CMD-->>OBS: /swarm/chat mirror (display only, no wake)
    deactivate CMD
    Note over D2: never addressed - stays asleep, 0 tokens

    activate D0
    Note over D0: LLM turn - take_off/goto via its own tools
    D0->>SIM: arm + takeoff (MAVSDK gRPC)
    SIM-->>D0: airborne ~12m
    D0->>CMD: /swarm/report/drone_0  (wakes Commander)
    D0-->>OBS: /swarm/chat mirror
    deactivate D0

    activate D1
    Note over D1: LLM turn - take_off to 7m
    D1->>SIM: arm + takeoff
    SIM-->>D1: airborne ~7m
    D1->>CMD: /swarm/report/drone_1  (wakes Commander)
    deactivate D1

    activate CMD
    Note over CMD: LLM turn - read reports, re-dispatch ONLY if goal unmet
    CMD-->>OBS: /swarm/chat mirror
    deactivate CMD
```

The wake graph is a strict hub: **you → Commander → only the addressed drones →
back to Commander**. A drone wakes only on its own `/swarm/cmd/drone_<i>`; it never
wakes another drone, and an unaddressed drone (drone_2 here) costs nothing. Each
drone's `report` re-wakes the Commander, whose prompt tells it to re-dispatch only
if the goal isn't met — so the chain terminates instead of ping-ponging.
`/swarm/chat` is display-only: nothing subscribes to it to act, so it wakes no one.

---

## Rendering: with GPU vs without

Camera rendering is the expensive part. The launcher has two paths:

| | **GPU (default, `GPU=1`)** | **No GPU (`GPU=0`)** |
|---|---|---|
| Renderer | Intel iGPU (Iris Xe, `i915`) via **EGL headless** | Software GL (**llvmpipe**) |
| Camera POV tiles | ✅ yes, real-time | ❌ disabled (too slow) |
| Real-time factor (3 cams) | **~1.0** | ~0.004 with cameras → flight only |
| Drone model | `gz_x500_depth` (has camera) | `gz_x500` (no camera) |
| Devices passed in | only `/dev/dri/renderD128` + `card1` | none |

Notes:
- Only **`renderD128`** is exposed to the container on purpose: if Mesa can see an
  NVIDIA node with no Mesa driver, `ogre2` segfaults. Intel iGPU → EGL is the
  reliable headless path here.
- Gazebo always runs **server-only** (`HEADLESS=1`) — no Qt GUI. The GUI aborts
  under offscreen Qt and would take down the gz-launching PX4 instance.
- **NVIDIA dGPU** is a future toggle (install `nvidia-container-toolkit`, run with
  `--gpus all`) for larger/faster feeds. Not required; the iGPU holds 3×640×360 at
  real time.

---

## How to run

### Prerequisites
- **Docker** on Linux.
- **Logged-in Claude CLI** on the host — the agents use your OAuth from `~/.claude`;
  **no API key needed.** (Run `claude` once to log in.) The launcher copies your
  creds to an isolated `/tmp/swarm-claude` and **never writes to the live
  `~/.claude`.**
- For cameras: a usable **GPU** (Intel iGPU works out of the box via `/dev/dri`).

### 1. Build the image (once, slow — compiles PX4 + pulls Gazebo/ROS)
```bash
docker build -f docker/Dockerfile.swarm -t squawd:dev .
```

### 2. Launch the swarm
```bash
# 3 drones, baylands world, GPU cameras (default)
./scripts/run_swarm_demo.sh 3

# more drones
./scripts/run_swarm_demo.sh 5

# procedural building world (adds building obstacle-scan)
WORLD=city ./scripts/run_swarm_demo.sh 3

# software rendering, no cameras (flight + chat only, much slower)
GPU=0 ./scripts/run_swarm_demo.sh 3
```
> First baylands launch downloads ~400MB of Gazebo Fuel terrain/water models
> (cached in `/tmp/swarm-gz-fuel`, reused after that — needs internet once).

Then open **http://localhost:8000** and command the swarm:
> *"everyone take off and climb to 12m, then spread out and scout"*
> *"drone_1 climb to 20m"* · *"all return and land"*

The Commander decomposes each command, the drones carry it out and report back in
the chat, and the camera tiles + map update live.

### Logs & stop
```bash
docker exec swarm-multi tail -f /tmp/swarm.log   # agents
docker exec swarm-multi tail -f /tmp/obs.log     # observatory / commands
docker rm -f swarm-multi                          # stop everything
```

### Ports
| Port | Service |
|------|---------|
| `8000` | Observatory web UI |
| `8888` | uXRCE-DDS Agent (UDP) |
| `50051 + i` | `mavsdk_server` gRPC for drone *i* |
| `14540 + i` | PX4 MAVLink (offboard) for drone *i* |

---

## Project layout

```
agents/                     # one package per responsibility; deps flow downward
  core/                     # RosBridge + QoS, GPS offset math, GzCameras (the one camera reader)
  world/                    # World: loads city_boxes.json, maps PX4 NED -> world ENU, resolves targets
  perception/               # pure trig/text: scan + situation readouts over a World
  flight/                   # FlightOps (take_off/goto/orbit/...) + their MCP tool bindings
  swarm/run.py              # thin assembler: build the agents + gather their run() loops
  swarm/commander.py        # CommanderAgent: user/report channels, dispatch(), run()
  swarm/drone.py            # DroneAgent: MAVSDK link, cmd inbox, report(), connect(), run()
  observatory/server.py     # web UI backend: state, cameras (MJPEG/WS), command intake
  observatory/static/       # the Observatory single-page UI
sim/
  launch/swarm_sim.sh       # in-container launch: gz + N×PX4 + uXRCE + N×mavsdk
  worlds/make_city_world.py # generate the 'city' world (buildings) from default.sdf
scripts/
  run_swarm_demo.sh         # one-command host launcher (build args, creds, GPU)
docker/Dockerfile.swarm     # Ubuntu 24.04 + Gazebo Harmonic + ROS2 Jazzy + PX4 + uXRCE
docs/superpowers/           # design specs + plans
```

> Dependencies form a clean DAG: `core <- world <- perception <- flight <- swarm`,
> and `observatory -> core`. Each package is importable and testable on its own.

---

## Known limitation / roadmap

- **Drones are commander-driven only.** Each drone acts when the Commander tasks it
  and holds its last command in between — it has its own thinking but no autonomous
  loop. A future option is letting a drone send an *unsolicited* report (e.g. on
  spotting something) that the Commander can react to.
- **Run drones on separate hardware.** The agents already talk only over ROS topics
  and are self-contained objects, so a per-process entrypoint that constructs a single
  `DroneAgent(i)` (or the `CommanderAgent`) and awaits its `run()` would put a drone's
  agent on its own onboard computer with no protocol change.
- NVIDIA dGPU path for larger camera feeds.
- Higher-level behaviors (follow, search patterns) as composable tools.
```
