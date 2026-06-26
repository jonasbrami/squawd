# Squawd Swarm — Bridge Spike Gate (Milestone 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove the one high-risk unknown of the whole swarm design — that a single OS process can host the asyncio-native Claude Agent SDK + MAVSDK **and** a callback-based `rclpy` ROS2 subscription at the same time, flying a PX4 SITL drone while reading its ROS2 telemetry. This is the **go/no-go gate**; nothing else in the swarm gets built until it passes.

**Architecture:** One process. `rclpy` spins in a dedicated daemon thread and writes the latest message of each subscribed topic into a thread-safe store. The asyncio event loop runs MAVSDK (drone control) and the Claude Agent SDK (reasoning). A Claude `@tool` arms+takes off via MAVSDK and reads altitude from the ROS2 store — exercising all three stacks in one tool call. PX4 publishes telemetry to ROS2 over the micro-XRCE-DDS Agent (no `ros_gz` needed for this gate).

**Tech Stack:** Ubuntu 24.04, ROS2 **Jazzy** (matches 24.04 + Gazebo Harmonic — overrides the spec's "Humble"), Gazebo Harmonic, PX4 SITL (`gz_x500`), micro-XRCE-DDS-Agent + `px4_msgs`, MAVSDK-Python ≥2.0, `claude-agent-sdk` ≥0.2, Python 3.12, `rclpy`.

**Scope:** Milestone 1 only (spec §8 step 1). Milestones 2–7 (single-drone slice, SLAM, map fusion, multi-drone, commander, chat) are deferred to follow-up plans written after this gate passes. The gate reads PX4's `/fmu/out/vehicle_local_position` topic rather than a SLAM map topic — same `rclpy` mechanism, far less setup; reading a real SLAM map is identical plumbing deferred to Milestone 3.

**Working directory:** the existing repo `/home/quenouille/drone` on branch `feat/squawd`. We add new top-level dirs (`docker/`, `sim/`, `agents/`, `spikes/swarm/`) alongside v1's `src/` (kept as salvage source).

---

### Task 1: Scaffold swarm directories and dependency manifest

**Files:**
- Create: `agents/__init__.py`
- Create: `agents/common/__init__.py`
- Create: `spikes/swarm/__init__.py`
- Create: `requirements-swarm.txt`

- [ ] **Step 1: Create package directories and init files**

```bash
mkdir -p agents/common spikes/swarm sim/launch docker
touch agents/__init__.py agents/common/__init__.py spikes/swarm/__init__.py
```

- [ ] **Step 2: Write the Python dependency manifest**

Create `requirements-swarm.txt` (rclpy is provided by the ROS2 apt install, NOT pip):

```
mavsdk>=2.0.0
claude-agent-sdk>=0.2.0
python-dotenv>=1.0.0
```

- [ ] **Step 3: Commit**

```bash
git add agents spikes/swarm sim docker requirements-swarm.txt
git commit -m "chore: scaffold swarm dirs + python deps manifest"
```

---

### Task 2: ROS2 Jazzy + micro-XRCE-DDS Agent + px4_msgs in the Docker image

**Files:**
- Create: `docker/Dockerfile.swarm`
- Test: manual — `ros2`, `MicroXRCEAgent`, and `python3 -c "import px4_msgs"` all resolve in the built image.

This extends v1's proven Ubuntu 24.04 + Gazebo Harmonic + PX4 base with the ROS2 layer. We keep v1's rendering/UID lessons by basing on the same OS.

- [ ] **Step 1: Write the Dockerfile**

Create `docker/Dockerfile.swarm`:

```dockerfile
# Swarm image: Ubuntu 24.04 + Gazebo Harmonic + ROS2 Jazzy + PX4 + uXRCE-DDS
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- base tools ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release ca-certificates software-properties-common \
        git python3 python3-pip python3-venv build-essential cmake \
    && rm -rf /var/lib/apt/lists/*

# --- ROS2 Jazzy (apt) ---
RUN add-apt-repository universe \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic (OSRF apt) ---
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --- micro-XRCE-DDS Agent (PX4 <-> ROS2 bridge) ---
RUN git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cd /tmp/xrce && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install && ldconfig \
    && rm -rf /tmp/xrce

# --- px4_msgs (ROS2 message defs for PX4 topics), built into a ws ---
RUN mkdir -p /opt/px4_ws/src \
    && git clone -b release/1.15 --depth 1 https://github.com/PX4/px4_msgs.git /opt/px4_ws/src/px4_msgs \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/px4_ws && colcon build --packages-select px4_msgs

# --- mavsdk_server binary (run standalone; pip's mavsdk is the client only) ---
# x86_64 dev host. On arm64 swap to mavsdk_server_musl_aarch64.
RUN curl -fsSL -o /usr/local/bin/mavsdk_server \
        https://github.com/mavlink/MAVSDK/releases/download/v2.12.2/mavsdk_server_musl_x86_64 \
    && chmod +x /usr/local/bin/mavsdk_server

# --- Python deps for the agent layer ---
COPY requirements-swarm.txt /tmp/requirements-swarm.txt
RUN pip3 install --break-system-packages -r /tmp/requirements-swarm.txt

# Source ROS + px4_msgs for every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
    && echo "source /opt/px4_ws/install/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
```

- [ ] **Step 2: Build the image**

Run: `docker build -f docker/Dockerfile.swarm -t squawd:dev .`
Expected: build completes; final layers show `px4_msgs` colcon build succeeded and pip install of mavsdk/claude-agent-sdk.

- [ ] **Step 3: Verify the three toolchains resolve inside the image**

Run:
```bash
docker run --rm squawd:dev bash -lc \
  'ros2 --help >/dev/null && which MicroXRCEAgent && which mavsdk_server && python3 -c "import px4_msgs.msg as m; print(m.VehicleLocalPosition)"'
```
Expected: prints paths like `/usr/local/bin/MicroXRCEAgent` and `/usr/local/bin/mavsdk_server`, then `<class 'px4_msgs.msg._vehicle_local_position.VehicleLocalPosition'>`. No import error.

- [ ] **Step 4: Commit**

```bash
git add docker/Dockerfile.swarm
git commit -m "feat: swarm Docker image (ROS2 Jazzy + uXRCE-DDS + px4_msgs)"
```

---

### Task 3: Thread-safe latest-message store (pure unit, TDD)

**Files:**
- Create: `agents/common/latest_store.py`
- Test: `tests/test_latest_store.py`

The bridge's core logic — "hold the most recent message per topic, written by the ROS thread, read by the asyncio thread" — is separable from ROS and unit-testable. Isolate it so the risky integration in Task 4 has a tested core.

- [ ] **Step 1: Write the failing test**

Create `tests/test_latest_store.py`:

```python
import threading
from agents.common.latest_store import LatestStore


def test_get_returns_none_before_any_set():
    store = LatestStore()
    assert store.get("/topic") is None


def test_set_then_get_returns_latest():
    store = LatestStore()
    store.set("/pos", {"z": -1.0})
    store.set("/pos", {"z": -2.0})
    assert store.get("/pos") == {"z": -2.0}


def test_independent_topics():
    store = LatestStore()
    store.set("/a", 1)
    store.set("/b", 2)
    assert store.get("/a") == 1 and store.get("/b") == 2


def test_concurrent_writes_do_not_crash_and_last_wins():
    store = LatestStore()

    def writer(n):
        for i in range(1000):
            store.set("/t", (n, i))

    threads = [threading.Thread(target=writer, args=(n,)) for n in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    # last value is a well-formed tuple from some writer (no torn read)
    val = store.get("/t")
    assert isinstance(val, tuple) and val[1] == 999
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_latest_store.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'agents.common.latest_store'`.

- [ ] **Step 3: Write minimal implementation**

Create `agents/common/latest_store.py`:

```python
"""Thread-safe holder for the most recent value per key.

Written by the rclpy thread, read by the asyncio loop. No torn reads: each
get/set is guarded by a single lock and returns the whole object reference.
"""
import threading
from typing import Any, Optional


class LatestStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._values: dict[str, Any] = {}

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self._values[key] = value

    def get(self, key: str) -> Optional[Any]:
        with self._lock:
            return self._values.get(key)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_latest_store.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add agents/common/latest_store.py tests/test_latest_store.py
git commit -m "feat: thread-safe LatestStore for the rclpy<->asyncio bridge"
```

---

### Task 4: The bridge — rclpy node spun in a thread, exposing latest() to asyncio

**Files:**
- Create: `agents/common/bus.py`
- Test: integration (needs ROS2 + running sim from Task 5); this task ships the code, Task 5 exercises it.

This is the heart of the §7 risk. The subtle correctness points baked into the code: (a) `rclpy.spin` runs in a **daemon thread**, not the main loop; (b) PX4 publishes with **BEST_EFFORT** sensor QoS — a default RELIABLE subscription receives **nothing**, so we set the QoS explicitly; (c) `latest()` is a non-blocking read of the tested `LatestStore`.

- [ ] **Step 1: Write the bridge**

Create `agents/common/bus.py`:

```python
"""RosBridge: run rclpy in a background thread, surface latest msgs to asyncio.

The asyncio side never calls blocking rclpy APIs; it only reads latest().
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from agents.common.latest_store import LatestStore

# PX4 uXRCE-DDS publishes /fmu/out/* with this profile. Must match to receive.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class RosBridge:
    def __init__(self, node_name: str = "squawd_bridge") -> None:
        rclpy.init()
        self._node: Node = rclpy.create_node(node_name)
        self._store = LatestStore()
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def subscribe(self, topic: str, msg_type) -> None:
        self._node.create_subscription(
            msg_type, topic, lambda m, t=topic: self._store.set(t, m), PX4_QOS
        )

    def latest(self, topic: str):
        """Non-blocking: return the most recent message for topic, or None."""
        return self._store.get(topic)

    def start(self) -> None:
        self._thread.start()

    def _spin(self) -> None:
        rclpy.spin(self._node)

    def shutdown(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Smoke-check it imports under ROS2 (no sim yet)**

Run:
```bash
docker run --rm -v "$PWD:/workspace" squawd:dev bash -lc \
  'python3 -c "from agents.common.bus import RosBridge, PX4_QOS; print(\"ok\", PX4_QOS.reliability)"'
```
Expected: prints `ok ReliabilityPolicy.BEST_EFFORT`. (Importing `rclpy` succeeds; we do not start() here.)

- [ ] **Step 3: Commit**

```bash
git add agents/common/bus.py
git commit -m "feat: RosBridge (rclpy in a thread, BEST_EFFORT PX4 QoS, latest())"
```

---

### Task 5: One-drone sim bring-up + bridge integration spike

**Files:**
- Create: `sim/launch/one_drone.sh`
- Create: `spikes/swarm/bridge_spike.py`
- Test: manual run — the spike prints both MAVSDK telemetry AND ROS2 `vehicle_local_position` from one process.

This proves MAVSDK and `rclpy` coexist. The Claude SDK layer is added in Task 6.

- [ ] **Step 1: Write the sim bring-up script**

Create `sim/launch/one_drone.sh`. PX4's `gz_x500` SITL target auto-starts the uXRCE-DDS client (PX4 ≥1.14); the Agent bridges it to ROS2. `mavsdk_server` runs standalone per v1's hard-won lesson (the SDK forks the CLI and would kill an in-process server).

```bash
#!/usr/bin/env bash
# Brings up: micro-XRCE-DDS Agent + PX4 SITL (gz_x500) + standalone mavsdk_server.
# Assumes PX4-Autopilot is cloned at /workspace/PX4-Autopilot (Task 5 Step 2).
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

# 1. uXRCE-DDS Agent (PX4 -> ROS2), UDP 8888
MicroXRCEAgent udp4 -p 8888 &
sleep 2

# 2. PX4 SITL with Gazebo Harmonic x500
pushd /workspace/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 &
popd
sleep 25   # PX4 + Gazebo cold start

# 3. standalone mavsdk_server bound to PX4's UDP offboard port
mavsdk_server -p 50051 udpin://0.0.0.0:14540 &

echo "Bring-up launched. ROS2 topics:"
ros2 topic list | grep /fmu/ || echo "WARN: no /fmu topics yet"
wait
```

- [ ] **Step 2: Build PX4-Autopilot once inside a container shell**

Run an interactive container and build PX4 (one-time, ~10–15 min):
```bash
docker run --rm -it -v "$PWD:/workspace" squawd:dev bash -lc '
  cd /workspace && [ -d PX4-Autopilot ] || git clone -b v1.15.4 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh --no-nuttx
  DONT_RUN=1 make px4_sitl gz_x500'
```
Expected: ends with `gz_x500` build success (the `DONT_RUN=1` builds without launching). `PX4-Autopilot/` is gitignored (Step 4).

- [ ] **Step 3: Write the integration spike**

Create `spikes/swarm/bridge_spike.py`:

```python
"""Prove MAVSDK + rclpy run together in one process.

Connects MAVSDK to PX4, subscribes to PX4's ROS2 vehicle_local_position via the
bridge, and for ~10s prints altitude from BOTH sources side by side.
"""
import asyncio

from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition

from agents.common.bus import RosBridge

TOPIC = "/fmu/out/vehicle_local_position"


async def main() -> None:
    bridge = RosBridge()
    bridge.subscribe(TOPIC, VehicleLocalPosition)
    bridge.start()

    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("MAVSDK connected to PX4")
            break

    for _ in range(10):
        ros_msg = bridge.latest(TOPIC)
        ros_alt = None if ros_msg is None else -ros_msg.z  # NED z -> altitude
        mav_pos = await anext(drone.telemetry.position())
        print(f"ROS2 alt={ros_alt!s:>8}  |  MAVSDK alt={mav_pos.relative_altitude_m:.2f} m")
        await asyncio.sleep(1)

    bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
```

- [ ] **Step 4: Gitignore the PX4 checkout**

Append to `.gitignore`:
```
PX4-Autopilot/
```

- [ ] **Step 5: Run the sim, then the spike (two shells, same container)**

Shell A (bring-up):
```bash
docker run --rm -it --name swarm-sim -v "$PWD:/workspace" squawd:dev \
  bash -lc 'chmod +x sim/launch/one_drone.sh && sim/launch/one_drone.sh'
```
Wait until it prints `/fmu/...` topics. Shell B (spike, into the same container):
```bash
docker exec -it swarm-sim bash -lc 'cd /workspace && python3 spikes/swarm/bridge_spike.py'
```
Expected: `MAVSDK connected to PX4`, then ~10 lines where **both** `ROS2 alt` (non-`None` after a second) and `MAVSDK alt` print. Both near `0.00` on the ground. **This is the first half of the gate: the two async stacks coexist.**

- [ ] **Step 6: Commit**

```bash
git add sim/launch/one_drone.sh spikes/swarm/bridge_spike.py .gitignore
git commit -m "feat: one-drone sim bring-up + MAVSDK/rclpy coexistence spike"
```

---

### Task 6: THE GATE — fly via a Claude Agent SDK tool that also reads ROS2

**Files:**
- Create: `spikes/swarm/gate_spike.py`
- Test: manual run — Claude calls a tool that arms+takes off (MAVSDK) and reports altitude read from the ROS2 bridge.

This unites all three stacks in one in-process tool call. `setting_sources=[]` slims the SDK context (v1's confirmed cost fix). Runs on Claude Code OAuth — no `ANTHROPIC_API_KEY` needed.

- [ ] **Step 1: Write the gate spike**

Create `spikes/swarm/gate_spike.py`:

```python
"""GO/NO-GO GATE: Claude Agent SDK in-process tool awaits MAVSDK AND reads rclpy,
all on one event loop in one process, against PX4 SITL.
"""
import asyncio

from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition
from claude_agent_sdk import (
    tool, create_sdk_mcp_server, ClaudeAgentOptions, ClaudeSDKClient,
)

from agents.common.bus import RosBridge

TOPIC = "/fmu/out/vehicle_local_position"

bridge = RosBridge()
drone = System(mavsdk_server_address="127.0.0.1", port=50051)


@tool("takeoff_and_report", "Arm, take off to 5m, and report altitude from ROS2 telemetry.", {})
async def takeoff_and_report(args):
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    await asyncio.sleep(8)  # let it climb
    ros_msg = bridge.latest(TOPIC)
    alt = None if ros_msg is None else -ros_msg.z
    text = f"Took off. ROS2-reported altitude: {alt:.2f} m" if alt is not None else \
           "Took off, but no ROS2 telemetry received."
    return {"content": [{"type": "text", "text": text}]}


async def main() -> None:
    bridge.subscribe(TOPIC, VehicleLocalPosition)
    bridge.start()
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    server = create_sdk_mcp_server(name="flight", tools=[takeoff_and_report])
    options = ClaudeAgentOptions(
        mcp_servers={"flight": server},
        allowed_tools=["mcp__flight__takeoff_and_report"],
        setting_sources=[],  # slim context: don't inherit Claude Code/superpowers session
    )
    async with ClaudeSDKClient(options=options) as client:
        await client.query("Take off and tell me the drone's altitude.")
        async for msg in client.receive_response():
            print(msg)

    bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
```

- [ ] **Step 2: Ensure the sim is running**

If Task 5's Shell A container (`swarm-sim`) is still up, reuse it. Otherwise re-run Task 5 Step 5 Shell A.

- [ ] **Step 3: Run the gate**

Run:
```bash
docker exec -it swarm-sim bash -lc 'cd /workspace && python3 spikes/swarm/gate_spike.py'
```
Expected: Claude invokes `takeoff_and_report`; the drone arms and climbs in Gazebo; the final printed assistant message contains an altitude near **5 m** read from ROS2 (e.g. "ROS2-reported altitude: 4.8 m"). No "Future attached to a different loop" error, no cross-thread crash.

- [ ] **Step 4: Record the gate outcome**

Create `spikes/swarm/GATE_RESULT.md` documenting PASS/FAIL, the observed altitude, and any surprises (loop/threading errors, QoS mismatch yielding `None` ROS alt, etc.). On PASS, the swarm design is validated and Milestone 2 planning begins. On FAIL, capture the exact error — it dictates the redesign (e.g. move to an executor-per-loop pattern or `rclpy` async executor).

- [ ] **Step 5: Commit**

```bash
git add spikes/swarm/gate_spike.py spikes/swarm/GATE_RESULT.md
git commit -m "feat: bridge gate spike — Claude tool flies PX4 + reads ROS2 (one process)"
```

---

## Deferred to follow-up plans (written after this gate passes)

- **Milestone 2** — single-drone vertical slice: NL → drone agent → fly + local SLAM + `look()` (port v1's control/perception onto ROS2; salvage `geo.py`).
- **Milestone 2b** — Observatory v0: minimal read-only web UI (one drone's camera + status + chat), needs the `ros_gz` image bridge. Salvage v1's `web/` cockpit. Pulled early so all later milestones are watchable.
- **Milestone 3** — per-drone SLAM (RTAB-Map on depth cam) + read a real map topic through the bridge.
- **Milestone 4** — map fusion server (`map_merge`/OctoMap in GPS frame).
- **Milestone 4b** — Observatory v1: full UI — N-drone grid + swarm chat panel + fused-map view.
- **Milestone 5** — second drone + `/drone_i` namespacing; fused map from both.
- **Milestone 6** — Commander agent: NL mission → decompose → delegate.
- **Milestone 7** — `/swarm/chat` free NL peer coordination; end-to-end "search this area", watched live in the observatory.
