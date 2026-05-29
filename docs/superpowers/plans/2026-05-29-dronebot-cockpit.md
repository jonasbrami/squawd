# Dronebot Cockpit & Dev Container Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make dronebot fully self-contained in a dev container and give it a browser "mission-control" cockpit (chat + live Gazebo 3D + telemetry + home-relative map + camera).

**Architecture:** One Ubuntu 24.04 container builds and runs PX4 SITL + Gazebo Harmonic (Intel-iGPU rendering via `/dev/dri`, software fallback). A supervisor starts Xvfb → PX4+Gazebo → x11vnc+noVNC → a FastAPI app. The FastAPI app reuses the v1 `CommandExecutor`/`DroneAgent` (same single asyncio loop) and exposes WS chat, WS telemetry, and an MJPEG camera stream to a vanilla-JS cockpit. The Claude Agent SDK uses the mounted host OAuth with a slimmed context.

**Tech Stack:** Docker / dev containers, Ubuntu 24.04, PX4 SITL, Gazebo Harmonic, Mesa, Xvfb/x11vnc/noVNC, FastAPI + uvicorn, vanilla JS, MAVSDK-Python, Claude Agent SDK.

**Reference specs:** `docs/superpowers/specs/2026-05-29-dronebot-cockpit-design.md` (this increment) and `docs/superpowers/specs/2026-05-29-llm-uav-chatbot-design.md` (v1 core).

**Prereq:** the v1 core branch `feat/dronebot-v1` (control/perception/agent modules) is implemented.

---

## File Structure

| Path | Responsibility |
|---|---|
| `.devcontainer/Dockerfile` | Self-contained image: PX4+Gazebo build, Mesa, Xvfb/x11vnc/noVNC, Node+claude CLI, Python deps. |
| `.devcontainer/devcontainer.json` | Mounts `/dev/dri` + `~/.claude`; render/video group; forwards 8000/6080. |
| `docker-compose.yml` | Single service wrapping the image (works without VS Code). |
| `scripts/gl_probe.sh` | Milestone-1 GL/render check. |
| `scripts/start-all.sh` | Supervisor: Xvfb → PX4+Gazebo → noVNC → uvicorn; GL fallback; signal teardown. |
| `src/dronebot/stack.py` | `build_stack`/`start_stack`/`stop_stack` — shared wiring for REPL + web (DRY). |
| `src/dronebot/web/framing.py` | Pure helpers: MJPEG part framing + telemetry-frame serializer. |
| `src/dronebot/web/server.py` | FastAPI app: lifespan wiring, WS `/chat`, WS `/telemetry`, MJPEG `/camera`, static. |
| `src/dronebot/web/static/index.html` | Cockpit markup (4 panels). |
| `src/dronebot/web/static/cockpit.css` | Cockpit styling. |
| `src/dronebot/web/static/cockpit.js` | Chat WS, telemetry WS, camera img, map canvas. |
| `src/dronebot/app.py` | (modified) reuse `build_stack`/`start_stack`/`stop_stack`. |
| `src/dronebot/agent/claude_agent.py` | (modified) slim `ClaudeAgentOptions` context. |
| `tests/test_framing.py` | Unit tests for `framing.py` (no sim). |
| `README.md` | How to launch the container + cockpit. |

---

## Milestone 1 — GPU rendering spike (GO / NO-GO GATE)

> Decides whether Gazebo renders in-container on the Intel iGPU, or whether we run software-rendered (slow). Build nothing else until a frame renders one way or the other.

### Task 1: Minimal base image + GL probe

**Files:**
- Create: `.devcontainer/Dockerfile.probe`
- Create: `scripts/gl_probe.sh`

- [ ] **Step 1: Create `scripts/gl_probe.sh`**

```bash
#!/usr/bin/env bash
# Milestone-1 gate: verify an OpenGL context is available (hardware via
# /dev/dri, else software llvmpipe). Prints the renderer string and exits
# non-zero only if NO GL context can be created at all.
set -euo pipefail

export DISPLAY=:99
Xvfb :99 -screen 0 1280x720x24 >/tmp/xvfb.log 2>&1 &
XVFB_PID=$!
sleep 2

echo "=== Hardware GL probe (/dev/dri) ==="
if glxinfo -B 2>/tmp/glx_hw.log | grep -E "OpenGL renderer"; then
  echo "HARDWARE_GL_OK"
else
  echo "hardware GL unavailable; trying software (llvmpipe)"
  if LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B 2>/tmp/glx_sw.log | grep -E "OpenGL renderer"; then
    echo "SOFTWARE_GL_OK"
  else
    echo "NO_GL" >&2
    kill "$XVFB_PID" 2>/dev/null || true
    exit 1
  fi
fi

kill "$XVFB_PID" 2>/dev/null || true
echo "GL PROBE PASS"
```

- [ ] **Step 2: Make it executable**

Run: `chmod +x scripts/gl_probe.sh`

- [ ] **Step 3: Create `.devcontainer/Dockerfile.probe`**

```dockerfile
# Minimal image just to validate GL/rendering in-container.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb x11-utils mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
COPY scripts/gl_probe.sh /gl_probe.sh
ENTRYPOINT ["/gl_probe.sh"]
```

- [ ] **Step 4: Build the probe image**

Run: `docker build -f .devcontainer/Dockerfile.probe -t dronebot-glprobe .`
Expected: image builds with no errors.

- [ ] **Step 5: Run the probe with the Intel GPU passed through (THE GATE)**

Run: `docker run --rm --device /dev/dri dronebot-glprobe`
Expected: prints an `OpenGL renderer` line and `GL PROBE PASS`.
- If it prints `HARDWARE_GL_OK` with an Intel/Mesa renderer → hardware path; record it.
- If it falls back to `SOFTWARE_GL_OK` (llvmpipe) → software path; the cockpit will work but Gazebo will be slow. Record this and set `LIBGL_ALWAYS_SOFTWARE=1` as the container default.
- If `NO_GL` → STOP and escalate (likely a `/dev/dri` permission/GID issue; capture `/tmp/glx_hw.log`).

- [ ] **Step 6: Record the verdict + commit**

Create `.devcontainer/RENDER_NOTES.md` with the renderer string and which path (hardware/software) we're on.
```bash
git add scripts/gl_probe.sh .devcontainer/Dockerfile.probe .devcontainer/RENDER_NOTES.md
git commit -m "spike: in-container GL/render gate (Intel iGPU vs software)"
```

---

## Milestone 2 — Self-contained container

### Task 2: Full Dockerfile (PX4 + Gazebo Harmonic + tools)

**Files:**
- Create: `.devcontainer/Dockerfile`

- [ ] **Step 1: Create `.devcontainer/Dockerfile`**

```dockerfile
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Host render/video GIDs so /dev/dri is usable (override at build time).
ARG RENDER_GID=110
ARG VIDEO_GID=44
ARG USERNAME=dev

# --- base + rendering + vnc + python + node ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo git curl wget gnupg lsb-release ca-certificates \
        python3 python3-pip python3-venv \
        xvfb x11-utils x11vnc mesa-utils libgl1-mesa-dri \
        novnc websockify \
        build-essential \
    && rm -rf /var/lib/apt/lists/*

# --- Node + Claude Code CLI (for the Agent SDK / OAuth) ---
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && npm install -g @anthropic-ai/claude-code \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic (OSRF apt repo) ---
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --- non-root user with render/video access ---
RUN groupadd -g ${RENDER_GID} render2 2>/dev/null || true \
    && groupmod -g ${VIDEO_GID} video 2>/dev/null || true \
    && useradd -m -s /bin/bash ${USERNAME} \
    && usermod -aG sudo,video,render2 ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# --- PX4-Autopilot (clone + deps + prebuild SITL target) ---
RUN git clone --depth 1 --recurse-submodules \
        https://github.com/PX4/PX4-Autopilot.git \
    && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
# Prebuild the gz_x500_depth target so first run is fast (DONT_RUN avoids launch).
RUN cd PX4-Autopilot && DONT_RUN=1 make px4_sitl gz_x500_depth || true

# --- Python deps + dronebot (editable) ---
COPY --chown=${USERNAME}:${USERNAME} . /workspace
WORKDIR /workspace
RUN pip3 install --break-system-packages -e ".[dev]" \
    && pip3 install --break-system-packages fastapi uvicorn

ENV PX4_DIR=/home/${USERNAME}/PX4-Autopilot
ENV DRONEBOT_CONNECTION_URL=udp://:14540
EXPOSE 8000 6080
CMD ["bash", "scripts/start-all.sh"]
```

- [ ] **Step 2: Verify the image builds (long — 10–20+ min)**

Run: `docker build -f .devcontainer/Dockerfile --build-arg RENDER_GID=$(getent group render | cut -d: -f3) --build-arg VIDEO_GID=$(getent group video | cut -d: -f3) -t dronebot .`
Expected: builds to completion. (PX4 build is slow; the `|| true` keeps a model-fetch hiccup from failing the image — the model is fetched again at first run if needed.)

- [ ] **Step 3: Commit**

```bash
git add .devcontainer/Dockerfile
git commit -m "feat: self-contained dronebot image (PX4 + Gazebo Harmonic + noVNC)"
```

### Task 3: Supervisor script

**Files:**
- Create: `scripts/start-all.sh`

- [ ] **Step 1: Create `scripts/start-all.sh`**

```bash
#!/usr/bin/env bash
# Supervisor: bring up the virtual display, sim, noVNC, and the web app,
# then wait. Tears everything down on signal.
set -uo pipefail

export DISPLAY=:99
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PIDS=()

cleanup() {
  echo "shutting down..."
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1. Virtual display
Xvfb :99 -screen 0 1600x900x24 >/tmp/xvfb.log 2>&1 &
PIDS+=($!)
sleep 2

# 1b. GL fallback: if hardware GL fails, force software rendering.
if ! glxinfo -B >/dev/null 2>&1; then
  echo "hardware GL unavailable -> LIBGL_ALWAYS_SOFTWARE=1"
  export LIBGL_ALWAYS_SOFTWARE=1
fi

# 2. PX4 SITL + Gazebo (renders to :99)
( cd "$PX4_DIR" && HEADLESS=0 make px4_sitl gz_x500_depth >/tmp/px4.log 2>&1 ) &
PIDS+=($!)

# 3. noVNC: expose the virtual display over the browser (port 6080)
x11vnc -display :99 -forever -shared -nopw -quiet >/tmp/x11vnc.log 2>&1 &
PIDS+=($!)
websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
PIDS+=($!)

# 4. Web app (FastAPI) on port 8000
uvicorn dronebot.web.server:app --host 0.0.0.0 --port 8000 >/tmp/web.log 2>&1 &
PIDS+=($!)

echo "dronebot up: cockpit http://localhost:8000  noVNC http://localhost:6080/vnc.html"
wait
```

- [ ] **Step 2: Make executable**

Run: `chmod +x scripts/start-all.sh`

- [ ] **Step 3: Commit**

```bash
git add scripts/start-all.sh
git commit -m "feat: container supervisor (display, sim, noVNC, web)"
```

### Task 4: devcontainer.json + docker-compose.yml

**Files:**
- Create: `.devcontainer/devcontainer.json`
- Create: `docker-compose.yml`

- [ ] **Step 1: Create `docker-compose.yml`**

```yaml
services:
  dronebot:
    build:
      context: .
      dockerfile: .devcontainer/Dockerfile
      args:
        RENDER_GID: "${RENDER_GID:-110}"
        VIDEO_GID: "${VIDEO_GID:-44}"
    devices:
      - "/dev/dri:/dev/dri"
    volumes:
      - "${HOME}/.claude:/home/dev/.claude"
      - ".:/workspace"
    ports:
      - "8000:8000"
      - "6080:6080"
    stdin_open: true
    tty: true
```

- [ ] **Step 2: Create `.devcontainer/devcontainer.json`**

```json
{
  "name": "dronebot",
  "dockerComposeFile": "../docker-compose.yml",
  "service": "dronebot",
  "workspaceFolder": "/workspace",
  "forwardPorts": [8000, 6080],
  "portsAttributes": {
    "8000": { "label": "cockpit" },
    "6080": { "label": "noVNC" }
  },
  "customizations": {
    "vscode": { "extensions": ["ms-python.python"] }
  }
}
```

- [ ] **Step 3: Verify compose config**

Run: `RENDER_GID=$(getent group render | cut -d: -f3) VIDEO_GID=$(getent group video | cut -d: -f3) docker compose config`
Expected: prints the resolved config with `/dev/dri` device and the `~/.claude` + `.` mounts, no errors.

- [ ] **Step 4: Commit**

```bash
git add .devcontainer/devcontainer.json docker-compose.yml
git commit -m "feat: dev container + compose (dri passthrough, OAuth mount, ports)"
```

---

## Milestone 3 — Web backend

### Task 5: Pure framing helpers (TDD)

**Files:**
- Create: `src/dronebot/web/__init__.py` (empty)
- Create: `src/dronebot/web/framing.py`
- Test: `tests/test_framing.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_framing.py
from dronebot.web.framing import mjpeg_part, telemetry_frame
from dronebot.control.state import StateStore
from dronebot.control.geo import GeoPoint
from dronebot.perception.store import PerceptionStore
from dronebot.perception.provider import PerceptionSnapshot, Obstacle


def test_mjpeg_part_has_boundary_and_payload():
    part = mjpeg_part(b"\xff\xd8jpeg")
    assert part.startswith(b"--frame\r\n")
    assert b"Content-Type: image/jpeg" in part
    assert b"Content-Length: 6" in part
    assert part.endswith(b"\xff\xd8jpeg\r\n")


def test_telemetry_frame_populated():
    state = StateStore()
    state.set_connection(True)
    state.set_armed(True)
    state.set_in_air(True)
    state.set_flight_mode("HOLD")
    state.set_battery(0.8)
    state.set_home(GeoPoint(47.0, 8.0, 500.0))
    state.set_position(GeoPoint(47.0, 8.0, 510.0))
    perception = PerceptionStore()
    perception.update(PerceptionSnapshot(timestamp=1.0, jpeg_frame=None,
                                         obstacles=[Obstacle("ahead", 4.0)]))
    frame = telemetry_frame(state, perception)
    assert frame["connected"] and frame["armed"] and frame["in_air"]
    assert frame["flight_mode"] == "HOLD"
    assert frame["battery"] == 0.8
    assert frame["rel_alt"] == 10.0
    assert frame["position"]["lat"] == 47.0
    assert "4" in frame["surroundings"]


def test_telemetry_frame_no_fix():
    frame = telemetry_frame(StateStore(), PerceptionStore())
    assert frame["position"] is None
    assert frame["rel_alt"] is None
    assert frame["connected"] is False
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_framing.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dronebot.web'`.

- [ ] **Step 3: Implement `framing.py`**

```python
# src/dronebot/web/framing.py
"""Pure helpers for the web layer. No I/O — unit-testable without the sim."""
from __future__ import annotations

from dronebot.control.state import StateStore
from dronebot.perception.store import PerceptionStore

_BOUNDARY = b"frame"


def mjpeg_part(jpeg: bytes) -> bytes:
    """Frame one JPEG as a multipart/x-mixed-replace part."""
    return (
        b"--" + _BOUNDARY + b"\r\n"
        + b"Content-Type: image/jpeg\r\n"
        + b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
        + jpeg + b"\r\n"
    )


def telemetry_frame(state: StateStore, perception: PerceptionStore) -> dict:
    """Serialize the authoritative state + surroundings for the cockpit."""
    snap = state.snapshot()
    pos = snap.position
    home = snap.home
    rel_alt = None
    if pos is not None and home is not None:
        rel_alt = pos.absolute_altitude_m - home.absolute_altitude_m
    return {
        "connected": snap.is_connected,
        "armed": snap.is_armed,
        "in_air": snap.in_air,
        "flight_mode": snap.flight_mode,
        "battery": state.battery_remaining,
        "rel_alt": rel_alt,
        "position": None if pos is None else {
            "lat": pos.latitude_deg,
            "lon": pos.longitude_deg,
            "abs_alt": pos.absolute_altitude_m,
        },
        "home": None if home is None else {
            "lat": home.latitude_deg,
            "lon": home.longitude_deg,
            "abs_alt": home.absolute_altitude_m,
        },
        "surroundings": perception.surroundings_summary(),
    }
```

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest tests/test_framing.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add src/dronebot/web/__init__.py src/dronebot/web/framing.py tests/test_framing.py
git commit -m "feat: pure web framing helpers (MJPEG + telemetry frame)"
```

### Task 6: Shared stack wiring (`stack.py`) + REPL refactor

**Files:**
- Create: `src/dronebot/stack.py`
- Modify: `src/dronebot/app.py`

> Extracts the layer wiring so the REPL and the web server share ONE path (DRY). No unit test (imports mavsdk/SDK); verified when the container runs.

- [ ] **Step 1: Implement `src/dronebot/stack.py`**

```python
# src/dronebot/stack.py
"""Shared wiring for the dronebot stack — used by both the terminal REPL
(app.py) and the web server. One construction + lifecycle path (DRY).
"""
from __future__ import annotations

import asyncio
import os
from dataclasses import dataclass, field

from mavsdk import System

from dronebot.agent.claude_agent import DroneAgent
from dronebot.config import Config
from dronebot.control.controller import DroneController
from dronebot.control.executor import CommandExecutor
from dronebot.control.safety import SafetyGuard
from dronebot.control.state import StateStore
from dronebot.control.telemetry import start_telemetry
from dronebot.flight_log import FlightLog
from dronebot.perception.gazebo_perception import GazeboPerception
from dronebot.perception.store import PerceptionStore

_RGB_TOPIC = os.environ.get("DRONEBOT_RGB_TOPIC", "/camera")
_DEPTH_TOPIC = os.environ.get("DRONEBOT_DEPTH_TOPIC", "/depth_camera")


@dataclass
class Stack:
    config: Config
    drone: System
    controller: DroneController
    state: StateStore
    perception_store: PerceptionStore
    perception: GazeboPerception
    executor: CommandExecutor
    agent: DroneAgent
    log: FlightLog
    telemetry_tasks: list = field(default_factory=list)


def build_stack(config: Config) -> Stack:
    drone = System()
    controller = DroneController(drone)
    state = StateStore()
    perception_store = PerceptionStore()
    perception = GazeboPerception(perception_store, _RGB_TOPIC, _DEPTH_TOPIC)
    guard = SafetyGuard(config.limits)
    executor = CommandExecutor(controller, state, guard)
    agent = DroneAgent(executor, perception_store, config.model)
    os.makedirs("flight_logs", exist_ok=True)
    log = FlightLog("flight_logs/session.jsonl")
    return Stack(config, drone, controller, state, perception_store,
                 perception, executor, agent, log)


async def start_stack(stack: Stack) -> None:
    await stack.controller.connect(stack.config.connection_url)
    stack.state.set_connection(True)
    stack.telemetry_tasks = start_telemetry(stack.drone, stack.state)
    for _ in range(40):  # wait for a position fix, then set home
        if stack.state.position is not None:
            stack.state.set_home(stack.state.position)
            break
        await asyncio.sleep(0.25)
    await stack.perception.start()


async def stop_stack(stack: Stack) -> None:
    await stack.perception.stop()
    for task in stack.telemetry_tasks:
        task.cancel()
```

- [ ] **Step 2: Rewrite `src/dronebot/app.py` to use the shared stack**

```python
# src/dronebot/app.py
"""Terminal entrypoint. Owns the single asyncio loop; reuses the shared stack."""
from __future__ import annotations

import asyncio

from dotenv import load_dotenv

from dronebot.chat.repl import run_repl
from dronebot.config import load_config
from dronebot.stack import build_stack, start_stack, stop_stack


async def main() -> None:
    load_dotenv()
    stack = build_stack(load_config())
    print(f"connecting to {stack.config.connection_url} ...")
    await start_stack(stack)
    print("connected.")
    try:
        async with stack.agent:
            await run_repl(stack.agent, stack.executor, stack.log)
    finally:
        await stop_stack(stack)


if __name__ == "__main__":
    asyncio.run(main())
```

- [ ] **Step 3: Verify it still imports/compiles**

Run: `python3 -m py_compile src/dronebot/stack.py src/dronebot/app.py`
Expected: no output. (Runtime needs the sim; deferred.)

- [ ] **Step 4: Run the full no-sim suite to confirm nothing broke**

Run: `python3 -m pytest -q`
Expected: still passes (framing + all v1 unit tests).

- [ ] **Step 5: Commit**

```bash
git add src/dronebot/stack.py src/dronebot/app.py
git commit -m "refactor: shared stack wiring for REPL + web (DRY)"
```

### Task 7: Slim the SDK context

**Files:**
- Modify: `src/dronebot/agent/claude_agent.py`

> The spike showed the SDK inherits the full Claude Code session (~53k tokens/turn). Restrict it so turns carry only the drone prompt + tools.

- [ ] **Step 1: Update `ClaudeAgentOptions` in `claude_agent.py`**

In `DroneAgent.__init__`, extend the options to NOT inherit host settings/hooks. Replace the `ClaudeAgentOptions(...)` construction with:
```python
        self._options = ClaudeAgentOptions(
            model=model,
            system_prompt=SYSTEM_PROMPT,
            mcp_servers={"flight": server},
            allowed_tools=ALLOWED_TOOLS,
            setting_sources=[],          # do NOT load user/project/local settings
            cwd="/tmp/dronebot-agent",   # clean cwd: no project CLAUDE.md / hooks
        )
```
Also add, before constructing options, `import os; os.makedirs("/tmp/dronebot-agent", exist_ok=True)`.

> NOTE: `setting_sources=[]` is the documented Agent SDK switch to stop loading filesystem settings (which is what pulls in the SessionStart/superpowers hook). If a future SDK version renames it, the goal is unchanged: exclude host settings/hooks from this agent.

- [ ] **Step 2: Verify it compiles**

Run: `python3 -m py_compile src/dronebot/agent/claude_agent.py`
Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add src/dronebot/agent/claude_agent.py
git commit -m "perf: slim Agent SDK context (no host settings/hooks per turn)"
```

### Task 8: FastAPI server

**Files:**
- Create: `src/dronebot/web/server.py`

> No unit test (owns the live stack + SDK); verified in the container (Milestone 5). Uses a FastAPI lifespan so the whole stack runs on uvicorn's single asyncio loop.

- [ ] **Step 1: Implement `src/dronebot/web/server.py`**

```python
# src/dronebot/web/server.py
"""FastAPI cockpit backend. Runs the shared dronebot stack inside the app
lifespan (single asyncio loop) and exposes chat, telemetry, and camera.
"""
from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from dronebot.config import load_config
from dronebot.stack import build_stack, start_stack, stop_stack
from dronebot.web.framing import mjpeg_part, telemetry_frame

_ABORT_WORDS = {"stop", "abort", "emergency", "land now"}
_STATIC = Path(__file__).parent / "static"


@asynccontextmanager
async def lifespan(app: FastAPI):
    stack = build_stack(load_config())
    await start_stack(stack)
    await stack.agent.__aenter__()
    app.state.stack = stack
    try:
        yield
    finally:
        await stack.agent.__aexit__(None, None, None)
        await stop_stack(stack)


app = FastAPI(lifespan=lifespan)


def _text_of(message) -> str:
    content = getattr(message, "content", None) or []
    return " ".join(getattr(b, "text", "") for b in content if getattr(b, "text", ""))


@app.websocket("/chat")
async def chat(ws: WebSocket) -> None:
    await ws.accept()
    stack = app.state.stack
    try:
        while True:
            user = (await ws.receive_text()).strip()
            if not user:
                continue
            if user.lower() in _ABORT_WORDS:
                stack.log.record("abort", {"trigger": user})
                await stack.agent.interrupt()
                result = await stack.executor.hold()
                await ws.send_text(f"[ABORT] {result.message}")
                continue
            stack.log.record("utterance", {"text": user})
            async for message in stack.agent.ask(user):
                text = _text_of(message)
                if text:
                    await ws.send_text(text)
            await ws.send_text("\n")
    except WebSocketDisconnect:
        return


@app.websocket("/telemetry")
async def telemetry(ws: WebSocket) -> None:
    await ws.accept()
    stack = app.state.stack
    period = 1.0 / max(stack.config.telemetry_rate_hz, 0.5)
    try:
        while True:
            frame = telemetry_frame(stack.state, stack.perception_store)
            frame["geofence_radius_m"] = stack.config.limits.geofence_radius_m
            await ws.send_text(json.dumps(frame))
            await asyncio.sleep(period)
    except WebSocketDisconnect:
        return


@app.get("/camera")
async def camera():
    stack = app.state.stack

    async def gen():
        while True:
            snap = stack.perception_store.latest()
            if snap is not None and snap.jpeg_frame is not None:
                yield mjpeg_part(snap.jpeg_frame)
            await asyncio.sleep(0.1)

    return StreamingResponse(
        gen(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


app.mount("/", StaticFiles(directory=str(_STATIC), html=True), name="static")
```

- [ ] **Step 2: Verify it compiles**

Run: `python3 -m py_compile src/dronebot/web/server.py`
Expected: no output (FastAPI is installed in the container; locally this only checks syntax).

- [ ] **Step 3: Commit**

```bash
git add src/dronebot/web/server.py
git commit -m "feat: FastAPI cockpit backend (chat/telemetry/camera)"
```

---

## Milestone 4 — Cockpit frontend

### Task 9: Cockpit markup + styling

**Files:**
- Create: `src/dronebot/web/static/index.html`
- Create: `src/dronebot/web/static/cockpit.css`

- [ ] **Step 1: Create `index.html`**

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>dronebot · mission control</title>
  <link rel="stylesheet" href="/cockpit.css" />
</head>
<body>
  <header><span class="logo">▲ dronebot</span><span id="conn" class="badge">offline</span></header>
  <main>
    <section class="panel chat">
      <h2>chat</h2>
      <div id="log" class="chat-log"></div>
      <form id="chat-form">
        <input id="msg" autocomplete="off" placeholder="tell the drone what to do…" />
        <button type="submit">send</button>
        <button type="button" id="abort" class="abort">ABORT</button>
      </form>
    </section>
    <section class="panel view">
      <h2>3D world</h2>
      <iframe id="sim" src="http://localhost:6080/vnc.html?autoconnect=1&resize=scale" title="Gazebo"></iframe>
      <div class="camera-row">
        <img id="camera" src="/camera" alt="drone camera" />
        <div id="obstacle" class="obstacle">surroundings: —</div>
      </div>
    </section>
    <section class="panel side">
      <h2>telemetry</h2>
      <dl id="telemetry" class="telemetry">
        <dt>mode</dt><dd id="t-mode">—</dd>
        <dt>armed</dt><dd id="t-armed">—</dd>
        <dt>in air</dt><dd id="t-air">—</dd>
        <dt>rel alt</dt><dd id="t-alt">—</dd>
        <dt>battery</dt><dd id="t-batt">—</dd>
      </dl>
      <h2>map</h2>
      <canvas id="map" width="280" height="280"></canvas>
    </section>
  </main>
  <script src="/cockpit.js"></script>
</body>
</html>
```

- [ ] **Step 2: Create `cockpit.css`**

```css
:root { --bg:#0b0f14; --panel:#121922; --line:#1f2a37; --fg:#cfe3f7; --accent:#3ddc97; --warn:#ff5c5c; --mono:"JetBrains Mono",ui-monospace,monospace; }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font-family:var(--mono); }
header { display:flex; justify-content:space-between; align-items:center; padding:10px 16px; border-bottom:1px solid var(--line); }
.logo { color:var(--accent); font-weight:700; letter-spacing:.5px; }
.badge { font-size:12px; padding:3px 8px; border:1px solid var(--line); border-radius:10px; }
.badge.online { color:var(--accent); border-color:var(--accent); }
main { display:grid; grid-template-columns:320px 1fr 300px; gap:10px; padding:10px; height:calc(100vh - 53px); }
.panel { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:10px; overflow:hidden; display:flex; flex-direction:column; }
.panel h2 { margin:0 0 8px; font-size:12px; text-transform:uppercase; letter-spacing:1px; color:#7da2c7; }
.chat-log { flex:1; overflow-y:auto; font-size:13px; line-height:1.5; }
.chat-log .you { color:#9fd0ff; } .chat-log .drone { color:var(--accent); } .chat-log .sys { color:var(--warn); }
#chat-form { display:flex; gap:6px; margin-top:8px; }
#msg { flex:1; background:#0b0f14; border:1px solid var(--line); color:var(--fg); padding:8px; border-radius:6px; font-family:var(--mono); }
button { background:var(--accent); border:none; color:#06121b; font-weight:700; padding:8px 10px; border-radius:6px; cursor:pointer; }
button.abort { background:var(--warn); color:#fff; }
.view { padding:0; }
.view h2 { padding:10px 10px 0; }
#sim { width:100%; flex:1; border:0; background:#000; }
.camera-row { display:flex; gap:8px; padding:8px; border-top:1px solid var(--line); align-items:center; }
#camera { width:220px; height:140px; object-fit:cover; background:#000; border:1px solid var(--line); border-radius:4px; }
.obstacle { font-size:13px; color:#ffd479; }
.telemetry { display:grid; grid-template-columns:auto 1fr; gap:4px 10px; font-size:13px; margin:0 0 14px; }
.telemetry dt { color:#7da2c7; } .telemetry dd { margin:0; text-align:right; }
#map { background:#06121b; border:1px solid var(--line); border-radius:6px; }
```

- [ ] **Step 3: Commit**

```bash
git add src/dronebot/web/static/index.html src/dronebot/web/static/cockpit.css
git commit -m "feat: cockpit markup + mission-control styling"
```

### Task 10: Cockpit JS (chat, telemetry, camera, map)

**Files:**
- Create: `src/dronebot/web/static/cockpit.js`

- [ ] **Step 1: Create `cockpit.js`**

```javascript
const $ = (id) => document.getElementById(id);
const wsURL = (path) => `ws://${location.host}${path}`;

// --- chat ---
const log = $("log");
function addLine(who, text) {
  const div = document.createElement("div");
  div.className = who;
  div.textContent = (who === "you" ? "you> " : who === "drone" ? "drone> " : "") + text;
  log.appendChild(div);
  log.scrollTop = log.scrollHeight;
}
let chat = new WebSocket(wsURL("/chat"));
chat.onmessage = (e) => { if (e.data.trim()) addLine("drone", e.data); };
chat.onclose = () => addLine("sys", "chat disconnected");
$("chat-form").addEventListener("submit", (e) => {
  e.preventDefault();
  const v = $("msg").value.trim();
  if (!v) return;
  addLine("you", v);
  chat.send(v);
  $("msg").value = "";
});
$("abort").addEventListener("click", () => { addLine("sys", "ABORT sent"); chat.send("abort"); });

// --- telemetry + map ---
const conn = $("conn");
let last = null;
let tele = new WebSocket(wsURL("/telemetry"));
tele.onmessage = (e) => {
  const t = JSON.parse(e.data);
  last = t;
  conn.textContent = t.connected ? "online" : "offline";
  conn.classList.toggle("online", !!t.connected);
  $("t-mode").textContent = t.flight_mode ?? "—";
  $("t-armed").textContent = t.armed ? "yes" : "no";
  $("t-air").textContent = t.in_air ? "yes" : "no";
  $("t-alt").textContent = t.rel_alt == null ? "—" : t.rel_alt.toFixed(1) + " m";
  $("t-batt").textContent = t.battery == null ? "—" : Math.round(t.battery * 100) + "%";
  $("obstacle").textContent = "surroundings: " + (t.surroundings ?? "—");
  drawMap(t);
};

const cv = $("map"), ctx = cv.getContext("2d");
function drawMap(t) {
  const w = cv.width, h = cv.height, cx = w / 2, cy = h / 2;
  ctx.clearRect(0, 0, w, h);
  // geofence ring (scaled so the fence radius ~ 0.42 * canvas)
  const R = (t.geofence_radius_m || 100);
  const scale = (Math.min(w, h) * 0.42) / R; // px per meter
  ctx.strokeStyle = "#1f2a37";
  ctx.beginPath(); ctx.arc(cx, cy, R * scale, 0, 2 * Math.PI); ctx.stroke();
  // home
  ctx.fillStyle = "#7da2c7"; ctx.fillRect(cx - 3, cy - 3, 6, 6);
  if (!t.position || !t.home) return;
  // north/east offset of drone from home, in meters (small-angle)
  const dN = (t.position.lat - t.home.lat) * 111320;
  const dE = (t.position.lon - t.home.lon) * 111320 * Math.cos(t.home.lat * Math.PI / 180);
  const px = cx + dE * scale, py = cy - dN * scale; // north = up
  ctx.fillStyle = "#3ddc97";
  ctx.beginPath(); ctx.arc(px, py, 4, 0, 2 * Math.PI); ctx.fill();
}
```

- [ ] **Step 2: Sanity-check the JS parses**

Run: `node --check src/dronebot/web/static/cockpit.js`
Expected: no output (valid syntax). (If `node` isn't on the host, this is verified in the container.)

- [ ] **Step 3: Commit**

```bash
git add src/dronebot/web/static/cockpit.js
git commit -m "feat: cockpit client (chat, telemetry, camera, map)"
```

---

## Milestone 5 — End-to-end in container

### Task 11: Bring it all up and verify

**Files:** none (verification only)

- [ ] **Step 1: Launch the container**

Run:
```bash
export RENDER_GID=$(getent group render | cut -d: -f3)
export VIDEO_GID=$(getent group video | cut -d: -f3)
docker compose up --build
```
Expected: build completes; logs show `dronebot up: cockpit http://localhost:8000 ...`. Give Gazebo ~30–60s.

- [ ] **Step 2: Verify the sim + noVNC**

Open `http://localhost:6080/vnc.html?autoconnect=1` in a browser.
Expected: the Gazebo 3D world with the x500 quadcopter is visible.

- [ ] **Step 3: Verify the cockpit**

Open `http://localhost:8000`.
Expected: cockpit loads; `online` badge; the 3D iframe shows Gazebo; telemetry populates (mode/battery); the map shows the geofence ring + home; the camera panel shows a frame after a moment.

- [ ] **Step 4: Confirm topic names if camera is blank**

If the camera panel stays blank, inside the container run `gz topic -l | grep -Ei 'camera|depth|image'`, set `DRONEBOT_RGB_TOPIC` / `DRONEBOT_DEPTH_TOPIC` (and adjust the pixel-format assumption in `gazebo_perception.py` if needed per v1 Task 13), and restart.

- [ ] **Step 5: Fly a conversation from the cockpit**

In the chat panel type, one at a time: `arm`, `take off to 10 meters`, `fly 20 meters north`, `what do you see?`, `come back and land`.
Expected: the drone arms/climbs/moves in the 3D view; `what do you see?` returns a camera-grounded answer; telemetry + map track the motion; RTL + land work.

- [ ] **Step 6: Verify abort + safety**

Type `take off to 10 meters` then click **ABORT** → expected `[ABORT] holding position` and the drone holds. Type `fly 500 meters north` → expected a geofence refusal, no motion.

- [ ] **Step 7: Cost check (SDK slimming)**

After a few turns, confirm per-turn token usage is small (system-prompt-sized, not the ~53k from the unslimmed spike) — inspect uvicorn/agent logs or `flight_logs/session.jsonl`.

- [ ] **Step 8: Write the README**

Create `README.md` documenting: prerequisites (Docker, `~/.claude` logged in), the `RENDER_GID`/`VIDEO_GID` + `docker compose up --build` launch, the cockpit (`:8000`) and noVNC (`:6080`) URLs, the Intel-iGPU/software-render note (from `RENDER_NOTES.md`), and the future NVIDIA-RTX toggle.

- [ ] **Step 9: Commit**

```bash
git add README.md
git commit -m "docs: cockpit + dev container usage"
```

---

## Self-Review Notes

- **Spec coverage:** container self-contained (Tasks 2–4), Intel-iGPU render + fallback (Tasks 1, 3), noVNC 3D (Tasks 2/3/9), FastAPI chat/telemetry/camera (Tasks 5/8), SDK slimming (Task 7), cockpit panels (Tasks 9/10), OAuth mount (Task 4), DRY shared wiring (Task 6), end-to-end + cost check (Task 11). All spec sections map to a task.
- **Deferred (non-goals):** NVIDIA RTX acceleration (documented toggle only), online tile maps, multi-container, auth/multi-user, heavy frontend framework.
- **Carried-forward open item:** Gazebo camera/depth topic names + pixel format (v1 Task 13), surfaced in Task 11 Step 4.
- **Risk:** Milestone 1 may force software rendering (slow) — acceptable, non-blocking, recorded in `RENDER_NOTES.md`.
