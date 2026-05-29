# dronebot

An LLM-piloted UAV chatbot. You talk to a simulated drone in plain language
("take off, fly 50m north, what do you see?, come back and land") and an LLM
agent flies it via high-level commands. Runs fully self-contained in a dev
container: PX4 SITL + Gazebo Harmonic + a browser "mission-control" cockpit.

> **Status:** v1 core + cockpit implemented. The container's heavy first build
> (PX4 + Gazebo) and the end-to-end sim run are the last steps — see below.

## Architecture (short)

A single asyncio process drives everything. A Claude Agent SDK client calls
in-process `@tool` adapters → a plain-Python `CommandExecutor` (the portable
command boundary) → a `DroneController` (MAVSDK) talking to PX4. An authoritative
`StateStore` (fed by telemetry) and a non-bypassable `SafetyGuard` sit below the
LLM. A `PerceptionProvider` (Gazebo sensors now, swappable later) feeds a camera
view + obstacle awareness. Two front-ends consume the same `CommandExecutor`:
the terminal REPL and the web cockpit.

Design + plans live in `docs/superpowers/specs/` and `docs/superpowers/plans/`.

## Prerequisites

- **Docker** (tested with 29.x) on Linux.
- Logged-in **Claude Code** on the host (`claude` CLI authenticated) — the agent
  uses your OAuth via the mounted `~/.claude`; **no API key needed.**
- A GPU for Gazebo. Intel iGPU works out of the box (`/dev/dri`); see *Rendering*.

## Run the cockpit (container)

```bash
# host render/video group GIDs (defaults 992/44 already match a typical Ubuntu host)
export RENDER_GID=$(getent group render | cut -d: -f3)
export VIDEO_GID=$(getent group video | cut -d: -f3)

docker compose up --build
```

- **First build is slow (~10–20 min)** — it clones and builds PX4-Autopilot and
  pulls Gazebo Harmonic. Subsequent starts are fast.
- Give Gazebo ~30–60s after startup, then open:
  - **Cockpit:** http://localhost:8000
  - **Raw 3D (noVNC):** http://localhost:6080/vnc.html?autoconnect=1

The cockpit shows: chat (talk to the drone) · the live Gazebo 3D world · the
drone camera + nearest-obstacle readout · telemetry · a home-relative map with
the geofence ring. There's an always-visible **ABORT** button that holds the
drone immediately, bypassing the LLM.

Or open the folder in VS Code → **Reopen in Container** (`.devcontainer/`).

### Terminal mode (no web UI)
Inside the container (or any env with the sim + deps): `python -m dronebot.app`.

## Rendering

Confirmed on this host (`.devcontainer/RENDER_NOTES.md`): **hardware EGL via the
Intel Iris Xe is available in-container**, so Gazebo camera/depth sensors are
GPU-accelerated. The noVNC GUI *view* renders in software (llvmpipe) — fine, it
only affects the picture, not the sensor data. If hardware GL is ever
unavailable, `start-all.sh` falls back to `LIBGL_ALWAYS_SOFTWARE=1` (slower).

**NVIDIA RTX (future toggle):** present on this host but the driver isn't loaded.
To use it later: fix the host NVIDIA driver, install `nvidia-container-toolkit`,
and run with `--gpus all`. Not required.

## Safety

Hard limits are enforced *below* the LLM and cannot be prompted away: altitude
cap, geofence radius, per-command distance, and flight-state preconditions
(`src/dronebot/control/safety.py`), backed by PX4's own geofence/collision
prevention. Limits are configurable via `DRONEBOT_*` env vars with conservative
fail-closed defaults (`src/dronebot/config.py`).

## Development

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
python -m pytest -q        # pure-logic unit tests (no sim required)
```

The control/perception/agent logic is unit-tested without the sim (geo math,
safety invariants, command executor, tool adapters, framing). Sim-dependent code
(`controller`, `gazebo_perception`, `app`, `web/server`) is verified by running
the container.
