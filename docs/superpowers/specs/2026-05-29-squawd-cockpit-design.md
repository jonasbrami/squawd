# Squawd Cockpit & Dev Container — Design (Increment 2)

**Date:** 2026-05-29
**Status:** Approved (pending spec review)
**Builds on:** `2026-05-29-llm-uav-chatbot-design.md` (v1 core)

## 1. Summary

Make the squawd fully self-contained and give it a browser-based "mission
control" cockpit. Everything — PX4 SITL, Gazebo Harmonic, the agent app, the
web UI — runs inside a single dev container. The cockpit shows the live Gazebo
3D world, a chat panel to talk to the drone, live telemetry, a home-relative
map with the geofence ring, and the drone's camera feed.

The v1 architecture is unchanged: the web cockpit is **another consumer** of the
existing `CommandExecutor` + `DroneAgent`, proving the layering. The terminal
REPL is retained as an alternate entrypoint.

## 2. Goals & non-goals

**Goals**
- One self-contained dev container: builds PX4 + Gazebo Harmonic, runs the sim,
  the app, and the cockpit. Nothing installed on the host.
- Gazebo rendered via the host **Intel iGPU** (`/dev/dri` + Mesa), with an
  automatic software-rendering fallback.
- Browser cockpit: chat + embedded Gazebo 3D (noVNC) + telemetry + home-relative
  map + camera feed.
- Agent authenticates via the host **Claude OAuth** (mounted `~/.claude`), no API
  key, with the SDK context **slimmed** so turns are cheap.
- Terminal REPL still works.

**Non-goals (deferred)**
- NVIDIA RTX GPU acceleration (host driver currently broken; Intel iGPU now,
  RTX later — a documented future toggle).
- Online tile maps (Leaflet/OSM) — home-relative canvas map for now.
- Multi-container orchestration — one container + a process supervisor.
- Authentication / multi-user / remote hosting — local dev only.
- Heavy frontend framework — vanilla JS/HTML/CSS.

## 3. Hardware/host context (confirmed)
- Host: Ubuntu 24.04, Docker 29.1, Wayland.
- GPUs: Intel Iris Xe (works now) + NVIDIA RTX 3070 Ti Laptop (driver not
  loaded; `nvidia-smi` fails; `nvidia-container-toolkit` absent).
- The container runs its **own Xvfb**, so host Wayland/X is irrelevant —
  display plumbing is avoided by design.

## 4. Top risk & first gate

**GPU rendering in-container is the cockpit's go/no-go.** Gazebo Harmonic's
`ogre2` renderer on Iris Xe-via-Mesa, in a container, with **offscreen GL for
camera/depth sensors**, is the riskiest unknown. Milestone 1 is a rendering
spike: render one Gazebo camera frame inside the container. If hardware GL via
`/dev/dri` fails, fall back to `LIBGL_ALWAYS_SOFTWARE=1` (llvmpipe) — correct but
slow. Do not build the cockpit until a frame renders.

## 5. Architecture

```
 Host browser ──HTTP/WS──┐
                         │   (ports published from the container)
 ┌───────────────────────┴───────────────────────────────────────┐
 │ DEV CONTAINER (Ubuntu 24.04)                                    │
 │                                                                 │
 │  supervisor (start-all) launches, in order:                     │
 │   1. Xvfb            virtual display :99                        │
 │   2. PX4 SITL + Gazebo Harmonic   (renders to :99 via Mesa)     │
 │   3. x11vnc + noVNC/websockify     (serves :99 over WS :6080)   │
 │   4. uvicorn  →  FastAPI app (port 8000)                        │
 │                                                                 │
 │  FastAPI app  (single asyncio loop, lifespan-managed):          │
 │   DroneController(MAVSDK) ── StateStore ── SafetyGuard          │
 │   PerceptionStore ←── GazeboPerception                          │
 │   DroneAgent (Claude Agent SDK, slimmed context, OAuth)         │
 │   routes: WS /chat · WS /telemetry · GET /camera (MJPEG) ·      │
 │           static cockpit                                        │
 │                                                                 │
 │  mounts: /dev/dri (Intel GPU) · ~/.claude (OAuth, rw)           │
 └─────────────────────────────────────────────────────────────────┘
```

Published ports: **8000** (cockpit), **6080** (noVNC). The cockpit embeds noVNC
(`:6080`) in an iframe.

## 6. Container

- `.devcontainer/Dockerfile` — Ubuntu 24.04 base; installs: PX4-Autopilot
  (cloned + `make px4_sitl` build deps via `Tools/setup/ubuntu.sh`), Gazebo
  Harmonic, Mesa + Intel userspace (`mesa-utils`, `libgl1-mesa-dri`), Xvfb,
  x11vnc, novnc + websockify, Node (for the `claude` CLI), Python deps
  (`mavsdk`, `pillow`, `fastapi`, `uvicorn`, `claude-agent-sdk`), and the
  `@anthropic-ai/claude-code` CLI.
- `.devcontainer/devcontainer.json` — mounts `/dev/dri` and `~/.claude` (rw),
  adds the container user to `render`/`video` groups (matching host GID),
  forwards ports 8000 and 6080, sets `runArgs` for `--device /dev/dri`.
- `docker-compose.yml` — single service wrapping the above (so plain
  `docker compose up` works without VS Code too).
- `scripts/start-all.sh` — the supervisor: starts Xvfb, launches PX4+Gazebo on
  `DISPLAY=:99`, x11vnc+noVNC, then uvicorn; uses `LIBGL_ALWAYS_SOFTWARE`
  fallback if the GL probe fails; traps signals and tears all children down.
- `scripts/gl_probe.sh` — Milestone-1 GL/render check (`glxinfo` + a one-frame
  Gazebo camera capture).

`/dev/dri` permissions: the Dockerfile creates a `render`/`video` group with the
host's GID (passed as build args) and adds the user; documented in the README.

## 7. Web backend — `src/squawd/web/server.py`

FastAPI app; all the async stack lives in a **lifespan** context (single loop,
per the v1 single-loop principle):
- On startup: connect `DroneController`, start telemetry tasks, start
  `GazeboPerception`, capture home, enter `DroneAgent`.
- On shutdown: stop perception, cancel telemetry, exit the agent.

Routes:
- **`WS /chat`** — receive a user message; stream the agent's reply text frames
  back; log to `FlightLog`. The hard abort word set (`stop`/`abort`/…) calls
  `executor.hold()` directly + `agent.interrupt()`, bypassing the LLM (v1 §5.2).
- **`WS /telemetry`** — push a JSON state frame (lat/lon, rel-alt, mode, armed,
  in_air, battery, nearest-obstacle, home, geofence radius) at the telemetry
  rate from `StateStore` + `PerceptionStore`.
- **`GET /camera`** — `multipart/x-mixed-replace` MJPEG stream of the latest
  `PerceptionStore` JPEG frame.
- **`GET /`** + static — serves the cockpit.

The FastAPI app reuses the exact wiring from `app.py` (controller/state/safety/
perception/executor/agent); shared setup is extracted into a small
`build_stack(config)` helper so both the REPL entrypoint and the web server use
one wiring path (DRY).

## 8. SDK context slimming (required, cost)

The spike showed the SDK inherits the full Claude Code/superpowers session
(~53k tokens, ~$0.35/call). For many-turn web chat that is untenable.
`DroneAgent`'s `ClaudeAgentOptions` will explicitly **not** inherit host
settings/hooks (e.g. `setting_sources=[]`, no `SessionStart` hook injection,
`cwd` set to a clean dir), so each turn carries only the drone system prompt +
tools. Verify the per-turn token count drops to ~system-prompt size.

## 9. Cockpit frontend — `src/squawd/web/static/`

Vanilla HTML/CSS/JS, built with the frontend-design skill for a real
"mission-control" aesthetic (dark, high-contrast, monospace telemetry).

Layout:
```
┌──────────────┬─────────────────────────────┬───────────────┐
│   CHAT       │    GAZEBO 3D (noVNC iframe)  │  TELEMETRY    │
│  (WS /chat)  │    :6080                     │  (WS /telem)  │
│              │                             ├───────────────┤
│  you> …      ├─────────────────────────────┤  MAP (canvas, │
│  drone> …    │  CAMERA (MJPEG)  │ obstacle  │  home-rel +   │
│  [abort]     │                  │ readout   │  geofence)    │
└──────────────┴─────────────────────────────┴───────────────┘
```
- **Chat panel**: WebSocket to `/chat`; streamed drone replies; an always-visible
  **Abort** button that sends the direct-abort signal.
- **3D panel**: `<iframe>` to the noVNC viewer (`:6080/vnc.html?autoconnect=1`).
- **Camera panel**: `<img src="/camera">` (MJPEG) + the nearest-obstacle line.
- **Telemetry panel**: live readout from `/telemetry`.
- **Map panel**: `<canvas>` top-down plot — drone position relative to home, a
  heading arrow, and the geofence circle; updates from `/telemetry`.

`files`: `index.html`, `cockpit.css`, `cockpit.js` (or a few small JS modules:
`chat.js`, `telemetry.js`, `map.js`).

## 10. Testing strategy

- **No-sim unit tests**: the MJPEG framing helper and the telemetry-frame
  serializer are pure functions → unit-tested without the sim/container.
- **Milestone-1 gate**: `gl_probe.sh` renders a frame in-container (hardware or
  software fallback).
- **Container integration**: `docker compose up` → all four processes healthy;
  cockpit reachable on `:8000`; noVNC on `:6080` shows Gazebo; `/telemetry`
  streams; `/camera` shows frames; a chat turn flies the drone (visible in 3D).
- **Cost check**: confirm slimmed per-turn token usage.

## 11. Build order (milestones)

1. **GPU rendering spike** (go/no-go) — base image + `/dev/dri` + render one
   Gazebo frame; establish hardware-vs-software path.
2. **Self-contained container** — full Dockerfile (PX4+Gazebo build), supervisor,
   noVNC, devcontainer.json, compose; `docker compose up` brings up sim + noVNC.
3. **Backend** — `build_stack` refactor, SDK context slimming, FastAPI lifespan,
   WS chat, WS telemetry, MJPEG camera; pure-function unit tests.
4. **Cockpit frontend** — the four panels; frontend-design pass.
5. **End-to-end in container** — talk → fly → see it in 3D + camera + telemetry;
   cost check; README.

## 12. Open items carried forward
- Gazebo camera/depth **topic names + pixel format** (v1 Task 13) confirmed
  against the running sim; MJPEG assumes RGB8 until then.
- NVIDIA RTX path documented as a future toggle (host driver +
  nvidia-container-toolkit + `--gpus all`), not implemented now.
