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

# 2. PX4 SITL + Gazebo SERVER (PX4 starts gz with -s = headless server only)
( cd "$PX4_DIR" && HEADLESS=0 make px4_sitl gz_x500_depth >/tmp/px4.log 2>&1 ) &
PIDS+=($!)

# 2b. Gazebo GUI client -> renders the 3D world to :99 for noVNC.
# PX4 only launches the headless server, so we attach the GUI ourselves once
# the server's transport is up.
(
  for _ in $(seq 1 60); do gz topic -l >/dev/null 2>&1 && break; sleep 1; done
  # Force software GL for the GUI: Xvfb provides no hardware GLX, so a
  # hardware attempt would crash. Sensors keep their own (EGL/GPU) path.
  LIBGL_ALWAYS_SOFTWARE=1 gz sim -g >/tmp/gzgui.log 2>&1
) &
PIDS+=($!)

# 3. noVNC: expose the virtual display over the browser (port 6080)
x11vnc -display :99 -forever -shared -nopw -quiet >/tmp/x11vnc.log 2>&1 &
PIDS+=($!)
websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
PIDS+=($!)

# 4. Web app (FastAPI) on port 8000.
# `python3 -m uvicorn` (not the `uvicorn` script): pip --user puts console
# scripts in ~/.local/bin, which isn't on PATH in the container.
python3 -m uvicorn dronebot.web.server:app --host 0.0.0.0 --port 8000 >/tmp/web.log 2>&1 &
PIDS+=($!)

echo "dronebot up: cockpit http://localhost:8000  noVNC http://localhost:6080/vnc.html"
wait
