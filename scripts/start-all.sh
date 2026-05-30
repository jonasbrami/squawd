#!/usr/bin/env bash
# Supervisor: bring up the virtual display, sim, noVNC, and the web app,
# then wait. Tears everything down on signal.
set -uo pipefail

export DISPLAY=:99
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SCREEN="${DRONEBOT_SCREEN:-1920x1080x24}"   # noVNC resolution (WxHxDepth)
SCREEN_W="${SCREEN%%x*}"; SCREEN_H="$(echo "$SCREEN" | cut -dx -f2)"
PIDS=()

cleanup() {
  echo "shutting down..."
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1. Virtual display
Xvfb :99 -screen 0 "$SCREEN" >/tmp/xvfb.log 2>&1 &
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

# 2b. mavsdk_server as its OWN process (NOT a child of the forking web app).
# The Claude Agent SDK spawns subprocesses (the claude CLI); an in-process
# bundled mavsdk_server gets killed by that fork (zombie -> "socket closed").
# Running it standalone keeps the MAVLink/gRPC link alive across agent turns.
MAVSDK_SERVER="$(python3 -c 'import mavsdk,os;print(os.path.join(os.path.dirname(mavsdk.__file__),"bin","mavsdk_server"))')"
"$MAVSDK_SERVER" udpin://0.0.0.0:14540 -p 50051 >/tmp/mavsdk_server.log 2>&1 &
PIDS+=($!)

# 2c. Gazebo GUI client -> renders the 3D world to :99 for noVNC, then resize
# it to fill the screen. PX4 only launches the headless server.
(
  for _ in $(seq 1 60); do gz topic -l >/dev/null 2>&1 && break; sleep 1; done
  # Once the GUI window appears, stretch it to fill the (high-res) display.
  ( for _ in $(seq 1 90); do
      if xdotool search --name "Gazebo Sim" >/dev/null 2>&1; then
        xdotool search --name "Gazebo Sim" windowsize "$SCREEN_W" "$SCREEN_H" windowmove 0 0
        break
      fi
      sleep 1
    done ) &
  # Pure-software Mesa (llvmpipe): /dev/dri is not passed through, so there is
  # no hardware EGL device for ogre2 to grab and segfault on.
  LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
    gz sim -g >/tmp/gzgui.log 2>&1
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
