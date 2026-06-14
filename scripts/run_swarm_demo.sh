#!/usr/bin/env bash
# One-command swarm demo: N drones with onboard camera POV + an interactive
# Commander you chat with. Open http://localhost:8000 and type commands.
#
# Usage:  ./scripts/run_swarm_demo.sh [N]        # default 3 drones, GPU cameras
#         GPU=0 ./scripts/run_swarm_demo.sh [N]  # software GL, no cameras (slower)
#
# GPU mode renders camera sensors on the Intel iGPU via EGL (RTF ~1.0). It exposes
# ONLY /dev/dri/renderD128 so Mesa can't grab the NVIDIA node (-> ogre2 segfault).
set -eo pipefail
cd "$(dirname "$0")/.."

N="${1:-3}"
IMG="dronebot-swarm:dev"
GPU="${GPU:-1}"
RENDER_GID="$(getent group render | cut -d: -f3 || echo 992)"
VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"

# Isolated copy of Claude OAuth creds (never write to the live ~/.claude).
CRED=/tmp/swarm-claude
rm -rf "$CRED" "$CRED.json" 2>/dev/null || true
mkdir -p "$CRED"
cp "$HOME/.claude/.credentials.json" "$CRED/" 2>/dev/null || {
  echo "ERROR: ~/.claude/.credentials.json not found (log in with 'claude' first)"; exit 1; }
printf '{}' > "$CRED.json"

GPU_ARGS=()
if [ "$GPU" = "1" ]; then
  GPU_ARGS=(--device /dev/dri/renderD128 --device /dev/dri/card1
            --group-add "$RENDER_GID" --group-add "$VIDEO_GID"
            -e GPU_RENDER=1 -e PX4_MODEL=gz_x500_depth)
else
  GPU_ARGS=(-e PX4_MODEL=gz_x500)
fi

echo "Launching swarm-multi (N=$N, GPU=$GPU)…"
docker rm -f swarm-multi >/dev/null 2>&1 || true
docker run -d --name swarm-multi -p 8000:8000 \
  "${GPU_ARGS[@]}" \
  --log-opt max-size=20m --log-opt max-file=2 \
  -v "$PWD:/workspace" -v "$CRED:/root/.claude" -v "$CRED.json:/root/.claude.json" \
  -e SWARM_N="$N" "$IMG" bash -lc 'sim/launch/swarm_sim.sh' >/dev/null

echo "Waiting for the sim (PX4 + cameras)…"
for _ in $(seq 1 25); do
  c=$(docker exec swarm-multi bash -lc 'ros2 topic list 2>/dev/null | grep -c vehicle_local_position' 2>/dev/null | tr -dc '0-9')
  [ -n "$c" ] && [ "$c" -ge "$N" ] 2>/dev/null && break
  sleep 6
done

echo "Starting observatory + agents…"
docker exec -d swarm-multi bash -lc "cd /workspace && PYTHONPATH=/workspace:\$PYTHONPATH SWARM_N=$N uv run --no-project python agents/observatory/server.py > /tmp/obs.log 2>&1"
docker exec -d swarm-multi bash -lc "cd /workspace && PYTHONPATH=/workspace:\$PYTHONPATH SWARM_N=$N PYTHONUNBUFFERED=1 uv run --no-project python agents/swarm/run_swarm.py > /tmp/swarm.log 2>&1"

echo
echo "  ✈  Swarm up.  Open  http://localhost:8000  and command the swarm."
echo "     e.g. \"everyone take off and spread out\" · \"drone_1 climb to 20m\" · \"all return and land\""
echo "     Logs: docker exec swarm-multi tail -f /tmp/swarm.log"
echo "     Stop: docker rm -f swarm-multi"
