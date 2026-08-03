#!/usr/bin/env bash
# run_single_demo.sh — one drone, one pilot agent, gated by doctor_sim.sh.
# Usage:  ./scripts/run_single_demo.sh [world]     # world: baylands (default) | city | dynamic
# Env:    SQUAWD_BACKEND=claude|kimi  RENDER_BACKEND=intel|nvidia|cpu  CAM_W/H/FPS
set -eo pipefail
cd "$(dirname "$0")/.."

WORLD="${1:-baylands}"
IMG="squawd:dev"
RENDER_BACKEND="${RENDER_BACKEND:-intel}"
SQUAWD_BACKEND="${SQUAWD_BACKEND:-claude}"

RENDER_GID="$(getent group render | cut -d: -f3 || echo 992)"
VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"
FUEL=/tmp/swarm-gz-fuel
mkdir -p "$FUEL"

# Agent credentials: Claude OAuth copy for the claude backend; Kimi needs only
# the API key in env (no ~/.claude mount required).
CRED=/tmp/pilot-claude
rm -rf "$CRED" "$CRED.json" 2>/dev/null || true
mkdir -p "$CRED"
if [ "$SQUAWD_BACKEND" != "kimi" ]; then
  cp "$HOME/.claude/.credentials.json" "$CRED/" 2>/dev/null || {
    echo "ERROR: ~/.claude/.credentials.json not found (log in with 'claude' first, or set SQUAWD_BACKEND=kimi)"; exit 1; }
fi
printf '{}' > "$CRED.json"

GPU_ARGS=()
# Intel device nodes are resolved from the PCI by-path symlinks AT LAUNCH:
# card/renderD numbering flips across boots (hardcoding can point Mesa at the
# dGPU), and the by-path links themselves contain ':' which breaks --device.
INTEL_CARD=$(readlink -f /dev/dri/by-path/pci-0000:00:02.0-card)
INTEL_RENDER=$(readlink -f /dev/dri/by-path/pci-0000:00:02.0-render)
case "$RENDER_BACKEND" in
  intel)  GPU_ARGS=(--device "$INTEL_RENDER" --device "$INTEL_CARD"
            --group-add "$RENDER_GID" --group-add "$VIDEO_GID"
            -e RENDER_BACKEND=intel -e PX4_MODEL=gz_x500_depth) ;;
  nvidia) GPU_ARGS=(--gpus all -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display
            -e RENDER_BACKEND=nvidia
            -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
            -e PX4_MODEL=gz_x500_depth) ;;
  *)      GPU_ARGS=(-e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500) ;;
esac

# W2 (design §4): the demo world runs the general COCO detector at 640 by
# default (p50 39.6 ms, inside the 10 Hz budget); other worlds keep the
# 2-class mover model. VISION_MODEL env always wins. The ONNX graph fixes its
# own input size, so no imgsz flag flows — OnnxBackend reads it at load.
VISION_MODEL_DEFAULT="mover-nano-seg-v1.onnx"
[ "$WORLD" = "demo" ] && VISION_MODEL_DEFAULT="coco-nano-seg-v1-640.onnx"

ENV_ARGS=(-e SWARM_N=1 -e PX4_GZ_WORLD="$WORLD" -e GZ_WORLD="$WORLD"
          -e CAM_W="${CAM_W:-640}" -e CAM_H="${CAM_H:-360}" -e CAM_FPS="${CAM_FPS:-10}"
          -e SQUAWD_BACKEND="$SQUAWD_BACKEND"
          -e VISION_BACKEND="${VISION_BACKEND:-onnx}"
          -e VISION_MODEL="${VISION_MODEL:-$VISION_MODEL_DEFAULT}")
[ -n "${KIMI_API_KEY:-}" ] && ENV_ARGS+=(-e KIMI_API_KEY="$KIMI_API_KEY")
[ -n "${SQUAWD_MODEL:-}" ] && ENV_ARGS+=(-e SQUAWD_MODEL="$SQUAWD_MODEL")

echo "Launching pilot-sim (N=1, world=$WORLD, backend=$SQUAWD_BACKEND, render=$RENDER_BACKEND)…"
docker rm -f pilot-sim >/dev/null 2>&1 || true
docker run -d --name pilot-sim -p 8000:8000 \
  --dns 8.8.8.8 --dns 1.1.1.1 \
  "${GPU_ARGS[@]}" \
  --log-opt max-size=20m --log-opt max-file=2 \
  -v "$PWD:/workspace" -v "$CRED:/root/.claude" -v "$CRED.json:/root/.claude.json" \
  -v "$FUEL:/root/.gz/fuel" \
  "${ENV_ARGS[@]}" \
  "$IMG" bash -lc 'sim/launch/swarm_sim.sh' >/dev/null

echo "Waiting for the sim (world load + PX4 + camera)…"
for _ in $(seq 1 50); do
  c=$(docker exec pilot-sim bash -lc 'ros2 topic list 2>/dev/null | grep -c vehicle_local_position' 2>/dev/null | tr -dc '0-9' || true)
  if [ -n "$c" ] && [ "$c" -ge 1 ] 2>/dev/null; then echo "  sim ready ($c/1 drones)."; break; fi
  sleep 6
done

echo "Preflight (doctor_sim.sh — hard gate)…"
if ! docker exec pilot-sim bash -lc 'bash scripts/doctor_sim.sh'; then
  echo "ERROR: doctor_sim.sh FAILED — refusing to start the pilot. Container left"
  echo "running for diagnosis: docker exec -it pilot-sim bash"
  exit 1
fi

echo "Starting pilot agent…"
# --with onnxruntime: the M2.5 nano-seg detector (VISION_BACKEND=onnx) — the
# production perception, not the interim blob (which merges the box's
# shadowed face with its ground shadow at low level view).
docker exec -d pilot-sim bash -lc "cd /workspace && PYTHONPATH=/workspace:\$PYTHONPATH PYTHONUNBUFFERED=1 uv run --no-project --with onnxruntime python agents/pilot/run.py > /tmp/pilot.log 2>&1"

echo
echo "  ✈  Single drone up. Command it with:"
echo "     docker exec pilot-sim bash -lc \"ros2 topic pub --once /pilot/user_input std_msgs/String \\\"{data: 'take off to 12m and orbit bldg_1'}\\\"\""
echo "     Logs: docker exec pilot-sim tail -f /tmp/pilot.log"
echo "     Estop: docker exec pilot-sim bash -lc \"ros2 topic pub --once /pilot/estop std_msgs/String \\\"{data: 'hold'}\\\"\""
echo "     Stop:  docker rm -f pilot-sim"
