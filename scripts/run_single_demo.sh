#!/usr/bin/env bash
# run_single_demo.sh — one drone, one pilot agent, gated by doctor_sim.sh.
# Usage:  ./scripts/run_single_demo.sh [world]     # supported demo world: demo
# Env:    SQUAWD_BACKEND=codex|kimi|claude  RENDER_BACKEND=intel|nvidia|cpu
set -eo pipefail
cd "$(dirname "$0")/.."

WORLD="${1:-baylands}"
IMG="squawd:dev"
RENDER_BACKEND="${RENDER_BACKEND:-intel}"
SQUAWD_BACKEND="${SQUAWD_BACKEND:-claude}"
case "$SQUAWD_BACKEND" in
  codex|kimi|claude) ;;
  *) echo "ERROR: SQUAWD_BACKEND must be codex, kimi, or claude"; exit 2 ;;
esac

RENDER_GID="$(getent group render | cut -d: -f3 || echo 992)"
VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"
FUEL=/tmp/swarm-gz-fuel
mkdir -p "$FUEL"

# Copy only the selected provider's login artifact into a fresh runtime home.
# Do not mount the owner's unrelated MCP, plugin, or workspace configuration.
CRED_ARGS=()
if [ "$SQUAWD_BACKEND" = "codex" ]; then
  CODEX_CRED=/tmp/pilot-codex
  rm -rf "$CODEX_CRED" 2>/dev/null || true
  mkdir -p "$CODEX_CRED"
  cp "$HOME/.codex/auth.json" "$CODEX_CRED/auth.json" 2>/dev/null || {
    echo "ERROR: ~/.codex/auth.json not found (run 'codex login' first)"; exit 1; }
  chmod 600 "$CODEX_CRED/auth.json"
  CRED_ARGS=(-v "$CODEX_CRED:/root/.codex")
elif [ "$SQUAWD_BACKEND" = "claude" ]; then
  CLAUDE_CRED=/tmp/pilot-claude
  CLAUDE_STATE=/tmp/pilot-claude.json
  rm -rf "$CLAUDE_CRED" "$CLAUDE_STATE" 2>/dev/null || true
  mkdir -p "$CLAUDE_CRED"
  cp "$HOME/.claude/.credentials.json" "$CLAUDE_CRED/" 2>/dev/null || {
    echo "ERROR: ~/.claude/.credentials.json not found (run 'claude' login first)"; exit 1; }
  printf '{}' > "$CLAUDE_STATE"
  CRED_ARGS=(-v "$CLAUDE_CRED:/root/.claude"
             -v "$CLAUDE_STATE:/root/.claude.json")
fi

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

ENV_ARGS=(-e PX4_GZ_WORLD="$WORLD" -e GZ_WORLD="$WORLD"
          -e CAM_W="${CAM_W:-640}" -e CAM_H="${CAM_H:-360}" -e CAM_FPS="${CAM_FPS:-10}"
          -e SQUAWD_BACKEND="$SQUAWD_BACKEND"
          -e VISION_BACKEND="${VISION_BACKEND:-onnx}"
          -e VISION_MODEL="${VISION_MODEL:-$VISION_MODEL_DEFAULT}")
[ -n "${KIMI_API_KEY:-}" ] && ENV_ARGS+=(-e KIMI_API_KEY="$KIMI_API_KEY")
[ -n "${SQUAWD_MODEL:-}" ] && ENV_ARGS+=(-e SQUAWD_MODEL="$SQUAWD_MODEL")
[ -n "${SQUAWD_CODEX_EFFORT:-}" ] && ENV_ARGS+=(-e SQUAWD_CODEX_EFFORT="$SQUAWD_CODEX_EFFORT")
[ "$SQUAWD_BACKEND" = "codex" ] && ENV_ARGS+=(-e CODEX_HOME=/root/.codex)

# Deep-perception sidecar (M2): the host-GPU service is reached from the
# container via the docker gateway (host.docker.internal, mapped below). The
# bearer token is read from the mounted repo at script time and exported into
# the container env — never echoed. No token file -> the pilot still boots
# with look/pinpoint answering UNAVAILABLE (agents/pilot/run.py logs one line).
DEEP_URL="${DEEP_PERCEPTION_URL:-http://host.docker.internal:8100}"
DEEP_TOKEN_VAL=""
if [ -f .deep_token ]; then
  DEEP_TOKEN_VAL="$(tr -d '[:space:]' < .deep_token)"
fi
if [ -n "$DEEP_TOKEN_VAL" ]; then
  ENV_ARGS+=(-e DEEP_PERCEPTION_URL="$DEEP_URL" -e DEEP_TOKEN="$DEEP_TOKEN_VAL")
else
  echo "  (no .deep_token — deep tools will answer UNAVAILABLE)"
fi

# M3 slowlane gate controls (agents/vision/slowlane.py): DEEP_SLOWLANE=on
# forces, =off disables; unset leaves the default gate (off when
# RENDER_BACKEND=nvidia or armed). HZ/VOCAB/CONF tune the sampler.
for v in DEEP_SLOWLANE DEEP_SLOWLANE_HZ DEEP_SLOWLANE_VOCAB DEEP_SLOWLANE_CONF; do
  [ -n "${!v:-}" ] && ENV_ARGS+=(-e "$v=${!v}")
done

echo "Launching pilot-sim (world=$WORLD, backend=$SQUAWD_BACKEND, render=$RENDER_BACKEND)…"
docker rm -f pilot-sim >/dev/null 2>&1 || true
docker run -d --name pilot-sim -p 8000:8000 \
  --dns 8.8.8.8 --dns 1.1.1.1 \
  --add-host host.docker.internal:host-gateway \
  "${GPU_ARGS[@]}" \
  --log-opt max-size=20m --log-opt max-file=2 \
  -v "$PWD:/workspace" "${CRED_ARGS[@]}" \
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
# The M2.5 nano-seg detector (VISION_BACKEND=onnx) is the production
# perception, not the interim blob (which merges the box's
# shadowed face with its ground shadow at low level view).
docker exec -d pilot-sim bash -lc "cd /workspace && PYTHONPATH=/workspace:\$PYTHONPATH PYTHONUNBUFFERED=1 uv run --no-project python agents/pilot/run.py > /tmp/pilot.log 2>&1"

# Deep sidecar hint (M2): probe /v1/health from INSIDE the container (token
# via env, never printed). Advisory only — the pilot boots regardless.
if [ -n "$DEEP_TOKEN_VAL" ]; then
  if docker exec pilot-sim bash -lc \
      'curl -sf -m 3 -H "Authorization: Bearer $DEEP_TOKEN" "$DEEP_PERCEPTION_URL/v1/health" >/dev/null 2>&1'; then
    echo "  deep sidecar reachable from the container (look/pinpoint live)."
  else
    echo "  hint: deep sidecar NOT reachable from the container — look/pinpoint"
    echo "        will answer UNAVAILABLE. Start it on the host: ./scripts/deep_perception.sh"
  fi
fi

echo
echo "  ✈  Single drone up. Command it with:"
echo "     docker exec pilot-sim bash -lc \"ros2 topic pub --once /pilot/user_input std_msgs/String \\\"{data: 'take off to 12m and orbit bldg_1'}\\\"\""
echo "     Logs: docker exec pilot-sim tail -f /tmp/pilot.log"
echo "     Estop: docker exec pilot-sim bash -lc \"ros2 topic pub --once /pilot/estop std_msgs/String \\\"{data: 'hold'}\\\"\""
echo "     Stop:  docker rm -f pilot-sim"
