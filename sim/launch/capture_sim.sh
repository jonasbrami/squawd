#!/usr/bin/env bash
# capture_sim — gz-only capture boot for the W2.5b demo-capture world
# (sim/worlds/make_demo_capture_world.py): no PX4, no drone — the demo cast
# on the mover plugin plus the profile's static capture cameras, then
# scripts/demo_dataset.py fills the (aspect x band x clip) cell quotas.
#
#   docker run -d --name w25-capture --device $INTEL_RENDER --device $INTEL_CARD \
#     --group-add $RENDER_GID --group-add $VIDEO_GID \
#     -v "$PWD:/workspace" -v /tmp/swarm-gz-fuel:/root/.gz/fuel \
#     -e CAPTURE_PROFILE=car_1 -e CAPTURE_MINUTES=35 \
#     squawd:dev bash -lc 'sim/launch/capture_sim.sh'
#
# Env: CAPTURE_PROFILE (car_1|car_2|car_3|walkers|negatives, default car_1),
#      CAPTURE_MINUTES (capture deadline, default 35),
#      CAPTURE_FRESH=1 to ignore capture_state.json (start quotas over).
set -eo pipefail
# NB: no ROS setup needed (no PX4) — but ros_gz's vendored gz must not shadow
# the system Gazebo Harmonic on PATH (swarm_sim.sh lesson).
export PATH=/usr/bin:$PATH
export GZ_CONFIG_PATH=/usr/share/gz

# Isolate this boot's gz transport from other gz instances on the docker
# bridge (pilot-sim, stale/overlapping capture boots): the capture subscriber
# wedges on a stale duplicate demo_capture publisher (zero callbacks forever;
# W2.5b dud boots, 2026-08-02). One partition per capture fleet is enough —
# the real rule is: never two demo_capture servers alive at once.
export GZ_PARTITION="${GZ_PARTITION:-w25cap}"

# Headless EGL on the Intel iGPU (mirrors swarm_sim.sh's intel branch).
unset LIBGL_ALWAYS_SOFTWARE
export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-iris}"
export QT_QPA_PLATFORM=offscreen
export HEADLESS=1

# Fuel texture path fixes (cache-side, idempotent — swarm_sim.sh demo branch):
# the car meshes reference textures by bare name and model:// URIs; provide
# the version-less model dir links + copy textures next to the meshes.
LINKS="$HOME/.gz/model-links"
mkdir -p "$LINKS"
for d in "$HOME"/.gz/fuel/fuel.gazebosim.org/openrobotics/models/*/; do
  latest="$(ls -d "$d"*/ 2>/dev/null | sort -V | tail -1)"
  [ -n "$latest" ] || continue
  ln -sfn "$latest" "$LINKS/$(basename "$d")"
  if [ -d "${latest}materials/textures" ]; then
    cp -n "${latest}materials/textures"/*.png "${latest}meshes/" 2>/dev/null || true
  fi
done
# "Hatchback red" ships without wheels3.png (its mtl references it) — borrow
# the plain Hatchback's or the wheels render untextured (W1a 2026-08-01).
RED="$(ls -d "$HOME"/.gz/fuel/fuel.gazebosim.org/openrobotics/models/hatchback\ red/*/ 2>/dev/null | sort -V | tail -1)"
PLAIN="$(ls -d "$HOME"/.gz/fuel/fuel.gazebosim.org/openrobotics/models/hatchback/*/ 2>/dev/null | sort -V | tail -1)"
if [ -n "$RED" ] && [ -n "$PLAIN" ]; then
  cp -n "${PLAIN}meshes/wheels3.png" "${RED}materials/textures/wheels3.png" 2>/dev/null || true
  cp -n "${PLAIN}meshes/wheels3.png" "${RED}meshes/wheels3.png" 2>/dev/null || true
fi

PROFILE="${CAPTURE_PROFILE:-car_1}"
SRC=/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
SDF="/tmp/demo_capture_${PROFILE}.sdf"
python3 /workspace/sim/worlds/make_demo_capture_world.py "$SRC" "$SDF" "$PROFILE"
export GZ_SIM_RESOURCE_PATH="$LINKS:/workspace/sim/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/workspace/sim/plugins"
export MOVERS_JSON="${SDF%.sdf}_boxes.json"

echo "starting gz server (demo_capture, profile=$PROFILE)…"
gz sim -v1 -r -s "$SDF" >/tmp/gz.log 2>&1 &
for _ in $(seq 1 90); do
  gz service -l 2>/dev/null | grep -q "/world/demo_capture/create" && break
  sleep 2
done
# Warmup gate: the world service appears before the transport graph settles;
# subscribing that early wedged the capture node on a stale publisher (dud
# boots saved 0 frames, 2026-08-02). Wait for real pose traffic first.
for _ in $(seq 1 30); do
  timeout 5 gz topic -e -t "/world/demo_capture/dynamic_pose/info" -n 1 \
    >/dev/null 2>&1 && break
  sleep 2
done

cd /workspace
EXTRA=()
[ "${CAPTURE_FRESH:-0}" = "1" ] && EXTRA+=(--fresh)
uv run --no-project python scripts/demo_dataset.py \
  --sidecar "$MOVERS_JSON" \
  --out /workspace/evals/out/w25b_dataset \
  --minutes "${CAPTURE_MINUTES:-35}" "${EXTRA[@]}"
# frames are written as root; hand the dataset back to the host user so
# host-side tooling (replay merge, QA) can write alongside
chown -R 1000:1000 /workspace/evals/out/w25b_dataset 2>/dev/null || true
