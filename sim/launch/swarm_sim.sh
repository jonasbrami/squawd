#!/usr/bin/env bash
# Single-vehicle PX4 SITL in one Gazebo world. The filename is historical.
# PX4 uses /px4_0/fmu/*, MAVLink UDP 14540, and mavsdk_server port 50051.
# `-d` runs px4 detached (no interactive pxh> console -> no log spam).
# NB: no `-u` — ROS setup.bash references unbound vars.
set -eo pipefail

# Rendering backend selector.
#   RENDER_BACKEND=cpu     -> software GL (llvmpipe), no GPU needed (default if GPU_RENDER!=1)
#   RENDER_BACKEND=intel   -> headless EGL on the Intel iGPU (current GPU_RENDER=1 behaviour)
#   RENDER_BACKEND=nvidia  -> headless EGL on the NVIDIA dGPU (needs NVIDIA EGL libs + /dev/nvidia*)
# Back-compat: GPU_RENDER=1 with no RENDER_BACKEND == intel.
RENDER_BACKEND="${RENDER_BACKEND:-}"
if [ -z "$RENDER_BACKEND" ]; then
  if [ "${GPU_RENDER:-0}" = "1" ]; then
    RENDER_BACKEND=intel
  else
    RENDER_BACKEND=cpu
  fi
fi
case "$RENDER_BACKEND" in
  nvidia)
    unset LIBGL_ALWAYS_SOFTWARE
    # ogre2 must use NVIDIA's EGL, never Mesa: force the glvnd vendor ICD and
    # make sure no Mesa driver override is set.
    unset MESA_LOADER_DRIVER_OVERRIDE
    NV_ICD="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
    # The nvidia-container-toolkit (`--gpus all`, caps incl. graphics) injects
    # libEGL_nvidia.so.0 but does NOT create the glvnd vendor ICD json. Without it
    # __EGL_VENDOR_LIBRARY_FILENAMES points at a missing file -> glvnd loads no
    # NVIDIA ICD -> ogre2 null-derefs in Ogre2RenderEngine::CreateRenderSystem
    # (the reported segfault). Create it here, idempotently, when the NVIDIA EGL
    # lib is actually present.
    if [ ! -f "$NV_ICD" ] && ls /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.0 >/dev/null 2>&1; then
      mkdir -p "$(dirname "$NV_ICD")"
      printf '{\n  "file_format_version":"1.0.0",\n  "ICD":{"library_path":"libEGL_nvidia.so.0"}\n}\n' > "$NV_ICD"
    fi
    export __EGL_VENDOR_LIBRARY_FILENAMES="$NV_ICD"
    export QT_QPA_PLATFORM=offscreen
    # NVIDIA EGL needs the explicit headless-rendering surface or ogre2 segfaults
    # in CreateRenderSystem (the Mesa/iris path tolerates its absence).
    GZ_HR="--headless-rendering"
    ;;
  intel)
    unset LIBGL_ALWAYS_SOFTWARE
    # Headless EGL on the Intel iGPU. Expose ONLY renderD128 to the container so
    # Mesa can't grab the NVIDIA node (no Mesa driver there -> ogre2 segfault).
    export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-iris}"
    export QT_QPA_PLATFORM=offscreen
    ;;
  *)  # cpu / llvmpipe
    export LIBGL_ALWAYS_SOFTWARE=1
    unset MESA_LOADER_DRIVER_OVERRIDE
    ;;
esac

# Server-only Gazebo. There's no display in the container, and under the GPU path
# (QT_QPA_PLATFORM=offscreen) the gz GUI aborts in handleContextCreationFailure,
# which takes down the px4 -i 0 instance that spawned gz. HEADLESS=1 makes PX4
# launch `gz sim -s` (no GUI); cameras render from the server sensor pipeline.
export HEADLESS=1

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

# ros_gz vendor packages prepend a vendored `gz` (no `sim` subcommand) on PATH +
# GZ_CONFIG_PATH, which breaks PX4's `gz sim`. Force the system Gazebo Harmonic.
# (The ros_gz image_bridge uses the gz-transport lib directly, so it's unaffected.)
export PATH=/usr/bin:$PATH
export GZ_CONFIG_PATH=/usr/share/gz

cd /workspace/PX4-Autopilot
WORLDS="Tools/simulation/gz/worlds"

# Default world: 'baylands' — PX4's realistic coastal scene (road, grass, water,
# trees). NB: baylands' trees render as black silhouettes under headless EGL
# (known artifact, scene is fine).
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-baylands}"

# Per-world asset setup.
if [ "$PX4_GZ_WORLD" = "obstacles" ]; then
  # Flat default world + 6 static box buildings (evals obstacle ladder). Built
  # from default.sdf idempotently, with the obstacles_boxes.json sidecar the
  # agents (scan) and evals oracle both read for building geometry.
  if [ -f "$WORLDS/default.sdf" ] && [ -f /workspace/sim/worlds/make_obstacles_world.py ]; then
    python3 /workspace/sim/worlds/make_obstacles_world.py "$WORLDS/default.sdf" "$WORLDS/obstacles.sdf" || true
  fi
elif [ "$PX4_GZ_WORLD" = "dynamic" ]; then
  # Flat default world + scripted kinematic movers (evals dynamic ladder).
  # Movers are driven per physics step by sim/plugins/mover_system.py
  # (PythonSystemLoader); the plugin path and the trajectory sidecar must both
  # be visible to the gz server process launched below.
  if [ -f "$WORLDS/default.sdf" ] && [ -f /workspace/sim/worlds/make_dynamic_world.py ]; then
    python3 /workspace/sim/worlds/make_dynamic_world.py "$WORLDS/default.sdf" "$WORLDS/dynamic.sdf" || true
  fi
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}/workspace/sim/plugins"
  export MOVERS_JSON="/workspace/PX4-Autopilot/$WORLDS/dynamic_boxes.json"
elif [ "$PX4_GZ_WORLD" = "perceive" ]; then
  # Perceive ladder (M5): true orange rover + visually distinct ground decoys,
  # same mover plugin mechanics as the dynamic world.
  if [ -f "$WORLDS/default.sdf" ] && [ -f /workspace/sim/worlds/make_perceive_world.py ]; then
    python3 /workspace/sim/worlds/make_perceive_world.py "$WORLDS/default.sdf" "$WORLDS/perceive.sdf" || true
  fi
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}/workspace/sim/plugins"
  export MOVERS_JSON="/workspace/PX4-Autopilot/$WORLDS/perceive_boxes.json"
elif [ "$PX4_GZ_WORLD" = "demo" ]; then
  # W1b demo world (design 2026-07-28 §3): 5 mesh-visual movers (velocity-drive
  # mover_system plugin, heading_align) + Fuel landmark neighborhood. Fuel
  # models download once into the cache; the generator then resolves the
  # car/house/tree meshes and their scales/poses from the cached model.sdf
  # files.
  MARKER="$HOME/.gz/fuel/.w1b_demo_v1"
  if [ ! -f "$MARKER" ]; then
    echo "downloading demo-world Fuel models (one-time)…"
    for m in "Hatchback red" Hatchback SUV TruckDelivery "Walking person" \
             "Gas Station" "Pine Tree" "Oak Tree" "Lamp Post" "House 1" "House 2"; do
      gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/$m" -t model || true
    done
    touch "$MARKER"
  fi
  # Walking person: strip <library_animations> from the cached dae into
  # walking_frozen.dae (idempotent) — the demo
  # renders walkers as STATIC mesh visuals (frozen stride; <actor> stalls the
  # headless render thread, W0.1 2026-08-01), and the keyframe-free file
  # parses far faster.
  python3 - <<'EOF' || true
import glob, re
for src in glob.glob("/root/.gz/fuel/fuel.gazebosim.org/openrobotics/models/"
                     "walking person/*/meshes/walking.dae"):
    dst = src.replace("walking.dae", "walking_frozen.dae")
    try:
        xml = open(src, encoding="utf-8", errors="ignore").read()
        xml = re.sub(r"<library_animations>.*?</library_animations>", "", xml,
                     flags=re.S)
        xml = re.sub(r"<library_animation_clips>.*?</library_animation_clips>",
                     "", xml, flags=re.S)
        open(dst, "w").write(xml)
        print("wrote", dst, len(xml) >> 20, "MB")
    except OSError as e:
        print("frozen-strip failed:", e)
EOF
  # Fuel texture path fixes (cache-side, idempotent): the car meshes reference
  # textures by bare name (resolved against
  # meshes/, where they do NOT live) and by model://<name>/... URIs (which
  # need a version-less model dir on the resource path). Without this the cars
  # render untextured gray (W0.1 lesson).
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
  # "Hatchback red" ships WITHOUT wheels3.png although its mtl references
  # ../materials/textures/wheels3.png (same wheel mesh as the plain Hatchback)
  # — borrow the texture or the wheels render untextured (found W1a 2026-08-01).
  RED="$(ls -d "$HOME"/.gz/fuel/fuel.gazebosim.org/openrobotics/models/hatchback\ red/*/ 2>/dev/null | sort -V | tail -1)"
  PLAIN="$(ls -d "$HOME"/.gz/fuel/fuel.gazebosim.org/openrobotics/models/hatchback/*/ 2>/dev/null | sort -V | tail -1)"
  if [ -n "$RED" ] && [ -n "$PLAIN" ]; then
    cp -n "${PLAIN}meshes/wheels3.png" "${RED}materials/textures/wheels3.png" 2>/dev/null || true
    cp -n "${PLAIN}meshes/wheels3.png" "${RED}meshes/wheels3.png" 2>/dev/null || true
  fi
  export GZ_SIM_RESOURCE_PATH="$LINKS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
  if [ -f "$WORLDS/default.sdf" ] && [ -f /workspace/sim/worlds/make_demo_world.py ]; then
    python3 /workspace/sim/worlds/make_demo_world.py "$WORLDS/default.sdf" "$WORLDS/demo.sdf" || true
  fi
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}/workspace/sim/plugins"
  export MOVERS_JSON="/workspace/PX4-Autopilot/$WORLDS/demo_boxes.json"
elif [ "$PX4_GZ_WORLD" = "baylands" ]; then
  # baylands.sdf <include>s two Gazebo Fuel models (terrain + Coast Water). Cache
  # them once (needs internet on the first run; mount /root/.gz/fuel to persist).
  if [ ! -d "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/baylands" ]; then
    echo "downloading baylands Fuel models (one-time, ~400MB)…"
    gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/baylands" -t model || true
    gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coast Water" -t model || true
  fi
fi

MODEL=${PX4_MODEL:-gz_x500_depth}

# PX4_GZ_STANDALONE needs the model/world resource path so it can spawn into the
# already-running gz server (below).
export GZ_SIM_RESOURCE_PATH="/workspace/sim/models:${GZ_SIM_RESOURCE_PATH}:/workspace/PX4-Autopilot/$WORLDS/../models:/workspace/PX4-Autopilot/$WORLDS"

# Patch the OakD-Lite camera (idempotent): drop the shared
# `<topic>camera</topic>` override so Gazebo scopes it to the model, and cut it
# to 640x360@10Hz.
CAM_W="${CAM_W:-640}"; CAM_H="${CAM_H:-360}"; CAM_FPS="${CAM_FPS:-10}"
OAKD="Tools/simulation/gz/models/OakD-Lite/model.sdf"
if [ -f "$OAKD" ] && grep -q "<topic>camera</topic>" "$OAKD"; then
  sed -i \
    -e "s|<width>1920</width>|<width>${CAM_W}</width>|" \
    -e "s|<height>1080</height>|<height>${CAM_H}</height>|" \
    -e "s|<update_rate>30</update_rate>|<update_rate>${CAM_FPS}</update_rate>|g" \
    -e "/<topic>camera<\/topic>/d" \
    "$OAKD"
fi

# Start gz as a persistent standalone server first, then attach PX4 to it
# (PX4_GZ_STANDALONE=1). Previously PX4 launched gz itself and raced
# its own create-service — on a busy host (or the heavier baylands world) it timed
# out and killed the gz it spawned. Starting gz up
# front and waiting until its spawn service is live removes the race entirely.
echo "starting gz server ($PX4_GZ_WORLD)…"
gz sim -v1 -r -s ${GZ_HR:-} "$WORLDS/$PX4_GZ_WORLD.sdf" >/tmp/gz.log 2>&1 &
for _ in $(seq 1 60); do
  gz service -l 2>/dev/null | grep -q "/world/$PX4_GZ_WORLD/create" && break
  sleep 2
done

# Bridge PX4 to ROS 2.
MicroXRCEAgent udp4 -p 8888 >/tmp/xrce.log 2>&1 &
sleep 2

# PX4's sitl launcher creates its working dir NON-recursively and dies when
# rootfs is absent (observed: "Error creating directory .../rootfs/0").
mkdir -p build/px4_sitl_default/rootfs

# Boot PX4 with FACTORY state. The rootfs persists parameters.bson/dataman/
# eeprom across container rebuilds (host bind-mount), and PX4 auto-saves its
# learned EKF2_MAG_DECL at disarm — a declination learned in one world
# (baylands CA ≈ +12.8°) poisons boots in another (dynamic Zurich ≈
# +2.5°): mag innovations flap across the arming gate and arming is denied
# (root-caused 2026-07-21; wiped state + hold→arm→takeoff restored flight).
rm -f build/px4_sitl_default/rootfs/0/parameters.bson \
      build/px4_sitl_default/rootfs/0/parameters_backup.bson \
      build/px4_sitl_default/rootfs/0/dataman
rm -rf build/px4_sitl_default/rootfs/0/eeprom

# SIM_GZ_EN_LIDAR=0: PX4's gz_bridge must NOT ingest the composite's forward
# ToF beam as a distance_sensor — a horizontal beam feeding the EKF
# destabilizes it (observed: ~700m physical drift + arming denial, fable-MAJOR-3).
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL="${MODEL}" \
  SIM_GZ_EN_LIDAR=0 PX4_GZ_MODEL_POSE="0,0,0.5" PX4_UXRCE_DDS_NS="px4_0" \
  ./build/px4_sitl_default/bin/px4 -i 0 -d >/tmp/px4_0.log 2>&1 &

mavsdk_server -p 50051 "udpin://0.0.0.0:14540" >/tmp/mav_0.log 2>&1 &

echo "single-drone bring-up launched (world=$PX4_GZ_WORLD)"
wait
