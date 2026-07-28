#!/usr/bin/env bash
# Multi-vehicle PX4 SITL: N drones in one Gazebo world.
# Each instance i: spawns at (0, i*3), namespaces ROS2 topics as /px4_<i>/fmu/*,
# and gets MAVLink offboard on udp 14540+i -> mavsdk_server on 50051+i.
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
# trees). Override with PX4_GZ_WORLD=city for the procedural building world (which
# also gives the drones building obstacle-awareness via scan). NB: baylands' trees
# render as black silhouettes under headless EGL (known artifact, scene is fine).
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-baylands}"

# Per-world asset setup.
if [ "$PX4_GZ_WORLD" = "city" ]; then
  # Build the 'city' world (buildings) from default.sdf, idempotently — and the
  # matching city_boxes.json the agents read for obstacle/proximity awareness.
  if [ -f "$WORLDS/default.sdf" ] && [ -f /workspace/sim/worlds/make_city_world.py ]; then
    python3 /workspace/sim/worlds/make_city_world.py "$WORLDS/default.sdf" "$WORLDS/city.sdf" || true
  fi
elif [ "$PX4_GZ_WORLD" = "obstacles" ]; then
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
elif [ "$PX4_GZ_WORLD" = "baylands" ]; then
  # baylands.sdf <include>s two Gazebo Fuel models (terrain + Coast Water). Cache
  # them once (needs internet on the first run; mount /root/.gz/fuel to persist).
  if [ ! -d "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/baylands" ]; then
    echo "downloading baylands Fuel models (one-time, ~400MB)…"
    gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/baylands" -t model || true
    gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coast Water" -t model || true
  fi
fi

N=${SWARM_N:-3}
MODEL=${PX4_MODEL:-gz_x500_depth}

# PX4_GZ_STANDALONE needs the model/world resource path so it can spawn into the
# already-running gz server (below).
export GZ_SIM_RESOURCE_PATH="/workspace/sim/models:${GZ_SIM_RESOURCE_PATH}:/workspace/PX4-Autopilot/$WORLDS/../models:/workspace/PX4-Autopilot/$WORLDS"

# Patch the OakD-Lite camera (idempotent) so each drone gets a UNIQUE, low-res
# camera topic: drop the shared `<topic>camera</topic>` override (-> gz scopes the
# topic per model: /world/default/model/x500_depth_<i>/.../IMX214/image) and cut
# it to 640x360@10Hz so N feeds render on software GL.
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

# Start gz as a PERSISTENT standalone server FIRST, then attach every PX4 instance
# to it (PX4_GZ_STANDALONE=1). Previously instance 0 launched gz itself and raced
# its own create-service — on a busy host (or the heavier baylands world) it timed
# out and KILLED the gz it spawned, taking the whole swarm down. Starting gz up
# front and waiting until its spawn service is live removes the race entirely.
echo "starting gz server ($PX4_GZ_WORLD)…"
gz sim -v1 -r -s ${GZ_HR:-} "$WORLDS/$PX4_GZ_WORLD.sdf" >/tmp/gz.log 2>&1 &
for _ in $(seq 1 60); do
  gz service -l 2>/dev/null | grep -q "/world/$PX4_GZ_WORLD/create" && break
  sleep 2
done

# Single uXRCE-DDS Agent bridges all instances to ROS2.
MicroXRCEAgent udp4 -p 8888 >/tmp/xrce.log 2>&1 &
sleep 2

# PX4's sitl launcher creates its working dir NON-recursively and dies when
# rootfs is absent (observed: "Error creating directory .../rootfs/0").
mkdir -p build/px4_sitl_default/rootfs

# Boot PX4 with FACTORY state. The rootfs persists parameters.bson/dataman/
# eeprom across container rebuilds (host bind-mount), and PX4 auto-saves its
# learned EKF2_MAG_DECL at disarm — a declination learned in one world
# (baylands CA ≈ +12.8°) poisons boots in another (dynamic/city Zurich ≈
# +2.5°): mag innovations flap across the arming gate and arming is denied
# (root-caused 2026-07-21; wiped state + hold→arm→takeoff restored flight).
for i in $(seq 0 $((N-1))); do
  rm -f "build/px4_sitl_default/rootfs/$i/parameters.bson" \
        "build/px4_sitl_default/rootfs/$i/parameters_backup.bson" \
        "build/px4_sitl_default/rootfs/$i/dataman"
  rm -rf "build/px4_sitl_default/rootfs/$i/eeprom"
done

for i in $(seq 0 $((N-1))); do
  y=$((i * 3))
  # SIM_GZ_EN_LIDAR=0: PX4's gz_bridge must NOT ingest the composite's forward
  # ToF beam as a distance_sensor — a horizontal beam feeding the EKF
  # destabilizes it (observed: ~700m physical drift + arming denial, fable-MAJOR-3).
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL="${MODEL}" \
    SIM_GZ_EN_LIDAR=0 \
    PX4_GZ_MODEL_POSE="0,${y},0.5" PX4_UXRCE_DDS_NS="px4_${i}" \
    ./build/px4_sitl_default/bin/px4 -i "${i}" -d >"/tmp/px4_${i}.log" 2>&1 &
  sleep 3                                  # stagger model spawns into the live gz
done

# one mavsdk_server per drone
for i in $(seq 0 $((N-1))); do
  mavsdk_server -p $((50051 + i)) "udpin://0.0.0.0:$((14540 + i))" >"/tmp/mav_${i}.log" 2>&1 &
done

echo "swarm bring-up launched (N=$N, world=$PX4_GZ_WORLD)"
wait
