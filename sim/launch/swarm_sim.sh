#!/usr/bin/env bash
# Multi-vehicle PX4 SITL: N drones in one Gazebo world.
# Each instance i: spawns at (0, i*3), namespaces ROS2 topics as /px4_<i>/fmu/*,
# and gets MAVLink offboard on udp 14540+i -> mavsdk_server on 50051+i.
# `-d` runs px4 detached (no interactive pxh> console -> no log spam).
# NB: no `-u` — ROS setup.bash references unbound vars.
set -eo pipefail

# Force software GL (llvmpipe) so Gazebo renders camera sensors headless (no GPU).
export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

# ros_gz vendor packages prepend a vendored `gz` (no `sim` subcommand) on PATH +
# GZ_CONFIG_PATH, which breaks PX4's `gz sim`. Force the system Gazebo Harmonic.
# (The ros_gz image_bridge uses the gz-transport lib directly, so it's unaffected.)
export PATH=/usr/bin:$PATH
export GZ_CONFIG_PATH=/usr/share/gz

cd /workspace/PX4-Autopilot

N=${SWARM_N:-3}
MODEL=${PX4_MODEL:-gz_x500_depth}

# Patch the OakD-Lite camera (idempotent) so each drone gets a UNIQUE, low-res
# camera topic: drop the shared `<topic>camera</topic>` override (-> gz scopes the
# topic per model: /world/default/model/x500_depth_<i>/.../IMX214/image) and cut
# it to 640x360@10Hz so N feeds render on software GL.
OAKD="Tools/simulation/gz/models/OakD-Lite/model.sdf"
if [ -f "$OAKD" ] && grep -q "<topic>camera</topic>" "$OAKD"; then
  sed -i \
    -e "s|<width>1920</width>|<width>640</width>|" \
    -e "s|<height>1080</height>|<height>360</height>|" \
    -e "s|<update_rate>30</update_rate>|<update_rate>10</update_rate>|g" \
    -e "/<topic>camera<\/topic>/d" \
    "$OAKD"
fi

# Single uXRCE-DDS Agent bridges all instances to ROS2.
MicroXRCEAgent udp4 -p 8888 >/tmp/xrce.log 2>&1 &
sleep 2

for i in $(seq 0 $((N-1))); do
  y=$((i * 3))
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL="${MODEL}" \
    PX4_GZ_MODEL_POSE="0,${y},0.5" PX4_UXRCE_DDS_NS="px4_${i}" \
    ./build/px4_sitl_default/bin/px4 -i "${i}" -d >"/tmp/px4_${i}.log" 2>&1 &
  # first instance starts the gz server; give it time before others join
  if [ "$i" -eq 0 ]; then sleep 18; else sleep 6; fi
done

# one mavsdk_server per drone
for i in $(seq 0 $((N-1))); do
  mavsdk_server -p $((50051 + i)) "udpin://0.0.0.0:$((14540 + i))" >"/tmp/mav_${i}.log" 2>&1 &
done

echo "swarm bring-up launched (N=$N)"
wait
