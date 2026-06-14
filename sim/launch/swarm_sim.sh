#!/usr/bin/env bash
# Multi-vehicle PX4 SITL: N drones in one Gazebo world.
# Each instance i: spawns at (0, i*3), namespaces ROS2 topics as /px4_<i>/fmu/*,
# and gets MAVLink offboard on udp 14540+i -> mavsdk_server on 50051+i.
# `-d` runs px4 detached (no interactive pxh> console -> no log spam).
# NB: no `-u` — ROS setup.bash references unbound vars.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash
cd /workspace/PX4-Autopilot

N=${SWARM_N:-3}

# Single uXRCE-DDS Agent bridges all instances to ROS2.
MicroXRCEAgent udp4 -p 8888 >/tmp/xrce.log 2>&1 &
sleep 2

for i in $(seq 0 $((N-1))); do
  y=$((i * 3))
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
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
