#!/usr/bin/env bash
# Brings up: micro-XRCE-DDS Agent + PX4 SITL (gz_x500) + standalone mavsdk_server.
# Assumes PX4-Autopilot is cloned/built at /workspace/PX4-Autopilot.
# NB: no `-u` — ROS2 setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

# 1. uXRCE-DDS Agent (PX4 -> ROS2), UDP 8888
MicroXRCEAgent udp4 -p 8888 &
sleep 2

# 2. PX4 SITL with Gazebo Harmonic x500 (headless server)
pushd /workspace/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 &
popd
sleep 25   # PX4 + Gazebo cold start

# 3. standalone mavsdk_server bound to PX4's UDP offboard port
mavsdk_server -p 50051 udpin://0.0.0.0:14540 &

echo "Bring-up launched. ROS2 topics:"
ros2 topic list | grep /fmu/ || echo "WARN: no /fmu topics yet"
wait
