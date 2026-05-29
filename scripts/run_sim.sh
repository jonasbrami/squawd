#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic with the depth-camera airframe.
# Usage: PX4_DIR=/path/to/PX4-Autopilot ./scripts/run_sim.sh
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
MODEL="${PX4_SIM_MODEL:-gz_x500_depth}"

if [[ ! -d "$PX4_DIR" ]]; then
  echo "PX4-Autopilot not found at $PX4_DIR. Set PX4_DIR." >&2
  exit 1
fi

echo "Launching PX4 SITL + Gazebo ($MODEL) from $PX4_DIR ..."
cd "$PX4_DIR"
make px4_sitl "$MODEL"
