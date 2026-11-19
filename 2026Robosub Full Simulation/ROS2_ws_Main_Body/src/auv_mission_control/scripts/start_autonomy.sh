#!/usr/bin/env bash
set -e
MISSION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "$MISSION_DIR/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export AMENT_PREFIX_PATH="/home/liuhao/auv_mission_runtime/install/auv_mission_control:/home/liuhao/auv_perception_runtime/install/auv_perception:$AMENT_PREFIX_PATH"
export PYTHONPATH="/home/liuhao/auv_mission_runtime/build/auv_mission_control:$MISSION_DIR:$PYTHONPATH"
export PYTHONNOUSERSITE=1
exec ros2 launch auv_mission_control autonomy.launch.py config:="$MISSION_DIR/config/autonomy.yaml" "$@"
