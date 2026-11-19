#!/usr/bin/env bash
set -e
WS='/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/ROS2_ws_Main_Body'
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
PREFIX='/home/liuhao/auv_perception_runtime/install/auv_perception'
export AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH"
export PYTHONPATH="/home/liuhao/auv_perception_runtime/build/auv_perception:$WS/src/auv_perception:$PYTHONPATH"
exec ros2 launch auv_perception perception.launch.py "$@"
