#!/usr/bin/env bash
set -e
WS='/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/ROS2_ws_Main_Body'
PKG="$WS/src/auv_perception"
if pgrep -f '[p]ython.*auv_perception/scripts/vision_scene.py' >/dev/null; then
  echo 'Perception simulation is already running; do not start a second stream.'; exit 1
fi
export PYTHONPATH=/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
export LD_LIBRARY_PATH=/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/bin
/isaacsim/python.sh "$PKG/scripts/prepare_scene.py"
unset PYTHONPATH
export LD_LIBRARY_PATH=/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
export ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec /isaacsim/python.sh -u "$PKG/scripts/vision_scene.py" --duration 7200
