# Validation record — 2026-11-11

26 tests passed on server (11.01s), with PYTHONNOUSERSITE=1 and third-party pytest plugins disabled.

- Pure planner/arrival tests: model/body/world coordinate conventions, both entry sides,
  full rotated/translated world poses, semantic SOS/Repair offsets, 11-point order and loop,
  turn-in-place, rotation wrap, speed/dwell/hysteresis, directional plane crossing,
  distinct/fresh stable gate samples, locked path, timeouts and simulation rollback.
- ROS integration in domain 62: synthetic bounded kinematic robot receives real PoseStamped
  goals, completes all 11 points, stops after one configured lap, rejects competing goal
  publishers, handles stale state and time resets. Trigger start service exercised.
- Launch integration in domain 63: starts original PID once, no autonomous goal before explicit
  start, search yaw produces PID actuator output. All isolated launch processes stopped afterward.
- Existing auv_controller_pid tests included unchanged in the same 26-test suite.

No autonomous goal was published to the live user simulation during installation/testing.
Live domain checks show old keyboard/hold/web_keyboard entry points and new autonomy_node available.
Original PID source, gains, keyboard manager and USD scene were not modified.
Original setup.py/package.xml backed up under runs/mission_autonomy/backups/20261111_124241.
Mission runtime install/build are persistent under /home/liuhao/auv_mission_runtime.

This validates software integration, not a live Isaac Sim lap, physical clearance, obstacle avoidance
or the accuracy of visual localization. Live task completion remains to be verified by the user.

The existing Foxglove bridge whitelist continues to carry the original scene/current/target markers.
Its whitelist is read-only at runtime. Optional /auv/mission/status and /auv/mission/waypoints
require restarting just the Foxglove bridge with ^/auv/mission/.*$ added to the low-bandwidth whitelist;
no bridge restart was performed solely for these optional monitoring topics.
