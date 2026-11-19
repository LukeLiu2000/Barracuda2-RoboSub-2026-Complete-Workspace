# Task-relative PID autonomy

Task 1 planner lives in `auv_mission_control`, while geometric perception remains in `auv_perception`.
No changes to keyboard/hold goal managers, PID gains/control law, thruster allocation, USDs or sensors.
The launch command is an explicit switch to autonomy, not an additional keyboard goal source.

## Start / stop

Keep Isaac Sim playing and its existing `isaac/bootstrap.py` bridge running. Keep perception running.
Stop the keyboard terminal with Ctrl+C (also stop web_keyboard / hold / experiments if active).
Leave an existing PID running: launch checks for it and reuses it. If none exists, launch starts one.

From the workspace:

```bash
bash src/auv_mission_control/scripts/start_autonomy.sh
```

Optional: inspect first without motion, then start explicitly:

```bash
bash src/auv_mission_control/scripts/start_autonomy.sh autostart:=false
source /opt/ros/jazzy/setup.bash
ros2 service call /auv_autonomy/start std_srvs/srv/Trigger '{}'
```

Stop task and HOLD current pose (not emergency stop):

```bash
ros2 service call /auv_autonomy/stop std_srvs/srv/Trigger '{}'
```

Resume same locked waypoint with `/auv_autonomy/resume`. Use `/auv_autonomy/start` to discard
the old route, search and acquire a new gate. External `/auv/control/estop` remains the existing
zero-thrust emergency-stop mechanism. Ctrl+C attempts to capture current pose and reset PID's
latched distant goal; a forcibly killed process cannot perform cleanup. Mission failure/stale
state requires explicit start/resume; it never skips a blocked point automatically.

For keyboard handover: stop the autonomy terminal (Ctrl+C), then start the existing keyboard node.
Do not keep two goal publishers active. The autonomous node detects conflicts and ceases goal
publication; it is not a general priority mux for legacy publishers.

## Edit thresholds and path

`config/autonomy.yaml`:

- `arrival.position_tolerance_m`: independent XYZ distance threshold.
- `arrival.orientation_tolerance_deg`: full SO(3) angle, including +/-180 wrap and quaternion signs.
- `arrival.dwell_s`, linear/angular speed limits: stable settle-point completion.
- `arrival.exit_ratio`: hysteresis; strict thresholds still required at completion.
- `reference`: bounded-speed reference for search/2D guidance only. In EXECUTE the full fixed waypoint pose is sent directly to PID, unchanged until arrival advances the sequence.
- `acquisition`: fresh accepted map samples, distinct timestamps, stable duration and pose dispersion.
- `progress`: waypoint deadline and lack-of-progress deadline. Timers use simulation time.

`config/tasks/task1_gate.yaml`:

Each waypoint has `name`, `position_m: [X,Y,Z]`, `rpy_deg: [roll,pitch,yaw]`, `mode`.
Angles are in degrees about fixed task axes (Rz Ry Rx). Optional `arrival` dictionary overrides
global thresholds per waypoint. Restart mission to reload YAML; running targets do not jump.

Task frame origin is beam center, X through gate from selected entry side, Y left, Z up.
The initial robot position chooses which normal is forward (`entry_side: auto`); X and Y rotate
together for opposite-side approaches. This side is LOCKED for the complete mission.
The vision model uses x=lateral/right-to-left, y=down, z=normal, not these task axes.
Body axes remain x=left, y=up, z=forward. Pose conversion accounts for this explicitly.

Semantic `lateral_anchor: sos|repair` adds the template sign-center lateral coordinate;
`height_anchor: middle_bottom` adds middle red rod bottom elevation (currently -0.63785m).
Numeric offsets are then editable in meters without duplicating vision geometry. Remove the anchor
to specify absolute task-frame coordinates yourself. Geometry comes from
`package://auv_perception/config/templates/task1_gate.yaml`.

Configured route (all at middle-red-rod-bottom elevation):

1. Center front 3m, face through.
2. SOS center front 2m, align and settle.
3. SOS gate plane, pass (no low-speed/dwell requirement).
4. SOS rear 1m, settle.
5. Gate center rear 3m, keep original heading and settle.
6. Same position, turn 180 degrees and settle.
7. Repair center rear 2m, align facing back through gate.
8. Repair gate plane, pass in reverse direction.
9. Repair front 1m, settle.
10. Gate center front 3m, keep reverse heading and settle.
11. Same position, turn 180 degrees and settle. Repeat from 1.

`repeat: true`, `max_laps: 0` means indefinite repetition. Set `max_laps: 1` for one complete cycle.
Pass mode accepts entry into the configured position/attitude tolerance OR correct-direction plane
crossing within its corridor and with correct attitude. Teleport/discontinuous motion is not an
assumed valid navigation input. Stop/turn points require velocity and dwell limits.

When neither a 2D nor a fresh 3D target is available, search holds location/depth and ramps yaw. A valid map is not enough by itself:
at least 3 fresh, distinct accepted estimates over >=1s must be stable. The selected gate and
route remain frozen during execution and repeated laps, including while gate is out of view.
Use start to deliberately reacquire. The last map alone after a long outage is not fresh evidence.
Map clear/epoch change or sim clock rollback invalidates execution; explicit restart required.

## Extension points

`make_waypoint(name, task_to_world, position, rpy_deg, mode, **tolerances)` creates a world pose.
`Mission.set_waypoint(waypoint, now)` switches active point and resets arrival/progress state.
`Mission.step(...)` updates the sequence. These pure functions/classes have no ROS dependency.
New gate-like routes can use another task YAML. Other task geometries require another planner
(current `planner: gate_pass` is explicit), while arrival/goal ownership/execution remain reusable.

Monitoring: `/auv/mission/status` JSON and `/auv/mission/waypoints` blue arrows in `auv_world`.
These are lightweight. Foxglove whitelist must include `^/auv/mission/.*$` to see them.

## Validation limits

Numerical and isolated ROS-domain tests validate axes, relative geometry, arrival logic, search,
ordering, looping, goal ownership, stale-state hold and unchanged PID's interface. They do not
certify live collision-free passage or localization accuracy. Scene/handbook geometry, physical
clearance and perception errors still matter. This feature does not add obstacle avoidance.
The live user scene is not moved automatically during installation/testing.


## Direct waypoint goals (2026-11-11 follow-up)

EXECUTE now publishes the complete current waypoint position AND attitude directly to
/auv/control/goal. The 0.6m reference lead and reference slew are bypassed for this stage.
The same pose is republished for freshness until arrival, then the full next pose is sent.
Settle points still require configured tolerances, velocity and dwell (default 0.5s);
pass points still do not require stopping. Original PID gains/allocation are unchanged.
Search/2D approach retain their smooth reference. Relevant mission/PID suite: 37 passed.
Restart autonomy launch to load the updated node; no Isaac restart is needed.
