# Shared visual acquisition state machine — 2026-11-11

No new terminal is required. Leave Isaac Sim playing and ZED acquisition running. Stop the
old autonomous launch with Ctrl+C, wait for the prompt, then run the same start_autonomy.sh.
Installation does not resume motion automatically. The old live mission was put into HOLD.

## States

- SEARCH: neither a fresh 2D image target nor accepted fresh 3D pose; hold and rotate.
- ALIGN_2D: image target visible, no reliable 3D pose; hold translation and align horizontal bearing.
- APPROACH_2D: at least 2 consistent frames and yaw error <=6 degrees; feed a bounded-speed
  PID reference toward a virtual target 10m ahead along the image target's world bearing.
- WAIT_2D: target lost or stale; cancel advancing reference immediately and hold. Search resumes
  after the 1.5s loss grace if target is not recovered.
- WAIT_3D: image target covers >=65% width or >=75% height, or bearing is near vertical; hold
  while waiting for valid geometry. Do not continue blindly through a large image target.
- CONFIRM: fresh accepted 3D estimate seen; hold and require the existing distinct stable samples.
- EXECUTE: freeze 3D template-relative route and execute original waypoints. Temporary loss of
  the image does not take the robot out of this locked route.
- FAILED: approach time/travel or original acquisition/waypoint deadlines exhausted. Explicit
  start is required if no suspended waypoint route exists.

The 2D controller is not metric object localization or obstacle avoidance. A 10m lookahead is
only a reference, not estimated distance to the gate. Max approach travel defaults to 12m,
accumulated APPROACH_2D time to 600s, speed to 0.18m/s. These limits remain tunable in config/autonomy.yaml.

## Detection interface and coordinates

`/auv/perception/detections_2d` is small JSON: simulation capture stamp, optical frame, original
image width/height, CameraInfo.P, and each class/confidence/bbox/mask fraction/clipped flag.
It is produced directly after YOLO, before cloud decoding and fitting. Empty detections are
published as well. No matched cloud is required (wait at most 0.15s for one); missing/invalid
depth cannot prevent 2D observations. No 3D world pose is invented from the masks.

Mission Control selects targets using task.visual_target in config/tasks/task1_gate.yaml:

1. Trusted whole Gate instance wins over component instances, even with higher component scores.
   The bbox center is used rather than area-weighted mask centroid.
2. Without whole Gate: strongest instance per semantic component; prefer horizontal center from
   middle rod or beam, vertical center from visible component bounds. This is approximate.
3. Component-only movement uses 40% of normal approach speed and holds depth by default,
   because a lone beam/flag does not determine the full gate's vertical center.

Camera intrinsics plus verified camera_to_body rotation convert the selected pixel to a bearing.
The robot pose nearest the image CAPTURE timestamp (<=0.12s) rotates that bearing into world
coordinates. AUV axes remain left/up/forward; optical axes remain right/down/forward. This is
ray projection for heading guidance, not depth backprojection. Matching cloud projection and
original 3D fitting are retained. Both simulation timestamp and wall-clock observation freshness
are checked, and large bearing jumps reset the two-frame confirmation.

RGB image publishing remains 480x270 JPEG45, max1Hz. Internal inference uses original imagery.
No full masks/clouds are transmitted for the mission-control target message.

## Reuse for other tasks

VisualApproach is independent of Gate geometry and ROS. Change whole_labels/part_labels and
optional center_labels/horizontal_labels for another task. Register its 3D geometry planner in
waypoint_mission.PLANNERS and provide its perception model/template/waypoints. The same
SEARCH/ALIGN_2D/APPROACH_2D/CONFIRM/EXECUTE flow is reused, including immediate 2D acquisition
if a target is already in view. This update implements Task1, not an untrained Task2 detector
or an automatic multi-task competition sequencer.

## Tests

40 regression tests passed before the additional CUDA check (2 opt-in tests skipped).
The real YOLO weights then passed RGB-only integration in isolated ROS domain64: Gate detections
and 480x270 compressed overlays were produced with NO PointCloud2 or Odometry input.
ROS domain62 tests also exercise live JSON -> ALIGN/APPROACH, empty detections -> hold,
3D takeover, and all 11 original waypoints. Real PID launch integration stays in domain63.
No autonomous movement was started in the user scene during this deployment.
Live scene verification received the new 2D JSON including MiddleRedPillar, LeftPillar,
FlagRepair and HorizontalBar; still not a completed live autonomous gate lap.


## Follow-up: premature holds fixed

Live failure was FAILED / 2D approach travel/time limit reached with travel only 2.142m:
the former 180s elapsed timer (including aligning/waiting) had expired. Measured live 2D
packet gaps included 1.301s and 1.700s, exceeding the old 0.8s freshness setting and causing
frequent stops/reconfirmations. Follow-up configuration: 10m virtual lookahead, 2s freshness,
600 accumulated seconds in APPROACH_2D only. Search deadline now counts only consecutive
SEARCH time and cannot abort an ongoing 2D approach. Distance cap remains 12m and near-image
size stops remain enabled. Reference speed and 0.6m position lead limits are unchanged.
Lost-target grace starts when loss is detected; no advancing goal is sent during grace.
Status distinguishes distance limit / forward-command time limit / oversized image target /
insufficient consistent frames, with elapsed approach time, traveled distance and image age.
Final suite after this fix: 44 passed, 1 skipped (14.66s), including true RGB-only YOLO.
Restart old autonomy launch to load code/config; deployment does not silently resume motion.


## 2026-11-12: individual component acquisition and direct PID goal

Task1 first chooses the highest-confidence valid Gate bbox. If none exists,
select exactly ONE highest-confidence detection across LeftPillar, RightPillar,
FlagSOS and FlagRepair (including multiple instances); use that bbox center.
MiddleRedPillar/HorizontalBar are not acquisition anchors in this Task1 profile.
min_confidence=0.35 matches inference filtering; min_bbox_fraction=0.00003 permits
small remote flags. Two consistent frames and yaw alignment remain required.

APPROACH_2D now publishes the full current-position + 10m world bearing target,
bypassing the former 0.6m lead and linear-reference ramp, just like EXECUTE sends
full frozen waypoints. Therefore approach_speed_mps/part_speed_factor no longer
limit the published APPROACH_2D goal; actual motion follows the unchanged PID and
its actuator limits. Single-part approach holds depth by default. Lost/stale
images, near-image limits, ownership guard and motion/distance budgets remain.
3D acquisition preempts 2D and retains the original 11-point locked route.

Live diagnosis: SDK image timestamps were about 1.2s AHEAD of bridge state time,
so max_future_s=0.05 rejected detections before target selection. New GUI-only
sensor_time_alignment estimates a fixed capture-clock offset from eight fresh
RGB arrivals and bridge clocks BEFORE inference. It preserves same-stamp RGB/XYZ
association, raw SDK timestamps in detections, and relative capture intervals;
it does not re-stamp old detections at inference publication time. This is an
estimated arrival-based clock-domain correction, NOT calibrated hardware time
synchronization; residual error includes transport/callback jitter. Non-GUI
profiles default to unchanged SDK time. No-clock RGB-only inference still works.
A backwards /clock reset recalibrates; future sensor-clock jumps stop alignment
until another stable window is available. Original auv_sensing is unchanged.

Validation: isolated domain62 suite 48 passed, 1 skipped, including each of the
four individual labels, highest-confidence selection, actual ROS-published 10m
goal with FlagRepair only/no cloud, 3D takeover + all 11 waypoints, clock adapter,
real YOLO RGB-only JPEG, and existing PID regression. Live read-only validation:
offset -1.383370509s; post-correction image age 0.05-0.20s. Current fixed camera
view emitted empty detections, so no live Gate lock or autonomous lap is claimed.
Robot was explicitly stopped into HOLD before deployment. Perception alone was
reloaded (PID41726, runs/perception/component_approach_clock.log); SDK receiver,
PID, Isaac, scene publisher and Foxglove kept running. Restart the user's old
autonomy launch to load the new mission code; no automatic motion was resumed.
Backup: runs/mission_autonomy/backups/component_approach_20261112_140854/sources_before.tgz.
