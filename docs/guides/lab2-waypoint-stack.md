# Lab 2 waypoint stack: what runs and why

Lab 2 (from 2026-10-13): an AutoSDV car drives a closed waypoint loop in an
indoor hall, localized by ArUco markers on the walls fused with odometry in
Autoware's EKF. Students write the `PathTracker` (steering and speed from the
localized pose and the path) and the `SpeedPID` (the actuator's inner loop).

| Recipe | Launch file | Where |
| --- | --- | --- |
| `just lab2 vehicle` | `waypoint_follow.launch.xml` | the car |
| `just lab2 bench` | `waypoint_follow_bench.launch.xml` | a workstation: the car's stack, a plant model for the hardware |
| `just lab2 bench-checks` | (starts benches) | the safety behaviour, PASS/FAIL |
| `just lab2 preflight` | (reads the running stack) | before engaging: nodes, rates, marker fixes, start box, TF, availability, MRM, graph |

The design and the decisions behind it are in the course repository,
`2026/labs/Lab2/lab2-system-design.md`. The coach-board lab this replaced is
`just coach` ([lab2-stack.md](lab2-stack.md)); its system, API and control
components are shared with this one.

## The chain

```
ZED X Mini rgb/color/rect/image ─► ar_tag_based_localizer (in the ZED container)
     ─► /localization/pose_estimator/pose_with_covariance ─┐
velocity_status ─► vehicle_velocity_converter ─┐           │
ZED IMU ─► imu_corrector ─► gyro_odometer ◄─────┘           │
     ─► /localization/twist_estimator/twist_with_covariance ┴► ekf_localizer
     ─► /localization/kinematic_state, TF map→base_link ─► twist2accel
start_pose_init ─► the EKF's initial pose (the course's start box)

course path.csv ─► waypoint_path_publisher ─► /planning/trajectory (+ ~/progress)
     ─► path_tracker [student PathTracker] ─► /control/trajectory_follower/control_cmd
     ─► vehicle_cmd_gate ─► actuator [safety shell + student SpeedPID] ─► PCA9685
localization_monitor ─► /diagnostics ─► autosdv-waypoint-main.yaml ─► MRM
```

Everything from Autoware is the installed 1.5.0, unmodified, except the
AR-tag localizer: that is AutoSDV's fork
(`src/localization/external/autoware_ar_tag_based_localizer`, branch
`autosdv-1.5.0`), which adds one parameter, `dictionary`, and overlays the
installed package. The lab's nodes are in `src/lab/autosdv_waypoint_lab`.

## What is new, and why each piece is there

**`start_pose_init`.** The stock AR-tag localizer publishes nothing until the
EKF has a pose to gate against (it reads the EKF's biased pose and drops any
fix more than `ekf_position_tolerance` from it), and the EKF has no pose until
something initializes it. There is no NDT to do that here, so the car starts
in a taped box whose pose is in `course.yaml`, and this node sends that pose
to the EKF once, then triggers it. `just lab2 reinit` repeats it.

**`consider_orientation: true`.** With the stock default (`false`) the
localizer corrects position only, and the heading drifts with the gyro for
the whole run.

**`ekf_position_tolerance: 1.0`.** Also rejects a far marker whose orientation
was misjudged: one rendered at 4.4 m and half resolution put the car 1.1 m
and 20° off, and the EKF's own Mahalanobis gate would have accepted it.

**The localizer runs in the ZED's container.** A 1920×1200 RGB frame is
6.9 MB; at 15 Hz through UDP to another process, a best-effort subscriber
(the localizer's) loses fragments under load. In one process the DDS delivery
is local. It is not `use_intra_process_comms`: the localizer's map subscription
is transient-local, which Humble's intra-process refuses.

**The base covariance is constant in practice.** The upstream node scales it
by `max(1, (d/5)^3)` where `d` is the fix's distance from the EKF pose, not
the marker's range, whatever its comment says; under a 1 m gate that is 1.

**`path_tracker` in place of `trajectory_follower`.** It runs the student law
through `TrackerShell`: the steering limit, the path's own speed as a ceiling,
speed 0 on an exception or a non-finite output, and the law disabled after 3
faults in a row. It publishes on trajectory_follower's topic, so the gate and
the transition manager are unchanged, and `path_tracker: tracker_state` in
place of trajectory_follower's `control_state` diagnostic
(`control-waypoint.yaml`).

**`localization_monitor`.** Two statuses the graph reads:
`marker_fix` (ERROR before the first fix and after 5 s without one) and
`path_deviation` (ERROR beyond 0.8 m from the course). There is no WARN tier
because the graph's mode availability treats WARN as unavailable: measured on
the bench, a WARN at 2 s triggered the MRM.

**The steering ratio.** `actuator.yaml`'s −1000 saturates the servo at
0.05 rad; the course's 2 m turns need 0.16 rad. The lab loads
`config/actuator_lab2.param.yaml` (−143, which maps the servo's ±50 PWM onto
±0.349 rad) through the new `actuator_param_overrides` argument of
`vehicle_interface.launch.xml`. It is a stand-in until the ratio and its sign
are measured on each car.

**The sensor kit runs with its drivers off.** No LiDAR in this lab, and the
ZED driver is the lab's (`zed_aruco.launch.xml`: rectified image and IMU, depth
and tracking off). `camera_model:=zedxm` keeps the kit's ZED URDF frames and
the left-camera-to-IMU edge, and stops it starting its own IMU-only ZED.

## Courses

A course is a directory: `course.yaml` (path file, laps, `v_ref`, start box,
marker size, dictionary, every marker's `id, x, y, yaw` with yaw the direction
it faces), `path.csv` (`x, y[, v]`), and the generated `lanelet2_map.osm`,
`ar_tag.param.yaml` and `map_projector_info.yaml`. `ar_tag.param.yaml` is the
localizer's `dictionary`, `marker_size` and `target_tag_ids`, loaded after the
lab's `ar_tag_based_localizer.param.yaml`, so a course's markers are described
once. `just lab2 course-map DIR` regenerates both and
prints how well the markers cover the path for a 100°, 5 m camera, including
whether a car anywhere in the start box (±0.3 m, ±10°) sees a marker.

The shipped `hall_demo` is a 10 m × 4 m oval with eleven markers: one in view
on 89 % of the loop, the longest gap 0.8 m, and the start box always covered.
It is a stand-in until a real hall is surveyed.

`hall_demo`'s markers are ARUCO_MIP_36h12, decoded under the default
`dictionary: ALL_DICTS`; AprilTag 36h11 rendered the same way is not detected.

The detector is the aruco library's, not OpenCV's, and it ships none of
OpenCV's `DICT_*` families. Those are decoded from a file exported by
`config/dictionaries/export_opencv_dictionary.py`. `DICT_5X5_1000` is shipped,
for the LCTK calibration boards: the solid 600 mm board is one 480 mm marker,
ID 24, so a course using it sets

```yaml
marker_size: 0.48
dictionary: DICT_5X5_1000
markers:
  - {id: 24, x: ..., y: ..., yaw: ..., z: ...}   # z: the marker centre's height
```

Checked through the real node with `synthetic_camera/run.sh` on such a course:
with the EKF pose 0.3 m off, the fixes land 2 cm and 0.3° from the true pose.
A 480 mm marker is about twice the 250 mm sheet, so `distance_threshold` (5 m)
could be raised for it. That has not been measured.

## The bench

`vehicle_bench_plant` (from the coach lab) starts in the course's start box
and integrates the PWM the dry-run actuator writes. `virtual_marker_localizer`
stands in for the ZED and the AR-tag localizer: from the plant's true pose and
the same OSM, it publishes the fix the stock node would publish. The model
follows the stock node rather than an ideal detector:
- a marker counts as seen only if it is whole in the field of view,
  within range and within 60° of head-on (checked against rendered images);
- detection noise grows with range, and the orientation error moves the fix
  sideways by range × angle;
- the EKF gate and the covariance rule are the stock node's.

`scripts/testing/lab2/synthetic_camera/run.sh` covers the image path the bench
skips. It runs the real localizer against rendered images of a course, which
tests the map's corner order, the marker axes, the dictionary and the camera
frame end to end.

## Measured on the bench (2026-10-13, hall_demo, reference laws)

| Run | Laps | Cross-track RMSE / max | Localization error RMSE / max | Speed RMSE | MRM |
| --- | --- | --- | --- | --- | --- |
| default noise | 3/3 | 0.02 / 0.06 m | 0.03 / 0.10 m | 0.12 m/s | 0 |
| 4° yaw noise, 40 % dropout, 0.01 rad/s gyro bias | 3/3 | 0.04 / 0.12 m | 0.08 / 0.17 m | 0.12 m/s | 0 |

The car stops within 5 cm of the start. `just lab2 bench-checks`: 5/5:
- follow;
- marker loss: MRM 5.2 s after the last fix, stopped 0.3 s later;
- path publisher loss: MRM 1.0 s;
- a raising tracker: autonomous unavailable, engage refused;
- no markers: autonomous unavailable, engage refused.

## Not checked without a car

- The ZED driver with `zed_aruco.yaml`.
- The localizer's CPU use on the Orin at full resolution.
- Real detection range and noise in the hall's lighting, which set
  `distance_threshold` and the bench's noise model.
- The steering ratio and sign.
- The IMU's mounting in TF: `gyro_odometer` uses the TF rotation, where the
  coach lab's dead reckoning had a manual sign.
