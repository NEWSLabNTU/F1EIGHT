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
ZED X Mini rgb/color/rect/image ─► marker_localizer (marker_survey, the course's marker_map.yaml)
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

Everything from Autoware is the installed 1.5.0, unmodified. The marker
survey and the marker localizer are the `marker_survey` and
`marker_localizer` packages of the marker_survey repository, a submodule at
`src/lab/external/marker_survey` pinned to a release tag (0.3.0); their
README and `docs/integration.md` describe both nodes and every parameter.
The lab's own nodes and launch files are in `src/lab/autosdv_waypoint_lab`.

## What is new, and why each piece is there

**`start_pose_init`.** The marker localizer publishes nothing until the EKF
has a pose to gate against (`require_prior`: it reads the EKF's biased pose
and drops any fix more than 1 m or 15° from it), and the EKF has no pose
until something initializes it. There is no NDT to do that here, so the car
starts in a taped box whose pose is in `course.yaml`, and this node sends
that pose to the EKF once, then triggers it. `just lab2 reinit` repeats it.

**The marker localizer** (`launch/marker_localizer.launch.xml`,
`config/marker_localizer.param.yaml`). marker_survey's `marker_localizer` in
`3d` mode, started through its stock `localizer.launch.xml` with the lab's
topics as arguments and the parameter file as its `param_file` (read last, so
the file's values win): it fits the full 6-DoF pose to the corners of every marker in
view together, with a robust loss, and leaves out a marker that disagrees
with the others. Its output goes to the EKF's pose input in place of the
AR-tag localizer's.
- The gate (1 m, 15°) also rejects a far marker whose orientation was
  misjudged, and a fix whose own uncertainty is over 0.33 m is not
  published (a lone oblique marker at 4.3 m, for one).
- The gate widens to 3 sd of the EKF's own covariance where that is wider
  (`gate_sigmas`), so an EKF that has lost the car still takes a fix. With
  the lab's EKF that never happens to the 1 m (0.6 m from the start box,
  under 0.7 m after 30 s without a fix); the angle gate is 17° from
  `start_pose_init` to the first fix, whose roll and pitch sd is 5.7°.
- The covariance comes from the fit (a few cm), plus floors of 0.22 m and
  5.7° that keep the weights the EKF and the bench were tuned with, the
  AR-tag localizer's constant 0.05 m² and 0.01 rad².
- No range limit: a far marker helps a near one's fix. On marker_survey's
  rendered wall course, a 5 m limit cut the fixes from 391 to 295 of 426
  frames and doubled the worst error.
- It finds markers in a half-size copy of the 1920×1200 image and refines
  the corners on the full one (`detect_scale` 0.5).
- Per marker in view, `~/markers` (`diagnostic_msgs/DiagnosticArray`,
  published only while subscribed) gives its state, its reprojection
  residual and how far the image alone puts it from its surveyed pose: the
  first place to look when one board seems wrong.
  `ros2 topic echo /localization/pose_estimator/marker_localizer/markers`.

**The previous localizer, `localizer:=ar_tag`.** Autoware's
`ar_tag_based_localizer`, AutoSDV's fork
(`src/localization/external/autoware_ar_tag_based_localizer`, branch
`autosdv-1.5.0`, which adds `dictionary`), reading the course's generated
`lanelet2_map.osm` and `ar_tag.param.yaml`. Kept until the marker localizer
has run on the car. Its settings: `consider_orientation: true` (with the
stock `false` the heading drifts with the gyro for the whole run),
`ekf_position_tolerance: 1.0`, a constant covariance (the upstream node
scales it by `max(1, (d/5)^3)` with `d` the fix's distance from the EKF
pose, which under a 1 m gate is 1), and it runs in the ZED's container.

**Where the localizer runs.** A 1920×1200 RGB frame is 6.9 MB; at 15 Hz
through UDP to another process, a best-effort subscriber loses fragments
under load. The AR-tag localizer is a composable node and is loaded into the
ZED's container, where the DDS delivery is local (not
`use_intra_process_comms`: its map subscription is transient-local, which
Humble's intra-process refuses). The marker localizer is a Python node in
its own process, so its frames do cross DDS: not yet measured on the car.

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

A course is a directory:

| File | What |
| --- | --- |
| `course.yaml` | the route: path file, `closed`, laps, `v_ref`, the start box |
| `path.csv` | `x, y[, v]` waypoints |
| `marker_map.yaml` | the markers, marker_survey's map file: dictionary, `marker_size`, and per marker `id, x, y, z` and its angles (`facing_deg` the direction a wall board faces; `lean_deg`, `turn_deg` for a board that does not stand upright). Written by the survey, read as it is by the marker localizer. |
| `lanelet2_map.osm`, `ar_tag.param.yaml` | generated from `marker_map.yaml`: the map and the marker settings of the AR-tag localizer (`localizer:=ar_tag`) |
| `map_projector_info.yaml` | `projector_type: local` |

`just lab2 course-map DIR` regenerates the two generated files and prints how
well the markers cover the path for a 100°, 5 m camera, including whether a
car anywhere in the start box (±0.3 m, ±10°) sees a marker. A course
directory from before `marker_map.yaml`, with its markers in `course.yaml`,
is refused with the command that converts it once
(`ros2 run marker_survey marker_map_import course ...`); the shipped courses
were converted that way.

The shipped `hall_demo` is a 10 m × 4 m oval with eleven markers: one in view
on 89 % of the loop, the longest gap 0.8 m, and the start box always covered.
It is a stand-in until a real hall is surveyed. Its markers are
`DICT_5X5_1000` of 0.25 m (before marker_map.yaml it named no family and the
AR-tag localizer tried all of its own).

The marker localizer decodes any OpenCV ArUco or AprilTag family on OpenCV
4.5 to 4.10 (`DICT_ARUCO_MIP_36h12` needs 4.7, so not on JetPack 6). The
AR-tag localizer's detector is the aruco library's, which ships none of
OpenCV's `DICT_*` families; for it those are decoded from a file exported by
`config/dictionaries/export_opencv_dictionary.py`, and only `DICT_5X5_1000`
is shipped. The lab's boards: a 500 mm plate with a 380 mm black square,
`DICT_5X5_1000`. (The LCTK calibration boards are 600 mm with a 480 mm
square, ID 24; a course using those has `marker_size: 0.48`.) A Lab 2
course's `marker_map.yaml`:

```yaml
version: 1
frame_id: map
dictionary: DICT_5X5_1000
marker_size: 0.38     # the black square, not the plate: measure it
mount: wall
markers:              # z: the square's centre above the floor (plate 0.50 m, standing)
  - {id: 24, x: ..., y: ..., z: 0.250, facing_deg: ...}
```

Checked through the real nodes with `synthetic_camera/run.sh` on `hall_demo`
(0.25 m markers, full resolution, the EKF pose 0.3 m off): the marker
localizer's fixes land 0.4 to 4.8 cm and under 0.6° from the true pose; the
AR-tag localizer's 14 cm and 1.8° at the start box. Earlier AR-tag runs: 2 cm
and 0.3° with a 480 mm marker, about 9 cm and 1.6° with a 380 mm one. The
bench's noise model (`marker_sim.NoiseModel`: 2 cm + 1 cm per metre of range,
1.5° yaw) is not scaled by marker size; to see what a noisier fix would cost,
run the bench with `pose_noise_std` 0.09 and yaw noise 1.6° (below).

## Surveying a hall with the car (`just lab2 survey`)

Boards that cannot stay on the walls are placed anew each visit, so the course
is surveyed each visit too, with the car's own ZED:
`just lab2 survey COURSE_DIR ORIGIN_ID` (`launch/marker_survey.launch.xml`:
the ZED with positional tracking, `config/zed_survey.yaml`, and marker_survey's
survey node in `origin_marker` mode with the ZED odometry as its pose source).
Drive slowly past every board; the survey discovers the markers live,
bundle-adjusts the recording (linked by the ZED's visual-inertial tracking)
and writes `marker_map.yaml`, `survey_report.yaml` and `survey_data.npz` in
the frame of the origin marker. Then `lab2_course_from_survey` adds
`course.yaml` for a new course (the recording's start as the start box) and
regenerates the lanelet2 map. Write `path.csv` in that frame. Arguments after
the origin go to the launch file (`dictionary:=`, `marker_size:=`,
`floor_offset:=` for a car on a stand); with a display, rqt_image_view shows
`/marker_survey/debug/image`. `just lab2 survey-save` saves without stopping,
`just lab2 survey-solve [DIR] [--origin-marker ID] [--output-dir OTHER]`
solves the recording again.

Accuracy, marker_survey's rendered recordings through this launch file
(marker_survey 0.3.0, the same as 0.2.0): the wall course of 0.38 m boards,
median 0.89 cm, max 1.45 cm (0.99 cm and 1.45 cm with 3 % of the odometry
messages lost); `hall_demo`'s 0.25 m markers, median 1.21 cm, max 1.66 cm.
Not yet measured on a real hall. The report's per-marker verdicts follow the
real errors (0.1.0 put every `hall_demo` marker at `bad`, 1σ 14 to 21 cm,
for real errors of at most 2.2 cm): on these runs every marker is `good`,
1σ 0.2 to 1.1 cm. The frame's 1σ (`frame_sd_xy_cm`) also counts a marker
size off by 0.5 % (`marker_size_sd`), which stretches the whole map: up to
5.0 cm on the wall course and 6.5 cm on `hall_demo` (0.2.0: 1.2 and 3.9 cm),
far from the origin marker. The rendered markers have their exact size, so
the real errors stay at the figures above; on a real hall, measure the black
square, and compare a few markers with a tape measure before driving past
them again.

## The grading routes (`lab2_straight`, `lab2_circle_cw`, `lab2_circle_ccw`)

The three graded bags each have a course template, already in the frame the
survey writes (origin on the floor below the origin board's marker, x the way
it faces), so the TA lays the boards out as the template's header says,
surveys a copy of the directory with that board as the origin (the survey
rewrites only `marker_map.yaml` and what is generated from it), and keeps its
`path.csv` and `start_pose`:

| course | route | origin board | boards |
|---|---|---|---|
| `lab2_straight` | open 10 m, x 11.5 → 1.5 heading −x, 1 run at 0.8 m/s, end ramp stops it | the end board, (0, 0) facing +x, 1.5 m past the stop | end board + 2 angled side boards (3 boards) |
| `lab2_circle_cw` | clockwise R 1.5 m about (3.5, 0), 3 laps at 0.8 m/s | a ring board, (0, 0) facing +x | ring of 6 at R 3.5 m facing the centre |
| `lab2_circle_ccw` | counter-clockwise R 2.5 m about (3.5, 0), 3 laps | the same ring board | the same ring |

All the lab's 500 mm boards standing on the floor (`DICT_5X5_1000`, black
square 0.38 m, centre 0.25 m up); the ids are placeholders the survey replaces. The two
circles share one ring, so one recording serves both: survey into one
directory, then `just lab2 survey-solve <dir> --output-dir <other dir>`.

```bash
just lab2 vehicle route:=lab2_circle_cw                      # the template as shipped
just lab2 vehicle course_dir:=$PWD/courses/circle_cw_today   # a surveyed copy
```

`just lab2 course-map` on the templates: a marker in view on 100 % of every
route, no gap, the start box always covered. The kinematic bench (reference
tracker, default lookahead, 5 cm / 2° pose noise) completes each with cross-track
under 3 cm, a 1.5 m circle at mean radius ±3 mm, and the straight stopping
within 7 cm of its last waypoint. With the 0.38 m boards' measured 9 cm / 1.6°
instead, it still completes each: cross-track under 5 cm, mean radius within
6 mm, the straight stopping within 14 cm. Coverage is unchanged at 0.38 m
(the field-of-view test uses the marker's width, and it stays inside the 100°
camera). Course-file checks are tests in `test_course.py`.
The R 1.5 m circle needs 0.21 rad of tire angle (0.13 rad for R 2.5 m): it is meaningless until the
car's `tire_angle_to_steer_ratio` is measured (`lab2-field-calibration.md`,
car gate; `known-config-defects.md` §8).

## Checking localization by hand (`just lab2 loc-check`)

Before letting the tracker drive a freshly surveyed course, drive it by hand
and watch where the localizer puts the car:
`just lab2 loc-check COURSE_DIR`, with the car in the start box (the EKF is
seeded there) and the selector switch on the joystick. It runs the waypoint
stack's localization unchanged and nothing that drives: no path publisher, no
tracker, no engage, and the actuator as a dry run, so the transmitter has the
car. With a display, RViz opens on `rviz/loc_check.rviz`: the course path
(white), the localized trail (red), the surveyed markers, the latest marker
fixes (green arrows) and the camera's detection image.

`loc_check` logs a summary every 10 s and on exit:
- fixes per second while moving, and the longest stretch without one (seconds
  and metres): where the EKF coasts on odometry alone;
- fix vs odometry: how far each fix lands from the EKF's pose at the fix's
  stamp. A few centimetres on a good survey; one marker's fixes consistently
  far off means that marker was surveyed wrong, or knocked since;
- off the course path: the trail's distance from `path.csv`, if you drive
  along it.

On the bench (`loc_check.launch.xml` beside `just lab2 bench`, hall_demo,
the reference tracker driving): 12 fixes/s, longest gap 1.2 s / 0.9 m, fix vs
odometry median 10 cm / p95 24 cm with the bench's default detection noise
(1.4 cm / 2.9 cm with that noise set to zero), off path median 1 cm, max 6 cm.

## Running a student's submission (gate, bench, car)

One argument loads a team's `SpeedPID` and calibration into `gate`, `bench` and
`vehicle`:

```bash
just lab2 gate    submission:=~/lab2/B12345678_lab_2_1
just lab2 vehicle submission:=~/lab2/B12345678_lab_2_1 course_dir:=...
python3 scripts/testing/lab2/submission.py DIR     # shows what it passes
```

`DIR` is the `<student_id>_lab_2_1/` directory or the `lab2_waypoint/` package
itself. `scripts/testing/lab2/submission.py` puts the package on `PYTHONPATH`,
sets `speed_controller_class:=lab2_waypoint.speed_pid:SpeedPID` and builds
`speed_controller_params` from `config/speed_pid.yaml` with `ff_deadband` and
`ff_gain` from `config/calibration.yaml` (the keys `speed_pid.yaml` sets itself
win). It reads the files itself instead of importing the student's bench, so a
modified bench changes nothing. An uncalibrated file (gain 0) is a warning, not an
error; the stub then fails the gate by not moving the car. Do not
combine it with explicit `speed_controller_*:=` arguments. The shell side is `scripts/testing/lab2/submission.sh`.

## The bench

`vehicle_bench_plant` (from the coach lab) starts in the course's start box
and integrates the PWM the dry-run actuator writes. `virtual_marker_localizer`
stands in for the ZED and the marker localizer: from the plant's true pose and
the course's map, it publishes the fix the AR-tag localizer would publish,
which it was written to model; its 1 m gate and constant covariance are what
the marker localizer's gate and covariance floors reproduce (it has no angle
gate and fixes one marker at a time). The model follows a real node rather
than an ideal detector:
- a marker counts as seen only if it is whole in the field of view,
  within range and within 60° of head-on (checked against rendered images);
- detection noise grows with range, and the orientation error moves the fix
  sideways by range × angle;
- the EKF gate and the covariance rule are the stock node's.

`scripts/testing/lab2/synthetic_camera/run.sh` covers the image path the bench
skips. It runs the stack's localizer launch file (`LOCALIZER=ar_tag`: the
AR-tag localizer) against rendered images of a course, which tests the map,
the marker axes, the dictionary and the camera frame end to end.

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
- `just lab2 loc-check` on the car, and its RViz view (checked headless: the
  file parses and every topic it names is published).
- The marker localizer on the car: its CPU use on the Orin at full
  resolution (8 ms detection and 0.4 to 23 ms fit per frame on a
  workstation core), and whether the 1920×1200 frames reach it through DDS
  without loss outside the ZED's container. Until then `localizer:=ar_tag`
  runs the previous localizer.
- The survey on the car with marker_survey's node (the ZED, its tracking and
  the URDF are as before).
- Real detection range and noise in the hall's lighting, which set the
  bench's noise model.
- The steering ratio and sign.
- The IMU's mounting in TF: `gyro_odometer` uses the TF rotation, where the
  coach lab's dead reckoning had a manual sign.
