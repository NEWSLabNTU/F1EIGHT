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
| `just lab2 gate` | (starts a bench) | a team's SpeedPID drives `lab2_uturn`, PASS/FAIL before car time |
| `just lab2 make-uturn R [--leg L]` | (no ROS) | regenerate the graded course from its radius and leg length |

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
it faces, and optionally `z`, `pitch` and `roll` for a board that does not
stand upright), `path.csv` (`x, y[, v]`), and the generated `lanelet2_map.osm`,
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
for the lab's boards: a 500 mm plate with a 380 mm black square. (The LCTK
calibration boards are 600 mm with a 480 mm square, ID 24; a course using those
sets `marker_size: 0.48`.) A Lab 2 course sets

```yaml
marker_size: 0.38     # the black square, not the plate: measure it
dictionary: DICT_5X5_1000
marker_height: 0.25   # the square's centre above the floor (plate 0.50 m, standing)
markers:
  - {id: 24, x: ..., y: ..., yaw: ..., z: ...}   # z overrides marker_height per board
```

Checked through the real node with `synthetic_camera/run.sh` on a 480 mm
course: with the EKF pose 0.3 m off, the fixes land 2 cm and 0.3° from the true
pose. The same synthetic check with a 380 mm marker gave about 9 cm and 1.6°,
so a smaller marker is a noisier one. The bench's noise model
(`marker_sim.NoiseModel`: 2 cm + 1 cm per metre of range, 1.5° yaw) is not
scaled by marker size, and the 9 cm / 1.6° figure comes from one synthetic
camera run, so it is not fitted in. To see what it would cost, run the bench
with `pose_noise_std` 0.09 and yaw noise 1.6° (below). `distance_threshold`
(5 m) could be raised for a 480 mm marker; that has not been measured.

## Surveying a hall with the car (`just lab2 survey`)

Boards that cannot stay on the walls are placed anew each visit, so the course
is surveyed each visit too, with the car's own ZED:
`just lab2 survey COURSE_DIR ORIGIN_ID`. Drive slowly past every board; the
survey discovers the markers live, bundle-adjusts the recording (linked by the
ZED's visual-inertial tracking) and writes `course.yaml` and the map in the
frame of the origin marker. Write `path.csv` in that frame. Details and
accuracy: `src/lab/autosdv_marker_survey/README.md` (simulated: 1–3 cm median,
under 5 cm worst).

## The graded route (`lab2_uturn`)

One graded bag, one course: straight 10 m, a **right U-turn of radius 1.5 m**
(legs 3 m apart, centre to centre), straight 10 m back in the other lane, one
run at 0.8 m/s. The path publisher's end ramp stops the car on the last
waypoint, the stop line, level with the start box. The template is already in
the frame the survey writes (origin on the floor below the origin board's
marker, x the way it faces), so the TA lays the boards out as `course.yaml`'s
header says, surveys a copy of the directory with that board as the origin,
and keeps its `path.csv` and `start_pose`.

| | course frame (m) |
|---|---|
| origin board (id 24) | (0, 0), facing +x, 1 m beyond the turn's apex on the out leg's line |
| start box (rear axle) | (12.5, 0), car facing −x |
| turn centre / apex | (2.5, 1.5) / (1.0, 1.5) |
| stop line (rear axle) | (12.5, 3.0), car facing +x |
| path | 24.71 m; the turn is s = 10.00 to 14.71 m |

Eight floor-standing 500 mm boards (`DICT_5X5_1000`, black square 0.38 m,
`marker_height: 0.25`; the ids are placeholders the survey replaces). The
origin board cannot stand at the suggested (0, 0) of a route starting at
x = 11.5: that point is 0.6 m from the arc, so the route is shifted 1 m along x.

**Everything in the directory is generated** from the radius and the leg length:

```bash
just lab2 make-uturn 1.5                    # dry run: summary, a paste block for the spec, a diff
just lab2 make-uturn 2.0 --leg 8 --write    # a new geometry: path, start/stop, boards, map, header
python3 scripts/testing/lab2/make_uturn.py --radius 1.5 --leg 10 --write   # the same, no just
```

The Lab 2 spec (course repository, `2026/labs/Lab2/spec/`) draws its map and
prints the route's numbers from `uturn_geometry.tex`, which the generator writes
with `--tex`. After a new geometry, regenerate that file too and rebuild the PDF:

```bash
just lab2 make-uturn 2.0 --leg 8 --write --tex ~/repos/ntu-sdv-course/2026/labs/Lab2/spec/uturn_geometry.tex
make -C ~/repos/ntu-sdv-course/2026/labs/Lab2/spec
```

It writes `path.csv`, `course.yaml` (start pose, boards, `uturn: {radius, leg}`
and a header whose numbers it computes), `lanelet2_map.osm`,
`ar_tag.param.yaml` and `map_projector_info.yaml`. The boards come from a
coverage search with the bench's detector model (`marker_sim.visible`: 100°
hfov, 5 m = `ar_tag` `distance_threshold`, 60° incidence, 0.38 m marker): boards
on a 0.5 m grid facing every 15°, at least 1.4 m from the car's track (1 m clear
of the car), the fewest that give

1. every start-box placement (±0.3 m, ±10°) sees ≥ 2 boards,
2. no blind stretch over 3 m (localization_monitor's `error_age` is 5 s),
3. a board in view through the whole U-turn,
4. a car at rest near the stop line (±0.3 m, ±10°) sees ≥ 1, so the stop is
   measured on a fix rather than on dead reckoning.

For R 1.5 m that is 8 (no layout of 7 meets all four; without goal 4 seven
would do): start box ≥ 2, the stop ≥ 1, a board in view on 85 % of the path,
the turn on 100 %, the longest blind stretch 1.3 m (1.6 s). It refuses to
write a layout that misses a goal (`--force` overrides). The grader reads the
turn's extent from `path.csv`, so a new radius or leg needs no code change;
`test_course.py` checks the shipped files are exactly what the generator
writes, and runs it at R 1.0 m / 6 m legs and R 2.5 m / 14 m legs.

```bash
just lab2 vehicle route:=lab2_uturn                         # the template as shipped
just lab2 vehicle course_dir:=$PWD/courses/uturn_today      # a surveyed copy
```

The kinematic bench (reference tracker, default lookahead, 5 cm / 2° pose
noise) completes it with cross-track under 8 cm and the car stopping 6 cm from
the stop line; with the 0.38 m boards' measured 9 cm / 1.6°, 8 cm and 8 cm.
The R 1.5 m turn needs 0.21 rad of tire angle: it is meaningless until the
car's `tire_angle_to_steer_ratio` is measured (`lab2-field-calibration.md`,
car gate; `known-config-defects.md` §8).

Grading (`route_grade.py`, `lab2-submission-bags.md`): speed RMSE on the legs
(15) and in the turn (10), overshoot (5), stop error (15), each against the
car's reference bag; a bag off the path by more than 0.3 m, unfinished, with an
MRM or a disengage is invalid and scores 0.

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
- `just lab2 loc-check` on the car, and its RViz view (checked headless: the
  file parses and every topic it names is published).
- The localizer's CPU use on the Orin at full resolution.
- Real detection range and noise in the hall's lighting, which set
  `distance_threshold` and the bench's noise model.
- The steering ratio and sign.
- The IMU's mounting in TF: `gyro_odometer` uses the TF rotation, where the
  coach lab's dead reckoning had a manual sign.
