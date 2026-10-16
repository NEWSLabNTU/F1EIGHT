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
| `just lab2 run` | (starts the stack) | ONE graded run: stack, preflight, record, engage (asks), stop, audit |
| `just lab2 audit` | (no stack) | the latest bag: VALID/INVALID, every metric, the violations, `audit.png` |
| `just lab2 integration-test` | (starts a bench) | TA: the whole Part 2 pipeline on the bench, rendered boards, PASS/FAIL |
| `just lab2 fuzz BAG` | (no stack) | TA: mutated copies of a valid bag through check-bags, grade and audit |
| `just lab2 record` | (reads the running stack) | the submission bag, `rosbags/lab2_uturn_<next>` |
| `just lab2 check-bags` | (no stack) | would the grader accept the latest bag? |
| `just lab2 grade` | (no stack) | score the latest bag against the car's reference bag |
| `just lab2 plot` | (no stack) | the latest bag in PlotJuggler |
| `just lab2 config` / `init` | (no ROS) | show the settings and their sources / write `~/lab2/lab2.yaml` |

## Settings (`~/lab2/lab2.yaml`)

The student recipes take their car, team and directories from one file instead
of the command line, so the whole run is short:

```bash
just lab2 init car:=A team:=3     # once: writes ~/lab2/lab2.yaml, then fill in the paths
just lab2 gate                    # workstation, before car time: `submission` on the graded route
just lab2 vehicle                 # car, terminal 1: `route` with the controller in `team_dir`
just lab2 preflight               # terminal 2
just lab2 record                  # terminal 2: rosbags/lab2_uturn_1, then _2, ...
just lab2 engage                  # terminal 3; Ctrl-C the recording when the car has stopped
just lab2 check-bags              # the latest bag, with car and team
just lab2 grade                   # the latest bag against `reference`
just lab2 plot                    # the latest bag
```

| Key | Used by | Default |
| --- | --- | --- |
| `car` | `record`, `check-bags` | none (required by `record`) |
| `team` | `record`, `check-bags` (the team number) | none (required by `record`) |
| `team_dir` | `vehicle`: the team's controller and `path_tracker.param.yaml` (what `team:=DIR` passes) | none (required by `vehicle`) |
| `submission` | `gate`: the controller to gate (what `submission:=DIR` passes) | none (required by `gate`) |
| `route` | `vehicle`, and the bag name (`lab2_uturn` -> `uturn_N`) | `lab2_uturn` |
| `reference` | `grade`: the car's TA reference bag | `~/lab2/reference/uturn` |
| `bag_dir` | where `record` writes and `grade`/`check-bags`/`plot` look | `rosbags` |

Precedence, highest first: `key:=value` on the command line (and the tools' own
`--car`, `--team`, `--name`, `--reference`), the user's file (`~/lab2/lab2.yaml`,
or `config:=FILE`), then the built-in defaults in
`src/lab/autosdv_waypoint_lab/config/lab2_defaults.yaml`. A missing user file is
fine while nothing required is missing; a missing required key is one error line
naming the key and the file to add it to. `~` is expanded, and a relative path
in the user's file is relative to that file. In `vehicle` and `gate`, `team:=`
is the team directory as before; elsewhere it is the team number. Every explicit
argument of the older commands still works and wins over the file.

The latest bag is the `bag_dir/lab2_*` whose `metadata.yaml` was written last;
with no `reference` bag on disk, `grade` warns and scores against the fixed
absolute pass lines. `just lab2 config` prints every value and where it came
from; each recipe prints one dim line with the values it used. The resolution is
`scripts/testing/lab2/lab2_config.py`.

The design and the decisions behind it are in the course repository,
`2026/labs/Lab2/lab2-system-design.md`. The coach-board lab this replaced is
`just coach` ([lab2-stack.md](lab2-stack.md)); its system, API and control
components are shared with this one.

## A graded run from one terminal (`just lab2 run`, `just lab2 audit`)

On the car there is no monitor: students work over ssh, in tmux so a dropped
connection does not end the run. One command does the run, a second looks at
it:

```bash
ssh car-a
tmux new -s lab2                  # or `tmux attach -t lab2`
just lab2 run                     # settings from ~/lab2/lab2.yaml
just lab2 audit                   # the latest bag again, any time later
just lab2 audit --show            # also open audit.png: `export DISPLAY=:1` first, view over RDP
```

`just lab2 run`, in one terminal, with one status line at a time:

1. starts the stack exactly as `just lab2 vehicle` would (`route`, `team_dir`)
   in a session of its own; its output goes to
   `<bag_dir>/run_logs/lab2_uturn_N/stack.log`, not the terminal;
2. waits until autonomous mode has been available for 3 s, then runs the
   preflight; any FAIL stops the stack and ends the run (WARN lines are shown
   and allowed);
3. starts recording `<bag_dir>/lab2_uturn_N` (the next free N; car and team
   from the settings);
4. asks: *Ready. Engage only when the TA holds the transmitter. Press Enter to
   engage, or q then Enter to abort.* It refuses to engage, with the reason,
   while the localization is not usable (below) or autonomous mode is not
   available, and asks again;
5. prints `t, v, s along the route, mode, MRM` once a second until the car has
   stopped at the end of the route; an MRM, a disengage (the transmitter
   included), a localization loss, the timeout (3 x the route's driving time,
   93 s for `lab2_uturn`) or Ctrl-C end it early, and anything but a finish
   disengages at once;
6. records 3 s more (check-bags wants 3 s past the stop), disengages if still
   engaged (before the bag closes: the car is not left in autonomous mode while
   the recorder shuts down; a disengage after the finish does not count),
   stops the recording, and stops the stack: SIGINT, then SIGTERM after
   20 s, then SIGKILL, to every process group of its session (play_launch gives
   its nodes groups of their own);
7. audits the new bag and prints the verdict and the plot's path.

Ctrl-C at any stage (or the terminal going away) disengages first if engaged,
then closes the bag, then stops the stack; a second Ctrl-C during that only
prints "please wait". `team:=` here is the team NUMBER, as in `record`; the
controller directory is `team_dir:=`.

```
config ~/lab2/lab2.yaml: route lab2_uturn, team_dir ~/lab2/team3
[1/7] stack: vehicle (car), log rosbags/run_logs/lab2_uturn_4/stack.log
[2/7] waiting for the stack (autonomous available 3 s, up to 180 s)
      up after 41 s
      preflight 16 passed, 0 warnings, 0 failed (all lines: .../preflight.log)
[3/7] recording rosbags/lab2_uturn_4
[4/7] Ready. Engage only when the TA holds the transmitter. Press Enter to engage, or q then Enter to abort.
      engaged
[5/7] driving; ends at the stop, on an MRM, a disengage, a localization loss, or after 93 s
  t  12.0 s  v 0.80 m/s  s  9.6/24.7 m  AUTONOMOUS  MRM NORMAL
END   stopped at the end of the route
[6/7] stopping the recording
...
[7/7] audit
```

**Localization loss stops the car.** While engaged, `run` disengages (Autoware's
stop mode brakes the car) and prints the reason as soon as any of these holds,
without waiting for the MRM the stack raises itself:

| Condition | Threshold | From |
| --- | --- | --- |
| no accepted marker fix | `error_age` 5 s | `localization_monitor: marker_fix` ERROR |
| off the path | `error_distance` 0.8 m | `localization_monitor: path_deviation` ERROR |
| no course progress | `progress_timeout` 1 s | `localization_monitor: path_deviation` ERROR |
| the monitor silent | 2 s | `run` |
| no `/localization/kinematic_state` | 0.5 s | `run` |
| EKF position sigma `sqrt(var_x + var_y)` | 0.5 m (`--max-pose-std`) | `run`; the bench runs at 0.06-0.10 m |

The first three are `localization_monitor`'s
(`config/localization_monitor.param.yaml`) and, in the diagnostic graph, also
make autonomous mode unavailable, so an engage with no fix is refused by the
stack itself (`just lab2 bench-checks --only no_markers`); `run` additionally
refuses before asking the stack, and the preflight fails with no fixes. The
monitor has no covariance check; the 0.5 m sigma limit is `run`'s, five times
what the bench's EKF reaches and above the 0.3 m validity line, so it only
catches an EKF that has lost its pose.

**`just lab2 audit [BAG]`** (default: the latest bag) runs the bag checker and
the grade (against `reference` when it is a bag, else the fixed lines, as
`grade` does) and prints the verdict, each metric against its full-marks and
zero lines with its points, and the violations: every check-bags rejection,
every validity gate, every metric below full marks, every stretch more than
0.3 m off the path (with its time after engage), every MRM and early
disengage. It writes `BAG/audit.png` and `BAG/audit.json`:

- speed vs time since engage: measured and commanded speed, the target, the
  scored steady window shaded, a strip naming the straight and U-turn
  segments (from where the localized car was on `path.csv`'s turn interval,
  as the grade splits them), the start overshoot peak, the engage, the stop
  ramp start, MRM and disengage moments as red lines, the samples whose speed
  error is over their segment's full-marks line in red, and the stop error;
- the run on the floor: the route path (U-turn section highlighted), the
  localized trajectory, the raw marker fixes, the true pose on the bench, the
  boards with their facing arrows, the start and stop lines, and in red every
  stretch more than 0.3 m off the path;
- the distance from the path vs time, the 0.3 m line, red where exceeded, and
  grey outside the judged interval (engage to rest);
- the title: bag, car, team, VALID/INVALID with the first violation, points.

A bench bag also gets its localization measured against the plant's true pose
(fix count and rate, fix and EKF error median/p95, the longest gap without a
fix).

## Bench bags and the TA integration test (`just lab2 integration-test`)

`just lab2 run --bench` is the same flow on the vehicle bench, and
`just lab2 integration-test` runs it as a TA test of the whole Part 2
pipeline without a car: stack -> record -> engage -> finish -> bag ->
check-bags -> grade -> audit, with the reference controller by default:

```bash
just lab2 integration-test                          # the reference SpeedPID, rendered boards
just lab2 integration-test submission:=~/lab2/B12345678_lab_2_1
just lab2 integration-test team_dir:=~/lab2/team3   # Part 2: a team's tracker and lookahead
just lab2 integration-test --camera virtual         # the modelled localizer (faster)
```

It exits 0 only if all of these pass, and prints one line each:

- the run finished (stopped at the end of the route, no MRM, no disengage);
- `check-bags --bench` ACCEPTs the bag;
- the grade is valid;
- `audit.png` was written;
- the localization against the plant's true pose is within `audit.LOC_LIMITS`:
  at least 2 accepted fixes/s, fix error p95 <= 0.50 m, EKF error p95 <=
  0.20 m, no gap without a fix over 3 s. The scale is the grade's: the
  validity line is 0.3 m from the path, so a pose 0.2 m off at p95 leaves no
  margin. A single fix is noisier than the EKF that averages it: the localizer
  declares a 0.22 m sigma per axis, whose 2-D p95 is 0.54 m. The gap stays
  well inside `error_age` (`lab2_uturn`'s design gap is 1.6 s).

Bags go to `tmp/lab2_integration/bags` as car `SIM`, team `0` (override with
`bag_dir:=`, `car:=`, `team:=`); `ROS_DOMAIN_ID` defaults to 73 and 0 is refused.

**Rendered boards (`camera:=mock`).** The integration test runs the bench with
`camera:=mock`: `mock_aruco_camera` renders the course's boards (cv2.aruco, the
course's dictionary, each marker on a white plate at its surveyed pose) from
the plant's TRUE pose at 15 Hz, 960 x 600, a 403 px pinhole (the ZED X Mini's
~100 deg field of view at half resolution), and publishes them on the ZED's
image and camera_info topics. The real `ar_tag_based_localizer` then detects
them and feeds the EKF, exactly as on the car; `virtual_marker_localizer` is
not started. A board behind the camera, facing away or beyond 12 m is not
drawn; the localizer's own range limit and EKF gate do the rest. The camera's
pose is the stack's TF to the ZED optical frame when it exists (it does not on
the bench, whose kit has no ZED frames), else the prototype's mount, 0.20 m
forward and 0.25 m up, published as `mock_camera_optical`. The image is mono8:
a 1.7 MB colour frame is larger than the host's socket receive buffer
(`net.core.rmem_max` 1 MB) and the localizer's best-effort subscription lost
all but 21 of 650; the 576 kB mono frame arrives whole. The frames also fill
the recorder's camera topic, so the bag passes the camera checks.

Cost, measured on the shared workstation (play_launch's per-node metrics,
median over a run): `mock_aruco_camera` 10 % of one core (2.5 ms a frame),
the real `ar_tag_based_localizer` 13 %, against 6.5 % for
`virtual_marker_localizer`. Localization on `lab2_uturn` with the reference
laws: 450 fixes (12.7/s), fix error median 0.05 m / p95 0.35 m, EKF error
median 0.025 m / p95 0.08 m, longest gap 1.6 s (virtual: fix p95 0.23 m, EKF
p95 0.08 m). The localizer must be AutoSDV's fork (the submodule
`src/localization/external/autoware_ar_tag_based_localizer`, built): the stock
node in `/opt/autoware` has no `dictionary` parameter, cannot decode the
boards' DICT_5X5_1000, and finds nothing without saying so. The preflight now
fails on the stock node (`localizer` line).

The student gate (`just lab2 gate`) keeps `camera:=virtual`: it is
deterministic per seed, cheaper, and gates the SpeedPID, which the camera
path does not affect. `just lab2 run --bench` uses `virtual` too unless
`--camera mock`.

**A bench bag is never a submission.** `lab2_record --bench` writes
`"bench": true` into the bag's metadata (and, without the mock camera, fills
the camera topic with a labelled synthetic card, "BENCH - simulated car - not
a camera"). `lab2_check_bags` rejects such a bag ("bench bag (simulated car,
rendered or synthetic camera), not a run on the car") unless it is given `--bench`, which
only the integration test and `audit` of a bench bag pass.

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
waypoint, the stop line, level with the start box; once the run is finished
the whole trajectory reads 0 m/s, so the car stays stopped even when the
localized pose settles a little short of the line. The template is already in
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
car's reference bag; a bag off the path by more than 0.3 m, with a localization
jump (pose vs integrated wheel speed over 1 s, 0.3 m), a non-finite pose or
speed, unfinished, with an MRM or a disengage is invalid and scores 0.

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
just lab2 gate    submission:=~/lab2/B12345678_lab_2_1     # or `submission:` in ~/lab2/lab2.yaml
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
frame end to end. `camera:=mock` (above) is the same idea as a node that
follows the plant, so a whole run goes through the real localizer.

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

## Fuzzing the Part 2 gates (`scripts/testing/lab2/fuzz_bags.py`)

A TA-side test that the bag gates catch bad runs. It takes one bag of the
graded route that passes both gates, writes mutated copies under
`tmp/fuzz2/bags/`, and runs `check_bags`, the route grade
(`route_grade.read_route_bag` + `score_route`) and, when present, the audit on
each:

```bash
python3 scripts/testing/lab2/fuzz_bags.py tmp/lab2_cfg/bags/lab2_uturn_2 \
    --reference tmp/uturn_bags/lab2_ref [--seed 1] [--n 30] [--out tmp/fuzz2]
python3 scripts/testing/lab2/fuzz_bags.py --list      # the catalogue and its expectations
python3 scripts/testing/lab2/fuzz_bags.py BAG --gate  # plus the bench gate on mutant SpeedPIDs
```

The source bag is never modified. A bench bag recorded before the synthetic
camera has an empty camera topic, which `check_bags` rejects on its own, so the
script first writes a copy with a synthetic 960 x 600, 10 Hz camera to
`tmp/fuzz2/base/` and checks that copy passes both gates.

The catalogue (about 65 cases plus `--n` seeded random combinations):

| Group | Mutations | Expected |
| --- | --- | --- |
| topics | each required topic dropped; kinematic state, PWM, camera declared but empty | REJECT (INVALID too when the grade needs the topic) |
| metadata | route_hash, route_kind, v_ref, car, team altered or missing; no metadata at all; route swapped for hall_demo or for a modified U-turn; bench flag | REJECT |
| run | MRM for 0.1 s mid-run; disengage mid-run; never engaged; truncated before the stop; time-reversed bag; laps changed in the progress | REJECT + INVALID |
| localization | 0.35 m lateral offset for 1 s; 0.6 m lateral jump for 2 s; one NaN pose; a 1 m jump along the path | REJECT + INVALID |
| speed | all speeds x1.3; commanded plateau 0.90 m/s; NaN speed samples | REJECT, or INVALID |
| benign | MRM or disengage after the finish; 0.25 m lateral offset; duplicated timestamps; reversed header stamps; adjacent samples swapped; commanded 0.83 m/s; course_hash altered (by design: markers are free) | VALID, metrics unchanged |
| metrics | measured speed x(1+e) in the steady window, e = 0.01..0.10; speed noise sigma 0.005..0.04 | VALID, speed RMSE rises monotonically |
| | +0.15 m/s spike after t90 | VALID, overshoot captures it |
| | final pose +0.2 m past / -0.2 m short of the stop line | VALID, stop_error counted double / single |

Each case prints CAUGHT (a bad bag was caught), OK (a good bag stayed good and
its metrics moved as expected), MISSED (a bad bag passed), FALSE (a good bag was
rejected), METRIC (a metric did not move as expected) or CRASH (a gate raised
instead of giving a verdict). Exit status 1 on any MISSED, FALSE, METRIC or
CRASH; `tmp/fuzz2/results.json` has every verdict and metric. Failing mutants
are kept for inspection, passing ones deleted unless `--keep`. The verdict
logic is unit-tested without ROS: `python3 -m pytest scripts/testing/lab2/test_fuzz_bags.py`.

### Results (2026-10-16, base `lab2_uturn_2`, reference `lab2_ref`, seed 1)

95 cases in 94 s with 4 jobs: 72 CAUGHT, 21 OK, 1 MISSED, 1 CRASH. Every bad
metadata, topic, MRM, disengage, truncation, speed and camera mutant was
caught, and so was every random combination. The 0.35 m offset is invalid at
0.357 m and the 0.25 m one stays valid at 0.257 m. Speed RMSE rises
monotonically with both the scale (0.006 -> 0.082 m/s at x1.10) and the noise.
A 0.15 m/s spike reads as 0.162 m/s of overshoot. Stopping 0.2 m past the line
gives stop_error 0.315 m (counted double) and 0.2 m short gives 0.243 m.

Findings of the first fuzz run, all fixed since (the fuzz now exits 0 on a
bench base with `--reference`; 75 CAUGHT, 20 OK):

- **was MISSED, `loc:jump_along_1.0m_1s`**: the pose jumping 1 m ahead along the
  path for 1 s passed, because the path deviation is measured across the path.
  Now a validity gate in `route_grade` (and a check-bags reason): over every
  1 s from engage to rest the pose's displacement and the integrated wheel
  speed must agree within `LOC_JUMP_MAX` = 0.30 m ("localization jump"). Clean
  bench runs (virtual and rendered camera, the TA reference) reach 0.05-0.07 m.
  Not yet calibrated on a car bag: a marker fix after a gap moves the EKF
  legitimately, so check the first car bags' `localization jump` line.
- **was CRASH, `loc:nan_pose_1_sample`**: one non-finite pose crashed
  check-bags, the grade and the audit (`Path.project` returned None). Now
  non-finite poses are dropped before measuring and named: "N non-finite
  localized pose(s)" (INVALID, REJECT); `Path.project` raises ValueError for a
  non-finite point instead of returning None.
- **NaN speed reports** are named likewise, "N non-finite speed report(s)",
  in both gates (it used to read "no steady window").
- **A bag cut 0.3 s after the stop** was accepted, and the stop pose came from
  one sample instead of the settle median. check-bags now rejects a bag that
  ends less than `SETTLE[1]` (3 s) after the car came to rest; `just lab2 run`
  records 3 s past the end plus the 1 s it waits to see the car at rest.

Bench gate (`--gate`, 4 mutants, 6 min): sign flip, always neutral, max PWM and
NaN every 50th call were all FAIL, for the reasons "did not finish / median
0.27 m/s", "did not move", "MRM + peak 2.85 m/s" and "SpeedPID fault".
