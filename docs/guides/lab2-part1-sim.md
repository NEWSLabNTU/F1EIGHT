# Lab 2 Part 1: speed and brake control on the simulated car

In Part 1 you write the speed controller of the AutoSDV car and improve it in a test
loop: run, read the curves and the violations, change the controller, run again. The car
is the RC simulator's model (motor, ESC, rear-wheel drive with slip, hall sensor) behind
the car's real vehicle interface (`hardware:=sim`). Around it runs the Lab 2 stack: the
waypoint path publisher, the TA's pure pursuit for steering, Autoware's command gate, and
the actuator's safety shell with **your** `SpeedPID` inside it, in the Part 1 simulation
vehicle model, `lab2_sim_vehicle` (one package, the same name for everyone; the car models,
`teamN_vehicle`, are Part 2's). It is the car's own launch
(`waypoint_follow.launch.xml`, `localization:=ground_truth hardware:=sim`): only the
localization differs, which here is the simulator's true pose. The test harness around it
(the plant, the scenario that engages the car and aligns its start, the grader) is
`lab2_part1_sim.launch.xml`'s own.

Design: `docs/design/lab2-part1-sim-design.md`; the vehicle models:
`docs/design/lab2-team-vehicle-models.md`. The simulator itself: `docs/guides/rc-sim.md`.

## One command

Once, to install the simulation model: copy `lab2_sim_vehicle_launch/` from the handout into
this repository's `src/lab2_vehicles/` (gitignored) and build it with the repository:

```bash
cp -r <handout>/lab2_sim_vehicle_launch src/lab2_vehicles/
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to lab2_sim_vehicle_launch
source install/setup.bash  # or open a new shell
```

Then the test loop:

```bash
just lab2 sim              # the graded set: nominal, low_batt, low_mu; the mark out of 25, ~3 min
just lab2 sim --no-gui     # the same without windows (SSH, batch)
just lab2 plot-sim         # redraw the latest run's curves.png (every car)
```

One command leaves nothing running. It checks the model, starts the stack in its own
process group with `vehicle_model:=lab2_sim_vehicle vehicle_id:=sim`, opens RViz and the
dashboard, engages the car, drives the course, grades the run live, and does that for each
of the three cars: `nominal`, `low_batt` (weaker battery) and `low_mu` (more slippery
floor). Then it draws `curves.png` for every car, prints a table of the cars (checks
passed, the plot's path) and the mark out of 25, and stops everything (Ctrl-C included).
From a terminal, the windows of the **last** car stay up with its final curves and marks
until the dashboard window is closed or Enter is pressed (`--no-hold` skips it).

**The model** resolves from this repository's install (its ament index). Nothing to name: no
`team:=` (it is accepted and ignored), nothing from `~/lab2/lab2.yaml`. Not installed, and it
stops with the copy and the colcon command above.

**Built, and current.** Build with `--symlink-install` (the command above): the installed
files are then symlinks into `src/lab2_vehicles/`, so an edit to `speed_pid.py` or to
`config/sim/speed_controller.param.yaml` runs on the next `just lab2 sim` without a rebuild.
A **new** file, a new package, or a change to `setup.py`, `package.xml` or `CMakeLists.txt`
needs the colcon command again. An install without `--symlink-install`, or one whose
`speed_pid.py` is not the one in your source, is refused with the same command, because
your edits would not run.

**Checked before it runs** (`scripts/testing/lab2/submission.py --part1 DIR`, which you can
also run yourself on `src/lab2_vehicles/lab2_sim_vehicle_launch`): the three packages are there,
`lab2_sim_vehicle_interface/speed_pid.py` defines `class SpeedPID`, the sim tuning file
parses and its keys, types and ranges are the table below, and nothing in the model imports
the TA's reference controllers (`autosdv_waypoint_lab.reference`), and the model runs its own
`speed_pid.py`'s `SpeedPID`: AutoSDV's built-in `AckermannPID` / `AckermannSpeedPID`
(`speed_controller.py`, which ships in every model) may be read, but `speed_pid.py` and the
modules it imports may not import, subclass or select them, the launch's
`speed_controller_class` default must stay `lab2_sim_vehicle_interface.speed_pid:SpeedPID`,
and no config file may set it. Any of these stops the run with the file and the key named.
After each car, the runner also reads the class the actuator actually loaded ("Speed
controller: ..." in its log, recorded as `speed_controller` in `marks.json`); a run of any
other class is invalid and counts 0.

**Windows.** `--no-gui` opens none (the same as `rviz:=false gui:=false`). With no
working X display (an SSH session without X) the run goes headless by itself and says so
in one line; the run and its marks are the same, and `curves.png` is still drawn.

TA only: `submission:=DIR` runs a team's model source (below), `variant:=low_mu` runs one
car (no mark out of 25), `--name LABEL` labels the run directory, `--out DIR` writes `DIR/<variant>/` (the old layout, which
the TA's calibration and tolerance tools read back), and `--final` is accepted as a
no-op (the graded set is the default).

Setup is the repository's standard one: `./setup.sh`, then `just build`, then the two
model commands above.

## The course

One straight, `config/courses/lab2_part1/` in `autosdv_waypoint_lab`. Planner limits:
2.5 m/s² accelerating and 2.5 m/s² braking.

| # | Segment | What it tests |
|---|---|---|
| 1 | accelerate 0 → 4 m/s, hold 4 s | launch from rest (no spin, no lurch), low-speed hold |
| 2 | accelerate 4 → 11 m/s, hold 4 s | the speed change: full throttle, hold at 40 km/h |
| 3 | brake 11 → 0 onto the stop line (s = 108.4 m), run-out 10 m | planned braking, stop error |

About 17 s of driving (16.8 s nominal). There is no emergency stop on the course.
`part1_profile.yaml` in the course directory is the contract the scenario and the grader
both read; nothing in either hard-codes a position.

## What you see

- **RViz**: the lane, the stop line, the car and its path, and a
  marker wherever a check failed.
- **Dashboard window**:
  - speed against time: the target, the measured speed (what your controller sees) and
    the true speed;
  - PWM against time, with neutral and the brake zone shaded;
  - distance to the stop line;
  - each failed check's window in red;
  - the checks live ("pending" until each window closes), then the marks.

  It plots exactly what the grader graded, and saves itself to `dashboard.png` in the
  run's output directory when it closes.

## The checks

Nine checks per run, computed live from the plant's **true** state, the target, the PWM
and the shell's mode, each over a window that `part1_profile.yaml` defines.
`autosdv_waypoint_lab/part1/metrics.py` is the definition. Its docstring gives every
window; `LIMITS` has the numbers.

| Check | Window | Meaning | Limit |
|---|---|---|---|
| `hold_rmse_4`, `hold_rmse_11` | the 4 m/s and the 11 m/s hold, from 1 s in | RMSE of true speed minus the hold speed | 0.05 / 0.15 m/s |
| `overshoot` | the two steps up (0 → 4, 4 → 11) | peak above the hold speed, % of the step | 3.8 % |
| `reach_s` | the same steps | time from entering the hold until within 10 % of the step | 0.2 s |
| `launch_slip` | 1 s from the first motion | max rear-wheel slip (spin) [m/s] | 0.5, physical |
| `launch_accel` | the same | max acceleration over 50 ms (a lurch) | 4.5 m/s² |
| `stop_lag` | the final brake, from its start to the stop line | max of true speed − target: a car that cannot slow down | 0.88 m/s |
| `stop_error` | the final rest | distance from the stop line [m] | 0.5, fixed |
| `stop_kappa` | final braking, speed > 0.3 m/s | min rear slip ratio (wheel lock below −0.2) | −0.2, physical |

**Limits.** The physical limits and the stop error are fixed. The others are
*calibrated*: 1.5 × the reference controller's worst case over the graded variants,
rounded up to 2 significant figures, and never below a floor (hold RMSE 0.05 m/s, reach
0.2 s, overshoot 2 %, stop lag 0.3 m/s), so a reference that is nearly
perfect does not set an unreachable limit. Calibrated 2026-10-19 on this course with
`scripts/testing/lab2/part1_calibrate.py --repeats 2` (6 live runs on nominal, low_batt and
low_mu, all valid and passing, the repeats bit-identical; `LIMITS_CALIBRATED = True` in `metrics.py`):

| Check | Reference worst | Limit |
|---|---|---|
| `hold_rmse_4` | 0.027 m/s (low_mu) | 0.05 (floor) |
| `hold_rmse_11` | 0.098 m/s (low_batt) | 0.15 |
| `overshoot` | 2.49 % (nominal) | 3.8 |
| `reach_s` | 0.00 s | 0.2 (floor) |
| `launch_accel` | 2.94 m/s² (nominal) | 4.5 |
| `stop_lag` | 0.581 m/s (low_mu) | 0.88 (floor 0.3) |

Re-run the calibration after any change to the plant, the stack or the reference
(the TA notes below). Each run prints the limit it was graded against.

**Validity.** A run is invalid, and scores 0 of 9 with the reason printed, only if:

- the clock handshake was invalidated (`lockstep invalidated: ...`);
- the controller faulted the shell (`controller fault: ...`);
- the actuator never started (`the actuator is not running: ...`): the class did not
  import or its `__init__` raised, so nothing of it ran;
- an MRM was raised (`MRM: ...`);
- the scenario aborted;
- an e-stop the profile asks for was never raised (the course has none);
- the stack never engaged.

A car that never moves, or stops early, is a **valid** run whose unreached checks fail.
The grader says so in plain words: *"Your car did not move ... Is your speed_pid.py
still the stub?"*

**Marks.** Each car shows passed/9; the run writes `final.json` with
`marks = 25 × sum(passed) / sum(total)` over the three cars (27 checks); an invalid run
counts 0 of its 9.

## Outputs

A new directory per run, so nothing is overwritten:
`tmp/lab2/part1/<model>/<YYYY-mm-dd_HH-MM-SS>/<variant>/` (`lab2_sim_vehicle`), and
`tmp/lab2/part1/<model>/latest` points at the newest run. Per car:

- `trace.csv` (100 Hz: `t, s, y, target, measured, speed, pwm, mode, slip_r, kappa_r,
  emergency, segment`);
- `marks.json` (`variant, valid, invalid_reason, passed, total, checks`);
- `run.json` (the command, the vehicle model and its source directory, the timings, the
  `play_log` path);
- `curves.png` (speed, PWM and every failed check's window; drawn when the run ends, no
  display needed; `just lab2 plot-sim` redraws it, `--t0/--t1` zoom in);
- `dashboard.png` (the dashboard as it was when it closed; with the dashboard only);
- `play_log/` (every node's log).

`final.json` (the mark out of 25) sits beside the three car directories.

## The controller

You edit two files of the model, `src/lab2_vehicles/lab2_sim_vehicle_launch/`:

- `lab2_sim_vehicle_interface/lab2_sim_vehicle_interface/speed_pid.py`, the `SpeedPID` (it starts
  as the student stub; the model's launch file loads it as
  `lab2_sim_vehicle_interface.speed_pid:SpeedPID`);
- `lab2_sim_vehicle_launch/config/sim/speed_controller.param.yaml`, its tuning on the
  simulated car (`vehicle_id` sim). The parameters sit under `speed_controller_params`:

  ```yaml
  /**:
    ros__parameters:
      speed_controller_params:
        kp: 12.0
        ki: 10.0
        # ... every key of the table below
  ```

The interface is the actuator's plug-in contract
(`autosdv_vehicle_interface/speed_controller.py`): `__init__(params)`, `reset()`,
`update(target_speed, measured_speed, dt) -> int`.

Two modes:

- **Driving**: feed-forward plus PID, from neutral up to `max_pwm_offset`. Driving never
  goes below neutral. P and I see the error clamped to `error_limit`: a large error is a
  new target (the feed-forward's job) or the hall sensor still reading 0 after a start.
- **Braking**: entered when the car is faster than the target by more than
  `brake_enter`, left when the excess falls below `brake_exit` (hysteresis). The
  feed-forward and the integral are set aside. The request `-(kp_brake·e + D)` goes to
  the brake band through the given `brake_pwm()`, which skips the ESC's dead band.

What is given and what you write:

| Given | TODO |
|---|---|
| reading the parameters, `reset()`, `feed_forward()`, `filter_derivative()`, `brake_pwm()`, `clamp_pwm()`, `debug_terms()` | 1 the error and the clamped driving error, 2 P, 3 D on the measurement (filtered), 4 the braking mode, 5 driving: I with clamp, the integral band and conditional integration, the command and its throttle limit |

`config/sim/speed_controller.param.yaml`, checked when the run starts
(`scripts/testing/lab2/submission.py --part1`): unknown keys, wrong types and out-of-range
values stop the run with the key named; a key left out warns, and the class falls back to
its own default. **The starting values are not tuned**: a correct implementation with them drives the
course but fails several checks. Tuning is your work.

| Key | Meaning | Range | Starting value |
|---|---|---|---|
| `kp`, `ki`, `kd` | driving gains [counts per m/s, per m, per m/s²] | 0..1000, 0..1000, 0..100 | 12, 10, 0 |
| `integral_limit` | the integral's clamp [counts] | (0, 1000] | 5 |
| `error_limit` | the largest error P and I see while driving [m/s] | (0, 20] | 5 |
| `integral_band` | integrate only when \|e\| ≤ this [m/s]; 20 is off on this course | (0, 20] | 20 |
| `enable_conditional_integration` | freeze the integral while the output saturates | bool | true |
| `derivative_filter_alpha` | the derivative's EMA | (0, 1] | 0.15 |
| `max_pwm_offset` | throttle authority above neutral [counts] | 1..100 | 100 |
| `max_brake_offset` | full brake, counts below neutral | 0..10 | 10 |
| `kp_brake` | brake request per m/s too fast [counts per m/s] | 0..1000 | 6 |
| `brake_enter`, `brake_exit` | the braking mode's thresholds [m/s] | (0, 5], [0, 5], exit < enter | 0.15, 0.05 |
| `brake_deadband` | counts below neutral the ESC ignores | 0..10 | 5 |
| `ff_deadband`, `ff_gain` | the given feed-forward: PWM = init + d + v/g | 0..50, (0, 1] | 5.27, 0.1433 |

The feed-forward was calibrated on a fresh 12.4 V pack. The car runs at 11.1 V, so it
under-drives: about 10 counts short at 11 m/s, which the integral must make up.

Part 2 drives the car model, `teamN_vehicle`, on the real cars, with that car's tuning
file, `config/car_a/speed_controller.param.yaml` and so on (`lab2-waypoint-stack.md`). The car
gives about 0.03 m/s per count against the simulated car's 0.143, so its gains are about
five times larger.

## Physics you need

- **Light regeneration.** A forward PWM below what the motor's speed already generates
  brakes by regeneration, lightly: with rolling resistance and drag, 1.5 to 2.0 m/s² at
  3 to 11 m/s. At neutral the car only coasts, 1.0 to 1.3 m/s². A controller that never
  goes below neutral cannot follow the course's 2.5 m/s² braking.
- **The brake band and its dead band.** Below neutral is the ESC's brake: brake level =
  (365 − PWM) / 5. So 365..369 do nothing (the 5-count dead band) and 360 (`min_pwm`) is
  full brake, 3.0 to 3.5 m/s² depending on speed and variant. Brake decel by count, from
  11 m/s: 363 → 1.9, 362 → 2.4, 361 → 2.8, 360 → 3.2 m/s². The shell's own `brake_pwm`
  (340) is full brake too.
- **The hall sensor is blind at very low speed.** The speed comes from 12 magnets on a
  rear wheel. From rest the first reading arrives only when the car is already at about
  0.4 m/s, and until then `measured_speed` is 0. A controller that pushes hard on that 0
  spins the rear wheel at launch (`launch_slip`). The sensor is on a driven wheel, so a spinning wheel also reads
  as speed. Under throttle the wheel turns slightly faster than the car, and under brake
  slightly slower, so the reading jumps by 0.2 to 0.4 m/s at each throttle/brake
  switch. A braking threshold inside that band makes the controller chatter between
  the modes at about 5 Hz (the reference uses `brake_enter` 0.4).
- **Rear-wheel drive with slip.** Full throttle from rest spins the rear tyres; on
  `low_mu` both spin and lock come sooner.
- The variants: `low_batt` is a 10.8 V pack (less top speed, more under-driven
  feed-forward); `low_mu` has μ 0.75 instead of 0.8.

## Troubleshooting

| Symptom | Cause / remedy |
|---|---|
| `no working X display ...: running headless` | an SSH session without X; the run is the same, and `curves.png` is drawn. Use a desktop session or VNC to see the windows. |
| `error: lab2_sim_vehicle is not built` | copy `lab2_sim_vehicle_launch` into `src/lab2_vehicles/`, run the colcon command it prints, open a new shell |
| `... installed without --symlink-install` / `is built from another copy` | the same colcon command (with `--symlink-install`), then run again |
| `error: the model does not pass its Part 1 check: .../speed_controller.param.yaml ...: kp is ...` | a key misspelled, out of range, the wrong type, or not under `speed_controller_params`; the message names it |
| `... uses the TA reference (autosdv_waypoint_lab.reference) at ...` | the controller must be your own: remove that import |
| "Your car did not move ... still the stub?" | `update()` returns neutral: the TODOs are not done, or the output never leaves `init_pwm` |
| invalid: `controller fault: ...` | `update()` raised or returned a non-number; the traceback is in `play_log/` |
| invalid: `the actuator is not running: ...` | the actuator died at start-up: `speed_pid.py` does not import, or `SpeedPID.__init__` raised (or the model's launch file is broken); the traceback is in `play_log/` |
| invalid: `lockstep invalidated: ...` | the simulation lost a participant (a node crashed or hung); run again, and report it if it repeats |
| invalid: `MRM: ...` / `the stack did not engage` | a stack fault, not your controller; run again; report it with `run.json` |
| the car stops about 9 m past the line, `stop_lag` fails | no braking mode: regeneration alone cannot follow 2.5 m/s² |
| `launch_slip` fails | too much push while the measured speed is still 0 (see the hall sensor above) |

## TA notes

- **A controller by import path**: `just lab2 sim --class
  autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID --params "{kp: 10.0, ...}"`
  (the TA reference, on the reference model `autosdv_vehicle`;
  `config/speed_pid_part1_reference.yaml` holds its **tuned** parameters, which calibrate
  and tolerance use). `--class` excludes `model:=` and `submission:=`. `model:=NAME` runs another model
  than `lab2_sim_vehicle` from the sourced install, checked the same way.
- **Starting values vs tuned**: `config/speed_pid_part1.yaml` (= the stub's defaults, and
  the seed of `lab2_sim_vehicle`'s `config/sim/speed_controller.param.yaml`, where these
  keys sit under `speed_controller_params`) is deliberately detuned: kp 12,
  ki 10, kd 0, integral_limit 5, error_limit 5, kp_brake 6, brake_enter 0.15. The
  reference code with those values scored **18/27 on `--final` (16.67/25)**, 2026-10-19:
  `hold_rmse_11` (the integral clamp cannot cover the feed-forward deficit), `stop_lag` and
  `stop_error` (brake gain too weak) fail on every variant. Tuned: kp 10, ki 15, kd 0.3,
  integral_limit 30, error_limit 0.3, kp_brake 20, brake_enter 0.4 (27/27). The same
  numbers hold for the reference class dropped into the model (its `speed_pid.py`, its
  sim tuning file) as for `--class`: the model path and the class path run the same
  actuator. `scripts/testing/lab2/part1_run.py --help` documents the rest: `--out`,
  `--no-rviz --no-gui`, `--timeout`, `--stall`.
- **Reference and stub**: `autosdv_waypoint_lab/reference/speed_pid_part1.py` (removed
  from the student branch) and `autosdv_waypoint_lab/stub/speed_pid_part1.py` (copied byte
  for byte as `lab2_sim_vehicle`'s `speed_pid.py`; the reference's tests are
  `test/test_speed_pid_part1_reference.py`).
- **Student branch**: `scripts/testing/lab2/strip_student_branch.sh` lists every TA-only
  file and section (dry run by default).
- **Calibrate the limits**: `python3 scripts/testing/lab2/part1_calibrate.py
  [--repeats 3]`. It runs the reference live on the three graded variants and prints
  1.5 × worst per calibrated metric, as a `LIMITS` snippet for `part1/metrics.py`. Then
  set `LIMITS_CALIBRATED = True`. It refuses a reference that fails a fixed check.
- **Load tolerance**: `python3 scripts/testing/lab2/part1_tolerance.py --class ... -n 3
  --busy-mult 2 --interleave` runs quiet against busy-loop-loaded hosts and compares the
  verdicts and spreads (the design's acceptance test; PASS on 2026-10-19, tightest
  `stop_lag` 0.0138 of 0.0176 m/s). Run it alone on the host (`flock tmp/bench.lock`).
- **Grading**: `just lab2 sim submission:=<dir> --no-gui`, `<dir>` a student's
  `<student_id>_lab_2_1/` holding `lab2_sim_vehicle_launch/` (and the report), or the
  `lab2_sim_vehicle_launch/` directory itself. The runner checks it as above, builds that
  directory alone (`colcon build`, no `--symlink-install`, the AutoSDV install as the
  underlay) into the run's own `ws/` (log in `ws/build.log`), sources it on top and runs it.
  Every submission has the same package names, so each one gets its own workspace;
  nothing in `src/lab2_vehicles/` is used or changed, and the mark is for the code as
  handed in. A
  build failure stops with the log's tail. The run directory is labelled with the
  submission's directory name, `tmp/lab2/part1/lab2_sim_vehicle/<date>_<time>_<dir>/`
  (`--name` overrides it); read `final.json` there.
- **Course**: `just lab2 make-part1-course` regenerates `config/courses/lab2_part1/`
  (committed).
- Several stacks on one host: `ROS_DOMAIN_ID` (yours if exported and non-zero, else
  1 + uid % 100) and `LAB2_WEB_ADDR` (the play_launch web UI, default 127.0.0.1:8081).
