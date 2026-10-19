# Lab 2 Part 1: speed and brake control on the simulated car

In Part 1 you write the speed controller of the AutoSDV car and improve it in a test
loop: run, read the curves and the violations, change the controller, run again. The car
is the RC simulator's model (motor, ESC, rear-wheel drive with slip, hall sensor) behind
the car's real vehicle interface (`hardware:=sim`). Around it runs the Lab 2 stack: the
waypoint path publisher, the TA's pure pursuit for steering, Autoware's command gate, and
the actuator's safety shell with **your** `SpeedPID` inside it. It is the car's own launch
(`waypoint_follow.launch.xml`, `localization:=ground_truth hardware:=sim`): only the
localization differs, which here is the simulator's true pose. The test harness around it
(the plant, the scenario that engages the car and aligns its start, the grader) is
`lab2_part1_sim.launch.xml`'s own.

Design: `docs/design/lab2-part1-sim-design.md`. The simulator itself: `docs/guides/rc-sim.md`.

## One command

```bash
just lab2 sim              # the graded set: nominal, low_batt, low_mu; the mark out of 25, ~3 min
just lab2 sim --no-gui     # the same without windows (SSH, batch)
just lab2 plot-sim         # redraw the latest run's curves.png (every car)
```

One command builds nothing by hand and leaves nothing running. It finds your submission,
starts the stack in its own process group, opens RViz and the dashboard, engages the car,
drives the course, grades the run live, and does that for each of the three cars:
`nominal`, `low_batt` (weaker battery) and `low_mu` (more slippery floor). Then it draws
`curves.png` for every car, prints a table of the cars (checks passed, the plot's path)
and the mark out of 25, and stops everything (Ctrl-C included). From a terminal, the
windows of the **last** car stay up with its final curves and marks until the dashboard
window is closed or Enter is pressed (`--no-hold` skips it).

**The submission** is found in this order:

1. `submission:=DIR` on the command line (TA, or a second copy);
2. `part1_submission:` in `~/lab2/lab2.yaml`;
3. the one directory `~/lab2/*_lab_2_1` that holds `lab2_speed/`.

None, or several, and it stops with a list of what it found and how to fix it. `DIR` is
either `<student_id>_lab_2_1/` holding `lab2_speed/`, or the `lab2_speed/` package
itself. `LAB2_HOME` moves `~/lab2` (tests; a TA grading from another directory).

**Windows.** `--no-gui` opens none (the same as `rviz:=false gui:=false`). With no
working X display (an SSH session without X) the run goes headless by itself and says so
in one line; the run and its marks are the same, and `curves.png` is still drawn.

TA only: `variant:=low_mu` runs one car (no mark out of 25), `--name LABEL` labels the
run directory, `--out DIR` writes `DIR/<variant>/` (the old layout, which
`part1_calibrate.py` and `part1_tolerance.py` read back), and `--final` is still
accepted (it is the default now).

Setup is the repository's standard one: `./setup.sh`, then `just build`.

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
*calibrated* by the course staff on a reference run of the course. Each run prints the limit it was graded against.

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
`tmp/lab2/part1/<submission name>/<YYYY-mm-dd_HH-MM-SS>/<variant>/`, and
`tmp/lab2/part1/<submission name>/latest` points at the newest run. Per car:

- `trace.csv` (100 Hz: `t, s, y, target, measured, speed, pwm, mode, slip_r, kappa_r,
  emergency, segment`);
- `marks.json` (`variant, valid, invalid_reason, passed, total, checks`);
- `run.json` (the command, the timings, the `play_log` path);
- `curves.png` (speed, PWM and every failed check's window; drawn when the run ends, no
  display needed; `just lab2 plot-sim` redraws it, `--t0/--t1` zoom in);
- `dashboard.png` (the dashboard as it was when it closed; with the dashboard only);
- `play_log/` (every node's log).

`final.json` (the mark out of 25) sits beside the three car directories.

## The controller

You edit `lab2_speed/speed_pid.py` and `config/speed_pid.yaml` of the handout package
(course repository `2026/labs/Lab2/handout/lab2_speed/`; the README there is the
reference). The interface is the actuator's plug-in contract
(`autosdv_vehicle_interface/speed_controller.py`): `__init__(params)`, `reset()`,
`update(target_speed, measured_speed, dt) -> int`. It is loaded as
`lab2_speed.speed_pid:SpeedPID`.

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

`config/speed_pid.yaml`, checked when the run starts (`scripts/testing/lab2/submission.py
--part1`): unknown keys, wrong types and out-of-range values stop the run with the key
named. **The starting values are not tuned**: a correct implementation with them drives the
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

`python3 selfcheck.py` in the package checks the rules, the yaml and each TODO (no ROS, no
grading).

Part 2 drives the same class on the real car with `config/speed_pid_car.yaml` (given: kp 20,
ki 25, integral_limit 10, error_limit 0.5, integral_band 0.25, brake mode on) plus the
team's `config/calibration.yaml` as `ff_deadband`/`ff_gain`; `selfcheck.py --car` runs the
offline car bench on it, and `just lab2 gate submission:=DIR` loads it
(`lab2-waypoint-stack.md`, "Running a student's submission"). The car gives about 0.03 m/s
per count against the simulated car's 0.143, so its gains are about five times larger; the
band, off here, stops the integral charging while the bench holds the wheels.

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
| `error: no Part 1 submission found` | no `~/lab2/<student_id>_lab_2_1/lab2_speed/`: put it there, or pass `submission:=DIR` |
| `error: several Part 1 submissions` | more than one `~/lab2/*_lab_2_1`: name one with `submission:=DIR` or `part1_submission:` in `~/lab2/lab2.yaml`, or move the others away |
| `error: submission DIR: submission error: .../speed_pid.yaml: kp is ...` | a key misspelled, out of range, or the wrong type; the message names it |
| "Your car did not move ... still the stub?" | `update()` returns neutral: the TODOs are not done, or the output never leaves `init_pwm` |
| invalid: `controller fault: ...` | `update()` raised or returned a non-number; the traceback is in `play_log/` |
| invalid: `the actuator is not running: ...` | the actuator died at start-up: `speed_pid.py` does not import, or `SpeedPID.__init__` raised; the traceback is in `play_log/` |
| invalid: `lockstep invalidated: ...` | the simulation lost a participant (a node crashed or hung); run again, and report it if it repeats |
| invalid: `MRM: ...` / `the stack did not engage` | a stack fault, not your controller; run again; report it with `run.json` |
| the car stops about 9 m past the line, `stop_lag` fails | no braking mode: regeneration alone cannot follow 2.5 m/s² |
| `launch_slip` fails | too much push while the measured speed is still 0 (see the hall sensor above) |

