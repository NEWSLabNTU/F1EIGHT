# Lab 2 Part 1: speed and brake control on the simulated car

In Part 1 you write the speed controller of the AutoSDV car and improve it in a test
loop: run, read the curves and the violations, change the controller, run again. The car
is the RC simulator's model (motor, ESC, rear-wheel drive with slip, hall sensor) behind
the car's real vehicle interface (`hardware:=sim`). Around it runs the Lab 2 stack: the
waypoint path publisher, the TA's pure pursuit for steering, Autoware's command gate, and
the actuator's safety shell with **your** `SpeedPID` inside it. It is the car's own launch
(`waypoint_follow.launch.xml`, `localization:=ground_truth hardware:=sim`): only the
localization differs, which here is the simulator's true pose. The test harness around it
(the plant, the scenario that engages and e-stops the car, the grader) is
`lab2_part1_sim.launch.xml`'s own.

Design: `docs/design/lab2-part1-sim-design.md`. The simulator itself: `docs/guides/rc-sim.md`.

## One command

```bash
just lab2 sim submission:=~/lab2/B12345678_lab_2_1            # one run, nominal car, ~1.5 min
just lab2 sim submission:=~/lab2/B12345678_lab_2_1 --final    # the graded set, ~5 min
just lab2 sim submission:=DIR variant:=low_mu                  # one run on another car
just lab2 sim submission:=DIR rviz:=false gui:=false           # no windows (SSH, batch)
just lab2 sim                                                  # part1_submission from ~/lab2/lab2.yaml
```

One command builds nothing by hand and leaves nothing running. It starts the stack in
its own process group, opens RViz and the dashboard, engages the car, drives the course,
grades the run live, prints the marks and stops everything (Ctrl-C included). A single
run from a terminal first holds the finished run open: the dashboard and RViz stay up with
the final curves and marks until the dashboard window is closed or Enter is pressed
(`--no-hold` skips it; `--final` never holds).

`submission:=DIR` is either `<student_id>_lab_2_1/` holding `lab2_speed/`, or the
`lab2_speed/` package itself. To stop typing it, put it in `~/lab2/lab2.yaml`:

```yaml
part1_submission: ~/lab2/B12345678_lab_2_1
```

`--final` runs the variants `nominal`, `low_batt` (weaker battery) and `low_mu` (more
slippery floor) one after the other. Without a submission anywhere the runner starts
**the stub**, which holds neutral, and prints a loud warning: the car does not move.

Setup is the repository's standard one: `./setup.sh`, then `just build`.

## The course

One straight, `config/courses/lab2_part1/` in `autosdv_waypoint_lab`. Planner limits:
2.5 m/s² accelerating and 2.5 m/s² braking.

| # | Segment | What it tests |
|---|---|---|
| 1 | accelerate 0 → 2 m/s, hold 3 s | launch from rest (no spin, no lurch), low-speed hold |
| 2 | accelerate 2 → 6, hold 3 s | step response, hold |
| 3 | accelerate 6 → 11, hold 3 s | full throttle, hold at 40 km/h |
| 4 | brake 11 → 6, hold 8 s, **e-stop** at s = 113.2 m | active braking; emergency stop and resume |
| 5 | accelerate 6 → 11, hold 3 s | the second climb |
| 6 | brake 11 → 0 onto the stop line (s = 220.4 m), run-out 10 m | planned braking, stop error |

The e-stop is raised through the actuator's own emergency input and held until the car
has been at rest for 1 s. Then the stack ramps the target back to 6 m/s at 2.5 m/s².
`part1_profile.yaml` in the course directory is the contract the scenario and the grader
both read; nothing in either hard-codes a position.

## What you see

- **RViz**: the lane, the stop line, the e-stop trigger, the car and its path, and a
  marker wherever a check failed.
- **Dashboard window**:
  - speed against time: the target, the measured speed (what your controller sees) and
    the true speed;
  - PWM against time, with neutral and the brake zone shaded;
  - distance to the stop line;
  - each failed check's window in red, the e-stop in grey;
  - the 16 checks live ("pending" until each window closes), then the marks.

  It plots exactly what the grader graded, and saves itself to `dashboard.png` in the
  run's output directory when it closes.

## The checks

Sixteen checks per run, computed live from the plant's **true** state, the target, the PWM
and the shell's mode, each over a window that `part1_profile.yaml` defines.
`autosdv_waypoint_lab/part1/metrics.py` is the definition. Its docstring gives every
window; `LIMITS` has the numbers.

| Check | Window | Meaning | Limit |
|---|---|---|---|
| `hold_rmse_2`, `hold_rmse_6`, `hold_rmse_11` | the first hold at each speed, from 1 s in | RMSE of true speed minus the hold speed | 0.05 / 0.05 / 0.13 m/s |
| `overshoot` | the three steps up | peak above the hold speed, % of the step | 7.2 % |
| `reach_s` | the same steps | time from entering the hold until within 10 % of the step | 0.2 s |
| `launch_slip` | 1 s from the first motion | max rear-wheel slip (spin) [m/s] | 0.5, physical |
| `launch_accel` | the same | max acceleration over 50 ms (a lurch) | 4.5 m/s² |
| `stepdown_undershoot` | 11 → 6 braking and the hold until the e-stop | lowest speed below 6, % of the step | 2 % |
| `stepdown_kappa` | the same, speed > 0.3 m/s | min rear slip ratio (wheel lock below −0.2) | −0.2, physical |
| `stepdown_lag` | the 11 → 6 decel, up to the e-stop | max of true speed − target: a car that cannot slow down | 0.88 m/s |
| `estop_reverse` | e-stop to the end of the resume | lowest true speed (reversing) | −0.05 m/s, physical |
| `estop_resume_slip` | from the clear until 2 s after the ramp reaches 6 m/s | max rear slip | 0.5, physical |
| `estop_resume_overshoot` | the same | peak above 6 m/s, % | 2.4 % |
| `stop_error` | the final rest | distance from the stop line [m] | 0.5, fixed |
| `stop_kappa` | final braking, speed > 0.3 m/s | min rear slip ratio | −0.2, physical |
| `hold_rmse_11b` | the second 11 m/s hold | RMSE | 0.13 m/s |

**Limits.** The physical limits and the stop error are fixed. The others are
*calibrated* by the course staff on a reference run of the course. Each run prints the limit it was graded against.

**Validity.** A run is invalid, and scores 0 of 16 with the reason printed, only if:

- the clock handshake was invalidated (`lockstep invalidated: ...`);
- the controller faulted the shell (`controller fault: ...`);
- the actuator never started (`the actuator is not running: ...`): the class did not
  import or its `__init__` raised, so nothing of it ran;
- an MRM was raised (`MRM: ...`);
- the scenario aborted;
- the e-stop was never raised;
- the stack never engaged.

A car that never moves, or stops early, is a **valid** run whose unreached checks fail.
The grader says so in plain words: *"Your car did not move ... Is your speed_pid.py
still the stub?"*

**Marks.** An everyday run shows passed/16 on the nominal car. `--final` writes
`final.json` with `marks = 25 × sum(passed) / sum(total)` over the three runs (48
checks); an invalid run counts 0 of its 16.

## Outputs

`tmp/lab2/part1/<submission name>/<variant>/`:

- `trace.csv` (100 Hz: `t, s, y, target, measured, speed, pwm, mode, slip_r, kappa_r,
  emergency, segment`);
- `marks.json` (`variant, valid, invalid_reason, passed, total, checks`);
- `run.json` (the command, the timings, the `play_log` path);
- `dashboard.png` (the dashboard as it was when it closed; with the dashboard only);
- `play_log/` (every node's log).

`--final` adds `final.json` beside them.

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
  spins the rear wheel at launch and at the e-stop resume (`launch_slip`,
  `estop_resume_slip`). The sensor is on a driven wheel, so a spinning wheel also reads
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
| `warning: no DISPLAY: running without RViz and the dashboard` | an SSH session without X; the run is the same. Use a desktop session or VNC to see the windows. |
| `WARNING: no submission given: running the STUB` | no `submission:=` and no `part1_submission` in `~/lab2/lab2.yaml` |
| `error: submission DIR: submission error: .../speed_pid.yaml: kp is ...` | a key misspelled, out of range, or the wrong type; the message names it |
| "Your car did not move ... still the stub?" | `update()` returns neutral: the TODOs are not done, or the output never leaves `init_pwm` |
| invalid: `controller fault: ...` | `update()` raised or returned a non-number; the traceback is in `play_log/` |
| invalid: `the actuator is not running: ...` | the actuator died at start-up: `speed_pid.py` does not import, or `SpeedPID.__init__` raised; the traceback is in `play_log/` |
| invalid: `lockstep invalidated: ...` | the simulation lost a participant (a node crashed or hung); run again, and report it if it repeats |
| invalid: `MRM: ...` / `the stack did not engage` | a stack fault, not your controller; run again; report it with `run.json` |
| the car stops about 9 m past the line | no braking mode: regeneration alone cannot follow 2.5 m/s² |
| `estop_resume_slip` or `launch_slip` fails | too much push while the measured speed is still 0 (see the hall sensor above) |

