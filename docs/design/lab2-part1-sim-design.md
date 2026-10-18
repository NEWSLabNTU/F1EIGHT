# Lab 2 Part 1: speed and brake control on the simulated car

Decided 2026-10-18. Supersedes Part 1 of `docs/design/rc-sim-design.md` §"What students
do" (tune `AckermannSpeedPID` coefficients, headless grade) and the Lab 2 Part 1
"calibration + speed bench" of the course repository. The RC simulator's plant, its
model and its headless harness stay; what changes is the stack around them and what
students write.

## What a student does

```bash
just lab2 sim submission:=~/lab2/B12345678_lab_2_1            # one run, live, ~1.5 min
just lab2 sim submission:=~/lab2/B12345678_lab_2_1 --final    # the graded set, ~5 min
```

One command builds the course, starts the Lab 2 stack on the simulated car, opens RViz
and a dashboard window, engages, drives the run, grades it, shows the marks, and stops
everything. The student edits `speed_pid.py` and `config/speed_pid.yaml`, runs again,
and improves the controller from what the curves and the violations show. Planning and
steering are not theirs: steering only keeps the car on the straight.

Setup is the repository's standard one: `./setup.sh`, then `just build`. Nothing else
is installed or configured by hand.

## The stack (Approach 1: the Lab 2 bench's parts on the RC simulator's car)

```
just lab2 sim  ─ runner: course, setsid stack, wait for the result, print marks, kill group

waypoint_path_publisher (part1 straight course, v column = speed profile) ─► /planning/trajectory
kinematic_state + trajectory ─► path_tracker (TA reference pure pursuit; speed = path speed here)
  ─► vehicle_cmd_gate ─► /control/command/control_cmd (+ emergency_cmd)
  ─► actuator (hardware:=sim): safety shell + STUDENT SpeedPID   (loaded by import path)
  ─► /rc_sim/pwm ─► rc_sim_plant  (1 kHz physics, owns /clock, tick handshake)
  ─► /rc_sim/hall_edges ─► velocity_report ─► /vehicle/status/velocity_status ─► actuator
rc_sim_plant ─► /localization/kinematic_state + TF map→base_link (TRUE pose), /rc_sim/state
lab2_part1_scenario: engage; e-stop injection at the course's trigger; release; resume
lab2_part1_grader:   live checks ─► /lab2/part1/status, /lab2/part1/violations, trace + marks.json
lab2_part1_dashboard (window): curves, violation spans, final marks    RViz: car, lane, marks
```

- **Localization is the plant's true pose.** The stop error then measures control, not
  localization noise. The EKF and the virtual markers return in Part 2.
- **Planning** is the Lab 2 waypoint path publisher with a Part 1 parameter file
  (`v_max` 11.5 m/s, stop ramp `decel` 2.5 m/s^2), reading a generated straight course.
- **Control** is the TA's pure pursuit (steering) and the path's speed as the target, the
  command gate, and the car's actuator with the student's `SpeedPID`.
- Reused from `waypoint_follow_bench.launch.xml` wherever possible; the bench plant,
  the EKF and the marker localizer are replaced by the RC simulator's plant.

## The course and the run

One straight, generated (`just lab2 make-part1-course`, default written to the package's
`config/courses/lab2_part1`). Planner limits a = d = 2.5 m/s^2. Segments in order:

| # | Segment | Purpose |
|---|---|---|
| 1 | accel 0 → 2, hold 2 m/s for 3 s | launch from rest (no spin, no lurch), low-speed hold |
| 2 | accel 2 → 6, hold 3 s | step response, hold |
| 3 | accel 6 → 11, hold 3 s | full-throttle authority, hold at 40 km/h |
| 4 | decel 11 → 6, hold 4 s; **e-stop** triggered in this hold | active braking to a lower speed; emergency stop and clean resume |
| 5 | accel 6 → 11, hold 3 s | |
| 6 | decel 11 → 0 onto the stop line, run-out 10 m | planned braking, stop error |

The e-stop trigger is a position in segment 4, chosen so the nominal car is mid-hold at
6 m/s when it fires. The scenario node raises the emergency through the same input the
car uses (the actuator's `~/input/emergency`, `VehicleEmergencyStamped`), holds it until
the car is at rest for 1 s, then clears it; the path's speed there (6 m/s) is the resume
target.

`config/courses/lab2_part1/part1_profile.yaml` is the contract between the course
generator, the scenario node and the grader:

```yaml
speeds: [2.0, 6.0, 11.0, 6.0, 11.0]   # the hold speeds, in order
holds: [3.0, 3.0, 3.0, 4.0, 3.0]
accel: 2.5
decel: 2.5
runout: 10.0
segments:          # s in metres along the path from the start pose; kind accel|decel|hold|brake|runout
  - {kind: accel, s0: 0.0, s1: 0.8, v0: 0.0, v1: 2.0}
  # ...
stop_line_s: <m>
estop_trigger_s: <m>
```

## Clock synchronization (load-independent within a tolerance)

The plant owns `/clock`. The speed loop — plant → `velocity_report` → actuator/`SpeedPID`
→ PWM → plant — runs in lockstep: before the plant advances simulated time past a
participant's next deadline it waits until that participant has acknowledged its
previous one. Under load the simulation slows (real-time factor below 1); the controller
never misses a tick and never acts on a stale speed. The rest of the stack runs on the
same clock at its own rates.

- Participants acknowledge on `/rc_sim/lockstep/ack` (`builtin_interfaces/Time` inside a
  small message, or `std_msgs/String` `"<node> <sec> <nanosec>"`, implementer's choice,
  documented in the plant), published in `hardware:=sim` only, after the work for that
  deadline is done: `velocity_report` after publishing its report, the actuator after
  writing its PWM. Real hardware never publishes or waits.
- The plant learns each participant's period and phase from its acks and advances the
  clock deadline by deadline in time order; at equal instants `velocity_report` goes
  first.
- A missing ack for `lockstep_timeout` (default 2 s wall) invalidates the run (reported
  on `/rc_sim/lockstep/status` and by the grader), never hangs it.
- `just sim rc` keeps working (handshake on or off by parameter; the smoke test must pass).

**Acceptance:** the same submission run 3 times on a quiet host and 3 times with busy
loops on every core gives the same verdict on every check, for the reference controller
and for two degraded ones, wherever a metric is more than 3 % away from its limit; each
metric's spread is within max(2 %, its absolute floor). The test is a script
(`scripts/testing/lab2/part1_tolerance.py`) and its numbers go in the design's
validation section.

## What students write

A `SpeedPID` skeleton modelled on the car's `AckermannSpeedPID`, with the crucial parts
as TODOs and everything else given:

| Given | TODO |
|---|---|
| reading params, output clamp to [min_pwm, max_pwm], interface to the shell, the derivative's low-pass filter helper, the feed-forward constants and formula | the error; P; I with the integral clamp and conditional integration (anti-windup); D on the measurement (filtered); the **brake branch**: a command below neutral maps to [init_pwm − max_brake_offset, init_pwm), never past `min_pwm` |

The feed-forward constants are **given** (`ff_deadband`, `ff_gain`, matched to the
nominal model), not identified by the student. `config/speed_pid.yaml` exposes every
practical parameter of the production controller, so students see and may change them:

| Key | Meaning |
|---|---|
| `kp`, `ki`, `kd` | gains [counts per m/s, per m, per m/s^2] |
| `integral_limit` | the integral's clamp [counts] |
| `enable_conditional_integration` | freeze the integral while the output saturates |
| `derivative_filter_alpha` | the derivative's EMA, (0, 1] |
| `max_pwm_offset` | throttle authority above neutral [counts], ≤ max_pwm − init_pwm |
| `max_brake_offset` | brake authority below neutral [counts], ≤ init_pwm − min_pwm |
| `ff_deadband`, `ff_gain` | the feed-forward (given; changing them is allowed but explained in the report) |

The interface is the actuator's plug-in contract (`speed_controller.py`): `__init__(params)`,
`reset()`, `update(target_speed, measured_speed, dt) -> int`. Loaded by import path with
`speed_controller_class`; `submission:=DIR` resolves the package, the class and the
parameters (`scripts/testing/lab2/submission.py`, extended for the Part 1 layout).

The TA reference implementation lives in AutoSDV under `autosdv_waypoint_lab/reference/`
and is removed from the student branch; the student branch defaults to the stub, which
holds neutral.

## Checks and marks

Computed live by the grader from the plant's true state, the commanded target, the PWM
and the shell's mode, on the windows `part1_profile.yaml` defines. Fifteen checks per run:

| Check | Window |
|---|---|
| `hold_rmse_2`, `hold_rmse_6`, `hold_rmse_11` | steady part of the first hold at each speed |
| `overshoot` | worst over the three steps up |
| `reach_s` | worst time to enter the band after a step up |
| `launch_slip`, `launch_accel` | first second from rest (wheel spin, lurch) |
| `stepdown_undershoot`, `stepdown_kappa` | 11 → 6 decel (undershoot below 6, rear wheel lock) |
| `estop_reverse`, `estop_resume_slip`, `estop_resume_overshoot` | e-stop to resumed hold |
| `stop_error`, `stop_kappa` | final braking (distance to the line, wheel lock) |
| `hold_rmse_11b` | the second 11 m/s hold |

Validity (not a check): the run finished, no MRM, no controller fault, the lockstep was
never invalidated. An invalid run scores 0 and says why.

Limits follow the RC simulator's rule: calibrated limits are 1.5× the reference
controller's worst case across the graded variants, physical limits (spin, lock,
reversing) and the stop error are fixed. `scripts/testing/lab2/part1_calibrate.py`
reproduces them.

**Marks:** an everyday run shows passed/15 on the nominal car. `--final` runs nominal,
`low_batt` and `low_mu` in sequence; Part 1's run mark is 25 × passed / 45. The TA
grades by running `--final` on the submission.

## GUI

- **RViz** (Autoware's config trimmed for the lab): the lane, the stop line, the car, the
  path, violation markers placed where they happened (`/lab2/part1/violations`,
  `MarkerArray`), the e-stop trigger.
- **Dashboard window** (matplotlib in a Qt window; both come with ROS Humble's desktop
  install and are added to `setup.sh` where missing): speed vs time with the target and
  the true speed; PWM vs time with neutral and the brake zone shaded; distance to the stop
  line; violations as red spans; at the end, the marks table. It subscribes to the
  grader's status topic, so it shows exactly what was graded.
- `rviz:=false gui:=false` for TA batch runs and CI.

## Outputs

`tmp/lab2/part1/<submission name>/<variant>/`: `trace.csv` (100 Hz: t, s, y, target,
measured, true speed, PWM, mode, rear slip, emergency, segment), `marks.json`, and the
run's `play_log` path. `--final` adds `final.json` with the 25-point mark.

## Not in scope

The path tracker (TA's, unchanged), curves, Part 2's EKF and markers, feed-forward
identification, the headless grader's role as an official grade (it stays as a fast
developer check: `just sim grade`).
