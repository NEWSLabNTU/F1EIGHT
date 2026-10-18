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
`config/courses/lab2_part1`). Planner limits a = d = 2.5 m/s^2 (3.5 m/s^2 braking was
tried and is above every variant's full brake, 2.98-3.36 m/s^2). Segments in order:

| # | Segment | Purpose |
|---|---|---|
| 1 | accel 0 → 2, hold 2 m/s for 3 s | launch from rest (no spin, no lurch), low-speed hold |
| 2 | accel 2 → 6, hold 3 s | step response, hold |
| 3 | accel 6 → 11, hold 3 s | full-throttle authority, hold at 40 km/h |
| 4 | decel 11 → 6, hold 8 s; **e-stop** triggered 15 m into this hold | active braking to a lower speed; emergency stop and clean resume |
| 5 | accel 6 → 11, hold 3 s | |
| 6 | decel 11 → 0 onto the stop line, run-out 10 m | planned braking, stop error |

The e-stop trigger is a position in segment 4: 15 m (2.5 s at 6 m/s) into the hold, so the
step-down transient has settled before it fires, and the rest of the 8 s hold (33 m) leaves
room to stop, wait 1 s and resume onto 6 m/s before the next climb. The scenario node
raises the emergency through the same input the
car uses (the actuator's `~/input/emergency`, `VehicleEmergencyStamped`), holds it until
the car is at rest for 1 s, then clears it. The resume target is ramped from 0 at
2.5 m/s^2 up to the path's speed (6 m/s): a step would spin the rear wheels, and the hall
sensor on the driven wheel cannot see that. The ramp is applied by the scenario node
upstream of the command gate (path_tracker publishes to `/control/part1/tracker_cmd`, the
scenario node forwards it), so it appears in `control_cmd` where the grader reads the
target. The gate's emergency output is remapped and OR-ed with the e-stop by the same
node before it reaches the actuator.

`config/courses/lab2_part1/part1_profile.yaml` is the contract between the course
generator, the scenario node and the grader:

```yaml
speeds: [2.0, 6.0, 11.0, 6.0, 11.0]   # the hold speeds, in order
holds: [3.0, 3.0, 3.0, 8.0, 3.0]
accel: 2.5
decel_brake: 2.5
runout: 10.0
engage_velocity: 0.25
segments:   # s: metres along the path from the start pose; t0/t1 the nominal plan from engage
  - {kind: accel, s0: 0.0, s1: 0.8, v0: 0.0, v1: 2.0, rate: 2.5, group: 1, t0: 0.0, t1: 0.8}
  - {kind: hold,  s0: 0.8, s1: 6.8, v0: 2.0, v1: 2.0, rate: 0.0, group: 1, t0: 0.8, t1: 3.8, hold_index: 0}
  # ... kinds accel | decel | hold | brake | runout; group = the row of the table above
stop_line_s: 220.4
length: 230.4
estop_hold_index: 3
estop_trigger_s: 113.2     # duplicate of estop.trigger_s kept for simple readers
estop: {hold_index: 3, hold_s0: 98.2, hold_s1: 146.2, speed: 6.0, trigger_s: 113.2,
        rest_time: 1.0, resume_accel: 2.5}
nominal_duration: 32.8
```

## Clock synchronization (load-independent within a tolerance)

The plant owns `/clock` and is the only node not on simulated time. Every node whose
timing decides a graded number is a *participant*: it acknowledges each of its timer
deadlines after the work for that deadline is done, and the plant never publishes a
`/clock` past a deadline it has no acknowledgement for. Under load the simulation slows
(real-time factor below 1) instead of leaving the controller behind; nothing is skipped
and nothing acts on stale input. The rest of the stack runs on the same clock at its own
rates. Implementation: `autosdv_rc_sim/lockstep_sync.py` (the plant's side, no ROS),
`plant_node.py`, and `autosdv_vehicle_interface/hal.py` (`format_ack`, `ReportGate`,
`LockstepParticipant`); all of it exists only under `hardware:=sim`, real hardware never
publishes or waits.

| Participant | Period | Acknowledges after |
|---|---|---|
| `velocity_report` | 50 ms | publishing its report; it holds a report until the hall edges up to its deadline are in (each edge message is marked complete up to the `/clock` that follows it) |
| `actuator` | 10 ms | writing its PWM; it holds the tick behind three gates (below) |
| `path_tracker` | 33.3 ms | publishing the control command for the odometry of its instant (it waits for that odometry) |
| `emergency` (`lab2_part1_scenario`'s 20 Hz e-stop timer) | 50 ms | publishing the emergency state, stamped with the deadline; the scheduled e-stop and clear apply at the first tick at or after their mark |

- **Acks:** `std_msgs/String` on `/rc_sim/lockstep/ack`, `"<name> <sec> <nanosec>
  <period_ns> [key=value ...]"`. The plant learns each participant's period and phase from
  its first ack; nothing is configured. Extras record what was used: the actuator sends
  `report=`, `cmd=`, the gate flags `gate`/`cgate`/`egate`, and the PWM it wrote (the
  plant applies the PWM from the ack, so a PWM message overtaken by its ack cannot leave
  the plant on the previous value); `velocity_report` sends `late_edges=`.
- **The actuator's gates** order equal instants, which the plant cannot (both nodes see
  the same `/clock`). A tick is held until it has (1) the velocity report of
  `velocity_report`'s latest deadline at or before the tick, (2) the control command
  stamped with `path_tracker`'s latest deadline at or before it (the forwarder and
  `vehicle_cmd_gate` keep the stamp), and (3) the emergency message of the `emergency`
  participant's latest deadline. Each gate arms only once its participant has
  acknowledged, and only while the plant's status says the handshake is on and ok. The
  command and emergency gates share a **0.5 s wall guard**: no matching message within it
  disarms the gate with a warning (the next message re-arms it), so a missing upstream
  node never stalls the actuator. Stale inputs are counted in the plant's status
  (`stale_reports`, `stale_commands`); both are 0 in every validated run.
- **The plant's limit:** the furthest `/clock` it may publish is the earliest pending
  deadline, rounded up to the 1 ms physics step (the tracker's 33.3 ms deadlines are off
  the grid). At the limit it waits for the ack.
- **Timeout:** a participant that leaves a pending deadline unacknowledged for
  `lockstep_timeout` (default **2 s wall**) times out: `/rc_sim/lockstep/status` (1 Hz, JSON)
  reports `ok: false` with the reason for good, and the plant stops waiting for that
  participant until it acknowledges again, so a dead node never hangs the simulation.
  The grader makes the run invalid: `lockstep invalidated: ...` after the engage, and
  `the simulator stalled before the run started (machine too slow / suspended?); run
  again` before it (nothing of the student's has run yet; the runner exits non-zero and
  prints it).
- **Aligned start:** after the engage the scenario holds the speed passed to the gate at 0
  until the next multiple of 0.1 s (at least 0.3 s later), a multiple of all the loop
  periods, so every run starts at the same loop phase.
- `just sim rc` keeps working (handshake on or off with `lockstep:=`; its smoke test passes).

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
| reading params, output clamp to [min_pwm, max_pwm], interface to the shell, the derivative's low-pass filter helper, the feed-forward constants and formula, the brake band mapping `brake_pwm()` (skips the ESC's 5-count dead band) | the error and the driving error clamped to `error_limit`; P (on the clamped error); D on the measurement (filtered); the **braking mode**: entered when the car is faster than the target by more than `brake_enter`, left below `brake_exit` (hysteresis), feed-forward and integral set aside, request −(kp_brake·e + D) into the brake band; driving: I with the integral clamp and conditional integration (anti-windup), FF + PID limited to [0, max_pwm_offset] |

Why a braking mode (decided 2026-10-18): the plant's ESC regenerates only lightly (a
drag brake, `esc_regen_current_limit` 2.5 A: 1.5-2.0 m/s² with road load, against
3.0-3.5 m/s² for its brake), so following the course's 2.5 m/s² braking takes the brake
band below neutral, and a controller whose command is FF + PID only reaches below
neutral once the feedback cancels the whole feed-forward (about 60 counts at 8 m/s).

The feed-forward constants are **given** (`ff_deadband`, `ff_gain`, calibrated on a
fresh 12.4 V pack while the car runs at 11.1 V, so the integral must make up the rest),
not identified by the student. `config/speed_pid.yaml` exposes every practical parameter
of the controller, so students see and may change them:

| Key | Meaning |
|---|---|
| `kp`, `ki`, `kd` | gains [counts per m/s, per m, per m/s^2] |
| `integral_limit` | the integral's clamp [counts] |
| `error_limit` | the largest error P and I see while driving [m/s] |
| `enable_conditional_integration` | freeze the integral while the output saturates |
| `derivative_filter_alpha` | the derivative's EMA, (0, 1] |
| `max_pwm_offset` | throttle authority above neutral [counts], ≤ max_pwm − init_pwm |
| `max_brake_offset` | full brake below neutral [counts], ≤ init_pwm − min_pwm |
| `kp_brake` | the brake request's gain [counts per m/s too fast] |
| `brake_enter`, `brake_exit` | the braking mode's thresholds [m/s], exit < enter |
| `brake_deadband` | the ESC's dead band below neutral [counts] (given: 5) |
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
and the shell's mode, on the windows `part1_profile.yaml` defines. Sixteen checks per run:

| Check | Window |
|---|---|
| `hold_rmse_2`, `hold_rmse_6`, `hold_rmse_11` | steady part of the first hold at each speed |
| `overshoot` | worst over the three steps up |
| `reach_s` | worst time to enter the band after a step up |
| `launch_slip`, `launch_accel` | first second from rest (wheel spin, lurch) |
| `stepdown_undershoot`, `stepdown_kappa` | 11 → 6 decel (undershoot below 6, rear wheel lock; slip ratio only above 0.3 m/s) |
| `stepdown_lag` | 11 → 6 decel up to the e-stop: max of true speed − commanded target (a car that cannot slow down; added 2026-10-18 after the no-brake probe scored 23.33) |
| `estop_reverse`, `estop_resume_slip`, `estop_resume_overshoot` | e-stop to resumed hold |
| `stop_error`, `stop_kappa` | final braking (distance to the line where the final rest began, wheel lock above 0.3 m/s) |
| `hold_rmse_11b` | the second 11 m/s hold |

Validity (not a check): a run is invalid, scores 0 and says why, only for a lockstep
timeout (before the engage: a startup stall, "run again"), a controller fault, an actuator that never started
(no mode 2 s after the engage), an MRM, a
scenario abort, an e-stop that was never raised, or a stack that never engaged. A car that
does not move, stops early or does not finish in time has a valid run whose unreached
checks fail, with a message saying so.

Limits follow the RC simulator's rule: calibrated limits are 1.5× the reference
controller's worst case across the graded variants, physical limits (spin, lock,
reversing) and the stop error are fixed. `scripts/testing/lab2/part1_calibrate.py`
reproduces them.

**Marks:** an everyday run shows passed/16 on the nominal car. `--final` runs nominal,
`low_batt` and `low_mu` in sequence; Part 1's run mark is 25 × passed / 48. The TA
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

## Validation (2026-10-18)

**Load tolerance** (`scripts/testing/lab2/part1_tolerance.py`, the reference, 3 quiet + 3
loaded runs interleaved, busy loops on every core ×2; `tmp/lab2/part1/agents/P2-report.md`
round 5): **PASS**, all 6 runs valid 15/15 (before `stepdown_lag`). Quiet runs are bit-identical on every check.
The largest spreads: `stepdown_undershoot` 0.10 % (allowed 0.5), `stop_error` 0.003 m
(allowed 0.03), `hold_rmse_11b` 0.0012 m/s (allowed 0.011); every other check far inside
its allowance (at most 0.004 against 0.5 for `estop_resume_overshoot`).
`stepdown_lag`, evaluated afterwards on the same six traces: 0.5416-0.5481 m/s, spread
0.0065 (allowed max(2 % of 0.88, 0.01) = 0.018).

**Calibration** (`scripts/testing/lab2/part1_calibrate.py --repeats 2`: the reference live on
nominal, low_batt and low_mu, 6 runs, all passing, repeats bit-identical). Limit = 1.5 × the
worst case, 2 significant figures up, never below the floor in `part1_calibrate.py`
(hold RMSE 0.05 m/s, reach 0.2 s, undershoot and overshoots 2 %). Fixed limits unchanged.

| Check | Reference worst | Limit | | Check | Reference worst | Limit |
|---|---|---|---|---|---|---|
| `hold_rmse_2` | 0.023 m/s | 0.05 (floor) | | `launch_accel` | 2.94 m/s² | 4.5 |
| `hold_rmse_6` | 0.028 m/s | 0.05 (floor) | | `stepdown_undershoot` / `stepdown_lag` | 0.55 % / 0.586 m/s | 2 (floor) / 0.88 |
| `hold_rmse_11` | 0.085 m/s | 0.13 | | `estop_resume_overshoot` | 1.59 % | 2.4 |
| `hold_rmse_11b` | 0.084 m/s | 0.13 | | `launch_slip`, `estop_resume_slip` | 0.11, 0.27 m/s | 0.5 (physical) |
| `overshoot` | 4.77 % | 7.2 | | `stepdown_kappa`, `stop_kappa` | −0.074 | −0.2 (physical) |
| `reach_s` | 0.07 s | 0.2 (floor) | | `estop_reverse` / `stop_error` | 0.000 m/s / 0.066 m | −0.05 / 0.5 (fixed) |

**Separation** (`part1_run.py --final`, 16 checks, the calibrated limits, variants of the
reference's parameters; `tmp/lab2/part1/probes16/`; every run valid):

| Controller | Marks /25 | Fails (cars of 3) |
|---|---|---|
| reference | 25.00 | — |
| stub (default) | 0.00 | every check, "Your car did not move ... still the stub?" |
| P-only (`ki` 0, `kd` 0) | 15.62 | `hold_rmse_2/6/11/11b`, `reach_s`, `stepdown_undershoot` (3 each) |
| no brake (`max_brake_offset` 0) | 21.88 | `stepdown_lag` (2.5-2.6 m/s), `stop_error` (8.9-9.2 m) |
| no anti-windup (`enable_conditional_integration` false, `integral_limit` 1000) | 25.00 | none (accepted: report question 1 has students provoke windup) |
| aggressive (`kp` 30, `ki` 40) | 21.35 | `launch_slip`, `estop_resume_slip` (3), `hold_rmse_11b` (1) |

`stepdown_lag` (16th check, added after the 15-check no-brake probe scored 23.33) is
calibrated from the same six reference traces: worst 0.586 m/s (low_mu), limit 0.88.

**Coverage** (nominal car, `tmp/lab2/part1/coverage/`): which checks a parameter mistake
can fail.

| Probe | Passed /16 | Fails |
|---|---|---|
| strong brake (`kp_brake` 60, `brake_deadband` 0, `brake_enter` 0.1) | 12 | `hold_rmse_6/11/11b`, `stepdown_undershoot` 13 % |
| underdamped (`kp` 25, `ki` 60, `kd` 0) | 12 | `overshoot` 9.0 %, `launch_slip`, `stepdown_undershoot`, `estop_resume_slip` |
| lurch (`kp` 60, `error_limit` 20) | 9 | holds, `reach_s`, `launch_slip` 2.2, `estop_resume_slip` |
| windup (`error_limit` 20, no conditional integration, `integral_limit` 1000, `ki` 40) | 14 | `overshoot` 11.3 %, `stepdown_undershoot` |
| resume kick (`kp` 40, `ki` 80, `error_limit` 3, no conditional integration) | 8 | holds, `reach_s`, slips, `stepdown_undershoot` |
| heavy feed-forward (`ff_deadband` 20, `ff_gain` 0.09) | 8 | holds, `reach_s`, slips, `stepdown_undershoot` |

Never failed by any probe (worst value reached across all 30 probe runs):

- `launch_accel` (3.85 m/s² vs 4.5) and `estop_resume_overshoot` (1.94 % vs 2.4): traction
  caps the acceleration, so a hard launch spins the wheel and fails `launch_slip` first; the
  stack ramps the resume target at 2.5 m/s², so the car follows a ramp, not a step.
- `stepdown_kappa`, `stop_kappa` (−0.077 vs −0.2): the brake authority a student may set is
  at most 10 counts below neutral (`max_brake_offset`), which cannot lock the rear wheel on
  any graded car.
- `estop_reverse` (0.000 vs −0.05): during the e-stop the actuator's override drives the
  motor, not the student's controller.

These remain as physical-safety guards (they would catch a plant or shell change, or a
controller that escapes its limits), as the ruling accepts.

## Not in scope

The path tracker (TA's, unchanged), curves, Part 2's EKF and markers, feed-forward
identification, the headless grader's role as an official grade (it stays as a fast
developer check: `just sim grade`).
