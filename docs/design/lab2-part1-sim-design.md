# Lab 2 Part 1: speed and brake control on the simulated car

Decided 2026-10-18. Supersedes Part 1 of `docs/design/rc-sim-design.md` §"What students
do" (tune `AckermannSpeedPID` coefficients, headless grade) and the Lab 2 Part 1
"calibration + speed bench" of the course repository. The RC simulator's plant, its
model and its headless harness stay; what changes is the stack around them and what
students write.

## What a student does

```bash
just lab2 sim submission:=~/lab2/B12345678_lab_2_1            # one run, live, ~1 min
just lab2 sim submission:=~/lab2/B12345678_lab_2_1 --final    # the graded set, ~2 min
```

One command builds the course, starts the Lab 2 stack on the simulated car, opens RViz
and a dashboard window, engages, drives the run, grades it, shows the marks, and stops
everything. The student edits `speed_pid.py` and `config/speed_pid.yaml`, runs again,
and improves the controller from what the curves and the violations show. Planning and
steering are not theirs: steering only keeps the car on the straight.

Setup is the repository's standard one: `./setup.sh`, then `just build`. Nothing else
is installed or configured by hand.

## The stack (the car's launch, localization from the plant)

```
just lab2 sim  ─ runner: course, setsid stack, wait for the result, print marks, kill group

waypoint_path_publisher (part1 straight course, v column = speed profile) ─► /planning/trajectory
kinematic_state + trajectory ─► path_tracker (TA reference pure pursuit; speed = path speed here)
  ─► lab2_part1_scenario (test fixture: 0 until the aligned start)
  ─► vehicle_cmd_gate ─► /control/command/control_cmd (+ emergency_cmd, via the scenario)
  ─► actuator (hardware:=sim): safety shell + STUDENT SpeedPID   (loaded by import path)
  ─► /rc_sim/pwm ─► rc_sim_plant  (1 kHz physics, owns /clock, tick handshake)
  ─► /rc_sim/hall_edges ─► velocity_report ─► /vehicle/status/velocity_status ─► actuator
rc_sim_plant ─► /localization/kinematic_state + TF map→base_link (TRUE pose), /rc_sim/state
lab2_part1_scenario: engage; the aligned start (no e-stop on the course since 2026-10-19)
lab2_part1_grader:   live checks ─► /lab2/part1/status, /lab2/part1/violations, trace + marks.json
lab2_part1_dashboard (window): curves, violation spans, final marks    RViz: car, lane, marks
```

- **Localization is the plant's true pose.** The stop error then measures control, not
  localization noise. The EKF and the virtual markers return in Part 2.
- **Planning** is the Lab 2 waypoint path publisher with a Part 1 parameter file
  (`v_max` 11.5 m/s, stop ramp `decel` 2.5 m/s^2), reading a generated straight course.
- **Control** is the TA's pure pursuit (steering) and the path's speed as the target, the
  command gate, and the car's actuator with the student's `SpeedPID`.
- **The launch is the car's.** `lab2_part1_sim.launch.xml` includes
  `waypoint_follow.launch.xml` (`just lab2 vehicle`) with `localization:=ground_truth
  hardware:=sim`, and passes every Part 1 setting as an argument: the Part 1 path publisher
  and gate parameter files (`config/part1/`), rc_sim's actuator overrides, the graph
  `config/part1/part1-diagnostics.yaml` (the waypoint graph without the two marker checks),
  `use_sim_time`, path_tracker's topic and lockstep, the gate's emergency output and the
  student's `SpeedPID`. What it adds is the harness only: `rc_sim_plant`,
  `lab2_part1_scenario`, the grader, the dashboard, RViz and its lane map. A play_launch
  dump of the old composition and the new one has the same 40 nodes with the same
  parameters and remaps, apart from vehicle_info's global parameters, which the car's
  launch sets for every node, and `laps: 0` passed inline to the path publisher, which
  its file already sets (2026-10-19).
- **So Part 1 shares everything with the car but localization**; what differs is the
  car itself (the plant), the parameter files, and the harness.

### Why the scenario stays in the command path: the start

The scenario is the one harness node between the stack's nodes: path_tracker publishes
on `/control/part1/tracker_cmd` (`tracker_control_topic`), the scenario forwards it to
`/control/trajectory_follower/control_cmd`; the gate's emergency verdict goes to
`/control/command/emergency_cmd_gate` (`output_emergency_cmd`) and the scenario forwards it
to `/control/command/emergency_cmd` at once, unchanged. The car's software runs unchanged;
only the harness touches the command path.

Since the course lost its e-stop (2026-10-19, "The course and the run") the scenario stays
for the **aligned start** alone: after the engage it holds the speed passed to the gate at 0
until the 0.1 s grid. Without it `launch_slip` spread 0.027 m/s across runs, over the 0.02
the grading allows (measured with D11, below).

The e-stop is optional in `part1_profile.yaml`: `just lab2 make-part1-course --estop-hold K
--estop-offset M` writes an `estop` block, and only then does the scenario create its 20 Hz
emergency timer, join the clock handshake as participant `emergency`, raise the emergency at
the trigger, clear it after 1 s at rest and ramp the resume target at 2.5 m/s^2. Without the
block it has no emergency timer and no lockstep participant (the actuator's emergency gate
never arms, as in `just sim rc`), the grader's trace aligner waits for the command stream
alone, and "e-stop never raised" cannot invalidate a run. No check reads the e-stop.

Why an e-stop went through the scenario rather than the gate's external-emergency service
(D11 in the one-stack plan, measured 2026-10-19 on the previous course and rejected: the
scenario called `/control/vehicle_cmd_gate/external_emergency_stop` and
`clear_external_emergency_stop`, path_tracker published straight to the gate; reference
controller, 3 quiet runs and a 3 quiet + 3 loaded tolerance round; ntu-sdv-course
`2026/labs/Lab2/lab2-one-controller.md`, D11):

| | scenario (kept) | gate e-stop (D11) |
|---|---|---|
| e-stop to first emergency actuator tick | 0.01 s, every run | 0.02 or 0.06 s (4-tick spread; the car's position at it spreads 0.23 m) |
| why | the scenario's emergency timer is a lockstep participant | the gate's stop command carries a new stamp, so the actuator's command gate waits out its 0.5 s wall guard; the gate is not a participant |
| resume slip | 0.263 m/s, spread 0.0005 (quiet + loaded) | 4.34-4.58 m/s (limit 0.5), spread 0.242 vs 0.02 allowed: the gate steps back to 6 m/s in about 0.1 s, the rear wheel spins |
| `launch_slip` spread | 0 | 0.027 vs 0.02 allowed, without the aligned start |
| MRM | none | the gate's e-stop raises one (`emergency_stop_operation`), which the grader counts as an invalid run; its 10 Hz recovery spreads the release by 0.08 s |

## The course and the run

One straight, generated (`just lab2 make-part1-course`, default written to the package's
`config/courses/lab2_part1`). Planner limits a = d = 2.5 m/s^2 (3.5 m/s^2 braking was
tried and is above every variant's full brake, 2.98-3.36 m/s^2). Simplified 2026-10-19
from a five-hold course with a step down and an e-stop (32.8 s) to three segments, about
17 s of driving:

| # | Segment | s [m] | t [s] (nominal, from the engage) | Purpose |
|---|---|---|---|---|
| 1 | accel 0 → 4, hold 4 m/s for 4 s | 0-3.2, 3.2-19.2 | 0-1.6, 1.6-5.6 | launch from rest (no spin, no lurch), low-speed hold |
| 2 | accel 4 → 11, hold 4 s | 19.2-40.2, 40.2-84.2 | 5.6-8.4, 8.4-12.4 | the speed change: full-throttle authority, hold at 40 km/h |
| 3 | brake 11 → 0 onto the stop line, run-out 10 m | 84.2-108.4, 108.4-118.4 | 12.4-16.8 | planned braking, stop error |

The stop line is at s = 108.4 m. There is no e-stop (optional, see above). The
target reaches the gate through the scenario node (path_tracker publishes to
`/control/part1/tracker_cmd`, the scenario node forwards it), so the aligned start
appears in `control_cmd` where the grader reads the target.

`config/courses/lab2_part1/part1_profile.yaml` is the contract between the course
generator, the scenario node and the grader:

```yaml
speeds: [4.0, 11.0]   # the hold speeds, in order
holds: [4.0, 4.0]
accel: 2.5
decel_brake: 2.5
runout: 10.0
engage_velocity: 0.25
segments:   # s: metres along the path from the start pose; t0/t1 the nominal plan from engage
- {kind: accel, s0: 0.0, s1: 3.2, v0: 0.0, v1: 4.0, rate: 2.5, group: 1, t0: 0.0, t1: 1.6}
- {kind: hold, s0: 3.2, s1: 19.2, v0: 4.0, v1: 4.0, rate: 0.0, group: 1, t0: 1.6, t1: 5.6, hold_index: 0}
- {kind: accel, s0: 19.2, s1: 40.2, v0: 4.0, v1: 11.0, rate: 2.5, group: 2, t0: 5.6, t1: 8.4}
- {kind: hold, s0: 40.2, s1: 84.2, v0: 11.0, v1: 11.0, rate: 0.0, group: 2, t0: 8.4, t1: 12.4, hold_index: 1}
- {kind: brake, s0: 84.2, s1: 108.4, v0: 11.0, v1: 0.0, rate: 2.5, group: 3, t0: 12.4, t1: 16.8}
- {kind: runout, s0: 108.4, s1: 118.4, v0: 0.0, v1: 0.0, rate: 0.0, group: 3, t0: 16.8, t1: 16.8}
stop_line_s: 108.4
length: 118.4
nominal_duration: 16.8
# with --estop-hold/--estop-offset only: estop_hold_index, estop_trigger_s and
# estop: {hold_index, hold_s0, hold_s1, speed, trigger_s, rest_time: 1.0, resume_accel: 2.5}
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
| `emergency` (`lab2_part1_scenario`'s 20 Hz e-stop timer; **only with an e-stop in the profile**, so not on the course) | 50 ms | publishing the emergency state, stamped with the deadline; the scheduled e-stop and clear apply at the first tick at or after their mark |

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
  participant's latest deadline (never armed on the course, which has no e-stop). Each gate arms only once its participant has
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
and the shell's mode, on the windows `part1_profile.yaml` defines. Nine checks per run
(sixteen before the course was simplified on 2026-10-19):

| Check | Window |
|---|---|
| `hold_rmse_4`, `hold_rmse_11` | steady part of each hold (from 1 s in): RMSE of true speed − hold speed |
| `overshoot` | worst over the two steps up (0 → 4, 4 → 11), % of the step |
| `reach_s` | worst time to enter the 10 % band after a step up |
| `launch_slip`, `launch_accel` | first second from rest (wheel spin, lurch) |
| `stop_lag` | the final brake segment, its start to the stop line: max of true speed − commanded target (a car that cannot slow down; the former `stepdown_lag` on the final brake) |
| `stop_error`, `stop_kappa` | final braking (distance to the line where the final rest began, wheel lock above 0.3 m/s) |

Removed with the step down and the e-stop: `hold_rmse_2`, `hold_rmse_6`, `hold_rmse_11b`,
`stepdown_undershoot`, `stepdown_kappa`, `stepdown_lag`, `estop_reverse`,
`estop_resume_slip`, `estop_resume_overshoot`. No undershoot check at the stop: below a
target of 0 is reversing, which the ESC allows only after the car has stopped and sat at
neutral, and stopping short of or past the line is `stop_error`'s. Lock in the final brake
is `stop_kappa`'s, as before.

Validity (not a check): a run is invalid, scores 0 and says why, only for a lockstep
timeout (before the engage: a startup stall, "run again"), a controller fault, an actuator that never started
(no mode 2 s after the engage), an MRM, a
scenario abort, an e-stop that was never raised (only possible with an e-stop in the
profile, which the course does not have), or a stack that never engaged. A car that
does not move, stops early or does not finish in time has a valid run whose unreached
checks fail, with a message saying so.

Limits follow the RC simulator's rule: calibrated limits are 1.5× the reference
controller's worst case across the graded variants, physical limits (spin, lock,
reversing) and the stop error are fixed. `scripts/testing/lab2/part1_calibrate.py`
reproduces them.

**Marks:** an everyday run shows passed/9 on the nominal car. `--final` runs nominal,
`low_batt` and `low_mu` in sequence; Part 1's run mark is 25 × passed / 27 (the denominator
is 3 × `len(metrics.CHECK_NAMES)`, never hard-coded: `part1_run.py` reads it). The TA
grades by running `--final` on the submission.

## GUI

- **RViz** (Autoware's config trimmed for the lab): the lane, the stop line, the car, the
  path, violation markers placed where they happened (`/lab2/part1/violations`,
  `MarkerArray`) (and an e-stop trigger, when a profile has one).
- **Dashboard window** (matplotlib in a Qt window; both come with ROS Humble's desktop
  install and are added to `setup.sh` where missing): speed vs time with the target and
  the true speed; PWM vs time with neutral and the brake zone shaded; distance to the stop
  line; violations as red spans (an e-stop, if any, grey); at the end, the marks table. It subscribes to the
  grader's status topic, so it shows exactly what was graded.
- `rviz:=false gui:=false` for TA batch runs and CI.

## Outputs

`tmp/lab2/part1/<submission name>/<variant>/`: `trace.csv` (100 Hz: t, s, y, target,
measured, true speed, PWM, mode, rear slip, emergency, segment), `marks.json`, and the
run's `play_log` path. `--final` adds `final.json` with the 25-point mark.

## Validation (2026-10-19, the simplified course)

**Calibration** (`scripts/testing/lab2/part1_calibrate.py --repeats 2`: the reference live on
nominal, low_batt and low_mu, 6 runs, all passing, repeats bit-identical;
`tmp/lab2/simple/calibrate/`). Limit = 1.5 × the worst case, 2 significant figures up, never
below the floor in `part1_calibrate.py` (hold RMSE 0.05 m/s, reach 0.2 s, overshoot 2 %,
stop lag 0.3 m/s). Fixed limits unchanged.

| Check | Reference worst | Limit |
|---|---|---|
| `hold_rmse_4` | 0.027 m/s (low_mu) | 0.05 (floor) |
| `hold_rmse_11` | 0.098 m/s (low_batt) | 0.15 |
| `overshoot` | 2.49 % (nominal) | 3.8 |
| `reach_s` | 0.00 s (the car is inside the band when the hold begins) | 0.2 (floor) |
| `launch_slip` | 0.125 m/s (low_mu) | 0.5 (physical) |
| `launch_accel` | 2.94 m/s² (nominal) | 4.5 |
| `stop_lag` | 0.581 m/s (low_mu) | 0.88 |
| `stop_error` | 0.062 m (low_mu) | 0.5 (fixed) |
| `stop_kappa` | −0.075 (low_mu) | −0.2 (physical) |

Against the previous course, `overshoot` tightened from 7.2 to 3.8 (its worst case had been
a lower step) and `hold_rmse_11` loosened from 0.13 to 0.15 (a 4 s hold instead of 3 s).

**Load tolerance** (`scripts/testing/lab2/part1_tolerance.py`, the reference, nominal, 3 quiet
+ 3 loaded runs interleaved, busy loops on every core ×2; `tmp/lab2/simple/tolerance/`):
**PASS**, all 6 runs valid 9/9, quiet runs bit-identical, the same verdict everywhere. Spreads
against what is allowed: `stop_lag` 0.0138 m/s (0.0176, the tightest: 78 % of its
allowance, only under load), `stop_error` 0.0015 m (0.03), `stop_kappa` 0.00076 (0.01),
`hold_rmse_11` 0.00046 m/s (0.01); `hold_rmse_4`, `overshoot`, `reach_s`, `launch_slip`,
`launch_accel` 0.

**Marks** (`part1_run.py --final`, `tmp/lab2/simple/`; every run valid):

| Controller | Marks /25 | Passed /27 | Fails |
|---|---|---|---|
| reference (`config/speed_pid_part1_reference.yaml`) | 25.00 | 27 | — |
| stub (default; nominal only) | 0 | 0/9 | every check, "Your car did not move ... still the stub?" |
| reference code, detuned starting values (`config/speed_pid_part1.yaml`) | 16.67 | 18 | `hold_rmse_11` 0.32-0.41 m/s, `stop_lag` 1.68-1.78 m/s, `stop_error` 0.72-0.76 m, on every variant |

**Windup** (the report question; nominal car, reference code): the student-tuned file
(the starting values with `integral_limit` 15, `kp_brake` 15, `brake_enter` 0.35,
`brake_exit` 0.10; 9/9) plus `max_pwm_offset: 85, integral_limit: 500,
enable_conditional_integration: false` holds the command at its limit (455) through the
whole 11 m/s hold: the car cannot reach 11 m/s at 85 counts and tops out at 10.18 m/s
(`hold_rmse_11` 0.82 and `reach_s` fail, 7/9). On this course nothing follows the hold but
the final brake, and the braking mode sets the integral aside, so the wound integral shows
only as the throttle staying pinned at 455 for 0.11 s after the target falls below the car
(the car 0.29 m/s over the target when the brake takes over). With conditional integration
on, the same parameters release the throttle 0.03 s before the target crosses the car's
speed; the checks are identical (7/9). The previous course's overshoot after the climb
does not occur: no larger `ki` (40, 80) or tighter `max_pwm_offset` (93, 95) produced a
saturation long enough to wind up before the 11 m/s hold.

The separation and coverage probes of 2026-10-18 (P-only, no brake, aggressive, strong
brake, underdamped, lurch, heavy feed-forward) were run on the previous 16-check course
and have not been repeated; see this file's history before the simplification.

## Not in scope

The path tracker (TA's, unchanged), curves, Part 2's EKF and markers, feed-forward
identification, the headless grader's role as an official grade (it stays as a fast
developer check: `just sim grade`).
