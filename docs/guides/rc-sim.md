# The RC car simulator (Lab 2 Part 1)

The simulator runs the car's own vehicle interface — the safety shell and the
`AckermannSpeedPID` the actuator loads — against a model of the car fast
enough to matter: a motor that tops out at about 12 m/s, rear-wheel drive
that spins under hard throttle, tyres that slide. Design:
`docs/design/rc-sim-design.md`.

## Your task

Tune the car's speed controller. Nothing is written from scratch: study how
the textbook PID becomes `AckermannSpeedPID` (sampled integral, derivative
on the measurement with a low-pass filter, output clamp, integral limit,
conditional integration) in
`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/autosdv_vehicle_interface/speed_controller.py`,
then set its coefficients. The deliverable is one file and a report.

`<your_dir>/speed_controller_params.yaml`, a flat mapping of these keys and
no others (anything else is refused):

| Key | Meaning | Production value |
|---|---|---|
| `kp_speed` | proportional gain [PWM counts per m/s] | 12.0 |
| `ki_speed` | integral gain | 10.0 |
| `kd_speed` | derivative gain, on the measured speed | 0.001 |
| `integral_limit` | the integral's clamp [counts] | 50.0 |
| `enable_conditional_integration` | freeze the integral while the output saturates | true |
| `derivative_filter_alpha` | the derivative's low-pass filter, (0, 1] | 0.15 |
| `pwm_output_alpha` | the actuator's output filter, (0, 1] — **read but not applied by today's actuator: it changes nothing** | 0.25 |
| `max_pwm_offset` | the output limit around neutral (370) [counts], 1–100 | 55 |

With the production values the controller tops out at PWM 425, about 6.4 m/s
on the model: it cannot hold 11 m/s. That is the first thing to find.

## How you are graded

Headless and deterministic: one Python process steps the car (1 kHz), the
hall sensor, `velocity_report`'s estimate, the actuator's shell and your
coefficients (100 Hz) on a simulated clock, so the same file gives the same
marks on any machine. The target is what Autoware's planner commands on the
graded straight (115.4 m): 0 → 2 → 6 → 11 m/s, 3 s holds, then a stop on
the stop line. Two runs per model variant:

- **profile** — speed holds at 2, 6 and 11 m/s (steady error, overshoot,
  time to reach), the launch from rest (no wheel spin, no lurch), the
  braking to the stop line (stop error, no wheel lock);
- **e-stop** — at 11 m/s the target drops to 0 with Autoware's emergency
  flag; the shell brakes; after the car has stopped it resumes to 6 m/s
  (no reversing, clean resume, no spin).

Eight model variants (mass, friction, battery voltage, motor constant, ESC
deadband) replace the per-car models, so coefficients tuned to one exact
model lose marks. Each of the 16 checks per variant is worth the same; marks
= 15 × passed / total over all 8 × 16 checks. The stop error is the distance
from the stop line, at most 0.5 m on every variant.

```bash
just sim grade ~/lab2/<student_id>_lab_2_1              # all variants (a minute or two)
just sim grade ~/lab2/<student_id>_lab_2_1 --variant nominal --jobs 4
```

Each line names the variant, how many checks passed, and each failure as
`check (value vs limit)`. Traces (100 Hz CSV: target, measured and true
speed, PWM, shell mode, slip) and `marks.json` are written to
`tmp/rc_sim/grade/<dir name>/`.

### Limits

| metric | limit | kind |
|---|---|---|
| `hold_rmse_rel` | 0.05 m/s per m/s (floor `hold_rmse_floor` 0.10 m/s) | calibrated |
| `overshoot_pct` | 11 % | calibrated |
| `reach_s` | 1.0 s | calibrated |
| `launch_accel` | 4.8 m/s^2 (over 50 ms) | calibrated |
| `resume_overshoot_pct` | 10 % | calibrated |
| `launch_slip` | 0.5 m/s | physical |
| `brake_kappa` | -0.2 | physical |
| `reverse_speed` | -0.05 m/s | physical |
| `resume_slip` | 0.5 m/s | physical |
| `stop_error` | 0.5 m | fixed |

The reference coefficients (kp 7, ki 43, kd 1.5, integral limit 100, derivative
alpha 0.15) are in `src/sim/autosdv_rc_sim/config/reference_speed_controller.yaml`;
`python3 scripts/testing/rc_sim/calibrate.py --jobs 4` reproduces them and the table.

Calibrated limits are 1.5 times the reference controller's worst case across the variants, never tighter than the design's targets; physical limits (wheel spin, wheel lock, reversing) and the 0.5 m stop error are not calibrated, and the reference passes each of them with at least 10 % to spare.

## Interactive mode (not graded)

```bash
just sim rc                                          # production coefficients
just sim rc submission:=~/lab2/<student_id>_lab_2_1  # yours
just sim rc track:=tmp/rc_sim/tracks/mine variant:=low_mu
```

Autoware runs in planning-simulator mode (map, planning, control, system,
API, RViz; no sensing, localization, perception or GPU). The car's vehicle
interface runs unchanged with `hardware:=sim`: the actuator's PWM goes to the
plant instead of the PCA9685, and `velocity_report` times the plant's hall
edges instead of a GPIO pin. The plant publishes `/clock`; every other node
uses simulated time. In RViz, put a 2D Goal Pose at the stop line and press
Engage. Autoware's velocity planner decides the speeds, as on the car, but
with the planning caps raised for this run only: `max_vel` 4.17 -> 11.5 m/s and
acceleration/deceleration 1.0 -> 2.5 m/s^2 (jerk 1.0 -> 5.0), from the package's
`config/common.param.yaml`, a copy of upstream's `common.param.yaml`.
`rc_sim.launch.xml` composes planning through `rc_sim_planning_component.launch.xml`
(upstream's component with that one path changed), because `autoware.launch.xml`
exposes no argument for it; the car's own launch keeps 4.17 and 1.0. So the run
reproduces the lab's 11 m/s straight (the smoke test requires a peak of at least
9 m/s and a stop within 1 m of the stop line; measured 0.02 m short). Separate processes on a
simulated clock race within a tick, so results are close to, but not exactly,
the graded run.

`just sim make-track --speeds 2,6,11 --hold 3 --accel 2.5 --decel 2.5` prints
the segment table and writes a track for `track:=`.

## Seeing a graded run

```bash
just sim grade ~/lab2/<student_id>_lab_2_1 --view                     # nominal profile run
just sim grade DIR --view --view-variant low_mu --view-scenario estop
```

Grades first (headless), then replays the chosen run at real time in
Autoware's RViz: the straight's lanelet map, the vehicle model moving along
it, and every failed check marked where it happened. The replay publishes
the trace itself, so what you see is exactly what was graded. Target,
measured and true speed, PWM and slip are on `/rc_sim/replay/values` (plot
them in PlotJuggler).

## What it costs

Measured for `just sim rc` without RViz, pinned to four CPUs
(`smoke.py --taskset 0-3 --resources`), against the 4-core, 8 GB, no-GPU
laptop the simulator is sized for (budget: under 3 cores and 4 GB):

| | measured | budget |
|---|---|---|
| Peak memory | 2.6 GiB | 4 GB |
| CPU, steady | ~1.6 cores on a quiet host (load 12-21; max 1.9), 3.0-3.4 with the host's load at 22-37 | 3 cores |
| `rc_sim_plant` alone | 0.19 core (0.30 under that load) | 0.2 core |

Raising `clock_period` from 0.005 to 0.01 to save `/clock` traffic was tried
twice and the interactive run crept at about 0.15 m/s both times. The cause
turned out to be the emergency flap below, which crept the unmodified stack too,
so that rejection was not about `clock_period`; the lever was not re-measured and
stays at the default.

### The emergency flap (fixed) and the creep

Until Task 18b every `just sim rc` run, passing ones included, logged
`vehicle_cmd_gate: Emergency!` about once a second and `mrm_handler` toggled
EMERGENCY_STOP for 0.1 s each time, braking the actuator. The cause: the
diagnostic graph (`autoware-main.yaml`) gives `/autoware/localization/state`,
`/autoware/planning/routing/state` and `/adapi/mrm_request/delegate` the default
1.0 s timeout, and the `/adapi` nodes publish them every 1.0 s. On the plant's
simulated clock each publish is stamped on a whole second and the aggregator's
10 Hz tick falls on the same simulated instant, so every second the tick reads
the previous stamp at exactly age 1.0 s before the new one arrives, the unit
reads STALE for one tick, `autonomous` drops out of the graph and the MRM
handler stops the car. Upstream's CARLA graph (`autoware-carla.yaml`) sets
`timeout: 3.0` on `localization/state` and the MRM delegate too, but drops
`routing/state` altogether, and its stated reason is network latency and slow
update rates, not tick alignment: a loose precedent for the remedy, not for the
cause. rc_sim now passes its own graph,
`config/diagnostics/rc_sim_diagnostics.yaml` (autoware-main.yaml with the three
timeouts at 3.0; `localization.yaml` and `planning.yaml` are copies of
upstream's with the one line added; re-diff after an Autoware upgrade). The car's
own launch is untouched. After the change a run shows no MRM state change and no
loss of autonomous availability, and `smoke.py` asserts both (before: 48 MRM
changes and 46 availability drops in one run; after: 0 and 0, three runs of
three).

The creep, the car engaged but reaching only 0.15 m/s, was the flap, which
reproduced it on demand (`vmax` 0.14 on the first run of this task with the old
stack and default coefficients).

Whether the stock `just sim planning` (wall clock) has the same flap was not
measured. The lock needs the publisher and the aggregator on one stepped
simulated clock, so each 1 Hz stamp and each 10 Hz tick fall on the same
instant; on the wall clock their phases drift apart. Treat the 3.0 s timeouts
as specific to rc_sim's clock until someone runs the planning simulator
with the diagnostics logged.

### The stop overshoot (fixed): the hall speed saturated

With the flap gone the car overshot the stop line by 8-22 m (4-5 m with
`integral_limit` 60), although the grader stops the same controller within
0.5 m. It was not integrator windup. Recorded interactive run
(`/control/command/control_cmd`, `/vehicle/status/velocity_status`, the true
speed, `/rc_sim/pwm`): Autoware's profile matched the graded one (peak 11 m/s,
falling at 2.5 m/s^2), but `velocity_status` read at most about 8 m/s while the
car ran at 12, so the PID, seeing a car 4-6 m/s too slow, held PWM at 470
through the braking.

`VelocityEstimator` caps its estimate at `1.5 * marker_spacing / (now - last
edge)`; at 0.0275 m per marker that is 8.3 m/s when the last edge is 5 ms old.
The plant published `/clock` for a batch before the edges of that batch, so at
speed the clock always led the newest edge it could see by up to `clock_period`
(5 ms) and the cap cut in. The car has no such gap (it times an edge on arrival
with the same clock), and the lockstep grader has none either.
`rc_sim_plant` now steps the batch, then publishes the edges, then `/clock`, then
the stamped data, so an edge reaches `velocity_report` before the tick that
covers it. The same run stops at x 105.38 (line 105.40) with the reference
coefficients (`integral_limit` 100), and `smoke.py` now uses exactly those
and requires a stop within 1 m. Residual: the order is a publication order,
not a delivery guarantee across topics; a loaded host could in principle
reorder it, and the symptom would be the same overshoot.

Grading needs only Python: one variant takes about 4 s on one core (all eight in
32 s); `--jobs` runs variants in parallel with identical results (`--jobs 4`:
9 s, byte-identical `marks.json`).

## Calibrating the model against the car

Most of `vehicle.yaml` is a guess until the car has been measured. Take the
measurements in the order the notes in that file give them:

1. Weigh the car: `mass`.
2. Find its balance point: `lf`.
3. Do a coast-down at neutral on the flat, log `t,v` to a CSV (for example from
   a bag's `velocity_status`), and run `just sim fit-coastdown run.csv`. It
   prints the implied `rolling_resistance` and `aero_drag`.
4. Run a skidpad or tilt test for `mu`; `autosdv_rc_sim.fit.skidpad_mu(radius,
   speed)` gives it from the speed at which the car starts to slide on a circle.
5. Fit the motor with `just coach fit-motor` to replace the **mock**
   `motor_kv` and `gear_ratio`.

Each refitted value changes its tag to `measured` and gets a `note` with the
date and the car.
