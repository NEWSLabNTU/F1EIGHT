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
no others (anything else is refused, as is a key written twice). Write
small numbers as `0.001`, not `1e-3`, which YAML reads as text:

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
graded straight (131.5 m): 0 → 2 → 6 → 11 m/s at 2.5 m/s^2, 3 s holds, then a stop on
the stop line at 1.5 m/s^2. The ESC regenerates only lightly (a drag brake: 1.5-2.0 m/s^2
with road load, against 3.0-3.5 m/s^2 for its brake), and `AckermannSpeedPID` has no braking
mode, so the default track brakes at what regeneration gives. Two runs per model variant:

- **profile** — speed holds at 2, 6 and 11 m/s (steady error, overshoot,
  time to reach), the launch from rest (no wheel spin, no lurch), the
  braking to the stop line (stop error, no wheel lock);
- **e-stop** — at 11 m/s the target drops to 0 with Autoware's emergency
  flag; the shell brakes; after the car has stopped it resumes to 6 m/s
  (no reversing, clean resume, no spin).

Eight model variants (mass, friction, battery voltage, motor constant, ESC
thresholds) replace the per-car models, so coefficients tuned to one exact
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
alpha 0.15, `max_pwm_offset` 100, which is what makes 11 m/s reachable at all) are in `src/sim/autosdv_rc_sim/config/reference_speed_controller.yaml`;
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
acceleration 1.0 -> 2.5 m/s^2 and normal deceleration 1.0 -> 1.5 m/s^2 (jerk 1.0 -> 5.0), from the package's
`config/common.param.yaml`, a copy of upstream's `common.param.yaml`.
`rc_sim.launch.xml` composes planning through `rc_sim_planning_component.launch.xml`
(upstream's component with that one path changed), because `autoware.launch.xml`
exposes no argument for it; the car's own launch keeps 4.17 and 1.0. So the run
reproduces the lab's 11 m/s straight (the smoke test requires a peak of at least
9 m/s and a stop within 1 m of the stop line; measured 0.02-0.07 m short before
the clock handshake, 0.03-0.05 m past with it). The speed loop runs in lockstep with the clock (below); the rest of
Autoware still races within a tick, so results are close to, but not exactly,
the graded run.

### Clock handshake (lockstep)

`lockstep:=true` (the default; `lockstep:=false` turns it off) makes the plant
wait for the speed loop instead of running ahead of it:

- `velocity_report` and the actuator acknowledge every timer deadline on
  `/rc_sim/lockstep/ack` (`std_msgs/String`,
  `"<name> <sec> <nanosec> <period_ns> [key=value ...]"`) once its work is done:
  the report published, the PWM written. Only with `hardware:=sim`; the car never
  acknowledges or waits.
- The plant learns each one's period and phase from its acks and never publishes a
  `/clock` past a deadline it has no ack for. Under load simulated time slows (a
  real-time factor below 1) instead of ticks being skipped.
- At equal instants `velocity_report` goes first: the actuator holds a tick until
  the report of the latest `velocity_report` deadline at or before it has arrived.
  Likewise `velocity_report` holds a report until the plant has marked its hall edges
  complete up to that instant (the edge message's layout label `until_ns=<ns>`).
- Lab 2 Part 1 adds `path_tracker` (its `lockstep` parameter): the plant never skips
  one of its 30 Hz cycles, and the actuator also holds a tick until the control command
  stamped with the tracker's latest deadline has come through the forwarder and
  `vehicle_cmd_gate` (both keep the stamp). That hold is dropped after 0.5 s (wall)
  without such a command, and re-armed by the next command on the tracker's schedule
  (not by one the gate makes up on its own).
- It also adds the e-stop's source, participant `emergency`: `lab2_part1_scenario`'s
  20 Hz emergency timer applies a scheduled e-stop change at its tick, stamps the message
  with the tick and acknowledges it, and the actuator's third gate holds its tick until
  that message has arrived, so the e-stop acts on the same tick in every run. On stacks
  where nobody acknowledges as `path_tracker` or `emergency` (`just sim rc`, the car)
  these two gates never arm.
- A participant silent for `lockstep_timeout` (2 s wall) invalidates the run:
  `/rc_sim/lockstep/status` (latched JSON, 1 Hz and on change) says `"ok": false`
  and why, and the plant stops waiting for it, so the run never hangs. The status
  also carries the real-time factor (`rtf`) and the loop's own accounting:
  `stale_reports` (actuator ticks on a report older than the latest one due, 0
  expected), `max_report_age`, `late_edges`, `skipped_ticks`, `timeouts`.
  `"ok": false` is for good, and the participants' own holds (the actuator waiting for
  its report, command or emergency) follow it and stay off; the plant still waits for
  acks. So in `just sim rc` any 2 s stall (a suspended laptop, a debugger pause) leaves
  the equal-instant ordering off for the rest of that session: restart the simulator to
  get it back.

Measured with `smoke.py` (each run about 40 s; this shared 32-core host already ran
at a load average of 26-133): with the handshake, the actuator never used a report
older than 40 ms (the report period is 50 ms) and no tick was stale, quiet or with
32-128 busy loops added. With 32-64 loops the car stopped 0.03-0.05 m past the
line in every run; without the handshake, 19 % of ticks used the previous report
even on the quiet host (up to 70 ms old), and with 64 loops the car stopped 4.4-5.3 m
past the line. The real-time factor with the handshake averaged 0.78-0.98 per run
(lowest one-second value 0.54), following the host's other load more than the busy
loops; without it, 0.92-1.00. With 128 loops (real-time factor 0.64, lowest 0.35) the
speed loop stayed locked (no stale tick, reports at most 40 ms old) but the car stopped
2.6 m past the line: Autoware's planning and control, which produce the target speed,
are not in the handshake and fall behind under that load.
Design: `autosdv_rc_sim/lockstep_sync.py`.

`just sim make-track --speeds 2,6,11 --hold 3 --accel 2.5 --decel 1.5` prints
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

Development history of the interactive run (the emergency flap, the creep, the stop overshoot) and
its open items are in [rc-sim-interactive-fixes.md](../reports/rc-sim-interactive-fixes.md).

The grader and `just sim rc` both run the speed controller with
`speed_controller_dt_source: loop`, as the Lab launches do. The car's default
launch uses `velocity_report`, where the effective `ki` is about three times
larger, so coefficients tuned here do not transfer to that launch.

Grading needs only Python: one variant takes about 4 s on one core (all eight in
32 s); `--jobs` runs variants in parallel with identical results (`--jobs 4`:
9 s, byte-identical `marks.json`).

## Calibrating the model against the car

Most of `vehicle.yaml` is a guess until the car has been measured. Take the
measurements in the order the notes in that file give them:

1. Weigh the car: `mass`.
2. Find its balance point: `lf`.
3. Do a coast-down at neutral on the flat, log `t,v` to a CSV (for example from
   a bag's `velocity_status`), and run `just sim fit-coastdown run.csv`
   (options: `--variant`, `--v-min`, default 0.5 m/s, which drops slow and
   standstill samples). It fits three terms, `m_eff * decel = a0 + a1 v + a2
   v^2`, and prints them as **lumped estimates**: `a0` is rolling resistance plus
   the drivetrain's Coulomb drag, `a1` is the drivetrain's viscous drag
   (compare the model's `drivetrain_viscous / r^2`), `a2` is `aero_drag`.
   `rolling_resistance` is `(a0 - Coulomb / r) / (m g)`, which subtracts the
   model's Coulomb term. Coast from top speed to below 1 m/s on the flat, in
   both directions, and average them. Smooth the 20 Hz hall speed (low-pass it,
   or fit over windows of at least 0.2 s) before differencing, or the
   deceleration is noise. A warning is printed if any term comes out negative.
4. Run a skidpad or tilt test for `mu`; `autosdv_rc_sim.fit.skidpad_mu(radius,
   speed)` gives it from the speed at which the car starts to slide on a circle.
5. Fit the motor with `just coach fit-motor` to replace the **mock**
   `motor_kv` and `gear_ratio`.

Each refitted value changes its tag to `measured` and gets a `note` with the
date and the car.
