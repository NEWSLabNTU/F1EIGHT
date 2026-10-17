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
Engage. Speeds here come from Autoware's velocity planner, and separate
processes on a simulated clock race within a tick, so results are close to,
but not exactly, the graded run.

`just sim make-track --speeds 2,6,11 --hold 3 --accel 2.5 --decel 2.5` prints
the segment table and writes a track for `track:=`.
