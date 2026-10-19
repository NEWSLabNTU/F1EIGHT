# Lab 2 field calibration

> The recipes are under `just coach` since 2026-10-13, when `just lab2` became
> the waypoint lab. The speed steps, motor fit and braking report calibrate the
> actuator's SpeedPID and apply to both labs; the board detector sections
> apply to the coach-board lab only.

The numbers that the Lab 2 coach-board pursuit stack ships as guesses, and how
to measure them in the first field session. There are three measurements, one
recipe each, plus a speed step test that feeds the motor fit:

| Measurement | Record | Analyse | The result goes into |
|---|---|---|---|
| Motor plant | `record-cal` during `speed-steps` | `fit-motor` | `MotorPlantParams` in `motor_plant.py`, plus the course handout's vendored copy |
| Braking and reaction | `record` or `record-cal` around stops | `braking-report` | `envelope_decel` and `reaction_time` in `board_pursuit_planner.param.yaml` |
| Detector | `record-board` with the board held still | `detector-report` | the `autosdv_vlp16` / `autosdv_robin_w` profiles in `reflector_pose_init` |

Every recipe lives in `just/lab2.just`; `just coach` lists them. Bags go to
`rosbags/`. Field safety comes first: the protocol in the course repository
applies (spotter with the kill switch, 1.5 m/s cap, cones), and before any of
this you need the actuator watchdog in place and a straight, clear run of at
least 25 m.

Two software checks come before the car. On a workstation, `just coach
bench-checks` runs protocol 3.1 checks 1 to 4 against the real vehicle stack
with a plant model in place of the car, and every recipe below works against
`just coach bench` the same way it does against `just coach vehicle`. On the car,
`just coach preflight` after bring-up says whether the stack is ready to engage,
and which diagnostic blocks it if not. Both are described in
[lab2-stack.md](lab2-stack.md#the-vehicle-bench).

## Car gate: steering ratio and lock (`lab2_steer_cal`)

**Why.** `tire_angle_to_steer_ratio` converts a tire angle to servo counts
(`pwm = 489 + int(average_angle * ratio)`), and nothing on the car measures the
angle: no sensor, and `steering_status` republishes the command
(`known-config-defects.md` section 8). The Lab 2 launch loads a stand-in of
-143 (`config/actuator_lab2.param.yaml`) and `vehicle_info.param.yaml` claims
0.349 rad of lock; neither is measured. The graded U-turn (`lab2_uturn`) needs
`atan(0.319 / R)`: **0.282 rad at R 1.1 m**. The gate wants that to be at most
80 % of the lock, so the real lock must be at least about 0.35 rad (0.352). That
is the claimed 0.349 rad with no room to spare: a car whose lock is exactly the
claim, or the -143 stand-in's 0.350 rad, uses 81 % and FAILs. The gate reads the
radius from `lab2_uturn/course.yaml` (`uturn.radius`), so it follows the course
when `make-uturn` changes it. It decides the course before any reference bag is
recorded. Per car (A, B, C),
once, and again after the servo, linkage or horn is touched.

**Holding a fixed steer PWM.** The offset is the independent variable, so the
tool commands the tire angle that makes the actuator write exactly
`489 + offset`, whichever ratio is loaded (it inverts the actuator's Ackermann
averaging):

```bash
just control basic                  # terminal 1: actuator without the gate; the spotter is the stop
just lab2 steer-hold -25            # terminal 2: PWM held at 464, 0.4 m/s; Ctrl-C stops it
```

`steer-hold` assumes the actuator runs `actuator.yaml`'s -1000, which
`control basic` loads; pass `--ratio R` when it runs another. Offsets beyond
+-50 (the servo window) or beyond the 0.349 rad clamp are refused.

**Procedure** (car on the floor, 2 m clear radius around it, spotter with the
transmitter, G1 working):

1. For each offset in -50, -25, -10, +10, +25, +50: `steer-hold`, gear DRIVE
   (`just tool controller`, `x`), let it run one full slow circle, and mark the
   path (chalk dot under the rear axle centre every quarter turn, or the track
   of the inner rear tyre).
2. Measure the circle's radius from the marks: **R** for the axle centre, or
   for the inside or outside rear tyre track (pass `--measured inner|outer`).
   Note the direction: **positive radius = left turn** (counter-clockwise from
   above), negative = right.
3. Fit:

```bash
just lab2 steer-fit -50:+1.6 -25:+3.1 -10:+7.9 10:-7.9 25:-3.1 50:-1.6   # example numbers
```

It prints the ratio (counts/rad, with sign), the centre error (counts away from
489 that mean straight), the fit residual, the true lock (`delta` at +-50
counts) and the smallest radius, and says PASS when the U-turn's radius (R 1.1 m) needs at
most 80 % of the lock. Exit code 0 is PASS, 1 is FAIL, 2 is bad input. A positive ratio
means this servo turns left above 489, opposite to the shipped sign: that is
part of the value, not an error.

**Where the values go** (before any circle reference run):

| Value | File | Scope |
|---|---|---|
| `tire_angle_to_steer_ratio` (with sign) | a per-car copy of `src/lab/autosdv_waypoint_lab/config/actuator_lab2.param.yaml`, e.g. `actuator_lab2_carA.param.yaml`; pass `actuator_param_overrides:=$PWD/...` to `just lab2 vehicle` | per car |
| `max_steer_angle` (the fitted lock, rounded down) | `vehicle_info.param.yaml`; it is what the planner and the path tracker clamp to. Use the smallest of the three cars; the same key may be added to a car's override file for the actuator alone | all cars |

Re-run `steer-hold` once at -50 with the new ratio loaded
(`--ratio <fitted>`): the circle must match the first fit's radius within
10 %. Then drive the R 1.1 m U-turn (`lab2_uturn`) by hand and check the tracker's
commanded angle stays under 80 % of `max_steer_angle`. A car that fails the
margin cannot run the R 1.1 m U-turn reliably: tell the instructor; the course
changes (`just lab2 make-uturn` with a larger radius, if the field allows), not
the check. The measured values go on the car's sign-off sheet
(`field/lab2-car-signoff.md`, part C in the course repository).

## 1. Speed step test and motor plant fit

**Why.** The Part 1 inner-loop bench (`speed_bench`, and the handout's copy of
it) runs the student's speed PID against `MotorPlantParams`, and those
parameters are provisional guesses.

**Run.** The speed step planner takes `board_pursuit_planner`'s place, so start
the stack without the planner:

```bash
just coach vehicle launch_planner:=false                     # terminal 1 (Robin-W, ZED IMU)
just coach record-cal steps1                                  # terminal 2
just coach speed-steps                                        # terminal 3: 0,0.3,0.6,0 m/s, 4 s each (about 2 m)
just coach engage                                             # terminal 4: starts the schedule
```

The schedule starts on engage and ends stopped. It also stops early with a
braking ramp after `max_distance` metres (2.5 m by default, so give it about
3 m of clear straight), and disengaging aborts it. The default is sized for a
small floor and still identifies deadband, gain and time constant. With a
longer straight, pass steps, hold time and distance:
`just coach speed-steps 0,0.5,1.0,0.5,1.5,0 4 25`. A schedule with
**several stops from different speeds** gives the brake more data (see below).

It drives through the normal command path: a trajectory goes to Autoware's
trajectory follower, then `vehicle_cmd_gate`, then the actuator. The operation
mode, MRM and the actuator's watchdog and stop modes all stay live. It does not
publish `control_cmd` directly; that would bypass the gate.

**Fit.**

```bash
just coach fit-motor rosbags/lab2cal_steps1 -o tmp/motor_plant.yaml
```

The fit replays the recorded PWM (`/autosdv/actuator_node/debug/pwm_values`)
through `MotorPlant` and through `HallSpeedSensor`, the model of
`velocity_report`'s 12-marker EMA at 20 Hz. It then compares the simulated
sensor output with `/vehicle/status/velocity_status`. Because the sensor model
is in the loop, its lag and quantisation are not absorbed into the parameters.

The output is a parameter table that gives, per parameter, the range over
which the residual RMSE rises by at most 5 %. That range shows how tightly the
recording pins the parameter down. You also get the residual RMSE, overall and
per regime, and a YAML block.

- `deadband`, `gain` and `time_constant` come out tightly from the step test.
- `brake_decel` is usually marked **weak**. A stop from 1.5 m/s lasts a handful
  of 20 Hz reports, so the hall sensor barely sees it. Cross-check it against
  `a_hinge` from the braking report.
- `coast_time_constant` is **not identified**. The actuator's speed loop never
  leaves the motor in the deadband while the car is moving, so the parameter
  is held at its prior. That is harmless for the bench, which never coasts
  either.

**Where it goes.** Test the YAML first with
`ros2 run autosdv_vehicle_interface speed_bench --plant "$(cat tmp/motor_plant.yaml)"`.
The handout's bench takes the same `--plant`. Then write the values into the
`MotorPlantParams` defaults in
`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/autosdv_vehicle_interface/motor_plant.py`,
together with the date, the car and the bag name, and remove the PROVISIONAL
note. Copy the file unchanged to the course repository's
`2026/labs/Lab2/handout/lab2_speed/lab2_speed/bench/vendor/motor_plant.py`, then
update that entry in `SOURCES` of the `vendor/__init__.py` beside it: the
vehicle repository's commit (second field) and the sha256 of the copy (last
field, `sha256sum` of the file). The self-check compares the hash. Keep the two
copies identical.

## 2. Braking and reaction

**Why.** The planner's safety envelope lets the car go no faster than the speed
it can stop from inside the gap to the standoff floor:
`gap = v * reaction_time + v^2 / (2 * envelope_decel)`. Both numbers are
currently guesses (0.5 s and 0.6 m/s²).

**Record.** Drive at a steady speed, then stop the car. Any recording that
contains these topics works: `just coach record` or `just coach record-cal`.
Each kind of stop is a trigger:

| Trigger | How to make it | What it measures |
|---|---|---|
| `board_stop` | the TA walking the board stops dead | the whole chain: detector, tracker, headway law, control |
| `board_drop` | the TA drops the board or hides it | loss handling, including the tracker's 0.5 s timeout |
| `disengage` | `just coach disengage` | Autoware's stop mode on this car |
| `emergency` | MRM, or an emergency command | the emergency path |

Do at least three of each at 0.8 to 1.0 m/s.

**Analyse.**

```bash
just coach braking-report rosbags/lab2_run1 rosbags/lab2_run2 ...
```

For each stop the report gives the steady speed `v0` and the reaction, which
runs from the trigger to the onset of deceleration (from a hinge fit). It also
gives the deceleration three ways (hinge, mean from 90 % to 20 % of `v0`, and
peak), the stopping distance, and `a_eff`. `a_eff` is the deceleration that
makes the envelope formula reproduce the measured distance with the measured
reaction. Read `reaction` and `a_eff` together, as the envelope does. The hall
sensor's lag can move the hinge onset earlier while flattening its slope, but
the pair always predicts at least the true stopping distance.

The recommendation is conservative by a stated margin:

- `reaction_time` is the worst reaction over every stop plus 0.2 s
  (`--reaction-margin`).
- `envelope_decel` is the worst `a_eff` times 0.75 (`--decel-margin`). It
  counts only commanded stops (drop, disengage, emergency), because after a
  board stop the headway law chooses how hard to brake.

The report also prints the resulting speed cap at a 2 m range and the range
from which `v_max` is allowed.

On the car, a board stop is found from the detections themselves: the board's
position in the odometry frame at each scan's stamp. This needs the LiDAR
clock synced to the host. If it is not, use `--board-time receive`, and the
reaction then reads short by the detector latency.

**Where it goes.** Put `envelope_decel` and `reaction_time` in
`src/lab/autosdv_coach_lab/config/board_pursuit_planner.param.yaml`, and update
the speed-cap figures in that file's comment.

**Planning simulator, for reference** (2026-10-11, reference controller):

| Bag | Trigger | v0 | Reaction | a_eff |
|---|---|---|---|---|
| hardstop | board_stop (dead stop) | 0.74 | 0.77 s | 0.52 |
| dropout | board_stop | 0.84 | 0.45 s | 0.68 |
| dropout | board_drop | 0.80 | 0.55 s | 0.58 |
| speed steps | disengage | 1.62 | 0.08 s | 1.40 |

Combined, the recommendation is `reaction_time` 1.00 s and `envelope_decel`
0.40 m/s². That caps the car at 0.58 m/s at a 2 m range, against 0.84 m/s with
the shipped values. Even in simulation the shipped pair is optimistic for an
instant board stop. After a board drop the vehicle reaches only about
0.6 m/s², although the trajectory plans 1.0. Measure the car before relying on
either number.

## 3. Detector

**Why.** The intensity threshold in both profiles is unmeasured. Lab 2 runs
on Robin-W cars only (decided 2026-10-11), so the `autosdv_robin_w` profile
is the one to calibrate; a VLP-16 car sees a chest-height board only from
about 3 m, which is why it is not used.

**Record.** Park the car. A TA holds the board still, facing the car, at 2, 4,
6 and 8 m (add 1.5 and 10 m if there is room), for about 5 s at each, and
walks between the positions, at chest height (centre about 1.0 m). Make sure
the stack, or at least the LiDAR driver and the TF, is running:

```bash
just coach record-board robin-w chest
```

**Analyse.**

```bash
just coach detector-report rosbags/lab2board_robin-w_chest robin-w
```

The report replays every scan through `board_tracking_node`'s detector and its
publish rule, offline, and gives:

- the detection rate per range bin, with the gate that rejected each miss;
- the detection time per scan;
- the board's intensity distribution against the background (everything else
  in the range gate), with a suggested `intensity_threshold` in the middle of
  the gap between the two;
- the detected board-centre height against the profile's gate.

Misses take the range of the nearest detection in time. A position where the
board was never detected shows as `unassigned`.

**Where it goes.** The profiles are in reflector_pose_init, under
`packages/reflective_pose_core/reflective_pose_core/data/autosdv_vlp16.yaml`
and `autosdv_robin_w.yaml`:

- `intensity_threshold`;
- the centre-height gate (`board_centre_height`, `centre_height_tolerance`),
  set from the measured heights;
- `base_link_height_above_ground`, if the measured heights are offset by a
  constant.

Commit the profile change upstream first, then move the submodule pin. Write
the grip that works into the field protocol.

**Synthetic, for reference** (reflective_pose_sim, 20 scans per position):

- **VLP-16, knee height:** detected at every scan from 1.5 to 8 m; missed at
  10 m (`not_vertical`). Centre height 0.57 m (truth 0.55).
- **VLP-16, chest height:** missed at 2 m (`bad_width`), which reproduces the
  plan's finding; detected at every scan from 3 to 8 m.
- **Robin-W, chest height:** detected at every scan from 1.5 to 10 m.

Detection time is about 1 ms per scan on a VLP-16 and 6 ms on a Robin-W, on a
desktop core. The suggested threshold is about 160, in the simulator's gap
between 100 and 220. The simulator's intensity model is invented, so the real
value must come from a bag.
