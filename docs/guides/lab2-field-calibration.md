# Lab 2 field calibration

The numbers that the Lab 2 coach-board pursuit stack ships as guesses, and how
to measure them in the first field session. There are three measurements, one
recipe each, plus a speed step test that feeds the motor fit:

| Measurement | Record | Analyse | The result goes into |
|---|---|---|---|
| Motor plant | `record-cal` during `speed-steps` | `fit-motor` | `MotorPlantParams` in `motor_plant.py`, plus the course handout's vendored copy |
| Braking and reaction | `record` or `record-cal` around stops | `braking-report` | `envelope_decel` and `reaction_time` in `board_pursuit_planner.param.yaml` |
| Detector | `record-board` with the board held still | `detector-report` | the `autosdv_vlp16` / `autosdv_robin_w` profiles in `reflector_pose_init` |

Every recipe lives in `just/lab2.just`; `just lab2` lists them. Bags go to
`rosbags/`. Field safety comes first: the protocol in the course repository
applies (spotter with the kill switch, 1.5 m/s cap, cones), and before any of
this you need the actuator watchdog in place and a straight, clear run of at
least 25 m.

## 1. Speed step test and motor plant fit

**Why.** The Part 1 inner-loop bench (`speed_bench`, and the handout's copy of
it) runs the student's speed PID against `MotorPlantParams`, and those
parameters are provisional guesses.

**Run.** The speed step planner takes `board_pursuit_planner`'s place, so start
the stack without the planner:

```bash
just lab2 vehicle launch_planner:=false lidar_model:=vlp16   # terminal 1
just lab2 record-cal steps1                                  # terminal 2
just lab2 speed-steps                                        # terminal 3: 0,0.5,1.0,0.5,1.5,0 m/s, 4 s each
just lab2 engage                                             # terminal 4: starts the schedule
```

The schedule starts on engage and ends stopped. It also stops early with a
braking ramp after `max_distance` metres (25 m by default), and disengaging
aborts it. The arguments are steps, hold time and distance:
`just lab2 speed-steps 0,0.3,0.6,0,1.0,0.7,0 4 30`. A schedule with
**several stops from different speeds** gives the brake more data (see below).

It drives through the normal command path: a trajectory goes to Autoware's
trajectory follower, then `vehicle_cmd_gate`, then the actuator. The operation
mode, MRM and the actuator's watchdog and stop modes all stay live. It does not
publish `control_cmd` directly; that would bypass the gate.

**Fit.**

```bash
just lab2 fit-motor rosbags/lab2cal_steps1 -o tmp/motor_plant.yaml
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
note. Copy the file unchanged to the handout's
`lab2_pursuit/bench/vendor/motor_plant.py`. Keep the two copies identical.

## 2. Braking and reaction

**Why.** The planner's safety envelope lets the car go no faster than the speed
it can stop from inside the gap to the standoff floor:
`gap = v * reaction_time + v^2 / (2 * envelope_decel)`. Both numbers are
currently guesses (0.5 s and 0.6 m/s²).

**Record.** Drive at a steady speed, then stop the car. Any recording that
contains these topics works: `just lab2 record` or `just lab2 record-cal`.
Each kind of stop is a trigger:

| Trigger | How to make it | What it measures |
|---|---|---|
| `board_stop` | the TA walking the board stops dead | the whole chain: detector, tracker, headway law, control |
| `board_drop` | the TA drops the board or hides it | loss handling, including the tracker's 0.5 s timeout |
| `disengage` | `just lab2 disengage` | Autoware's stop mode on this car |
| `emergency` | MRM, or an emergency command | the emergency path |

Do at least three of each at 0.8 to 1.0 m/s.

**Analyse.**

```bash
just lab2 braking-report rosbags/lab2_run1 rosbags/lab2_run2 ...
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

**Why.** The intensity threshold in both profiles is unmeasured, and so is the
claim that a VLP-16 0.30 m up sees the board only at knee height at the 2 m
standoff.

**Record.** Park the car. A TA holds the board still, facing the car, at 2, 4,
6 and 8 m (add 1.5 and 10 m if there is room), for about 5 s at each, and
walks between the positions. Do this once at knee height (centre about
0.55 m) and once at chest height (about 1.0 m). Make sure the stack, or at
least the LiDAR driver and the TF, is running:

```bash
just lab2 record-board vlp16 knee        # or: robin-w
```

**Analyse.**

```bash
just lab2 detector-report rosbags/lab2board_vlp16_knee vlp16
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
