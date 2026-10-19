# Speed-control tests on a car

Tools for checking the wheel-speed sensor and step-testing the actuator's
speed controller on a real car with the motor capped low. Output goes to
`tmp/speed_control/<vehicle_id>/`.

| Script | Drives the motor | Purpose |
|---|---|---|
| `hall_edges.py` | no | Count hall-sensor edges and pulse widths while the car is pushed or driven |
| `run_trial.sh` | yes, capped | Run `velocity_report` + the actuator, then `step_test.py` |
| `step_test.py` | via the actuator | Publish a speed step profile, log, abort on any guard, print metrics |

## 1. The speed sensor first

The controller is only as good as `/vehicle/status/velocity_status`. Check the
KY-003 alone (BOARD pin 15, see `params/velocity_report.yaml`) before any
closed-loop run:

```bash
python3 scripts/testing/speed_control/hall_edges.py 45      # then drive or push the car
```

It blocks on `GPIO.wait_for_edge(..., GPIO.BOTH)`, so it does not poll. Expect
rising and falling counts that match, and about 36 rising edges per metre
(12 magnets, 10.5 cm wheel). A pin that sits LOW with a handful of edges while
the module's LED flashes means the signal is not reaching the Jetson.

Two findings from car A:

- **On a lift the suspension drops the wheel away from the sensor**, so it
  reads nothing. Test speed on the ground.
- **The A3144 on a KY-003 is specified from 4.5 V.** Powered from 3.3 V (pin 17),
  car A's module lit its LED but pin 15 stayed LOW. At 5 V the module's pull-up
  puts 5 V on pin 15, a 3.3 V pin: add a divider (10 k / 20 k), a level shifter,
  or remove the on-board pull-up and pull S up to 3.3 V instead.

## 2. A step test

Stop the stack first: `just launch` runs its own actuator and
`vehicle_cmd_gate`, which publishes `control_cmd` too. Put the car on the
ground with the space the profile needs clear ahead.

```bash
scripts/testing/speed_control/run_trial.sh car_c base                    # current gains
scripts/testing/speed_control/run_trial.sh car_c kp8 -p kp_speed:=8.0 -p ki_speed:=4.0
scripts/testing/speed_control/run_trial.sh car_c short -- --profile 0:1,0.3:2,0:2 --max-distance 1.0
scripts/testing/speed_control/run_trial.sh car_c dry -p dry_run:=true    # no PWM written
```

Arguments before `--` go to the actuator as ROS overrides; after `--`, to
`step_test.py` (`--help` lists them).

### Safety limits

| Limit | Default | Where |
|---|---|---|
| Motor PWM cap | `forward_start_pwm + 10`; `MAX_PWM=` may raise it to `+ 25` at most | `run_trial.sh` forces `-p max_pwm` last on the actuator's command line and reads it back before driving; the actuator clamps every motor command to it |
| Profile | 2 s stop, 3 s at 0.3 m/s, 3 s at 0.5 m/s, 3 s stop | `--profile` |
| Distance | 3 m, aborting at 2.7 m to leave room to stop | `--max-distance`, `--stop-margin` |
| Overspeed | 0.8 m/s | `--overspeed` |
| Stall | PWM at or above `forward_start_pwm + 3` with < 0.05 m/s for 1.5 s | `--stall-pwm`, `--stall-time` |
| Lost speed | no report for 0.5 s, or none within 2 s | `step_test.py` |
| Run time | 20 s, and the step test is killed at 30 s | `--max-run`, `run_trial.sh` |
| Watchdog | brake, then neutral, 0.3 s after commands stop | the actuator's own `control_cmd_timeout` |

An abort commands zero for 2 s and exits; the actuator's watchdog then brakes
to neutral. Steering is held centred throughout. The stall guard is the one
that has fired on a powered car (twice on car A, lifted, about 2 s after the
motor started); the distance, overspeed and lost-report guards have not yet
been exercised with real speed readings.

### Reading the result

`step_test.py` prints one line per step: steady mean and error, standard
deviation, rise time to within 10 %, peak, and the PWM range over the step's
last two seconds. The CSV holds every 20 ms sample: target, speed, PWM, the
P/I/D terms, the actuator's mode and its ESC state.

The gains are the actuator's `kp_speed`, `ki_speed`, `kd_speed`
(`params/actuator.yaml`). The controller adds its output to `init_pwm`, so with
`kp_speed` 12 a 0.3 m/s error is only about 4 counts: the integral does most of
the climb from neutral to `forward_start_pwm`.
