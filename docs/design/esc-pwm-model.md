# The ESC PWM model: one state machine, per-car thresholds

Status: implemented in the vehicle interface (2026-10-19): thresholds in each
car's profile, startup checks, the ESC state tracker and the reverse guard.
The motor model and the simulator (item 3 below) are not changed yet. From
keyboard_pwm_control measurements on cars A, B and C in 5-count steps.

## What the measurements show

All three cars' ESCs behave the same way; only the numbers differ. A single
"neutral" value cannot describe them, for two reasons:

1. **Hysteresis.** A stopped car needs a larger command to start than a moving
   car needs to keep going, in both directions.
2. **A reverse lock.** Below the stop point the ESC *brakes*; it reverses only
   after an arming sequence -- a pull far below neutral, then a return to near
   neutral -- and a forward command locks it again.

## The model

The motor command `u` (PCA9685 counts) moves the ESC between four states:

```
                 u >= FWD_START
   +--------------------------------------------+
   |                                            v
LOCKED  --- u <= ARM --->  ARMING  --- u >= ARM_RETURN --->  ARMED
(power-on;                                                    |  ^
 after any       <------------- u >= FWD_START --------------+  |
 forward drive)       (disarms, drives forward)                  |
   ^                                                             |
   |   u <= FWD_HOLD (released)           u <= REV_START: reverse;
FORWARD <------------------------------   keeps reversing while u < REV_STOP
   (drives while u > FWD_HOLD)            (REV_HOLD still moves); u >= REV_STOP: stops
```

| state | command | the car |
|---|---|---|
| LOCKED (power-on, and after any forward drive) | `u >= FWD_START` | starts forward -> FORWARD |
| | `STOP band` | still |
| | `u < STOP band` | brakes, never reverses |
| | `u <= ARM` | brakes, and -> ARMING |
| FORWARD | `u > FWD_HOLD` | drives; force grows with `u - FWD_HOLD` |
| | `u <= FWD_HOLD` | released -> LOCKED |
| ARMING | `u >= ARM_RETURN` | -> ARMED, still |
| ARMED | `u <= REV_START` from rest | starts backward |
| | `u < REV_STOP` while reversing | keeps reversing; force grows with `REV_STOP - u` |
| | `REV_STOP <= u < FWD_START` | still, **stays armed** |
| | `u >= FWD_START` | disarms, forward -> FORWARD |

Two consequences for any software that drives the motor:

- **A stop is a value strictly inside the band where nothing moves in either
  state**: above `REV_STOP` (armed reverse) and below `FWD_START` (forward from
  rest). That band, not a single number, is what the stop must sit in.
- **A stopping brake must stay above `ARM`.** A brake pulled to `ARM` or below
  arms reverse, and the car then sits armed: the next command below
  `REV_STOP` drives it backward.

## Parameters per car (measured 2026-10-19)

| parameter | meaning | car A | car B | car C |
|---|---|---|---|---|
| `FWD_START` | forward from rest; also disarms reverse | 385 | 395 | 400 |
| `FWD_HOLD` | moving forward continues above this | ? | ? | 390 (395 holds, 390 releases) |
| still (tested) | no motion from rest, either state | 380 | 385, 390 | 380..390 |
| `REV_STOP` | armed reverse stops at or above this | 375 | 385 | 380 |
| `REV_HOLD` | moving backward continues at this | 370 | ? | ? |
| `REV_START` | armed reverse starts from rest at or below this | 365 | 380 | 375 |
| `ARM` | pull to at or below this to arm | 345 | 355 | 340 |
| `ARM_RETURN` | then come back up to this | 370 | 385 | 380 |
| **stop (`init_pwm`)** | mid of `REV_STOP .. FWD_START` | **380** | **390** | **385** |
| **stopping brake (`brake_pwm`)** | above `ARM`, below the still band | 360, untested | 365, untested | 375, untested |

Values were found in 5-count steps, so each is good to about +-5; "?" is not
measured yet. A threshold the tests only bracketed (e.g. car C's `ARM`: 340
arms, a brake that never reached it did not) is given as the tested value.

## In the vehicle interface (implemented)

- Profile keys, all in `actuator.yaml` with 0 = not measured:
  `forward_start_pwm`, `forward_hold_pwm`, `reverse_stop_pwm`,
  `reverse_start_pwm`, `reverse_arm_pwm`. Unset, the shell behaves exactly as
  before (the golden and equivalence tests run that way, and so does `sim`).
- Startup refuses a profile where `brake_pwm` or `min_pwm` is at or below
  `reverse_arm_pwm` (every brake would arm reverse), or `init_pwm` is not in
  `reverse_stop_pwm .. forward_start_pwm - 1`.
- `longitudinal.EscModel` tracks the ESC from the commands sent; it starts
  as `armed` (a manual test may have left it so) until a command at or above
  `forward_start_pwm` disarms it. Published on `/autosdv/actuator_node/debug/esc_state`.
- While it is `armed`, a command below `reverse_stop_pwm` is replaced by
  `init_pwm` unless the target is backward: the stack never reverses the car
  unasked. It does not drive backward on purpose (no reverse sequence yet).

## Where it goes (the full plan)

1. **Car profiles** (`individual_params/config/<vehicle_id>/autosdv_vehicle/
   actuator.param.yaml`): `init_pwm` = the stop above; `brake_pwm` = the
   stopping brake (above `ARM`); new keys `forward_start_pwm`,
   `forward_hold_pwm`, `reverse_arm_pwm`, `reverse_stop_pwm`,
   `reverse_start_pwm` for the measured thresholds.
2. **Vehicle interface** (`longitudinal.py`): stopping uses `brake_pwm` only;
   a reverse request runs the explicit sequence (`reverse_arm_pwm`, then
   `init_pwm`, then below `reverse_stop_pwm`). Lab 2 never reverses, so the
   stack never arms the ESC. The speed controller's feed-forward starts a
   stopped car at `forward_start_pwm` and may hold a moving one down to
   `forward_hold_pwm`.
3. **Motor model and simulator** (`motor_plant.py`, rc_sim `esc.py`): the same
   state machine, so the motor fit and the simulated car match the real ESC.
   The simulators get their own instance (`sim` = `actuator.yaml`'s values)
   instead of `car_a`, which becomes a measured car.

## Still to measure

- `FWD_HOLD` on cars A and B: start forward, step down while moving.
- `REV_HOLD` on cars B and C: reverse, step up while moving.
- The stopping brake on each car: drive forward, pull to the candidate,
  return to the stop, then go 5 counts below `REV_STOP`. If the car reverses,
  the candidate armed it: choose a weaker brake.
