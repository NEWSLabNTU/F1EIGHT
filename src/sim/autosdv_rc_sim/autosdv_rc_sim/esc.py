"""The electronic speed controller: PWM counts in, motor duty and brake out.

The cars' ESC model (docs/design/esc-pwm-model.md): one state machine, per-car
thresholds. The state follows the command alone and is the vehicle
interface's own tracker, longitudinal.EscModel, so the simulated ESC and the
actuator's picture of it cannot diverge; this module adds what the car does in
each state. Counts are the sim instance's (individual_params config/sim):

    LOCKED   (power-on, and after forward) pwm >= forward_start_pwm drives
             forward; below reverse_stop_pwm brakes, never reverses
    FORWARD  drives while pwm > forward_hold_pwm, at
             duty = (pwm - forward_hold_pwm) / (max_pwm - forward_hold_pwm),
             one count about 1 % of the span, quantised as the PCA9685 is
    ARMING   pwm <= reverse_arm_pwm: full brake
    ARMED    any return above reverse_arm_pwm; from rest, pwm <=
             reverse_start_pwm reverses, and it keeps reversing while pwm <
             reverse_stop_pwm, at reverse_scale of the brake level. Still
             rolling forward, below reverse_stop_pwm brakes.

Brake level = (reverse_stop_pwm - pwm) / (reverse_stop_pwm - min_pwm), so
min_pwm (360) and anything lower, brake_pwm (340) included, is full brake.
Between reverse_stop_pwm and forward_start_pwm nothing drives; the ESC may
apply a drag brake (dynamic braking through the windings).

The ESC powers on LOCKED. (The actuator's tracker starts ARMED instead,
because it cannot know what a manual test left behind.)

|duty| rises at most throttle_ramp per second and falls at once.
"""
from dataclasses import dataclass

from autosdv_vehicle_interface.longitudinal import EscModel, EscState

from autosdv_rc_sim.params import VehicleParams

STOP_SPEED = 0.05  # [m/s] wheel surface speed under which the ESC treats the motor as stopped


@dataclass(frozen=True)
class EscOutput:
    duty: float    # applied voltage fraction, [-reverse_scale, 1]
    brake: float   # friction-brake fraction, [0, 1]
    short: float   # dynamic-braking fraction at neutral, [0, 1]
    mode: str      # neutral | forward | brake | reverse


class Esc:
    def __init__(self, p: VehicleParams):
        self.p = p
        self.duty = 0.0
        # EscModel reads only the thresholds and init_pwm, which VehicleParams names alike
        self.model = EscModel(p)  # type: ignore[arg-type]
        self.model.state = EscState.LOCKED
        self.reversing = False
        self.mode = 'neutral'

    @property
    def state(self) -> EscState:
        return self.model.state

    def step(self, t_ns: int, pwm: int, wheel_speed: float, dt: float) -> EscOutput:
        p = self.p
        state = self.model.update(pwm)
        level = min(1.0, max(0.0, (p.reverse_stop_pwm - pwm) / (p.reverse_stop_pwm - p.min_pwm)))
        target, brake, short = 0.0, 0.0, 0.0
        if state == EscState.FORWARD:
            target = min(1.0, (pwm - p.forward_hold_pwm) / (p.max_pwm - p.forward_hold_pwm))
            self.mode = 'forward'
        elif state == EscState.ARMED and pwm < p.reverse_stop_pwm and (
                self.reversing or (abs(wheel_speed) < STOP_SPEED and pwm <= p.reverse_start_pwm)):
            target = -level * p.reverse_scale
            self.mode = 'reverse'
        elif pwm < p.reverse_stop_pwm:
            brake = level
            self.mode = 'brake'
        else:
            short = p.drag_brake
            self.mode = 'neutral'
        self.reversing = self.mode == 'reverse'

        step = p.throttle_ramp * dt
        if target == 0.0:
            self.duty = 0.0
        elif self.duty * target < 0.0:
            self.duty = step if target > 0.0 else -step     # a direction change passes zero
            if abs(self.duty) > abs(target):
                self.duty = target
        elif abs(target) <= abs(self.duty):
            self.duty = target
        else:
            self.duty += step if target > 0.0 else -step
            if abs(self.duty) > abs(target):
                self.duty = target
        return EscOutput(self.duty, brake, short, self.mode)
