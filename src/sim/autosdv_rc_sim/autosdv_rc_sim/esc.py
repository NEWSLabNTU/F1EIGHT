"""The electronic speed controller: PWM counts in, motor duty and brake out.

Counts are actuator.yaml's. Forward: duty = (pwm - (init + deadband)) /
(max_pwm - (init + deadband)), one count about 1 % of the span, quantised as
the PCA9685 is. Below the deadband: level = ((init - deadband) - pwm) /
((init - deadband) - min_pwm), so min_pwm (360) and anything lower,
brake_pwm (340) included, is full brake. Below neutral means brake until the
car has stopped and the stick has sat at neutral for rearm_time; only then
does it mean reverse, at reverse_scale of the duty. At neutral the ESC may
apply a drag brake (dynamic braking through the windings).

|duty| rises at most throttle_ramp per second and falls at once.
"""
from dataclasses import dataclass
from typing import Optional

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
        self.reverse_armed = False
        self.mode = 'neutral'
        self._neutral_since_ns: Optional[int] = None
        self._rearm_ns = int(round(p.rearm_time * 1e9))

    def step(self, t_ns: int, pwm: int, wheel_speed: float, dt: float) -> EscOutput:
        p = self.p
        hi = p.init_pwm + p.esc_deadband
        lo = p.init_pwm - p.esc_deadband
        target, brake, short = 0.0, 0.0, 0.0
        if pwm > hi:
            target = min(1.0, (pwm - hi) / (p.max_pwm - hi))
            self.mode = 'forward'
            self.reverse_armed = False
            self._neutral_since_ns = None
        elif pwm < lo:
            level = min(1.0, (lo - pwm) / (lo - p.min_pwm))
            self._neutral_since_ns = None
            if self.reverse_armed:
                target = -level * p.reverse_scale
                self.mode = 'reverse'
            else:
                brake = level
                self.mode = 'brake'
        else:
            short = p.drag_brake
            self.mode = 'neutral'
            if abs(wheel_speed) < STOP_SPEED:
                if self._neutral_since_ns is None:
                    self._neutral_since_ns = t_ns
                elif t_ns - self._neutral_since_ns >= self._rearm_ns:
                    self.reverse_armed = True
            else:
                self._neutral_since_ns = None

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
