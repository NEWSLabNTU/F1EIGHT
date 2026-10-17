"""The steering servo: PWM counts -> tyre angle, with dead time, a rate limit and the lock.

The mapping inverts the actuator's (steer = init_steer + angle * ratio), with
the ratio the actuator runs in simulation (config/actuator_rc_sim.param.yaml,
-143), so +-50 counts span +-0.349 rad, the physical lock.
"""
from collections import deque

from autosdv_rc_sim.params import VehicleParams


class Servo:
    def __init__(self, p: VehicleParams):
        self.p = p
        self._n = max(1, int(round(p.servo_dead_time / p.physics_dt)))
        self.reset()

    def reset(self) -> None:
        self.line = deque([0.0] * self._n, maxlen=self._n)
        self.delta = 0.0

    def target_from_pwm(self, pwm: int) -> float:
        p = self.p
        pwm = max(p.min_steer, min(p.max_steer, int(pwm)))
        angle = (pwm - p.init_steer) / p.tire_angle_to_steer_ratio
        return max(-p.max_steer_angle, min(p.max_steer_angle, angle))

    def step(self, pwm: int, dt: float) -> float:
        self.line.append(self.target_from_pwm(pwm))
        delayed = self.line[0]
        limit = self.p.servo_rate_limit * dt
        change = max(-limit, min(limit, delayed - self.delta))
        self.delta = max(-self.p.max_steer_angle,
                         min(self.p.max_steer_angle, self.delta + change))
        return self.delta
