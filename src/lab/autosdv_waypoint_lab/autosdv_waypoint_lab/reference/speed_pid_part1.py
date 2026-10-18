"""Reference Lab 2 Part 1 SpeedPID (instructor): the skeleton's TODOs, completed.

Inherits everything the students are given (parameters, feed_forward,
filter_derivative, brake_pwm, clamp_pwm, reset) from the skeleton itself, so the
reference cannot drift from the handout's given parts; it adds only update(),
which is the TODOs: P and D as autosdv_vehicle_interface.speed_controller.AckermannPID
does them (D on the measurement through the EMA), the braking mode with hysteresis
(feed-forward and integral out, request -(kp_brake e + D) into the brake band), and driving
with conditional integration on the previous output's saturation and the integral
clamped always.

Selected with
    speed_controller_class:=autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID
and the parameters of config/speed_pid_part1.yaml.
"""

from autosdv_waypoint_lab.stub.speed_pid_part1 import SpeedPID as _Skeleton


class SpeedPID(_Skeleton):
    def update(self, target_speed: float, measured_speed: float, dt: float) -> int:
        error = target_speed - measured_speed
        e_d = max(-self.error_limit, min(self.error_limit, error))
        self.proportional = self.kp * e_d

        if dt > 0.0 and self.last_measured is not None:
            d = self.filter_derivative(-self.kd * (measured_speed - self.last_measured) / dt)
        else:
            d = 0.0
        self.last_measured = measured_speed

        if self.braking:
            if error > -self.brake_exit:
                self.braking = False
        elif error < -self.brake_enter:
            self.braking = True
        if self.braking:
            return self.brake_pwm(-(self.kp_brake * error + d))

        if (dt > 0.0 and abs(error) <= self.integral_band
                and not (self.enable_conditional_integration and self.saturated)):
            self.integral += self.ki * e_d * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        u = self.feed_forward(target_speed) + self.proportional + self.integral + d
        limited = max(0.0, min(u, float(self.max_pwm_offset), float(self.max_pwm - self.init_pwm)))
        self.saturated = limited != u
        return self.clamp_pwm(self.init_pwm + limited)
