"""The vehicle bench's car: motor PWM and steering PWM in, motion out. No ROS.

The bench (`just lab2 bench`) runs the real vehicle stack on a workstation and
replaces only the hardware. This module is the hardware:

- the drive train: `MotorPlant` from autosdv_vehicle_interface.motor_plant,
  the model the motor fit (`just lab2 fit-motor`) identifies and the Part 1
  speed bench runs, with the same `MotorPlantParams`;
- the steering servo: the actuator's PWM mapping inverted
  (tire angle = (steer_pwm - init_steer) / tire_angle_to_steer_ratio), with a
  first-order servo lag;
- kinematic bicycle motion about the rear axle (base_link);
- the wheel sensor: `HallSpeedSensor`, the model of velocity_report's
  12-marker edge timing, EMA and 0.5 s timeout.

Nothing here knows about the controller: it integrates whatever PWM the
actuator commanded, so the safety shell, the watchdogs and the student's
SpeedPID are all in the loop.
"""

from dataclasses import dataclass
import math
from typing import Optional

from autosdv_vehicle_interface.motor_plant import (
    HallSpeedSensor, MotorPlant, MotorPlantParams)


@dataclass
class SteeringParams:
    """The servo and the geometry; defaults are actuator.yaml / vehicle_info's."""

    init_steer: int = 489
    min_steer: int = 439
    max_steer: int = 539
    tire_angle_to_steer_ratio: float = -1000.0
    max_steer_angle: float = 0.349
    wheel_base: float = 0.319
    time_constant: float = 0.1     # servo lag [s]


def tire_angle_from_pwm(steer_pwm: float, p: SteeringParams) -> float:
    """Invert the actuator's mapping (steer = init_steer + angle * ratio)."""
    pwm = max(p.min_steer, min(p.max_steer, steer_pwm))
    if p.tire_angle_to_steer_ratio == 0.0:
        return 0.0
    angle = (pwm - p.init_steer) / p.tire_angle_to_steer_ratio
    return max(-p.max_steer_angle, min(p.max_steer_angle, angle))


class VehiclePlant:
    """Integrate the car's planar pose, speed and wheel-sensor output."""

    def __init__(self, motor: Optional[MotorPlantParams] = None,
                 steering: Optional[SteeringParams] = None,
                 wheel_diameter: float = 0.105, markers_per_rotation: int = 12):
        self.motor = MotorPlant(motor)
        self.steering = steering if steering is not None else SteeringParams()
        self.hall = HallSpeedSensor(wheel_diameter=wheel_diameter,
                                    markers_per_rotation=markers_per_rotation)
        self.motor_pwm = self.motor.params.init_pwm
        self.steer_pwm = self.steering.init_steer
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.tire_angle = 0.0
        self.yaw_rate = 0.0
        self.accel = 0.0
        self.time = 0.0

    @property
    def speed(self) -> float:
        return self.motor.speed

    @property
    def distance(self) -> float:
        return self.motor.distance

    def command(self, motor_pwm: float, steer_pwm: float) -> None:
        """Latch the PWM the actuator wrote (held until the next write, as the PCA9685 does)."""
        self.motor_pwm = int(round(motor_pwm))
        self.steer_pwm = int(round(steer_pwm))

    def step(self, dt: float) -> None:
        """Advance by `dt` seconds under the latched PWM."""
        if dt <= 0.0:
            return
        v0 = self.motor.speed
        v1 = self.motor.step(self.motor_pwm, dt)
        self.accel = (v1 - v0) / dt

        target = tire_angle_from_pwm(self.steer_pwm, self.steering)
        tau = self.steering.time_constant
        k = 1.0 - math.exp(-dt / tau) if tau > 0.0 else 1.0
        self.tire_angle += (target - self.tire_angle) * k

        v = 0.5 * (v0 + v1)
        self.yaw_rate = v * math.tan(self.tire_angle) / self.steering.wheel_base
        yaw_mid = self.yaw + 0.5 * self.yaw_rate * dt
        self.x += v * math.cos(yaw_mid) * dt
        self.y += v * math.sin(yaw_mid) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.yaw_rate * dt),
                              math.cos(self.yaw + self.yaw_rate * dt))
        self.time += dt
        self.hall.update(self.time, self.motor.distance)

    def wheel_speed_report(self) -> float:
        """What velocity_report would publish now (unsigned, quantised, lagged)."""
        return self.hall.report(self.time)


# ---------------------------------------------------------------- small maths

def quat_multiply(a, b):
    """Hamilton product of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    """Rotate vector `v` by unit quaternion `q`."""
    p = (v[0], v[1], v[2], 0.0)
    r = quat_multiply(quat_multiply(q, p), quat_conjugate(q))
    return (r[0], r[1], r[2])


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
