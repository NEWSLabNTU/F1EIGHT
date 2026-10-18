"""The car: PWM counts in, motion and hall edges out. No ROS.

    PCA9685 frame latch (60 Hz) -> ESC -> drivetrain -> rear axle  ┐
                                 -> servo -> steering angle         ├-> chassis -> hall edges
                                                                    ┘
The PCA9685 emits one pulse per frame, so a write reaches the ESC and the
servo at the next frame start: up to 16.7 ms of hold the real car has too.
"""
from dataclasses import dataclass
from typing import List, Tuple

from autosdv_rc_sim.chassis import Chassis, ChassisState
from autosdv_rc_sim.detmath import cos, sin
from autosdv_rc_sim.drivetrain import Drivetrain
from autosdv_rc_sim.esc import Esc, EscOutput
from autosdv_rc_sim.hall import HallSensor
from autosdv_rc_sim.params import VehicleParams
from autosdv_rc_sim.servo import Servo


class Pca9685Latch:
    def __init__(self, pwm_freq: float, init_motor: int, init_steer: int):
        self.frame_ns = int(round(1e9 / pwm_freq))
        self.next_ns = 0
        self.motor = int(init_motor)
        self.steer = int(init_steer)

    def sample(self, t_ns: int, motor: int, steer: int) -> Tuple[int, int]:
        if t_ns >= self.next_ns:
            self.motor, self.steer = int(motor), int(steer)
            while self.next_ns <= t_ns:
                self.next_ns += self.frame_ns
        return self.motor, self.steer


@dataclass(frozen=True)
class StepResult:
    state: ChassisState
    edges: List[int]


class Plant:
    def __init__(self, p: VehicleParams, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.p = p
        self.latch = Pca9685Latch(p.pwm_freq, p.init_pwm, p.init_steer)
        self.esc = Esc(p)
        self.drivetrain = Drivetrain(p)
        self.servo = Servo(p)
        self.chassis = Chassis(p)
        self.hall = HallSensor(p.markers_per_rotation)
        self.t_ns = 0
        self.esc_out = EscOutput(0.0, 0.0, 0.0, 'neutral')
        self.reset_pose(x, y, yaw)

    def reset_pose(self, x: float, y: float, yaw: float) -> None:
        """Place base_link (the rear axle centre) at (x, y, yaw), at rest."""
        lr = self.p.lr
        self.state = ChassisState(x=x + lr * cos(yaw), y=y + lr * sin(yaw), yaw=yaw,
                                  theta_r=self.hall.start_angle)
        self.hall.reset(self.hall.start_angle)
        self.servo.reset()

    @property
    def base_pose(self) -> Tuple[float, float, float]:
        s = self.state
        return s.x - self.p.lr * cos(s.yaw), s.y - self.p.lr * sin(s.yaw), s.yaw

    @property
    def base_twist(self) -> Tuple[float, float, float]:
        s = self.state
        return s.vx, s.vy - self.p.lr * s.yaw_rate, s.yaw_rate

    @property
    def delta(self) -> float:
        return self.servo.delta

    @property
    def slip_rear(self) -> float:
        """Rear wheel surface speed minus ground speed [m/s] (spin > 0, lock < 0)."""
        return self.state.omega_r * self.p.wheel_radius - self.state.vx

    @property
    def kappa_rear(self) -> float:
        return 0.0 if self.chassis.last is None else self.chassis.last.kappa_r

    def step(self, dt: float, pwm_motor: int, pwm_steer: int) -> StepResult:
        p = self.p
        dt_ns = int(round(dt * 1e9))
        motor, steer = self.latch.sample(self.t_ns, pwm_motor, pwm_steer)
        self.esc_out = self.esc.step(self.t_ns, motor, self.state.omega_r * p.wheel_radius, dt)
        tau, dtau = self.drivetrain.torque(self.esc_out.duty, self.esc_out.short, self.state.omega_r)
        delta = self.servo.step(steer, dt)
        theta0 = self.state.theta_r
        self.state = self.chassis.step(self.state, tau, dtau, self.esc_out.brake, delta, dt)
        edges = self.hall.step(self.t_ns, dt_ns, theta0, self.state.theta_r)
        self.t_ns += dt_ns
        return StepResult(self.state, edges)
