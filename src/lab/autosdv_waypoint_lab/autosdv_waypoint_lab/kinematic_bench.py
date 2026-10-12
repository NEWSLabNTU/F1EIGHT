"""Offline bench for a PathTracker: kinematic bicycle, no ROS, no actuator.

The Part 1 tests and the reference tuning run here. The car is a kinematic
bicycle about the rear axle with a first-order steering servo and a speed that
follows the command with a first-order lag and an acceleration limit -- a
stand-in for the actuator's SpeedPID, good enough to separate tracking from
speed control. The steering command is rate-limited to 1.0 rad/s before the
servo lag, as vehicle_cmd_gate limits it on the car (autoware_launch's
vehicle_cmd_gate.param.yaml, nominal steer_rate_lim_for_steer_cmd, 1.0 rad/s
below 20 m/s); the actuator itself applies no rate limit. The tracker sees exactly what `path_tracker` gives it on the
car: the course window ahead, from `CoursePath.window`, and a pose that can
carry localization noise (pose_noise_std, yaw_noise_std: Gaussian, redrawn
every cycle) while the metrics use the true pose.

    result = run(tracker, course_path, start=(x, y, yaw))
    result.cross_track_rmse, result.cross_track_max, result.finished
"""

from dataclasses import dataclass, field
import math
import random
from typing import List, Optional, Tuple

from autosdv_waypoint_lab.path import CoursePath, wrap_angle
from autosdv_waypoint_lab.tracker_api import (
    TrackerShell, VehicleGeometry, VehicleState)


@dataclass
class BenchParams:
    dt: float = 1.0 / 30.0           # tracker rate, as path_tracker
    steer_time_constant: float = 0.1
    steer_rate_max: float = 1.0      # [rad/s] vehicle_cmd_gate's steering rate limit
    speed_time_constant: float = 0.3
    accel_max: float = 1.0
    decel_max: float = 1.5
    window_behind: float = 1.0
    window_ahead: float = 6.0
    timeout: float = 300.0
    pose_noise_std: float = 0.0      # [m] on the x and y the tracker sees
    yaw_noise_std: float = 0.0       # [rad] on the yaw it sees
    seed: int = 1


@dataclass
class BenchResult:
    t: List[float] = field(default_factory=list)
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    yaw: List[float] = field(default_factory=list)
    speed: List[float] = field(default_factory=list)
    steer: List[float] = field(default_factory=list)
    cross_track: List[float] = field(default_factory=list)
    finished: bool = False
    laps: int = 0
    faults: int = 0
    disabled: bool = False
    max_steer_cmd: float = 0.0

    @property
    def moving_cross_track(self) -> List[float]:
        return [e for e, v in zip(self.cross_track, self.speed) if v > 0.05]

    @property
    def cross_track_rmse(self) -> float:
        e = self.moving_cross_track
        return math.sqrt(sum(v * v for v in e) / len(e)) if e else math.nan

    @property
    def cross_track_max(self) -> float:
        e = self.moving_cross_track
        return max(abs(v) for v in e) if e else math.nan

    @property
    def duration(self) -> float:
        return self.t[-1] if self.t else 0.0


def run(tracker, course: CoursePath, start: Tuple[float, float, float],
        vehicle: Optional[VehicleGeometry] = None,
        params: Optional[BenchParams] = None) -> BenchResult:
    vehicle = vehicle or VehicleGeometry()
    p = params or BenchParams()
    shell = TrackerShell(tracker, vehicle)
    shell.reset()
    x, y, yaw = start
    v = 0.0
    delta = 0.0
    delta_cmd = 0.0                  # after the gate's rate limit
    t = 0.0
    out = BenchResult()
    stopped_since = None
    rng = random.Random(p.seed)
    while t < p.timeout:
        course.update_progress(x, y)
        window = course.window(p.window_behind, p.window_ahead)
        seen = VehicleState(x + rng.gauss(0.0, p.pose_noise_std),
                            y + rng.gauss(0.0, p.pose_noise_std),
                            wrap_angle(yaw + rng.gauss(0.0, p.yaw_noise_std)), v)
        res = shell.step(seen, window, p.dt)
        if res.fault:
            out.faults += 1
        out.disabled = res.disabled
        cmd = res.command
        out.max_steer_cmd = max(out.max_steer_cmd, abs(cmd.steering_angle))
        cross = window.project(x, y).cross_track

        out.t.append(t)
        out.x.append(x)
        out.y.append(y)
        out.yaw.append(yaw)
        out.speed.append(v)
        out.steer.append(delta)
        out.cross_track.append(cross)

        # plant
        a = (cmd.speed - v) / p.speed_time_constant
        a = max(-p.decel_max, min(p.accel_max, a))
        v = max(0.0, v + a * p.dt)
        step = p.steer_rate_max * p.dt
        delta_cmd += max(-step, min(step, cmd.steering_angle - delta_cmd))
        k = 1.0 - math.exp(-p.dt / p.steer_time_constant)
        delta += (delta_cmd - delta) * k
        yaw_rate = v * math.tan(delta) / vehicle.wheel_base
        yaw_mid = yaw + 0.5 * yaw_rate * p.dt
        x += v * math.cos(yaw_mid) * p.dt
        y += v * math.sin(yaw_mid) * p.dt
        yaw = wrap_angle(yaw + yaw_rate * p.dt)
        t += p.dt

        if course.finished and v < 0.01:
            out.finished = True
            break
        # a car standing still with nothing more to do: the law never moves it
        if v < 0.01 and cmd.speed < 0.01:
            stopped_since = stopped_since if stopped_since is not None else t
            if t - stopped_since > 5.0:
                break
        else:
            stopped_since = None
    out.laps = course.laps_done
    return out
