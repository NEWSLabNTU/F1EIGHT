"""Deterministic lockstep simulation: the car's code against the plant, one process, no ROS.

    plant --edges--> VelocityEstimator --20 Hz--> ActuatorCore (shell + SpeedPID) --100 Hz--> PWM
      ^                                                ^ target, steering                      |
      |                                     scenario + pursuit tracker                         |
      +---------------------------- 1 kHz, PWM held between ticks -----------------------------+

VelocityEstimator and ActuatorCore are the classes velocity_report and the
actuator run on the car (autosdv_vehicle_interface). The actuator's
parameters come from the same files the launch loads: actuator.yaml,
vehicle_info.param.yaml and config/actuator_rc_sim.param.yaml, with
speed_controller_dt_source 'loop' as the Lab 2 launches use. No wall clock, no
messages, no libm (see detmath): a run is bit-identical on any machine.
"""
import csv
from dataclasses import dataclass
import hashlib
import io
from pathlib import Path
from typing import List, Optional

import yaml

from autosdv_vehicle_interface.actuator_core import (
    ActuatorCore, build_controller_params, normalize_params)
from autosdv_vehicle_interface.speed_controller import DEFAULT_SPEED_CONTROLLER, load_speed_controller
from autosdv_vehicle_interface.velocity_estimator import VelocityEstimator

from autosdv_rc_sim.detmath import atan, atan2, sin
from autosdv_rc_sim.params import VehicleParams
from autosdv_rc_sim.plant import Plant
from autosdv_rc_sim.track import TrackProfile

DT = 0.001
DT_NS = 1_000_000
CONTROL_NS = 10_000_000
REPORT_NS = 50_000_000
COLUMNS = ('t', 's', 'y', 'yaw', 'target', 'measured', 'speed', 'pwm', 'steer_pwm', 'mode',
           'kappa_r', 'slip_r', 'ax', 'delta', 'emergency', 'phase')
_INT_COLUMNS = {'pwm', 'steer_pwm', 'emergency'}
_STR_COLUMNS = {'mode', 'phase'}

REPO = Path(__file__).resolve().parents[4]
_SOURCE_DIRS = {
    'autosdv_vehicle_interface': REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface',
    'autosdv_vehicle_description': REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description',
    'autosdv_rc_sim': REPO / 'src/sim/autosdv_rc_sim',
}


def share_file(pkg: str, rel: str) -> Path:
    """A package's file: from this checkout if present, else from the installed share."""
    source = _SOURCE_DIRS[pkg] / rel
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(pkg)) / rel


def read_ros_params(path) -> dict:
    with open(path) as f:
        return dict(yaml.safe_load(f)['/**']['ros__parameters'])


def load_actuator_params(extra: Optional[dict] = None) -> dict:
    raw = {}
    raw.update(read_ros_params(share_file('autosdv_vehicle_interface', 'params/actuator.yaml')))
    raw.update(read_ros_params(share_file('autosdv_vehicle_description',
                                          'config/vehicle_info.param.yaml')))
    raw.update(read_ros_params(share_file('autosdv_rc_sim', 'config/actuator_rc_sim.param.yaml')))
    raw['speed_controller_dt_source'] = 'loop'
    raw.update(extra or {})
    return normalize_params(raw)


def load_velocity_report_params() -> dict:
    return read_ros_params(share_file('autosdv_vehicle_interface', 'params/velocity_report.yaml'))


def check_timing(vehicle: VehicleParams, actuator_params: dict, velocity_report_params: dict) -> None:
    """The harness's fixed step and periods must be the ones the car and the plant node use."""
    problems = []
    if int(round(vehicle.physics_dt * 1e9)) != DT_NS:
        problems.append(f'physics_dt {vehicle.physics_dt} s != the harness step {DT_NS} ns')
    if int(round(1e9 / float(actuator_params['rate']))) != CONTROL_NS:
        problems.append(f"actuator rate {actuator_params['rate']} Hz != the harness control "
                        f'period {CONTROL_NS} ns')
    if int(round(1e9 / float(velocity_report_params['rate']))) != REPORT_NS:
        problems.append(f"velocity_report rate {velocity_report_params['rate']} Hz != the "
                        f'harness report period {REPORT_NS} ns')
    if problems:
        raise ValueError('lockstep timing disagrees with the car: ' + '; '.join(problems))


class _QuietLogger:
    def _drop(self, *args, **kwargs):
        pass

    debug = info = warning = warn = error = fatal = _drop


def pursuit(x: float, y: float, yaw: float, v: float, wheel_base: float, max_angle: float) -> float:
    """Pure pursuit onto the straight's centre line (y = 0); stands in for Autoware's tracker."""
    lookahead = 1.0 + 0.3 * abs(v)
    alpha = atan2(-y, lookahead) - yaw
    delta = atan(2.0 * wheel_base * sin(alpha) / lookahead)
    return max(-max_angle, min(max_angle, delta))


class ProfileScenario:
    """The graded run: the straight's position-based profile to the stop line."""

    name = 'profile'

    def __init__(self, profile: TrackProfile, settle: float = 2.0):
        self.profile = profile
        self.settle_ns = int(round(settle * 1e9))
        self._still_since = None

    def command(self, t_ns: int, s: float, v: float):
        return self.profile.v_target(s), False, 'run'

    def done(self, t_ns: int, s: float, v: float, target: float) -> bool:
        if s > self.profile.stop_line_s - 2.0 and abs(v) < 0.01:
            if self._still_since is None:
                self._still_since = t_ns
            return t_ns - self._still_since >= self.settle_ns
        self._still_since = None
        return False


class EstopScenario:
    """The profile until mid top-speed hold; then target 0 with the gate's emergency; resume."""

    name = 'estop'

    def __init__(self, profile: TrackProfile, resume_speed: float = 6.0,
                 resume_accel: float = 2.5, resume_hold: float = 3.0):
        hold = [seg for seg in profile.segments if seg.kind == 'hold'][-1]
        self.profile = profile
        self.trigger_s = 0.5 * (hold.s0 + hold.s1)
        self.resume_speed = resume_speed
        self.resume_accel = resume_accel
        self.resume_ns = int(round((resume_speed / resume_accel + resume_hold) * 1e9))
        self.phase = 'run'
        self._still_since = None
        self._resume_at = None

    def command(self, t_ns: int, s: float, v: float):
        if self.phase == 'run':
            if s < self.trigger_s:
                return self.profile.v_target(s), False, 'run'
            self.phase = 'estop'
        if self.phase == 'estop':
            if abs(v) < 0.01:
                if self._still_since is None:
                    self._still_since = t_ns
                elif t_ns - self._still_since >= 1_000_000_000:
                    self.phase = 'resume'
                    self._resume_at = t_ns
            else:
                self._still_since = None
            if self.phase == 'estop':
                return 0.0, True, 'estop'
        elapsed = (t_ns - self._resume_at) / 1e9
        target = max(self.profile.engage_velocity,
                     min(self.resume_speed, self.resume_accel * elapsed))
        return target, False, 'resume'

    def done(self, t_ns: int, s: float, v: float, target: float) -> bool:
        return self.phase == 'resume' and t_ns - self._resume_at >= self.resume_ns


def _fmt(value) -> str:
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, float):
        return repr(value)
    return str(value)


@dataclass
class Trace:
    scenario: str
    variant: str
    rows: List[tuple]

    def column(self, name: str) -> list:
        i = COLUMNS.index(name)
        return [row[i] for row in self.rows]

    def csv_text(self) -> str:
        lines = [','.join(COLUMNS)]
        lines += [','.join(_fmt(v) for v in row) for row in self.rows]
        return '\n'.join(lines) + '\n'

    def digest(self) -> str:
        return hashlib.sha256(self.csv_text().encode()).hexdigest()

    def write_csv(self, path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(self.csv_text())

    @staticmethod
    def read_csv(path, scenario: str = '', variant: str = '') -> 'Trace':
        reader = csv.reader(io.StringIO(Path(path).read_text()))
        header = next(reader)
        assert tuple(header) == COLUMNS, f'{path}: unexpected columns {header}'
        rows = []
        for raw in reader:
            row = []
            for name, text in zip(COLUMNS, raw):
                if name in _STR_COLUMNS:
                    row.append(text)
                elif name == 'emergency':
                    row.append(text == '1')
                elif name in _INT_COLUMNS:
                    row.append(int(text))
                else:
                    row.append(float(text))
            rows.append(tuple(row))
        return Trace(scenario, variant, rows)


class Lockstep:
    def __init__(self, vehicle: VehicleParams, actuator_params: dict, controller_overrides: dict,
                 profile: TrackProfile, variant: str = 'nominal',
                 controller_class: Optional[str] = None):
        # controller_class: 'pkg.module:Class' as the actuator's speed_controller_class takes
        # it; None runs the car's default (AckermannSpeedPID), which the grader uses
        self.vehicle = vehicle
        self.controller_class = controller_class or DEFAULT_SPEED_CONTROLLER
        self.params = actuator_params
        self.overrides = dict(controller_overrides)
        self.profile = profile
        self.variant = variant
        vr = load_velocity_report_params()
        check_timing(vehicle, actuator_params, vr)
        # velocity_report's own expression, so the result is bit-identical to the node's
        self.distance_per_marker = vr['wheel_diameter'] / 100.0 * 3.141592653589793 / vr['markers_per_rotation']

    def run(self, scenario, max_time: float = 90.0) -> Trace:
        p = self.params
        controller = load_speed_controller(
            self.controller_class, build_controller_params(p, self.overrides))
        core = ActuatorCore(p, controller, logger=_QuietLogger(), start_time=0.0)
        estimator = VelocityEstimator(self.distance_per_marker)
        plant = Plant(self.vehicle, 0.0, 0.0, 0.0)
        motor, steer = p['init_pwm'], p['init_steer']
        measured = 0.0
        target, emergency, phase = 0.0, False, 'run'
        rows = []
        n_max = int(round(max_time * 1e9)) // DT_NS
        for k in range(n_max):
            t = k * DT_NS
            now = t / 1e9
            x, y, yaw = plant.base_pose
            v = plant.state.vx
            if t % REPORT_NS == 0:
                measured = estimator.report(t)
                core.on_velocity(now, now, measured)
            if t % CONTROL_NS == 0:
                target, emergency, phase = scenario.command(t, x, v)
                core.gate_emergency = emergency
                angle = pursuit(x, y, yaw, v, p['wheelbase'], p['max_steering_angle'])
                core.on_command(now, target, angle)
                motor, steer = core.tick(now, now)
                rows.append((now, x, y, yaw, target, measured, v, motor, steer,
                             core.longitudinal.mode.value, plant.kappa_rear, plant.slip_rear,
                             plant.state.ax, plant.delta, emergency, phase))
                if scenario.done(t, x, v, target):
                    break
            for edge in plant.step(DT, motor, steer).edges:
                estimator.on_edge(edge)
        return Trace(scenario.name, self.variant, rows)
