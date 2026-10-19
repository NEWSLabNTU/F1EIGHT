"""The RC simulator's vehicle parameters: one YAML file, every value tagged with its source.

See vehicle.yaml for the format and the meaning of the tags.
"""
from dataclasses import dataclass, fields
import math
from pathlib import Path
from typing import Any, Dict, List

import yaml

GRAVITY = 9.81
SOURCES = ('measured', 'config', 'literature', 'guess', 'mock', 'numerical')
DEFAULT_FILE = Path(__file__).resolve().parent / 'vehicle.yaml'


@dataclass(frozen=True)
class VehicleParams:
    """The model's parameters (SI units, PWM in PCA9685 counts); see vehicle.yaml."""

    mass: float
    yaw_inertia: float
    wheel_base: float
    lf: float
    cg_height: float
    wheel_radius: float
    wheel_axle_inertia: float
    rotor_inertia: float
    mu: float
    pacejka_c: float
    pacejka_e: float
    cornering_stiffness_front: float
    cornering_stiffness_rear: float
    longitudinal_stiffness_per_load: float
    rolling_resistance: float
    aero_drag: float
    drivetrain_coulomb: float
    drivetrain_breakaway: float
    drivetrain_viscous: float
    front_bearing_coulomb: float
    motor_kv: float
    motor_resistance: float
    gear_ratio: float
    battery_voltage: float
    battery_resistance: float
    pwm_freq: float
    init_pwm: int
    min_pwm: int
    max_pwm: int
    brake_pwm: int
    forward_start_pwm: int
    forward_hold_pwm: int
    reverse_stop_pwm: int
    reverse_start_pwm: int
    reverse_arm_pwm: int
    throttle_ramp: float
    drag_brake: float
    brake_torque_max: float
    esc_current_limit: float
    esc_regen_current_limit: float
    reverse_scale: float
    init_steer: int
    min_steer: int
    max_steer: int
    tire_angle_to_steer_ratio: float
    max_steer_angle: float
    servo_dead_time: float
    servo_rate_limit: float
    markers_per_rotation: int
    gyro_noise_std: float
    accel_noise_std: float
    gyro_bias_z: float
    physics_dt: float
    v_switch: float
    v_blend: float
    v_min: float
    stick_band: float

    @property
    def lr(self) -> float:
        return self.wheel_base - self.lf

    @property
    def kt(self) -> float:
        """Torque constant [N m/A] = back-EMF constant [V s/rad] = 60 / (2 pi Kv)."""
        return 60.0 / (2.0 * math.pi * self.motor_kv)

    @property
    def ke(self) -> float:
        return self.kt

    @property
    def front_axle_inertia(self) -> float:
        return self.wheel_axle_inertia

    @property
    def rear_axle_inertia(self) -> float:
        return self.wheel_axle_inertia + self.rotor_inertia * self.gear_ratio * self.gear_ratio

    @property
    def effective_mass(self) -> float:
        """Mass plus both axles' rotational inertia, referred to the ground."""
        r2 = self.wheel_radius * self.wheel_radius
        return self.mass + (self.front_axle_inertia + self.rear_axle_inertia) / r2

    @property
    def fz_front_static(self) -> float:
        return self.mass * GRAVITY * self.lr / self.wheel_base

    @property
    def fz_rear_static(self) -> float:
        return self.mass * GRAVITY * self.lf / self.wheel_base

    @property
    def b_lat_front(self) -> float:
        """Pacejka B, front lateral: C*B*mu*Fz equals the cornering stiffness at static load."""
        return self.cornering_stiffness_front / (self.pacejka_c * self.mu * self.fz_front_static)

    @property
    def b_lat_rear(self) -> float:
        return self.cornering_stiffness_rear / (self.pacejka_c * self.mu * self.fz_rear_static)

    @property
    def b_long(self) -> float:
        return self.longitudinal_stiffness_per_load / (self.pacejka_c * self.mu)


_INT_FIELDS = {f.name for f in fields(VehicleParams) if f.type in (int, 'int')}  # 'int' under postponed annotations
_NAMES = [f.name for f in fields(VehicleParams)]


@dataclass
class TaggedFile:
    entries: Dict[str, Dict[str, Any]]
    variants: Dict[str, Dict[str, Any]]

    def values(self) -> Dict[str, Any]:
        return {key: entry['value'] for key, entry in self.entries.items()}


def load_tagged(path=None) -> TaggedFile:
    """Read and check the tagged file: every field once, every tag known."""
    path = Path(path) if path else DEFAULT_FILE
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    entries = data.get('vehicle') or {}
    variants = data.get('variants') or {}
    missing = [n for n in _NAMES if n not in entries]
    unknown = [k for k in entries if k not in _NAMES]
    if missing or unknown:
        raise ValueError(f'{path}: missing {missing}, unknown {unknown}')
    for key, entry in entries.items():
        if not isinstance(entry, dict) or 'value' not in entry or 'source' not in entry:
            raise ValueError(f'{path}: {key} must be {{value: ..., source: ...}}')
        if entry['source'] not in SOURCES:
            raise ValueError(f"{path}: {key}: source {entry['source']!r} is not one of {SOURCES}")
        if entry['source'] == 'config' and 'from' not in entry:
            raise ValueError(f'{path}: {key} is tagged config but names no file (from: pkg/path:key)')
    if 'nominal' not in variants:
        raise ValueError(f'{path}: variants must include nominal')
    for name, over in variants.items():
        bad = [k for k in (over or {}) if k not in _NAMES]
        if bad:
            raise ValueError(f'{path}: variant {name} sets unknown {bad}')
    return TaggedFile(entries, {k: dict(v or {}) for k, v in variants.items()})


def _coerce(name: str, value):
    if name in _INT_FIELDS:
        if isinstance(value, bool) or int(value) != value:
            raise ValueError(f'{name} must be an integer, got {value!r}')
        return int(value)
    return float(value)


def validate(p: VehicleParams) -> None:
    """Refuse parameter sets the model cannot run."""
    if not 0.0 < p.lf < p.wheel_base:
        raise ValueError(f'lf ({p.lf}) must lie inside the wheelbase ({p.wheel_base})')
    if p.pacejka_c < 1.0 or not 0.0 <= p.pacejka_e < 1.0:
        raise ValueError('need pacejka_c >= 1 (the peak is reached) and 0 <= pacejka_e < 1')
    if not (p.reverse_arm_pwm < p.reverse_start_pwm < p.reverse_stop_pwm <= p.init_pwm
            <= p.forward_hold_pwm < p.forward_start_pwm < p.max_pwm):
        raise ValueError('the ESC thresholds must order as reverse_arm < reverse_start < '
                         'reverse_stop <= init <= forward_hold < forward_start < max_pwm')
    if not p.reverse_arm_pwm < p.min_pwm < p.reverse_stop_pwm:
        raise ValueError('min_pwm must lie between reverse_arm_pwm and reverse_stop_pwm: '
                         'the full brake, which never arms reverse')
    for name in ('mass', 'yaw_inertia', 'wheel_radius', 'wheel_axle_inertia', 'mu', 'motor_kv',
                 'motor_resistance', 'gear_ratio', 'battery_voltage', 'pwm_freq', 'physics_dt',
                 'esc_current_limit', 'esc_regen_current_limit',
                 'v_min', 'v_blend', 'stick_band', 'servo_rate_limit'):
        if getattr(p, name) <= 0.0:
            raise ValueError(f'{name} must be positive')


def load_vehicle(path=None, variant: str = 'nominal', **overrides) -> VehicleParams:
    """The parameters of one variant, with optional per-field overrides (tests)."""
    tagged = load_tagged(path)
    if variant not in tagged.variants:
        raise ValueError(f'unknown variant {variant!r}; known: {sorted(tagged.variants)}')
    values = tagged.values()
    values.update(tagged.variants[variant])
    values.update(overrides)
    p = VehicleParams(**{k: _coerce(k, v) for k, v in values.items()})
    validate(p)
    return p


def variant_names(path=None) -> List[str]:
    return list(load_tagged(path).variants)
