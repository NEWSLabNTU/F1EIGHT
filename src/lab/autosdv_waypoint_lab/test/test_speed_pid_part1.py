"""Lab 2 Part 1 SpeedPID: the stub (= the handout skeleton), its given parts, its detuned
defaults and the handout contract. No ROS. Survives the student branch, where the reference
and its tests (test_speed_pid_part1_reference.py) are removed.

    cd src/lab/autosdv_waypoint_lab && python3 -m pytest test/test_speed_pid_part1.py
"""
import ast
import importlib
import math
from pathlib import Path
import sys

import pytest
import yaml

from autosdv_waypoint_lab.stub import speed_pid_part1 as stub_module
from autosdv_waypoint_lab.stub.speed_pid_part1 import SpeedPID as Stub

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
DEFAULTS_YAML = PKG / 'config' / 'speed_pid_part1.yaml'
DEFAULTS = yaml.safe_load(DEFAULTS_YAML.read_text())
ACTUATOR = {'init_pwm': 370, 'min_pwm': 360, 'max_pwm': 470, 'brake_pwm': 340, 'rate': 100.0}
# the course repository's byte copy of the stub (every team model's speed_pid.py starts as it)
COURSE_STUB = Path.home() / 'repos/ntu-sdv-course/2026/labs/Lab2/ta/reference/stub/speed_pid.py'
# Modules the student's speed_pid.py may import (the Lab 2 handout's list)
ALLOWED = {'math', 'cmath', 'dataclasses', 'typing', 'collections', 'enum', 'functools',
           'itertools', 'numbers', 'statistics', 'abc', '__future__', 'numpy'}


def pid(cls=Stub, **over):
    """A controller on the lab car's PWM scale: no feed-forward, no D, unless asked for."""
    params = dict(ACTUATOR, kp=0.0, ki=0.0, kd=0.0, integral_limit=1000.0, error_limit=100.0,
                  enable_conditional_integration=False, derivative_filter_alpha=1.0,
                  max_pwm_offset=100, max_brake_offset=10, ff_deadband=0.0, ff_gain=0.0,
                  kp_brake=10.0, brake_enter=0.15, brake_exit=0.05, brake_deadband=5)
    params.update(over)
    return cls(params)


def submission_module():
    sys.path.insert(0, str(REPO / 'scripts/testing/lab2'))
    try:
        return importlib.import_module('submission')
    finally:
        sys.path.pop(0)


# ---------------------------------------------------------------- the given parts

def test_feed_forward_is_the_given_formula():
    c = Stub(dict(ACTUATOR, **DEFAULTS))
    want = DEFAULTS['ff_deadband'] + 6.0 / DEFAULTS['ff_gain']
    assert c.feed_forward(6.0) == pytest.approx(want)
    assert c.feed_forward(0.0) == 0.0 and c.feed_forward(-1.0) == 0.0
    assert pid(ff_deadband=5.0, ff_gain=0.0).feed_forward(3.0) == 0.0   # no division by zero


def test_defaults_are_the_fresh_pack_fit_and_under_drive_the_nominal_car():
    # fits (config comment): 12.4 V PWM = 375.273 + 6.9795 v; nominal 11.1 V 375.249 + 7.8219 v
    c = Stub(dict(ACTUATOR, **DEFAULTS))
    for v in (2.0, 6.0, 11.0):
        ff = 370 + c.feed_forward(v)
        assert ff == pytest.approx(375.273 + 6.9795 * v, abs=0.5)
        assert ff < 375.249 + 7.8219 * v - 0.5 * v      # short of what the 11.1 V car needs
    assert 375.249 + 7.8219 * 11 - (370 + c.feed_forward(11.0)) == pytest.approx(9.6, abs=0.5)


def test_filter_derivative_is_an_ema_reset_to_zero():
    c = pid(derivative_filter_alpha=0.25)
    assert c.filter_derivative(4.0) == pytest.approx(1.0)
    assert c.filter_derivative(4.0) == pytest.approx(1.75)
    c.reset()
    assert c.derivative == 0.0


def test_clamp_pwm_rounds_inside_the_scale():
    c = pid()
    assert c.clamp_pwm(400.4) == 400 and c.clamp_pwm(400.6) == 401
    assert c.clamp_pwm(1000) == 470 and c.clamp_pwm(0) == 360
    assert isinstance(c.clamp_pwm(400.0), int)


def test_offsets_are_capped_to_the_pwm_scale():
    c = pid(max_pwm_offset=500, max_brake_offset=50)
    assert c.max_pwm_offset == 100 and c.max_brake_offset == 10
    c = pid(init_pwm=365, max_brake_offset=10)
    assert c.max_brake_offset == 5


def test_class_defaults_match_the_defaults_file():
    c = Stub(dict(ACTUATOR))
    for key, value in DEFAULTS.items():
        assert getattr(c, key) == value, key


def test_brake_pwm_skips_the_dead_band():
    c = pid()
    assert c.brake_pwm(0.0) == 370 and c.brake_pwm(-3.0) == 370       # no request: coast
    assert c.brake_pwm(1.0) == 364                                     # first count of brake
    assert c.brake_pwm(3.0) == 362
    assert c.brake_pwm(50.0) == 360                                    # full brake
    assert pid(max_brake_offset=8).brake_pwm(50.0) == 362
    assert pid(max_brake_offset=5).brake_pwm(50.0) == 370              # no brake authority
    assert pid(max_brake_offset=0).brake_pwm(50.0) == 370
    assert pid(brake_deadband=0).brake_pwm(1.0) == 369
    c = pid(init_pwm=365, min_pwm=360, max_brake_offset=10)            # capped by min_pwm
    assert c.max_brake_offset == 5 and c.brake_pwm(50.0) == 365   # = the dead band: no brake


# ---------------------------------------------------------------- the stub

def test_stub_holds_neutral():
    c = Stub(dict(ACTUATOR, **DEFAULTS))
    for target, measured, dt in [(2.0, 0.0, 0.0), (6.0, 5.0, 0.01), (0.0, 3.0, 0.01),
                                 (11.0, 11.5, 0.02)]:
        pwm = c.update(target, measured, dt)
        assert pwm == 370 and isinstance(pwm, int)
    c.reset()
    assert Stub(dict(ACTUATOR, init_pwm=380)).update(2.0, 0.0, 0.01) == 380


def _violations(path: Path) -> list:
    out = []
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            out += [a.name for a in node.names if a.name.split('.')[0] not in ALLOWED]
        elif isinstance(node, ast.ImportFrom):
            if node.level or (node.module or '').split('.')[0] not in ALLOWED:
                out.append(node.module or '.')
        elif isinstance(node, ast.Name) and node.id in ('open', 'exec', 'eval', '__import__'):
            out.append(node.id)
    return out


def test_stub_keeps_the_handout_rules():
    assert _violations(Path(stub_module.__file__)) == []


def test_stub_has_no_reference_code():
    text = Path(stub_module.__file__).read_text()
    assert 'self.ki * error' not in text and 'reference' not in text.lower()


def test_defaults_file_passes_the_submission_check():
    sub = submission_module()
    assert set(DEFAULTS) == set(sub.part1_keys())
    checked, warnings = sub.check_part1_yaml(DEFAULTS_YAML)
    assert warnings == [] and checked == DEFAULTS


@pytest.mark.skipif(not COURSE_STUB.is_file(), reason='course repository not checked out')
def test_the_course_copy_matches_the_stub():
    assert COURSE_STUB.read_bytes() == Path(stub_module.__file__).read_bytes()


def test_math_is_the_only_import_needed():
    # documents the stub's own imports: none, so a student starts from a clean slate
    tree = ast.parse(Path(stub_module.__file__).read_text())
    assert not any(isinstance(n, (ast.Import, ast.ImportFrom)) for n in ast.walk(tree))
    assert math.isfinite(Stub(dict(ACTUATOR)).feed_forward(1.0))
