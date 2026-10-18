"""Lab 2 Part 1 SpeedPID: the TA reference's behaviour (the TODOs, completed) and its tuned
parameters. No ROS. TA-only: removed from the student branch with reference/speed_pid_part1.py
(scripts/testing/lab2/strip_student_branch.sh).

    cd src/lab/autosdv_waypoint_lab && python3 -m pytest test/test_speed_pid_part1_reference.py
"""
import importlib
from pathlib import Path
import sys

import pytest
import yaml

from autosdv_waypoint_lab.reference.speed_pid_part1 import SpeedPID as Reference
from autosdv_waypoint_lab.stub.speed_pid_part1 import SpeedPID as Stub

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
DEFAULTS_YAML = PKG / 'config' / 'speed_pid_part1_reference.yaml'   # tuned
DEFAULTS = yaml.safe_load(DEFAULTS_YAML.read_text())
ACTUATOR = {'init_pwm': 370, 'min_pwm': 360, 'max_pwm': 470, 'brake_pwm': 340, 'rate': 100.0}


def pid(cls=Reference, **over):
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


# ---------------------------------------------------------------- the reference's TODOs

def test_p_term():
    c = pid(kp=10.0)
    assert c.update(1.0, 0.5, 0.01) == 375
    assert c.proportional == pytest.approx(5.0)
    assert c.debug_terms() == (pytest.approx(5.0), 0.0, 0.0)


def test_error_limit_clamps_what_p_and_i_see_while_driving():
    assert pid(kp=10.0, error_limit=0.3).update(2.0, 0.0, 0.01) == 373
    c = pid(ki=10.0, error_limit=0.3)
    for _ in range(4):
        c.update(2.0, 0.0, 0.1)
    assert c.integral == pytest.approx(1.2)
    # braking uses the full error: 1 m/s too fast with kp_brake 10 is full brake
    assert pid(kp_brake=10.0, error_limit=0.3).update(6.0, 7.0, 0.01) == 360


def test_output_is_built_on_params_init_pwm():
    a, b = pid(kp=10.0), pid(kp=10.0, init_pwm=380, min_pwm=370, max_pwm=480)
    assert b.update(1.0, 0.5, 0.01) - a.update(1.0, 0.5, 0.01) == 10


def test_feed_forward_enters_the_output():
    c = pid(ff_deadband=5.0, ff_gain=0.1)
    assert c.update(2.0, 2.0, 0.01) == 370 + 5 + 20


def test_integral_accumulates_ki_e_dt_and_is_clamped():
    c = pid(ki=10.0, integral_limit=3.0)
    for _ in range(10):
        c.update(1.0, 0.0, 0.1)            # ki * e * dt = 1 count per call
    assert c.integral == pytest.approx(3.0)
    c = pid(ki=10.0, integral_limit=100.0)
    for _ in range(4):
        c.update(1.0, 0.5, 0.1)
    assert c.integral == pytest.approx(2.0)


def test_no_integration_when_dt_is_zero():
    c = pid(ki=10.0)
    c.update(1.0, 0.0, 0.0)
    assert c.integral == 0.0


def test_integral_clamp_is_symmetric():
    c = pid(ki=10.0, integral_limit=2.0, brake_enter=5.0, brake_exit=4.0)   # never brakes
    for _ in range(10):
        c.update(0.0, 1.0, 0.1)
    assert c.integral == pytest.approx(-2.0)


def test_conditional_integration_freezes_while_saturated():
    # kp alone saturates the throttle (kp * e = 500 > 100): the integral must not grow
    frozen = pid(kp=500.0, ki=10.0, enable_conditional_integration=True)
    free = pid(kp=500.0, ki=10.0, enable_conditional_integration=False)
    for _ in range(10):
        assert frozen.update(1.0, 0.0, 0.1) == 470
        free.update(1.0, 0.0, 0.1)
    assert frozen.saturated
    assert frozen.integral == pytest.approx(1.0)    # the first call, before any saturation
    assert free.integral == pytest.approx(10.0)


def test_conditional_integration_resumes_after_saturation():
    c = pid(kp=500.0, ki=10.0, enable_conditional_integration=True)
    c.update(1.0, 0.0, 0.1)
    c.update(1.0, 0.0, 0.1)
    before = c.integral
    c.update(1.0, 0.99, 0.1)                   # 5 counts: not saturated
    assert not c.saturated
    c.update(1.0, 0.99, 0.1)
    assert c.integral > before


def test_driving_never_goes_below_neutral():
    # too fast by 0.1 m/s, inside the hysteresis band: driving, the command clipped at neutral
    c = pid(kp=100.0)
    assert c.update(1.0, 1.1, 0.01) == 370
    assert not c.braking and c.saturated


def test_derivative_on_measurement_not_on_error():
    c = pid(kd=1.0)
    c.update(1.0, 0.5, 0.01)
    # a step in the target alone: no derivative kick
    c.update(3.0, 0.5, 0.01)
    assert c.derivative == 0.0
    # the measurement rising 0.1 m/s in 0.01 s: D = -kd * 10 = -10 (alpha 1: unfiltered)
    c.update(3.0, 0.6, 0.01)
    assert c.derivative == pytest.approx(-10.0)


def test_derivative_goes_through_the_filter():
    c = pid(kd=1.0, derivative_filter_alpha=0.5)
    c.update(1.0, 0.5, 0.01)
    c.update(1.0, 0.6, 0.01)                 # raw -10, filtered -5
    assert c.derivative == pytest.approx(-5.0)
    c.update(1.0, 0.6, 0.01)                 # raw 0, filtered -2.5
    assert c.derivative == pytest.approx(-2.5)


def test_no_derivative_on_the_first_call_or_without_dt():
    c = pid(kd=100.0)
    assert c.update(1.0, 0.9, 0.0) == 370    # first call, dt 0: D 0, P 0
    c = pid(kd=100.0)
    c.update(1.0, 0.5, 0.01)                 # first call with dt: no previous measurement
    assert c.derivative == 0.0
    c.update(1.0, 0.9, 0.0)                  # dt 0 later: no division, D 0
    assert c.update(1.0, 0.9, 0.0) == 370


def test_dt_is_used_not_assumed():
    fast, slow = pid(kd=1.0), pid(kd=1.0)
    fast.update(1.0, 0.5, 0.01)
    fast.update(1.0, 0.6, 0.01)
    slow.update(1.0, 0.5, 0.02)
    slow.update(1.0, 0.6, 0.02)
    assert slow.derivative == pytest.approx(fast.derivative / 2)


# ---------------------------------------------------------------- the braking mode

def test_braking_mode_enters_and_leaves_with_hysteresis():
    c = pid(kp=10.0)
    c.update(6.0, 6.1, 0.01)
    assert not c.braking                                  # 0.1 too fast: inside the band
    assert c.update(6.0, 6.3, 0.01) == 370 - 5 - 3        # 0.3 > 0.15: -kp_brake e = 3
    assert c.braking
    assert c.update(6.0, 6.1, 0.01) == 370 - 5 - 1        # 0.1: still above exit 0.05
    assert c.braking
    c.update(6.0, 6.02, 0.01)
    assert not c.braking                                  # 0.02 < exit: driving again


def test_braking_drops_the_feed_forward_and_freezes_the_integral():
    c = pid(kp=10.0, ki=10.0, ff_deadband=5.0, ff_gain=0.1)
    c.update(6.0, 5.9, 0.1)
    held = c.integral
    for _ in range(5):
        assert c.update(6.0, 7.0, 0.1) == 360             # FF (65 counts) gone, full brake
    assert c.integral == held


def test_braking_request_includes_the_derivative():
    c = pid(kp=10.0, kd=1.0)
    c.update(6.0, 6.5, 0.01)                              # enters braking, no D yet
    pwm = c.update(6.0, 6.4, 0.01)                        # slowing 10 m/s^2: D = +10 counts
    assert pwm == 370                                     # request -(-4 + 10) < 0: coast


def test_braking_uses_its_own_gain():
    c = pid(kp=0.0, kp_brake=20.0)
    assert c.update(6.0, 6.2, 0.01) == 370 - 5 - 4        # -20 * -0.2 = 4 counts, not kp's 0


def test_brake_disabled_with_max_brake_offset_zero():
    c = pid(kp=100.0, max_brake_offset=0)
    assert c.update(1.0, 3.0, 0.01) == 370


def test_reset_leaves_the_braking_mode():
    c = pid(kp=10.0)
    c.update(6.0, 7.0, 0.01)
    assert c.braking
    c.reset()
    assert not c.braking


def test_throttle_is_limited_by_max_pwm_offset():
    assert pid(kp=1000.0, max_pwm_offset=50).update(5.0, 0.0, 0.01) == 420
    assert pid(kp=1000.0, max_pwm_offset=100).update(5.0, 0.0, 0.01) == 470
    # a car with a wider scale still reaches its own max_pwm
    assert pid(kp=1000.0, init_pwm=380, min_pwm=370, max_pwm=480).update(20.0, 0.0, 0.01) == 480


def test_reset_restores_the_initial_behaviour():
    params = dict(ACTUATOR, **DEFAULTS)
    seq = [(6.0, v, 0.01) for v in (5.0, 5.5, 5.8, 6.1, 6.0, 5.9)]
    fresh = Reference(params)
    first = [fresh.update(*a) for a in seq]
    used = Reference(params)
    for _ in range(500):
        used.update(6.0, 5.8, 0.01)
    used.update(6.0, 7.0, 0.01)
    used.reset()
    assert [used.update(*a) for a in seq] == first


def test_outputs_are_ints_and_finite_over_a_sweep():
    c = Reference(dict(ACTUATOR, **DEFAULTS))
    for target in (0.0, 0.25, 2.0, 6.0, 11.0, 15.0):
        for measured in (0.0, 1.0, 6.0, 12.0):
            for dt in (0.0, 0.01, 0.02, 0.05):
                pwm = c.update(target, measured, dt)
                assert isinstance(pwm, int) and 360 <= pwm <= 470


def test_defaults_converge_on_a_first_order_motor():
    # v' = (g * (pwm - init - d) - v) / tau: the nominal fit with a 0.4 s lag, a 2 count
    # deadband error and a 5 % gain error. The defaults settle within 0.1 m/s at 6 m/s.
    c = Reference(dict(ACTUATOR, **DEFAULTS))
    v, dt = 0.0, 0.01
    g, d, tau = 0.95 * 0.12785, 5.249 + 2.0, 0.4
    for _ in range(1500):
        pwm = c.update(6.0, v, dt)
        drive = max(0.0, g * (pwm - 370 - d))
        v += (drive - v) / tau * dt
    assert v == pytest.approx(6.0, abs=0.1)


def test_stub_has_the_given_parts_the_reference_uses():
    assert Reference.feed_forward is Stub.feed_forward
    assert Reference.filter_derivative is Stub.filter_derivative
    assert Reference.clamp_pwm is Stub.clamp_pwm
    assert Reference.reset is Stub.reset
    assert Reference.__init__ is Stub.__init__


def test_reference_parameters_pass_the_submission_check_and_cover_the_deficit():
    sub = submission_module()
    assert set(DEFAULTS) == set(sub.part1_keys())
    checked, warnings = sub.check_part1_yaml(DEFAULTS_YAML)
    assert warnings == [] and checked == DEFAULTS
    assert DEFAULTS['integral_limit'] >= 15    # room for the feed-forward deficit, and low_batt's


def test_reference_is_tuned_differently_from_the_starting_values():
    start = yaml.safe_load((PKG / 'config' / 'speed_pid_part1.yaml').read_text())
    assert start != DEFAULTS                     # the students' defaults are not the answer
