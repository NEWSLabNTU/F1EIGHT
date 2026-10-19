"""The ESC: the cars' state machine with the sim's thresholds, counts, ramp, brake, drag brake."""
import pytest

from autosdv_vehicle_interface.longitudinal import EscModel, EscState

from autosdv_rc_sim.esc import Esc
from autosdv_rc_sim.params import load_vehicle

DT = 0.001


def run(esc, pwm, seconds, wheel_speed=1.0, t0=0):
    out = None
    for k in range(int(round(seconds / DT))):
        out = esc.step(t0 + k * 1_000_000, pwm, wheel_speed, DT)
    return out


def test_thresholds_and_one_count_resolution():
    p = load_vehicle()
    assert (p.forward_start_pwm, p.forward_hold_pwm, p.reverse_stop_pwm,
            p.reverse_start_pwm, p.reverse_arm_pwm) == (376, 375, 365, 355, 335)
    assert run(Esc(p), 375, 0.5).duty == 0.0                  # still band from rest
    assert run(Esc(p), 376, 0.5).duty == pytest.approx(1.0 / 95.0)
    assert run(Esc(p), 470, 0.5).duty == 1.0


def test_forward_hysteresis():
    """From rest forward needs forward_start_pwm; moving, it keeps driving above forward_hold_pwm."""
    p = load_vehicle(forward_start_pwm=380, forward_hold_pwm=373)
    esc = Esc(p)
    assert run(esc, 378, 0.1).mode == 'neutral'               # below the start: LOCKED
    run(esc, 380, 0.1, t0=100_000_000)
    out = run(esc, 378, 0.1, t0=200_000_000)                  # moving: held
    assert out.mode == 'forward' and out.duty == pytest.approx(5.0 / 97.0)
    assert run(esc, 373, 0.01, t0=300_000_000).mode == 'neutral'
    assert esc.state == EscState.LOCKED


def test_throttle_ramp():
    p = load_vehicle()
    esc = Esc(p)
    assert run(esc, 470, 0.05).duty == pytest.approx(0.5)     # 10 per second
    assert run(esc, 470, 0.06, t0=50_000_000).duty == 1.0
    assert run(esc, 420, 0.001, t0=110_000_000).duty == pytest.approx(45.0 / 95.0)  # falls at once


def test_min_pwm_is_full_brake():
    p = load_vehicle()
    assert run(Esc(p), 365, 0.01).brake == 0.0               # reverse_stop_pwm: still
    assert run(Esc(p), 364, 0.01).brake == pytest.approx(0.2)
    assert run(Esc(p), 360, 0.01).brake == 1.0               # min_pwm: the controller's full brake
    out = run(Esc(p), 340, 0.01)                              # brake_pwm: saturated, above the arm point
    assert (out.brake, out.duty, out.mode) == (1.0, 0.0, 'brake')


def test_the_stack_never_arms_reverse():
    """brake_pwm and min_pwm stay above reverse_arm_pwm: holding either at rest only brakes."""
    p = load_vehicle()
    for pwm in (p.brake_pwm, p.min_pwm):
        esc = Esc(p)
        run(esc, 400, 0.5)
        run(esc, pwm, 1.0, wheel_speed=0.0, t0=500_000_000)
        run(esc, p.init_pwm, 1.0, wheel_speed=0.0, t0=1_500_000_000)
        out = run(esc, p.min_pwm, 1.0, wheel_speed=0.0, t0=2_500_000_000)
        assert out.mode == 'brake' and out.duty == 0.0 and esc.state == EscState.LOCKED


def test_reverse_after_a_pull_and_return():
    p = load_vehicle()
    esc = Esc(p)
    assert run(esc, 330, 0.05, wheel_speed=0.0).mode == 'brake'    # ARMING brakes
    assert esc.state == EscState.ARMING
    run(esc, 370, 0.05, wheel_speed=0.0, t0=50_000_000)
    assert esc.state == EscState.ARMED
    assert run(esc, 360, 0.1, wheel_speed=0.0, t0=100_000_000).mode == 'brake'  # above reverse_start
    out = run(esc, 355, 0.5, wheel_speed=0.0, t0=200_000_000)
    assert out.mode == 'reverse' and out.duty == pytest.approx(-0.5)
    out = run(esc, 362, 0.1, wheel_speed=-0.5, t0=700_000_000)     # reversing continues below the stop
    assert out.mode == 'reverse' and out.duty == pytest.approx(-0.3)
    assert run(esc, 365, 0.01, wheel_speed=-0.5, t0=800_000_000).mode == 'neutral'
    assert esc.state == EscState.ARMED                              # still armed


def test_armed_but_rolling_forward_brakes():
    p = load_vehicle()
    esc = Esc(p)
    run(esc, 330, 0.05, wheel_speed=2.0)
    run(esc, 370, 0.05, wheel_speed=2.0, t0=50_000_000)
    assert run(esc, 350, 0.1, wheel_speed=2.0, t0=100_000_000).mode == 'brake'


def test_forward_disarms_reverse():
    p = load_vehicle()
    esc = Esc(p)
    run(esc, 330, 0.05, wheel_speed=0.0)
    run(esc, 370, 0.05, wheel_speed=0.0, t0=50_000_000)
    run(esc, p.forward_start_pwm, 0.01, wheel_speed=0.0, t0=100_000_000)
    run(esc, 370, 0.01, wheel_speed=0.5, t0=110_000_000)
    assert esc.state == EscState.LOCKED
    assert run(esc, 355, 0.1, wheel_speed=0.0, t0=120_000_000).mode == 'brake'


def test_state_is_the_actuators_tracker():
    """Once a forward command has settled the actuator's tracker, the two agree on every command."""
    p = load_vehicle()
    esc, tracker = Esc(p), EscModel(p)
    seq = [400, 380, 376, 375, 370, 360, 340, 335, 370, 355, 362, 365, 372, 376, 450, 374, 330, 336]
    for k, pwm in enumerate(seq):
        esc.step(k * 1_000_000, pwm, 0.0, DT)
        tracker.update(pwm)
        assert esc.state == tracker.state, (k, pwm)


def test_drag_brake_only_at_neutral():
    p = load_vehicle(drag_brake=0.05)
    assert run(Esc(p), 370, 0.01).short == 0.05
    assert run(Esc(p), 400, 0.01).short == 0.0
