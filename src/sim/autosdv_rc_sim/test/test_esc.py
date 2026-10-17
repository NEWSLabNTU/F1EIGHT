"""The ESC: deadband, counts, ramp, brake endpoint, drag brake, reverse re-arm."""
import pytest

from autosdv_rc_sim.esc import Esc
from autosdv_rc_sim.params import load_vehicle

DT = 0.001


def run(esc, pwm, seconds, wheel_speed=1.0, t0=0):
    out = None
    for k in range(int(round(seconds / DT))):
        out = esc.step(t0 + k * 1_000_000, pwm, wheel_speed, DT)
    return out


def test_deadband_and_one_count_resolution():
    p = load_vehicle()
    assert run(Esc(p), 375, 0.5).duty == 0.0
    assert run(Esc(p), 376, 0.5).duty == pytest.approx(1.0 / 95.0)
    assert run(Esc(p), 470, 0.5).duty == 1.0


def test_throttle_ramp():
    p = load_vehicle()
    esc = Esc(p)
    assert run(esc, 470, 0.05).duty == pytest.approx(0.5)     # 10 per second
    assert run(esc, 470, 0.06, t0=50_000_000).duty == 1.0
    assert run(esc, 420, 0.001, t0=110_000_000).duty == pytest.approx(45.0 / 95.0)  # falls at once


def test_min_pwm_is_full_brake():
    p = load_vehicle()
    assert run(Esc(p), 365, 0.01).brake == 0.0               # deadband edge
    assert run(Esc(p), 364, 0.01).brake == pytest.approx(0.2)
    assert run(Esc(p), 360, 0.01).brake == 1.0               # min_pwm: the controller's full brake
    out = run(Esc(p), 340, 0.01)                              # brake_pwm: saturated
    assert (out.brake, out.duty, out.mode) == (1.0, 0.0, 'brake')


def test_holding_brake_at_standstill_never_reverses():
    p = load_vehicle()
    esc = Esc(p)
    out = run(esc, 340, 2.0, wheel_speed=0.0)
    assert out.mode == 'brake' and out.duty == 0.0 and not esc.reverse_armed


def test_reverse_after_stop_and_neutral():
    p = load_vehicle()
    esc = Esc(p)
    run(esc, 370, 0.05, wheel_speed=0.0)
    assert not esc.reverse_armed                              # 0.05 s < rearm_time
    run(esc, 370, 0.06, wheel_speed=0.0, t0=50_000_000)
    assert esc.reverse_armed
    out = run(esc, 360, 0.5, wheel_speed=0.0, t0=110_000_000)
    assert out.mode == 'reverse' and out.duty == pytest.approx(-0.5)


def test_neutral_while_rolling_does_not_arm_reverse():
    p = load_vehicle()
    esc = Esc(p)
    run(esc, 370, 1.0, wheel_speed=2.0)
    assert run(esc, 360, 0.01, wheel_speed=2.0, t0=10**9).mode == 'brake'


def test_forward_disarms_reverse():
    p = load_vehicle()
    esc = Esc(p)
    run(esc, 370, 0.2, wheel_speed=0.0)
    run(esc, 400, 0.01, wheel_speed=0.0, t0=200_000_000)
    assert not esc.reverse_armed


def test_drag_brake_only_at_neutral():
    p = load_vehicle(drag_brake=0.05)
    assert run(Esc(p), 370, 0.01).short == 0.05
    assert run(Esc(p), 400, 0.01).short == 0.0
