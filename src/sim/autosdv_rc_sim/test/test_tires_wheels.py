"""Tyre forces and wheel friction."""
import pytest

from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.tires import combined
from autosdv_rc_sim.wheels import front_friction, friction_sign, rear_friction, sticks

P = load_vehicle()
ARGS = dict(mu=P.mu, bx=P.b_long, by=P.b_lat_rear, c=P.pacejka_c, e=P.pacejka_e)


def test_zero_slip_zero_force():
    assert combined(0.0, 0.0, 16.0, **ARGS) == (0.0, 0.0)


def test_small_slip_stiffness():
    fz = P.fz_rear_static
    fx, _ = combined(1e-5, 0.0, fz, **ARGS)
    _, fy = combined(0.0, 1e-5, fz, **ARGS)
    assert fx / 1e-5 == pytest.approx(P.longitudinal_stiffness_per_load * fz, rel=1e-3)
    assert fy / 1e-5 == pytest.approx(P.cornering_stiffness_rear, rel=1e-3)


def test_peak_is_mu_fz_and_sliding_is_less():
    fz = 16.0
    peak = max(combined(k / 2000.0, 0.0, fz, **ARGS)[0] for k in range(2001))
    assert peak == pytest.approx(P.mu * fz, rel=1e-3)
    locked = combined(-1.0, 0.0, fz, **ARGS)[0]
    assert -P.mu * fz < locked < -0.6 * P.mu * fz


def test_friction_circle_holds_for_any_combination():
    fz = 16.0
    for i in range(-20, 21):
        for j in range(-20, 21):
            fx, fy = combined(i / 10.0, j / 20.0, fz, **ARGS)
            assert fx * fx + fy * fy <= (P.mu * fz) ** 2 * (1.0 + 1e-12)


def test_antisymmetry():
    assert combined(-0.05, -0.02, 16.0, **ARGS) == pytest.approx(
        tuple(-f for f in combined(0.05, 0.02, 16.0, **ARGS)))


def test_rear_friction_includes_rolling_and_brake():
    fr = rear_friction(P, 16.0, 0.5)
    rr = P.rolling_resistance * 16.0 * P.wheel_radius
    assert fr.coulomb == pytest.approx(P.drivetrain_coulomb + rr + 0.5 * P.brake_torque_max)
    assert fr.hold == pytest.approx(P.drivetrain_breakaway + rr + 0.5 * P.brake_torque_max)
    assert front_friction(P, 18.0).hold == front_friction(P, 18.0).coulomb


def test_stick_and_sign():
    fr = rear_friction(P, 16.0, 0.0)
    assert sticks(0.5, 0.1, fr, 2.0)               # slow and under the breakaway torque
    assert not sticks(0.5, 0.5, fr, 2.0)           # torque beats the breakaway
    assert not sticks(5.0, 0.0, fr, 2.0)           # turning
    assert friction_sign(5.0, -1.0, 2.0) == 1.0    # rotation decides when turning
    assert friction_sign(0.5, -1.0, 2.0) == -1.0   # impending motion decides inside the band
