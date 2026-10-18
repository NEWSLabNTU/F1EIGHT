"""Tyre forces and wheel friction."""
import pytest

from autosdv_rc_sim.chassis import ChassisState
from autosdv_rc_sim.params import load_vehicle, variant_names
from autosdv_rc_sim.plant import Plant
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
    assert friction_sign(0.0, -1.0, 2.0) == -1.0   # impending motion decides from rest
    assert friction_sign(0.5, 1.0, 2.0) == 1.0     # ... and along the rotation inside the band
    # inside the band but turning against the external torque: still opposes the rotation
    # (the old rule pushed the wheel along it, and a coasting car crept at the band's edge)
    assert friction_sign(0.5, -1.0, 2.0) == 1.0
    assert friction_sign(-0.5, 1.0, 2.0) == -1.0


# State a Part 1 run reached after its stop (t = 40 s): with the old friction_sign the car
# crept on at 0.103 m/s, |omega_r| just under stick_band, the ESC at neutral and no current.
CREEP_STATE = dict(vx=0.10324279522588543, omega_f=1.9522387551987879,
                   omega_r=1.9665294328740084, theta_r=3782.1345707195433,
                   ax=0.21287264751740365)


def _coast(plant, seconds):
    for _ in range(int(round(seconds / 0.001))):
        plant.step(0.001, plant.p.init_pwm, plant.p.init_steer)
    return plant.state


@pytest.mark.parametrize('variant', variant_names())
def test_neutral_after_a_crawl_comes_to_rest_and_stays(variant):
    p = load_vehicle(variant=variant)
    plant = Plant(p)
    plant.state = ChassisState(**CREEP_STATE)
    s = _coast(plant, 1.0)
    # both axles held; vx decays geometrically through the held tyres (1e-164 after 1 s)
    assert abs(s.vx) < 1e-9 and s.omega_r == 0.0 and s.omega_f == 0.0
    x = s.x
    s = _coast(plant, 2.0)
    assert abs(s.vx) < 1e-9 and abs(s.x - x) < 1e-9


@pytest.mark.parametrize('variant', variant_names())
@pytest.mark.parametrize('pwm', [377, 380, 390])
def test_neutral_after_driving_comes_to_rest_within_two_seconds(variant, pwm):
    # crawl speeds 0.13 .. 1.6 m/s on the nominal car: every one ends held at rest
    p = load_vehicle(variant=variant)
    plant = Plant(p)
    for _ in range(4000):
        plant.step(0.001, pwm, p.init_steer)
    s = _coast(plant, 2.0)
    assert abs(s.vx) < 1e-9 and s.omega_r == 0.0
