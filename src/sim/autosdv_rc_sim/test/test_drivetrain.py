"""Motor law, battery sag, and the analytic figures the track generator and tests rely on."""
import pytest

from autosdv_rc_sim.drivetrain import (
    brake_decel, drive_force, Drivetrain, grip_accel, rear_brake_limit, road_load, top_speed)
from autosdv_rc_sim.params import GRAVITY, load_vehicle, variant_names


def test_stall_force_and_sag():
    p = load_vehicle()
    dt = Drivetrain(p)
    tau, _ = dt.torque(1.0, 0.0, 0.0)
    assert 11.1 / 0.06 > p.esc_current_limit                         # the motor alone would draw 185 A
    assert dt.current == p.esc_current_limit == 120.0                # the ESC limits it
    assert tau / p.wheel_radius == pytest.approx(105.5, rel=1e-3)    # still far above the rear grip
    assert dt.terminal_voltage == pytest.approx(11.1 - 120.0 * 0.03)
    assert Drivetrain(p).torque(0.1, 0.0, 0.0)[0] == pytest.approx(   # below the limit: the plain motor law
        14.5 * p.kt * 0.1 * 11.1 / (0.03 + 0.01 * 0.03))


def test_no_load_speed():
    p = load_vehicle()
    omega0 = p.battery_voltage / (p.ke * p.gear_ratio)
    assert Drivetrain(p).torque(1.0, 0.0, omega0)[0] == pytest.approx(0.0, abs=1e-12)
    assert omega0 * p.wheel_radius == pytest.approx(12.63, abs=0.01)


def test_slope_is_the_derivative():
    p = load_vehicle()
    dt = Drivetrain(p)
    for duty, short, omega in ((0.4, 0.0, 100.0), (0.0, 0.05, 100.0),   # linear
                               (0.4, 0.0, 400.0), (0.0, 1.0, 300.0)):    # at the regeneration limit: flat
        tau, slope = dt.torque(duty, short, omega)
        tau2, _ = dt.torque(duty, short, omega + 1e-4)
        assert slope == pytest.approx((tau2 - tau) / 1e-4, rel=1e-6, abs=1e-9)


def test_motoring_and_regeneration_stay_within_the_esc_limits():
    """Regeneration (the current against the duty, or the drag brake) never exceeds its cap."""
    p = load_vehicle()
    k = p.gear_ratio * p.kt
    worst_regen = worst_motor = 0.0
    for duty in (0.01, 0.05, 0.2, 0.5, 1.0, -0.3):
        for omega in range(0, 700, 7):
            dt = Drivetrain(p)
            tau, _ = dt.torque(duty, 0.0, float(omega))
            if duty * tau < 0.0 or (duty > 0.0 and dt.current < 0.0):
                worst_regen = max(worst_regen, abs(tau))
            else:
                worst_motor = max(worst_motor, abs(tau))
            assert abs(dt.current) <= p.esc_current_limit
        for short in (0.1, 1.0):
            dt = Drivetrain(p)
            tau, _ = dt.torque(0.0, short, 600.0)
            worst_regen = max(worst_regen, abs(tau))
    assert worst_regen > 0.0                                          # the sweep does reach the cap
    assert worst_regen <= k * p.esc_regen_current_limit * (1.0 + 1e-12)
    assert worst_motor <= k * p.esc_current_limit * (1.0 + 1e-12)
    assert k * p.esc_regen_current_limit <= p.brake_torque_max        # motor braking is no stronger than the ESC brake


def test_drag_brake_opposes_rotation_and_neutral_is_free():
    p = load_vehicle()
    assert Drivetrain(p).torque(0.0, 0.05, 100.0)[0] < 0.0
    assert Drivetrain(p).torque(0.0, 0.0, 100.0) == (0.0, 0.0)


def test_loaded_top_speed():
    p = load_vehicle()
    v = top_speed(p)
    assert 11.5 <= v <= 12.6
    assert drive_force(p, 1.0, v) == pytest.approx(road_load(p, v), abs=1e-6)


@pytest.mark.parametrize('variant', variant_names())
def test_every_variant_can_hold_11_and_launch_at_2_5(variant):
    p = load_vehicle(variant=variant)
    assert top_speed(p) >= 11.4
    assert grip_accel(p) >= 2.5 * 1.1


def rear_tyre_load(p, v, torque):
    """(tyre force, tyre grip) at the rear axle for a braking torque on it at speed v, no wheel lock.

    The torque and the road load decelerate the car; the load transfer to the front unloads the rear."""
    r = p.wheel_radius
    a = (torque / r + road_load(p, v)) / p.effective_mass
    fz_r = p.mass * (GRAVITY * p.lf - a * p.cg_height) / p.wheel_base
    axle = (torque + p.drivetrain_coulomb + p.rolling_resistance * fz_r * r + p.drivetrain_viscous * v / r)
    return axle / r - p.rear_axle_inertia * a / (r * r), p.mu * fz_r


@pytest.mark.parametrize('v', [1.0, 6.0, 11.0])
def test_full_brake_stays_under_the_rear_grip(v):
    p = load_vehicle()
    tyre, grip = rear_tyre_load(p, v, p.brake_torque_max)
    assert tyre <= 0.9 * grip


@pytest.mark.parametrize('v', [1.0, 6.0, 11.0])
def test_motor_braking_stays_under_the_rear_grip(v):
    """The regeneration cap, at its largest, loads the rear tyres no more than the 90 % the ESC brake does."""
    p = load_vehicle()
    tyre, grip = rear_tyre_load(p, v, p.gear_ratio * p.kt * p.esc_regen_current_limit)
    assert tyre <= 0.9 * grip


def test_the_spec_decel_is_beyond_rear_only_braking():
    """Spec finding: d = 3.5 m/s^2 exceeds what the rear tyres can give (about 3.4 on the nominal car)."""
    assert 3.0 < rear_brake_limit(load_vehicle()) < 3.5
    assert rear_brake_limit(load_vehicle(variant='low_mu')) < rear_brake_limit(load_vehicle())


@pytest.mark.parametrize('variant', variant_names())
def test_every_variant_follows_a_2_5_decel(variant):
    """d = 2.5 m/s^2 is within rear-only braking, and within the ESC brake, on every variant."""
    p = load_vehicle(variant=variant)
    assert rear_brake_limit(p) >= 2.5 * 1.1
    assert all(brake_decel(p, v) >= 2.5 for v in (1.0, 6.0, 11.0))
