"""Cross-check against the linear single-track (F1TENTH ST) model; numerics of the step."""
from dataclasses import replace

import pytest

from autosdv_rc_sim.drivetrain import top_speed
from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.plant import Plant


def hold_pwm(p, v):
    """PWM whose steady speed is closest to v."""
    lo = p.init_pwm + p.esc_deadband
    return min(range(lo + 1, p.max_pwm + 1),
               key=lambda pwm: abs(top_speed(p, (pwm - lo) / (p.max_pwm - lo)) - v))


def _cruise(p, v, steer_pwm, steps):
    plant = Plant(p)
    w = v / p.wheel_radius
    plant.state = replace(plant.state, vx=v, omega_f=w, omega_r=w)
    pwm = hold_pwm(p, v)
    for _ in range(steps):
        plant.step(p.physics_dt, pwm, steer_pwm)
    return plant


def test_steady_yaw_rate_matches_the_linear_single_track_model():
    """F1TENTH's ST model in its linear range: yaw rate = v delta / (L + K_us v^2)."""
    p = load_vehicle()
    plant = _cruise(p, 3.0, 486, 5000)      # 3 counts: 0.021 rad, 0.6 m/s^2 lateral
    s = plant.state
    k_us = p.mass * (p.lr / p.cornering_stiffness_front - p.lf / p.cornering_stiffness_rear) / p.wheel_base
    expected = s.vx * plant.delta / (p.wheel_base + k_us * s.vx * s.vx)
    assert s.yaw_rate == pytest.approx(expected, rel=0.05)


def test_steering_sign_is_mirror_symmetric_and_understeers():
    """A slip-angle sign error would still pass a mu*g cornering test; this one cannot."""
    p = load_vehicle()
    results = []
    for pwm in (p.init_steer + 8, p.init_steer - 8):
        plant = _cruise(p, 2.0, pwm, 4000)
        results.append((plant.delta, plant.state))
    for delta, s in results:
        assert delta != 0.0
        assert s.yaw_rate * delta > 0.0
        assert s.y * delta > 0.0
        assert abs(s.yaw_rate) <= abs(s.vx * delta / p.wheel_base) * (1 + 1e-6)   # understeer
    (d1, s1), (d2, s2) = results
    assert d1 == pytest.approx(-d2, rel=1e-9)
    assert s1.yaw_rate == pytest.approx(-s2.yaw_rate, rel=1e-6)
    assert s1.y == pytest.approx(-s2.y, rel=1e-6)


def test_stiff_wheel_mode_is_damped():
    """Explicit Euler would diverge here (dt * rate >> 2); the implicit step must not even oscillate."""
    p = load_vehicle()
    rate = p.longitudinal_stiffness_per_load * p.fz_front_static * p.wheel_radius ** 2 / (
        p.front_axle_inertia * p.v_min)
    assert p.physics_dt * rate > 2.0          # why the step is implicit
    plant = Plant(p)
    plant.state = replace(plant.state, omega_f=0.5 / p.wheel_radius)   # front spinning, car at rest
    slips = []
    for _ in range(50):
        plant.step(p.physics_dt, 370, 489)
        s = plant.state
        slips.append(s.omega_f * p.wheel_radius - s.vx)
    assert all(abs(b) <= abs(a) + 1e-12 for a, b in zip(slips, slips[1:]) if abs(a) > 0.01)
    assert all(x > 0.0 for x in slips if abs(x) > 0.01)              # never overshoots through zero
    assert abs(slips[-1]) < 0.01


def test_dt_convergence():
    coarse = load_vehicle()
    fine = load_vehicle(physics_dt=0.00025)
    results = []
    for p in (coarse, fine):
        plant = Plant(p)
        for _ in range(int(round(4.0 / p.physics_dt))):
            plant.step(p.physics_dt, 450, 489)
        results.append(plant.state)
    # Measured at the default dt: vx 5e-6, x 5e-4 relative; a 4 ms step gives x 4e-3.
    assert results[0].vx == pytest.approx(results[1].vx, rel=1e-4)
    assert results[0].x == pytest.approx(results[1].x, rel=2e-3)


def test_dt_convergence_steered():
    """Lateral dynamics and the stiff front path: 1 ms against 0.25 ms, +8-count cruise."""
    results = []
    for dt in (0.001, 0.00025):
        p = load_vehicle(physics_dt=dt)
        results.append(_cruise(p, 2.0, p.init_steer + 8, int(round(4.0 / dt))).state)
    # Measured: yaw rate agrees to 1e-12, y to 7e-4 relative (a 4 ms step gives 3e-3).
    assert results[0].yaw_rate == pytest.approx(results[1].yaw_rate, rel=1e-4)
    assert results[0].y == pytest.approx(results[1].y, rel=2e-3)
    assert results[0].vx == pytest.approx(results[1].vx, rel=1e-4)


def test_standstill_at_full_lock_does_not_drift():
    p = load_vehicle()
    plant = Plant(p)
    start = plant.state
    for _ in range(10000):
        plant.step(p.physics_dt, 370, 439)
    assert (plant.state.x, plant.state.y, plant.state.yaw) == (start.x, start.y, start.yaw)
