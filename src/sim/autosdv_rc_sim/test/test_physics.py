"""Physical validation of the plant (docs/design/rc-sim-design.md, Validation)."""
from dataclasses import replace
import math

import pytest

from autosdv_rc_sim.drivetrain import road_load, top_speed
from autosdv_rc_sim.params import GRAVITY, load_vehicle, variant_names
from autosdv_rc_sim.plant import Plant

STRAIGHT = 489


def rolling(p, v):
    plant = Plant(p)
    w = v / p.wheel_radius
    plant.state = replace(plant.state, vx=v, omega_f=w, omega_r=w)
    return plant


def drive(plant, pwm, seconds, steer=STRAIGHT):
    dt = plant.p.physics_dt
    return [plant.step(dt, pwm, steer) for _ in range(int(round(seconds / dt)))]


def test_base_pose_round_trip():
    p = load_vehicle()
    assert Plant(p, 1.0, 2.0, 0.5).base_pose == pytest.approx((1.0, 2.0, 0.5))


def test_top_speed_matches_the_motor():
    p = load_vehicle()
    plant = rolling(p, 11.0)
    out = drive(plant, 470, 6.0)
    v_top = top_speed(p)
    assert out[-1].state.vx == pytest.approx(out[-1001].state.vx, abs=0.01)         # steady
    assert out[-1].state.omega_r * p.wheel_radius == pytest.approx(v_top, rel=0.01)  # motor law
    assert 0.97 * v_top <= out[-1].state.vx <= v_top                                 # tyre slip
    assert 11.5 <= out[-1].state.vx <= 12.6


def test_coast_down_matches_drag_and_rolling():
    p = load_vehicle()
    out = drive(rolling(p, 6.0), 370, 0.3)
    decel = (out[50].state.vx - out[250].state.vx) / 0.2
    v_mid = 0.5 * (out[50].state.vx + out[250].state.vx)
    assert decel == pytest.approx(road_load(p, v_mid) / p.effective_mass, rel=0.03)


def test_steady_cornering_never_exceeds_mu_g():
    p = load_vehicle()
    plant = rolling(p, 8.0)
    out = drive(plant, 450, 3.0, steer=482)          # 0.049 rad: kinematic demand 9.8 m/s^2
    ay = [abs(r.state.ay) for r in out]
    assert max(ay) <= p.mu * GRAVITY * (1.0 + 1e-9)
    assert max(ay) >= 0.8 * p.mu * GRAVITY           # it did reach the limit


def test_full_throttle_from_rest_spins_the_rear_wheels():
    p = load_vehicle(throttle_ramp=1e6)
    plant = Plant(p)
    out = drive(plant, 470, 0.3)
    assert max(r.state.omega_r * p.wheel_radius - r.state.vx for r in out) > 1.0
    s = out[-1].state
    assert abs(s.omega_f * p.wheel_radius - s.vx) < 0.1      # the front rolls freely


def test_neutral_does_not_creep():
    p = load_vehicle()
    plant = Plant(p)
    start = plant.state
    out = drive(plant, 370, 10.0)
    assert out[-1].state.x == start.x and out[-1].state.y == start.y
    assert out[-1].state.vx == 0.0 and out[-1].state.omega_r == 0.0


def test_rolling_at_neutral_stops_and_stays():
    p = load_vehicle()
    out = drive(rolling(p, 1.0), 370, 5.0)
    assert abs(out[-1].state.vx) < 1e-6
    assert out[-1].state.x == pytest.approx(out[-1001].state.x, abs=1e-6)


def brake_run(p, v0=6.0, seconds=2.0):
    """(speed, rear kappa) per step while holding brake_pwm from v0."""
    plant = rolling(p, v0)
    out = []
    for _ in range(int(round(seconds / p.physics_dt))):
        plant.step(p.physics_dt, 340, STRAIGHT)
        out.append((plant.state.vx, plant.kappa_rear))
    return out


def test_brake_locks_only_beyond_friction():
    nominal = brake_run(load_vehicle())                       # ~9 N: under the rear grip
    assert min(k for v, k in nominal if v > 1.0) > -0.2
    strong = brake_run(load_vehicle(brake_torque_max=1.5))   # ~30 N: far beyond it
    assert min(k for v, k in strong if v > 1.0) < -0.9


def test_standstill_is_held_below_the_breakaway_torque():
    """PWM 376 gives 0.18 N m at standstill, under the 0.25 N m the rear axle holds."""
    light_rolling = dict(rolling_resistance=0.02, front_bearing_coulomb=0.002)   # drive torque clearly above it

    def creep(**overrides):
        plant = Plant(load_vehicle(**overrides))
        x0 = plant.state.x
        out = drive(plant, 376, 3.0)
        return out[-1].state.x - x0, out[-1].state.omega_r

    assert creep() == (0.0, 0.0)
    assert creep(**light_rolling) == (0.0, 0.0)                                  # held by stiction alone
    free = creep(drivetrain_breakaway=load_vehicle().drivetrain_coulomb, **light_rolling)
    assert free[0] > 0.05 and free[1] > 0.0                                      # without it, the same command creeps off


@pytest.mark.parametrize('variant', variant_names())
def test_motor_braking_under_a_forward_command_does_not_lock_the_rear_wheels(variant):
    """PWM just above the deadband at speed is a duty below the back-EMF: the ESC regenerates.

    The regeneration cap keeps that under the rear grip on every variant, and it is a light
    drag brake: regeneration plus road load slow the car at 1.5-2.0 m/s^2, clearly less than
    the ESC brake (3.0-3.5), so braking harder needs the brake band below neutral."""
    p = load_vehicle(variant=variant)
    plant = rolling(p, 11.0)
    kappas = []
    for _ in range(int(round(2.5 / p.physics_dt))):
        plant.step(p.physics_dt, p.forward_start_pwm, STRAIGHT)
        if plant.state.vx > 1.0:
            kappas.append(plant.kappa_rear)
    assert plant.esc_out.duty > 0.0 and plant.drivetrain.current < 0.0     # it is regenerating
    decel = (11.0 - plant.state.vx) / 2.5
    assert 1.5 < decel < 2.1                                               # lightly: a drag brake
    assert min(kappas) > -0.2


@pytest.mark.parametrize('variant', variant_names())
def test_the_esc_brake_is_clearly_stronger_than_regeneration(variant):
    """Full brake (min_pwm) from 11 m/s: at least 2.9 m/s^2, against regeneration's 2.1 at most."""
    p = load_vehicle(variant=variant)
    plant = rolling(p, 11.0)
    for _ in range(int(round(1.0 / p.physics_dt))):
        plant.step(p.physics_dt, p.min_pwm, STRAIGHT)
    assert 11.0 - plant.state.vx >= 2.9


def test_uncapped_motor_braking_would_lock_the_rear_wheels():
    """The contrast that makes the test above mean something."""
    p = load_vehicle(esc_regen_current_limit=185.0)
    plant = rolling(p, 11.0)
    kappas = []
    for _ in range(1000):
        plant.step(p.physics_dt, p.forward_start_pwm, STRAIGHT)
        kappas.append(plant.kappa_rear)
    assert min(kappas) < -0.9


def test_low_speed_blend_is_stable_at_full_lock():
    p = load_vehicle()
    plant = Plant(p)
    out = drive(plant, 378, 3.0, steer=439) + drive(plant, 370, 2.0, steer=439)
    tan_lock = math.tan(0.349)
    for r in out:
        s = r.state
        assert all(math.isfinite(x) for x in (s.vx, s.vy, s.yaw_rate, s.x, s.y))
        assert abs(s.yaw_rate) <= 1.2 * abs(s.vx) * tan_lock / p.wheel_base + 1e-9
        assert abs(s.vy) <= 0.5 * abs(s.vx) + 1e-9
    assert abs(out[-1].state.vx) < 1e-6


def test_hall_follows_rear_wheel_during_spin():
    p = load_vehicle(throttle_ramp=1e6)
    plant = Plant(p)
    x0 = plant.state.x
    edges = sum(len(r.edges) for r in drive(plant, 470, 0.25))
    marker = 2.0 * math.pi * p.wheel_radius / p.markers_per_rotation
    assert edges * marker >= 1.5 * (plant.state.x - x0)      # the sensor sees the spinning wheel


def test_twelve_edges_per_wheel_revolution_in_the_plant():
    p = load_vehicle()
    plant = rolling(p, 2.0)
    theta0 = plant.state.theta_r
    edges = sum(len(r.edges) for r in drive(plant, 400, 1.0))
    revs = (plant.state.theta_r - theta0) / (2.0 * math.pi)
    assert abs(edges - 12.0 * revs) <= 1.0


def _momentum(p, s):
    return p.mass * s.vx + (p.front_axle_inertia * s.omega_f + p.rear_axle_inertia * s.omega_r) / p.wheel_radius


def _frictionless_axles(**extra):
    return load_vehicle(drivetrain_coulomb=0.0, drivetrain_breakaway=0.0, drivetrain_viscous=0.0,
                        front_bearing_coulomb=0.0, rolling_resistance=0.0, aero_drag=0.0, **extra)


def test_slip_sign_change_ends_the_step_rolling_and_conserves_momentum():
    from autosdv_rc_sim.chassis import Chassis, ChassisState
    p = _frictionless_axles()
    chassis = Chassis(p)
    # The front axle spins slightly faster than the ground: sliding, and about to cross zero slip.
    s = ChassisState(vx=0.2, omega_f=0.2 / p.wheel_radius + 3.0, omega_r=0.2 / p.wheel_radius)
    q0 = _momentum(p, s)
    n = chassis.step(s, 0.0, 0.0, 0.0, 0.0, p.physics_dt)
    assert n.omega_f * p.wheel_radius - n.vx == pytest.approx(0.0, abs=1e-9)    # rolling, not overshot
    assert _momentum(p, n) == pytest.approx(q0, rel=1e-12)


def test_no_slip_chatter_when_a_sliding_front_wheel_catches_up():
    from autosdv_rc_sim.chassis import Chassis, ChassisState
    p = _frictionless_axles()
    chassis = Chassis(p)
    s = ChassisState(vx=0.3, omega_f=0.3 / p.wheel_radius + 20.0, omega_r=0.3 / p.wheel_radius)
    signs = []
    for _ in range(300):
        s = chassis.step(s, 0.0, 0.0, 0.0, 0.0, p.physics_dt)
        signs.append(s.omega_f * p.wheel_radius - s.vx)
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a * b < 0.0)
    assert flips <= 1
    assert abs(signs[-1]) < 1e-6
