"""Servo mapping and lag; hall edges; IMU sample."""
import math

import pytest

from autosdv_rc_sim.chassis import ChassisState
from autosdv_rc_sim.hall import HallSensor
from autosdv_rc_sim.imu import ImuModel
from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.servo import Servo

P = load_vehicle()


def test_full_steer_pwm_reaches_lock():
    s = Servo(P)
    assert s.target_from_pwm(439) == 0.349      # RIGHT on the hardware, positive angle (ratio < 0)
    assert s.target_from_pwm(539) == -0.349
    assert s.target_from_pwm(489) == 0.0
    assert s.target_from_pwm(300) == 0.349      # clamped to min_steer first


def test_dead_time_then_rate_limit():
    s = Servo(P)
    out = [s.step(439, 0.001) for _ in range(60)]
    assert out[18] == 0.0                         # inside the 20 ms dead time
    assert out[59] == pytest.approx(5.0 * 0.001 * (60 - 20), abs=0.006)


def test_twelve_edges_per_revolution_both_directions():
    h = HallSensor(12)
    theta, edges, t = h.start_angle, [], 0
    for _ in range(3000):                         # 3 revolutions forward in 3 s
        nxt = theta + 2.0 * math.pi * 3 / 3000
        edges += h.step(t, 1_000_000, theta, nxt)
        theta, t = nxt, t + 1_000_000
    assert len(edges) == 36
    assert edges == sorted(edges) and all(0 < e <= t for e in edges)
    back = 0
    for _ in range(1000):                         # one revolution backward
        nxt = theta - 2.0 * math.pi / 1000
        back += len(h.step(t, 1_000_000, theta, nxt))
        theta, t = nxt, t + 1_000_000
    assert back == 12


def test_edges_stay_correct_after_a_very_long_run():
    """Wheel angle ~1e7 rad (beyond detmath's range): edges match the exact schedule."""
    h = HallSensor(12)
    spacing = h.spacing
    omega = 2.0 * math.pi * 10.0                  # 10 rev/s, 120 edges/s
    big = 2.0 * math.pi * 1_600_000               # ~1.0e7 rad, a whole number of turns
    h.reset(big + h.start_angle)
    assert 0.0 <= h.phase < spacing
    assert h.phase == pytest.approx(h.start_angle, abs=1e-6)
    theta, t, edges = big + h.start_angle, 0, []
    for _ in range(2000):                         # 2 s, 1 ms steps
        nxt = theta + omega * 0.001
        edges += h.step(t, 1_000_000, theta, nxt)
        theta, t = nxt, t + 1_000_000
    assert len(edges) == 240
    # exact schedule from the start: marker m after (m - 0.5) * spacing of rotation
    for m, e in enumerate(edges, start=1):
        expect = (m * spacing - h.start_angle) / omega * 1e9
        assert abs(e - expect) < 100.0            # ns; wrap keeps error at rounding level
    assert 0.0 <= h.phase < spacing


def test_hall_phase_stays_bounded_over_synthetic_hours():
    h = HallSensor(12)
    for i in range(200_000):
        h.step(i * 1_000_000, 1_000_000, 0.0, 0.2)   # angle increments only
        assert 0.0 <= h.phase < h.spacing


def test_imu_sample():
    state = ChassisState(yaw_rate=0.3, ax=1.0, ay=-0.5)
    quiet = load_vehicle(gyro_noise_std=0.0, accel_noise_std=0.0, gyro_bias_z=0.01)
    assert ImuModel(quiet).sample(state) == pytest.approx((0.31, 1.0, -0.5))
    assert ImuModel(P, seed=4).sample(state) == ImuModel(P, seed=4).sample(state)
