"""The vehicle bench's plant model (bench_plant.py): ROS-free checks."""

import math

import pytest

from autosdv_coach_lab.bench_plant import (
    SteeringParams, VehiclePlant, quat_conjugate, quat_from_yaw, quat_multiply, quat_rotate,
    tire_angle_from_pwm)
from autosdv_vehicle_interface.motor_plant import MotorPlant, MotorPlantParams


def run(plant, seconds, dt=0.005):
    for _ in range(int(round(seconds / dt))):
        plant.step(dt)


def test_neutral_does_not_move():
    p = VehiclePlant()
    run(p, 3.0)
    assert p.speed == 0.0 and p.x == 0.0 and p.wheel_speed_report() == 0.0


def test_speed_matches_motor_plant_and_drives_straight():
    params = MotorPlantParams()
    p = VehiclePlant(params)
    p.command(400, 489)
    run(p, 5.0)
    expected = MotorPlant(params).steady_state_speed(400)
    assert p.speed == pytest.approx(expected, rel=1e-3)
    assert abs(p.y) < 1e-9 and p.yaw == 0.0 and p.yaw_rate == 0.0
    # hall-sensor model, at steady state, reads the true speed
    assert p.wheel_speed_report() == pytest.approx(expected, rel=0.02)


def test_brake_stops_without_reversing():
    p = VehiclePlant()
    p.command(420, 489)
    run(p, 4.0)
    p.command(340, 489)
    run(p, 2.0)
    assert p.speed == 0.0


def test_steering_is_the_actuator_mapping_inverted():
    s = SteeringParams()
    # actuator: steer = init_steer + int(angle * ratio), ratio -1000
    assert tire_angle_from_pwm(489, s) == 0.0
    assert tire_angle_from_pwm(479, s) == pytest.approx(0.01)
    assert tire_angle_from_pwm(499, s) == pytest.approx(-0.01)
    # PWM limits saturate the angle (0.05 rad with ratio -1000)
    assert tire_angle_from_pwm(300, s) == pytest.approx(0.05)


def test_left_turn_is_positive_yaw_with_bicycle_radius():
    s = SteeringParams(time_constant=0.0)
    p = VehiclePlant(steering=s)
    p.command(400, 469)       # 0.02 rad left
    run(p, 6.0)
    assert p.yaw_rate > 0.0
    assert p.yaw_rate == pytest.approx(p.speed * math.tan(0.02) / s.wheel_base, rel=1e-3)
    assert p.y > 0.0


def test_quaternion_helpers():
    q = quat_from_yaw(math.pi / 2)
    x, y, z = quat_rotate(q, (1.0, 0.0, 0.0))
    assert (x, y, z) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    back = quat_rotate(quat_conjugate(q), (x, y, z))
    assert back == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)
    assert quat_multiply(q, quat_conjugate(q)) == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)
