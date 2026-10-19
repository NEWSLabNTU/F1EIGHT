"""`just coach record-part1`'s arguments, without ROS: python3 -m pytest scripts/testing/lab2/test_record_part1.py"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_part1 as rp  # noqa: E402


def test_car_needs_a_vehicle_id(capsys):
    with pytest.raises(SystemExit) as e:
        rp.parse_args(['--name', 'x'])
    assert e.value.code == 2
    assert 'vehicle_id:= is required' in capsys.readouterr().err


def test_vehicle_id_is_checked_and_passed_through():
    opts = rp.parse_args(['vehicle_id:=car_b', 'lidar_model:=robin-w'])
    assert opts.vehicle_id == 'car_b'
    assert opts.launch_args == ['vehicle_id:=car_b', 'lidar_model:=robin-w']
    with pytest.raises(SystemExit):
        rp.parse_args(['vehicle_id:=car_z'])


def test_bench_defaults_to_car_a():
    assert rp.parse_args(['--bench', '--yes']).vehicle_id == 'car_a'
    assert rp.parse_args(['--bench', 'vehicle_id:=car_b']).vehicle_id == 'car_b'


def test_last_vehicle_id_wins():
    assert rp.launch_vehicle_id(['vehicle_id:=car_a', 'x:=1', 'vehicle_id:=car_c']) == 'car_c'
    assert rp.launch_vehicle_id(['x:=1']) is None
