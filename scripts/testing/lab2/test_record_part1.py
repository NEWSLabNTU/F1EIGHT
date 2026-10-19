"""`just coach record-part1`'s arguments, without ROS: python3 -m pytest scripts/testing/lab2/test_record_part1.py"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_part1 as rp  # noqa: E402


def test_car_needs_a_car_profile(capsys):
    with pytest.raises(SystemExit) as e:
        rp.parse_args(['--name', 'x'])
    assert e.value.code == 2
    assert 'car_profile:= is required' in capsys.readouterr().err


def test_car_profile_is_checked_and_passed_through():
    opts = rp.parse_args(['car_profile:=car_b', 'lidar_model:=robin-w'])
    assert opts.car_profile == 'car_b'
    assert opts.launch_args == ['car_profile:=car_b', 'lidar_model:=robin-w']
    with pytest.raises(SystemExit):
        rp.parse_args(['car_profile:=car_z'])


def test_bench_defaults_to_car_a():
    assert rp.parse_args(['--bench', '--yes']).car_profile == 'car_a'
    assert rp.parse_args(['--bench', 'car_profile:=car_b']).car_profile == 'car_b'


def test_last_car_profile_wins():
    assert rp.launch_car_profile(['car_profile:=car_a', 'x:=1', 'car_profile:=car_c']) == 'car_c'
    assert rp.launch_car_profile(['x:=1']) is None
