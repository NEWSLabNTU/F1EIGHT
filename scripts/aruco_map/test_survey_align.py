"""scripts/aruco_map: settings from the command line only, nothing of Lab 2."""
import re
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import survey_align as sa  # noqa: E402


def test_a_route_is_required():
    with pytest.raises(sa.Usage, match='which route'):
        sa.parse(['surveys/x'])


def test_the_nominal_car_by_default_and_key_value_spellings():
    a = sa.parse(['surveys/x', '--route', 'lab2_uturn'])
    assert (a.survey_dir, a.route, a.vehicle_id) == ('surveys/x', 'lab2_uturn', 'sim')
    a = sa.parse(['route:=/r', 'vehicle_id:=car_c', '--force'])
    assert (a.survey_dir, a.route, a.vehicle_id, a.force) == ('surveys/latest', '/r', 'car_c', True)
    assert sa.parse(['--route', 'r', '--vehicle-id', 'car_b']).vehicle_id == 'car_b'


def test_the_bench_needs_its_truth():
    with pytest.raises(sa.Usage, match='--truth'):
        sa.parse(['--route', 'r', '--bench'])


def test_nothing_of_lab_2_is_read():
    lab = re.compile(r'^\s*(import|from)\s+(lab2_\w+|autosdv_waypoint_lab|autosdv_coach_lab)\b|lab2\.yaml'
                     r'|lab2_config\.(sh|py)|sys\.path.*lab2|source scripts/testing/lab2', re.M)
    for f in sorted(HERE.glob('*.py')) + sorted(HERE.glob('*.sh')):
        if f.name.startswith('test_'):
            continue
        assert not lab.search(f.read_text()), f


def test_the_command_lists_its_commands():
    r = subprocess.run(['bash', str(HERE / 'aruco_map.sh'), 'help'], capture_output=True, text=True)
    assert r.returncode == 0
    for cmd in ('survey ORIGIN_ID', 'scan', 'align', 'solve', 'check', 'save'):
        assert cmd in r.stdout
    assert 'set -u' not in r.stdout
    r = subprocess.run(['bash', str(HERE / 'aruco_map.sh'), 'bogus'], capture_output=True, text=True)
    assert r.returncode == 2 and 'unknown command' in r.stderr


def test_a_scan_takes_no_route():
    r = subprocess.run(['bash', str(HERE / 'aruco_map.sh'), 'scan', '24', '--route', 'x'],
                       capture_output=True, text=True)
    assert r.returncode == 2 and 'survey ORIGIN_ID --route x' in r.stderr
