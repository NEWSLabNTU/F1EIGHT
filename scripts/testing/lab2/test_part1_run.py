"""part1_run.py: python3 -m pytest scripts/testing/lab2/test_part1_run.py"""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import part1_run as R  # noqa: E402


def test_key_value_and_options_intermix():
    a = R.parse(['submission:=~/x', '--out', 'o', 'rviz:=false', 'gui:=false', 'foo:=1'])
    assert a.submission == '~/x' and a.out == 'o' and not a.rviz and not a.gui
    assert a.launch_args == ['foo:=1'] and a.variant == 'nominal' and not a.final


def test_final_with_a_variant_is_an_error():
    with pytest.raises(SystemExit):
        R.parse(['--final', 'variant:=low_mu'])
    assert R.parse(['--final']).final


def test_runner_owned_launch_args_are_refused():
    for bad in ('speed_controller_class:=a:B', 'speed_controller_params:={}', 'out_dir:=x'):
        with pytest.raises(SystemExit):
            R.parse([bad])
    with pytest.raises(SystemExit):
        R.parse(['--params', '{}'])                 # --params needs --class
    with pytest.raises(SystemExit):
        R.parse(['--class', 'a.b:C', 'submission:=d'])
    with pytest.raises(SystemExit):
        R.parse(['rviz:=maybe'])


def test_no_controller_is_the_stub():
    args, _, name, stub = R.resolve_controller(R.parse([]))
    assert stub and name == 'stub' and args[0] == f'speed_controller_class:={R.STUB_CLASS}'
    assert args[1].startswith('speed_controller_params:={') and 'kp:' in args[1]


def test_class_by_import_path():
    args, pp, name, stub = R.resolve_controller(R.parse(['--class', 'p.m:C', '--params', '{kp: 1}']))
    assert not stub and pp is None and name == 'p.m.C'
    assert args == ['speed_controller_class:=p.m:C', 'speed_controller_params:={kp: 1}']


def test_missing_submission_is_a_usage_error(tmp_path):
    with pytest.raises(R.UsageError):
        R.resolve_controller(R.parse(['submission:=' + str(tmp_path / 'nope')]))


def test_domain_id():
    assert R.domain_id({'ROS_DOMAIN_ID': '42'}) == ('42', 'exported')
    for env in ({}, {'ROS_DOMAIN_ID': '0'}):
        d, _ = R.domain_id(env)
        assert 1 <= int(d) <= 100


def test_variants_include_the_final_set():
    assert set(R.VARIANTS_FINAL) <= set(R.known_variants())


def test_stall_startup_and_cap():
    args = SimpleNamespace(startup_timeout=120.0, stall=30.0, timeout=0.0)
    w = SimpleNamespace(sim_t=None, sim_t_wall=None, rtf=None, node=object())
    assert R.why_stop(args, w, 0.0, 100.0, 1.0) is None
    assert 'did not report' in R.why_stop(args, w, 0.0, 121.0, 1.0)
    w.sim_t, w.sim_t_wall = 10.0, 100.0
    assert R.why_stop(args, w, 0.0, 129.0, 1.0) is None
    assert 'stuck' in R.why_stop(args, w, 0.0, 131.0, 1.0)
    w.sim_t_wall = 700.0                       # still advancing; cap = 120 + 540 / rtf
    assert R.why_stop(args, w, 0.0, 700.0, 1.0) is not None
    w.rtf = 0.5
    assert R.why_stop(args, w, 0.0, 700.0, 1.0) is None
    w.sim_t_wall = 1201.0
    assert 'rtf 0.50' in R.why_stop(args, w, 0.0, 1201.0, 1.0)
