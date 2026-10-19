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
    assert a.launch_args == ['foo:=1'] and a.variant is None and a.final   # the graded set


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


def test_class_by_import_path():
    args, pp, name = R.resolve_controller(R.parse(['--class', 'p.m:C', '--params', '{kp: 1}']))
    assert pp is None and name == 'p.m.C'
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


def test_hold_defaults(monkeypatch):
    monkeypatch.setattr(R.sys.stdin, 'isatty', lambda: True, raising=False)
    assert R.parse([]).hold                        # from a terminal (after the last car)
    assert R.parse(['--final']).hold               # --final is the default set
    assert not R.parse(['--no-hold']).hold
    monkeypatch.setattr(R.sys.stdin, 'isatty', lambda: False, raising=False)
    assert not R.parse([]).hold                    # scripted: no one to press Enter
    assert R.parse(['--hold']).hold


def test_session_has_finds_a_member_by_command_line():
    import os
    import subprocess
    import uuid
    needle = f'probe-{uuid.uuid4().hex}'           # in no other command line of this session
    p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', needle],
                         start_new_session=True)
    try:
        import time
        deadline = time.monotonic() + 5.0          # until the child has exec'd
        while not R.session_has(p.pid, needle.encode()):
            assert time.monotonic() < deadline
            time.sleep(0.05)
        assert not R.session_has(p.pid, b'no-such-process-name')
        assert not R.session_has(os.getsid(0), needle.encode())
    finally:
        os.killpg(p.pid, 9)
        p.wait()


def test_hold_open_returns_without_a_dashboard(tmp_path):
    proc = SimpleNamespace(pid=2 ** 22 + 12345, poll=lambda: None)   # no such session
    R.hold_open(proc, tmp_path)                    # returns at once, prints nothing
    assert not R._holding


def test_web_addr_is_per_user(monkeypatch):
    assert R.default_web_addr(1000) == '127.0.0.1:8081'
    assert R.default_web_addr(1005) == '127.0.0.1:8086'
    monkeypatch.delenv('LAB2_WEB_ADDR', raising=False)
    assert R.parse([]).web_addr == R.default_web_addr()
    monkeypatch.setenv('LAB2_WEB_ADDR', '127.0.0.1:9000')
    assert R.parse([]).web_addr == '127.0.0.1:9000'


def test_keep_hint_names_the_run_and_latest(tmp_path):
    run = tmp_path / 'B1_lab_2_1' / '2026-10-19_14-03-07'
    text = R.keep_hint(run, dated=True)
    assert str(run) in text and str(run.parent / 'latest') in text
    assert R.keep_hint(tmp_path, dated=False).strip() == f'outputs in {tmp_path}'


def test_checks_per_run_is_the_metrics_check_list():
    import importlib.util
    path = R.REPO / 'src/lab/autosdv_waypoint_lab/autosdv_waypoint_lab/part1/metrics.py'
    spec = importlib.util.spec_from_file_location('part1_metrics_for_test', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert R.CHECKS_PER_RUN == len(m.CHECK_NAMES)


# ---- the simple command: `just lab2 sim` finds the submission, runs the graded set,
# ---- keeps every run, picks the windows, draws the plots

def sub(home, name, speed=True):
    d = home / name
    (d / 'lab2_speed' if speed else d).mkdir(parents=True)
    return d


def test_discovery_finds_the_single_submission(tmp_path):
    d = sub(tmp_path, 'B12345678_lab_2_1')
    sub(tmp_path, 'team3', speed=False)
    assert R.discover_submission(tmp_path) == d


def test_discovery_with_none_names_the_fix(tmp_path):
    sub(tmp_path, 'B1_lab_2_1', speed=False)             # no lab2_speed/ inside
    with pytest.raises(R.UsageError) as e:
        R.discover_submission(tmp_path)
    msg = str(e.value)
    assert str(tmp_path) in msg and 'lab2_speed' in msg and 'submission:=' in msg
    assert 'B1_lab_2_1' in msg                           # the near miss is named
    with pytest.raises(R.UsageError):
        R.discover_submission(tmp_path / 'missing')


def test_discovery_with_several_lists_them(tmp_path):
    a, b = sub(tmp_path, 'B1_lab_2_1'), sub(tmp_path, 'B2_lab_2_1')
    with pytest.raises(R.UsageError) as e:
        R.discover_submission(tmp_path)
    msg = str(e.value)
    assert str(a) in msg and str(b) in msg and 'part1_submission' in msg


def test_no_controller_discovers_under_lab2_home(tmp_path, monkeypatch):
    monkeypatch.setenv('LAB2_HOME', str(tmp_path))
    with pytest.raises(R.UsageError, match='lab2_speed'):
        R.resolve_controller(R.parse([]))


def test_the_graded_set_is_the_default():
    a = R.parse([])
    assert R.variants_of(a) == R.VARIANTS_FINAL and a.final
    assert R.parse(['--final']).final                    # old habit, same thing
    one = R.parse(['variant:=low_mu'])
    assert R.variants_of(one) == ('low_mu',) and not one.final


def test_no_gui_closes_every_window():
    a = R.parse(['--no-gui'])
    assert not a.rviz and not a.gui
    a = R.parse(['rviz:=false', 'gui:=false'])
    assert not a.rviz and not a.gui
    a = R.parse(['--no-rviz'])
    assert not a.rviz and a.gui


def test_hold_only_after_the_last_variant(monkeypatch):
    monkeypatch.setattr(R.sys.stdin, 'isatty', lambda: True, raising=False)
    a = R.parse([])
    assert a.hold
    assert not R.holds_after(a, 'nominal') and not R.holds_after(a, 'low_batt')
    assert R.holds_after(a, 'low_mu')
    assert R.holds_after(R.parse(['variant:=low_batt']), 'low_batt')
    assert not R.holds_after(R.parse(['--no-gui']), 'low_mu')


def test_each_run_gets_a_dated_directory_and_latest(tmp_path):
    import datetime
    now = datetime.datetime(2026, 10, 19, 14, 3, 7)
    r1 = R.run_dir(tmp_path, 'B1_lab_2_1', None, now)
    assert r1 == tmp_path / 'B1_lab_2_1' / '2026-10-19_14-03-07'
    r1.mkdir(parents=True)
    R.point_latest(r1)
    latest = tmp_path / 'B1_lab_2_1' / 'latest'
    assert latest.is_symlink() and latest.resolve() == r1.resolve()
    assert not Path(R.os.readlink(latest)).is_absolute()  # survives moving the tree
    r2 = R.run_dir(tmp_path, 'B1_lab_2_1', 'kd_off', now + datetime.timedelta(seconds=1))
    assert r2.name == '2026-10-19_14-03-08_kd_off'
    r2.mkdir()
    R.point_latest(r2)
    assert latest.resolve() == r2.resolve()


def test_out_root_explicit_out_keeps_the_old_layout(tmp_path):
    a = R.parse(['--out', str(tmp_path / 'o'), 'variant:=nominal'])
    assert R.out_root_for(a, 'x', base=tmp_path) == (tmp_path / 'o', False)
    root, dated = R.out_root_for(R.parse([]), 'x', base=tmp_path)
    assert dated and root.parent == tmp_path / 'x'


def test_no_working_display_runs_headless(monkeypatch, capsys):
    a = R.parse([])
    monkeypatch.setattr(R.le, 'display_works', lambda d, *k, **kw: False)
    R.check_display(a, {'DISPLAY': ':9'})
    assert not a.rviz and not a.gui
    assert 'headless' in capsys.readouterr().out
    b = R.parse([])
    monkeypatch.setattr(R.le, 'display_works', lambda d, *k, **kw: True)
    R.check_display(b, {'DISPLAY': ':0'})
    assert b.rviz and b.gui


def test_summary_table_names_every_variant_and_its_plot(tmp_path):
    results = [{'variant': 'nominal', 'valid': True, 'passed': 10, 'total': 11},
               {'variant': 'low_mu', 'valid': False, 'invalid_reason': 'MRM', 'passed': 0,
                'total': 11}]
    plots = {'nominal': tmp_path / 'nominal/curves.png', 'low_mu': None}
    text = R.summary_table(results, plots)
    assert 'nominal' in text and '10/11' in text and str(plots['nominal']) in text
    assert 'INVALID' in text and 'MRM' in text
