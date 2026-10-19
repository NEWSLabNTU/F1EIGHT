"""part1_run.py: python3 -m pytest scripts/testing/lab2/test_part1_run.py"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import part1_run as R  # noqa: E402
from test_submission import make_model  # noqa: E402


def test_key_value_and_options_intermix():
    a = R.parse(['submission:=~/x', '--out', 'o', 'rviz:=false', 'gui:=false', 'foo:=1'])
    assert a.submission == '~/x' and a.out == 'o' and not a.rviz and not a.gui
    assert a.launch_args == ['foo:=1'] and a.variant is None and a.final   # the graded set


def test_final_with_a_variant_is_an_error():
    with pytest.raises(SystemExit):
        R.parse(['--final', 'variant:=low_mu'])
    assert R.parse(['--final']).final


def test_runner_owned_launch_args_are_refused():
    for bad in ('speed_controller_class:=a:B', 'speed_controller_params:={}', 'out_dir:=x',
                'vehicle_id:=car_a', 'part1_submission:=x'):
        with pytest.raises(SystemExit):
            R.parse([bad])
    with pytest.raises(SystemExit):
        R.parse(['--params', '{}'])                 # --params needs --class
    for pair in (['--class', 'a.b:C', 'submission:=d'], ['model:=x_sim', '--class', 'a:B']):
        with pytest.raises(SystemExit):
            R.parse(pair)
    with pytest.raises(SystemExit):
        R.parse(['rviz:=maybe'])


def test_class_by_import_path():
    args, name, root = R.resolve_controller(R.parse(['--class', 'p.m:C', '--params', '{kp: 1}']))
    assert root is None and name == 'p.m.C'
    assert args == ['speed_controller_class:=p.m:C', 'speed_controller_params:={kp: 1}']


def test_missing_submission_is_a_usage_error(tmp_path):
    with pytest.raises(R.UsageError):
        R.resolve_controller(R.parse(['submission:=' + str(tmp_path / 'nope')]))


def test_model_and_vehicle_model_name_the_model():
    assert R.parse(['model:=x_sim']).model == 'x_sim'
    assert R.parse(['vehicle_model:=x_sim']).model == 'x_sim'
    assert R.parse(['team:=3']).launch_args == [] and R.parse(['--team', '3']).team == '3'
    assert R.parse(['model:=x_sim', 'submission:=d']).submission == 'd'    # TA: both fine


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

def ws_src(home):
    return home / 'ws' / 'src'


def lab2_home(tmp_path, monkeypatch):
    monkeypatch.setenv('LAB2_HOME', str(tmp_path))
    return {'LAB2_HOME': str(tmp_path)}


def install(tmp_path, root, model='lab2_sim_vehicle', symlink=True):
    """A fake install share dir for <model>_launch, its package.xml a symlink into root (as
    --symlink-install makes it) or a copy."""
    share = tmp_path / 'install' / f'{model}_launch' / 'share' / f'{model}_launch'
    share.mkdir(parents=True)
    src = root / f'{model}_launch' / 'package.xml'
    if symlink:
        (share / 'package.xml').symlink_to(src)
    else:
        (share / 'package.xml').write_text(src.read_text())
    return share


def test_the_model_is_found_through_the_install(tmp_path):
    model, root = make_model(tmp_path / 'repo')     # repo/src/lab2_sim_vehicle_launch
    assert model == R.SIM_MODEL == 'lab2_sim_vehicle'
    assert R.model_source(share=install(tmp_path, root)) == root.resolve()


def test_not_installed_names_the_copy_and_the_colcon_build(monkeypatch):
    monkeypatch.setattr(R, '_share', lambda pkg: None)
    with pytest.raises(R.UsageError) as e:
        R.model_source()
    msg = str(e.value)
    assert 'not built' in msg and 'src/lab2_vehicles/' in msg
    assert ('colcon build --base-paths src --symlink-install --cmake-args '
            '-DCMAKE_BUILD_TYPE=Release --packages-up-to lab2_sim_vehicle_launch') in msg


def test_an_install_without_symlinks_is_refused(tmp_path):
    _, root = make_model(tmp_path / 'repo')
    with pytest.raises(R.UsageError, match='(?s)without --symlink-install.*packages-up-to'):
        R.model_source(share=install(tmp_path, root, symlink=False))


def test_no_controller_and_no_install_is_a_usage_error(monkeypatch):
    monkeypatch.setattr(R, '_share', lambda pkg: None)
    with pytest.raises(R.UsageError, match='lab2_vehicles'):
        R.resolve_controller(R.parse([]))


def test_a_model_that_fails_its_check_stops_before_the_build_check(tmp_path, monkeypatch):
    _, root = make_model(tmp_path / 'repo', tuning='kp: 1\n')        # flat: the old layout
    share = install(tmp_path, root)
    monkeypatch.setattr(R, '_share', lambda pkg: share)
    with pytest.raises(R.UsageError, match='Part 1 check.*speed_controller_params'):
        R.resolve_controller(R.parse([]))


def test_the_reference_model_is_refused():
    with pytest.raises(R.UsageError, match='reference car model'):
        R.resolve_controller(R.parse(['model:=autosdv_vehicle']))


def test_not_built_names_the_colcon_build(tmp_path):
    model, root = make_model(tmp_path)
    with pytest.raises(R.UsageError) as e:
        R.check_built(model, root, installed=False)
    assert 'not built' in str(e.value) and '--packages-up-to lab2_sim_vehicle_launch' in str(e.value)


def test_built_from_another_copy_is_stale(tmp_path):
    model, root = make_model(tmp_path)
    pid = root / f'{model}_interface' / f'{model}_interface' / 'speed_pid.py'
    R.check_built(model, root, installed=True, imported=pid)          # the symlink build
    link = tmp_path / 'build' / 'speed_pid.py'
    link.parent.mkdir()
    link.symlink_to(pid)
    R.check_built(model, root, installed=True, imported=link)         # resolved through it
    copy = tmp_path / 'install' / 'speed_pid.py'
    copy.parent.mkdir()
    copy.write_text(pid.read_text())
    with pytest.raises(R.UsageError, match='(?s)another copy.*packages-up-to'):
        R.check_built(model, root, installed=True, imported=copy)


def test_the_model_resolves_to_its_launch_args(tmp_path, monkeypatch, capsys):
    model, root = make_model(tmp_path / 'repo')
    shares = {'lab2_sim_vehicle_launch': install(tmp_path, root)}
    o_model, o_root = make_model(tmp_path / 'other', model='other_sim')
    shares['other_sim_launch'] = install(tmp_path, o_root, model='other_sim')
    monkeypatch.setattr(R, '_share', shares.get)
    monkeypatch.setattr(R, 'check_built', lambda m, r: None)
    args = R.parse([])
    assert R.resolve_controller(args) == (['vehicle_model:=lab2_sim_vehicle'], model,
                                          root.resolve())
    assert args.model_name == model and args.model_root == str(root.resolve())
    assert R.resolve_controller(R.parse(['team:=3']))[1] == model      # team:= ignored
    assert 'not needed' in capsys.readouterr().out
    assert R.resolve_controller(R.parse(['model:=other_sim']))[:2] == \
        (['vehicle_model:=other_sim'], 'other_sim')


def test_a_submission_is_labelled_by_its_directory(tmp_path):
    d = tmp_path / 'B12345678_lab_2_1'
    model, _ = make_model(tmp_path / 'x')
    (tmp_path / 'x' / 'src').rename(d)               # <id>_lab_2_1/lab2_sim_vehicle_launch/
    root = d / 'lab2_sim_vehicle_launch'
    args = R.parse([f'submission:={d}'])
    assert R.resolve_controller(args) == (['vehicle_model:=lab2_sim_vehicle'], model, root)
    assert args.name == 'B12345678_lab_2_1'


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


def _play_log(tmp_path, line):
    d = tmp_path / 'play_log' / '2026-10-20_04-00-00' / 'node' / 'actuator_node'
    d.mkdir(parents=True)
    (d / 'err').write_text(f'[INFO] [1.0] [autosdv.actuator_node]: {line}\n')
    return tmp_path / 'play_log'


def _marks():
    return {'variant': 'nominal', 'valid': True, 'passed': 9, 'total': 9, 'checks': {}}


def test_a_run_of_the_builtin_controller_counts_zero(tmp_path):
    args = R.parse([])
    args.model_name = 'lab2_sim_vehicle'
    log = _play_log(tmp_path, 'Speed controller: '
                    'lab2_sim_vehicle_interface.speed_controller:AckermannSpeedPID')
    m = R.enforce_own_controller(_marks(), args, log, tmp_path)
    assert (m['valid'], m['passed']) == (False, 0) and 'only your own' in m['invalid_reason']
    assert json.loads((tmp_path / 'marks.json').read_text())['passed'] == 0


def test_a_run_of_the_models_own_speed_pid_keeps_its_marks(tmp_path):
    args = R.parse([])
    args.model_name = 'lab2_sim_vehicle'
    log = _play_log(tmp_path, 'Speed controller: lab2_sim_vehicle_interface.speed_pid:SpeedPID')
    m = R.enforce_own_controller(_marks(), args, log, tmp_path)
    assert (m['valid'], m['passed']) == (True, 9)
    assert m['speed_controller'] == 'lab2_sim_vehicle_interface.speed_pid:SpeedPID'
