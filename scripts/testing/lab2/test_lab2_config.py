"""lab2_config.py: python3 -m pytest scripts/testing/lab2/test_lab2_config.py"""
import os
import sys
import time
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lab2_config as lc  # noqa: E402


@pytest.fixture(autouse=True)
def no_real_home(tmp_path, monkeypatch):
    """Point ~ at an empty directory so the user's real ~/lab2/lab2.yaml never leaks in."""
    home = tmp_path / 'home'
    home.mkdir()
    monkeypatch.setenv('HOME', str(home))
    monkeypatch.delenv('LAB2_HOME', raising=False)
    return home


def bag(d: Path, name: str, mtime=None) -> Path:
    b = d / name
    b.mkdir(parents=True)
    (b / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    if mtime is not None:
        os.utime(b / 'metadata.yaml', (mtime, mtime))
    return b


def test_defaults_then_file_then_command_line(tmp_path):
    f = tmp_path / 'lab2.yaml'
    f.write_text('car: A\nroute: hall_demo\nteam: 3\n')
    c = lc.Config({'car': 'B'}, f)
    assert c.get('car') == 'B' and c.source('car') == 'command line'
    assert c.get('route') == 'hall_demo' and c.source('route') == str(f)
    assert c.get('team') == '3'
    assert c.get('bag_dir') == str(lc.REPO / 'rosbags') and c.source('bag_dir') == 'default'
    assert c.get('reference') == os.path.expanduser('~/lab2/reference/uturn')


def test_the_user_file_is_read_from_home_by_default(no_real_home):
    (no_real_home / 'lab2').mkdir()
    (no_real_home / 'lab2' / 'lab2.yaml').write_text('team_dir: team3\n')
    c = lc.Config()
    assert c.file == no_real_home / 'lab2' / 'lab2.yaml'
    assert c.get('team_dir') == str(no_real_home / 'lab2' / 'team3')   # relative to the file


def test_config_override_on_the_command_line_picks_another_file(tmp_path, no_real_home):
    (no_real_home / 'lab2').mkdir()
    (no_real_home / 'lab2' / 'lab2.yaml').write_text('car: A\n')
    other = tmp_path / 'other.yaml'
    other.write_text('car: C\nsubmission: ~/sub\n')
    for argv in (['config:=' + str(other)], ['--config', str(other)], ['--config=' + str(other)]):
        args, line = lc.plan('gate', argv)
        assert args == [f'submission:={no_real_home}/sub']
        assert str(other) in line or '~' in line
    assert lc.Config(None, other).get('car') == 'C'


def test_a_named_config_file_must_exist(tmp_path):
    with pytest.raises(lc.ConfigError, match='not found'):
        lc.Config(None, tmp_path / 'nope.yaml')


def test_a_missing_user_file_is_fine_when_nothing_required_is_missing():
    c = lc.Config()
    assert not c.file_exists and c.get('route') == 'lab2_uturn'


def test_missing_required_key_names_the_key_and_the_file(no_real_home):
    with pytest.raises(lc.ConfigError) as e:
        lc.plan('vehicle', [])
    msg = str(e.value)
    assert 'team_dir' in msg and str(no_real_home / 'lab2' / 'lab2.yaml') in msg
    assert '\n' not in msg
    with pytest.raises(lc.ConfigError, match='no submission set'):
        lc.plan('gate', [])
    with pytest.raises(lc.ConfigError, match='no car set'):
        lc.plan('record', ['team:=3'])


def test_unknown_key_in_the_file_is_an_error(tmp_path):
    f = tmp_path / 'x.yaml'
    f.write_text('cars: A\n')
    with pytest.raises(lc.ConfigError, match="unknown key 'cars'"):
        lc.Config(None, f)


def test_init_writes_a_template_and_refuses_to_overwrite(tmp_path):
    f = tmp_path / 'sub' / 'lab2.yaml'
    lc.init(['config:=' + str(f), 'car:=B'])
    text = f.read_text()
    assert 'car: B' in text and '# team: 3' in text
    c = lc.Config(None, f)
    assert c.get('car') == 'B' and c.get('team') is None     # placeholders stay commented
    with pytest.raises(lc.ConfigError, match='exists'):
        lc.init(['config:=' + str(f)])
    assert f.read_text() == text


def test_init_sets_only_the_given_keys_in_an_existing_file(tmp_path):
    f = tmp_path / 'lab2.yaml'
    original = ('# my notes, keep them\n'
                'team: 3          # our team\n'
                '# car: A                              # the letter\n'
                'bag_dir: rosbags\n'
                '\n'
                '# trailing note\n')
    f.write_text(original)
    os.chmod(f, 0o640)
    out = lc.init(['config:=' + str(f), 'car:=B'])
    text = f.read_text()
    assert 'car: B (was commented out)' in out and str(f) in out
    assert text.splitlines()[2].startswith('car: B') and text.splitlines()[2].endswith('# the letter')
    assert text.splitlines()[2].index('#') == original.splitlines()[2].index('# the')
    # every other line exactly as it was
    assert [ln for i, ln in enumerate(text.splitlines()) if i != 2] == \
        [ln for i, ln in enumerate(original.splitlines()) if i != 2]
    assert (f.stat().st_mode & 0o777) == 0o640
    c = lc.Config(None, f)
    assert c.get('car') == 'B' and str(c.get('team')) == '3'

    out = lc.init(['config:=' + str(f), 'team:=5', 'team_dir:=~/lab2/team5'])
    text = f.read_text()
    assert 'team: 3 -> 5' in out and 'team_dir: ~/lab2/team5 (added at the end)' in out
    assert 'team: 5          # our team' in text and text.endswith('team_dir: ~/lab2/team5\n')
    assert '# my notes, keep them' in text and '# trailing note' in text
    c = lc.Config(None, f)
    assert str(c.get('team')) == '5' and c.get('car') == 'B'

    before = f.read_text()
    out = lc.init(['config:=' + str(f), 'car:=B'])
    assert 'nothing to change' in out and 'car: B (unchanged)' in out
    assert f.read_text() == before


def test_init_quotes_a_value_yaml_would_misread(tmp_path):
    f = tmp_path / 'lab2.yaml'
    f.write_text('car: A\n')
    lc.init(['config:=' + str(f), 'route:=yes'])
    assert yaml.safe_load(f.read_text())['route'] == 'yes'


def test_init_refuses_to_edit_a_broken_file(tmp_path):
    f = tmp_path / 'lab2.yaml'
    f.write_text('car: [A\n')
    with pytest.raises(lc.ConfigError, match='not valid YAML'):
        lc.init(['config:=' + str(f), 'car:=B'])
    assert f.read_text() == 'car: [A\n'
    f.write_text('colour: red\n')
    with pytest.raises(lc.ConfigError, match='unknown key'):
        lc.init(['config:=' + str(f), 'car:=B'])


def test_init_on_a_template_it_wrote(tmp_path):
    f = tmp_path / 'lab2.yaml'
    lc.init(['config:=' + str(f)])
    lc.init(['config:=' + str(f), 'car:=C'])
    c = lc.Config(None, f)
    assert c.get('car') == 'C' and c.get('team') is None
    assert sum(ln.startswith('car:') for ln in f.read_text().splitlines()) == 1


def test_latest_bag_is_the_most_recently_finished(tmp_path):
    now = time.time()
    bag(tmp_path, 'lab2_uturn_9', now - 100)
    newest = bag(tmp_path, 'lab2_uturn_10', now)
    bag(tmp_path, 'lab2cal_steps', now + 50)              # not a lab2_* bag
    (tmp_path / 'lab2_unfinished').mkdir()                # no metadata.yaml yet
    assert lc.latest_bag(tmp_path) == newest
    with pytest.raises(lc.ConfigError, match='no bag'):
        lc.latest_bag(tmp_path / 'empty')


def test_next_bag_name_is_one_past_the_highest(tmp_path):
    assert lc.next_bag_name(tmp_path / 'none', 'lab2_uturn') == 'uturn_1'
    for n in (1, 2, 7):
        (tmp_path / f'lab2_uturn_{n}').mkdir()
    (tmp_path / 'lab2_uturn_x').mkdir()
    (tmp_path / 'lab2_hall_demo_3').mkdir()
    assert lc.next_bag_name(tmp_path, 'lab2_uturn') == 'uturn_8'
    assert lc.next_bag_name(tmp_path, 'hall_demo') == 'hall_demo_4'
    assert lc.next_bag_name(tmp_path, 'lab2_other') == 'other_1'
    assert lc.next_bag_name(tmp_path, '/some/dir/lab2_hall_demo/') == 'hall_demo_4'


def test_record_fills_car_team_and_the_next_name(tmp_path):
    f = tmp_path / 'x.yaml'
    f.write_text(f'car: A\nteam: 3\nbag_dir: {tmp_path / "bags"}\n')
    (tmp_path / 'bags' / 'lab2_uturn_1').mkdir(parents=True)
    args, line = lc.plan('record', ['config:=' + str(f)])
    assert args == ['--car', 'A', '--team', '3', '--out-dir', str(tmp_path / 'bags'),
                    '--name', 'uturn_2']
    assert 'name uturn_2' in line
    args, _ = lc.plan('record', ['config:=' + str(f), '--team', '5', '--name', 'mine', '--bench'])
    assert args == ['--car', 'A', '--team', '5', '--out-dir', str(tmp_path / 'bags'),
                    '--name', 'mine', '--bench']


def test_vehicle_and_gate_keep_the_explicit_arguments(tmp_path, monkeypatch):
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'surveys' / 'latest')
    f = tmp_path / 'x.yaml'
    f.write_text('car: B\nteam_dir: /t/team3\nsubmission: /s/sub\n')
    cf = 'config:=' + str(f)
    (tmp_path / 'c').mkdir()
    (tmp_path / 'c' / 'course.yaml').write_text('path: path.csv\n')
    assert lc.plan('vehicle', [cf])[0] == ['route:=lab2_uturn', 'team:=/t/team3', 'vehicle_id:=car_b']
    assert lc.plan('vehicle', [cf, 'route:=hall_demo', 'team:=/o/t'])[0] == \
        ['route:=hall_demo', 'team:=/o/t', 'vehicle_id:=car_b']
    assert lc.plan('vehicle', [cf, f'course_dir:={tmp_path / "c"}', 'submission:=/x'])[0] == \
        [f'course_dir:={tmp_path / "c"}', 'submission:=/x', 'vehicle_id:=car_b']
    with pytest.raises(lc.ConfigError, match='no course.yaml'):
        lc.plan('vehicle', [cf, 'course_dir:=/nonexistent'])
    assert lc.plan('vehicle', [cf, 'speed_controller_class:=a:B'])[0] == \
        ['speed_controller_class:=a:B', 'route:=lab2_uturn', 'vehicle_id:=car_b']
    assert lc.plan('gate', [cf])[0] == ['submission:=/s/sub']
    assert lc.plan('gate', [cf, '--team', '/o/t', '--only', 'hall_demo'])[0] == \
        ['--only', 'hall_demo', 'team:=/o/t']
    assert lc.plan('gate', ['speed_controller_class:=a:B'])[0] == ['speed_controller_class:=a:B']


def test_car_picks_the_vehicle_id(tmp_path):
    f = tmp_path / 'x.yaml'
    f.write_text('car: B\nteam_dir: /t/team3\n')
    cf = 'config:=' + str(f)
    args, line = lc.plan('vehicle', [cf])
    assert args[-1] == 'vehicle_id:=car_b' and 'vehicle_id car_b' in line
    for car, prof in (('A', 'car_a'), ('c', 'car_c'), ('car_b', 'car_b')):
        assert lc.plan('vehicle', [cf, f'car:={car}'])[0][-1] == f'vehicle_id:={prof}'
    # an explicit vehicle_id:= that agrees is kept once; one that disagrees is refused
    args = lc.plan('vehicle', [cf, 'vehicle_id:=car_b'])[0]
    assert args.count('vehicle_id:=car_b') == 1
    with pytest.raises(lc.ConfigError, match='vehicle_id:=car_a disagrees with car B'):
        lc.plan('vehicle', [cf, 'vehicle_id:=car_a'])
    with pytest.raises(lc.ConfigError, match='vehicle_id:=car_z'):
        lc.plan('vehicle', [cf, 'vehicle_id:=car_z'])
    with pytest.raises(lc.ConfigError, match="car 'D'"):
        lc.plan('vehicle', [cf, 'car:=D'])
    # sim is the simulators' nominal car, not a car `car` can name
    with pytest.raises(lc.ConfigError, match="car 'sim'") as e:
        lc.plan('vehicle', [cf, 'car:=sim'])
    assert 'SIM' not in str(e.value) and ', ,' not in str(e.value)
    assert lc.plan('loc-check', [cf, 'rviz:=true'])[0] == ['rviz:=true', 'vehicle_id:=car_b']
    # loc-check passes course_dir:= etc. through, and takes car:= as the setting
    assert lc.plan('loc-check', [cf, 'course_dir:=/d', 'car:=C'])[0] == \
        ['course_dir:=/d', 'vehicle_id:=car_c']


def test_no_car_is_refused_on_the_car_not_on_the_bench(no_real_home):
    with pytest.raises(lc.ConfigError, match='no car set') as e:
        lc.plan('vehicle', ['team_dir:=/t'])
    assert 'just lab2 init car:=A|B|C' in str(e.value) and '\n' not in str(e.value)
    # nothing drives in loc-check or the survey: no car is the nominal car
    assert lc.plan('loc-check', [])[0] == ['vehicle_id:=sim']
    # the bench keeps its launch default, sim; the hardware sim is passed sim
    # (waypoint_follow.launch.xml requires vehicle_id)
    args = lc.plan('vehicle', ['team_dir:=/t'], bench=True)[0]
    assert not any(a.startswith('vehicle_id:=') for a in args)
    for sim in ('hardware:=sim', 'hardware_sim:=true'):
        assert [a for a in lc.plan('vehicle', ['team_dir:=/t', sim])[0]
                if a.startswith('vehicle_id:=')] == ['vehicle_id:=sim']
    # the survey takes the car the same way loc-check does
    assert lc.plan('survey', ['car:=B', 'marker_size:=0.25'])[0] == \
        ['marker_size:=0.25', 'vehicle_id:=car_b']
    assert lc.plan('survey', ['marker_size:=0.25'])[0] == ['marker_size:=0.25', 'vehicle_id:=sim']
    # survey-align: the settings' route and car for `just lab2 survey-align`, which reads no settings
    assert lc.plan('survey-align', ['car:=B'])[0] == ['--route', 'lab2_uturn', 'vehicle_id:=car_b']
    assert lc.plan('survey-align', ['/s', '--force'])[0] == \
        ['/s', '--force', '--route', 'lab2_uturn', 'vehicle_id:=sim']
    assert lc.plan('survey-align', ['route:=lab2_straight_2m', '--vehicle-id', 'car_c'])[0] == \
        ['--vehicle-id', 'car_c', '--route', 'lab2_straight_2m']
    assert lc.plan('survey-align', ['--route', 'x', 'car:=B', '--bench', '--truth', '/t'])[0] == \
        ['--route', 'x', '--bench', '--truth', '/t']
    # a car set on the bench is still that car's profile
    assert lc.plan('vehicle', ['team_dir:=/t', 'car:=B'], bench=True)[0][-1] == 'vehicle_id:=car_b'
    # an explicit vehicle_id:= alone names the car
    args = lc.plan('vehicle', ['team_dir:=/t', 'vehicle_id:=car_c'])[0]
    assert [a for a in args if a.startswith('vehicle_id:=')] == ['vehicle_id:=car_c']


def test_grade_scores_the_latest_bag_against_the_reference(tmp_path):
    ref = bag(tmp_path, 'ref')
    latest = bag(tmp_path / 'bags', 'lab2_uturn_1')
    f = tmp_path / 'x.yaml'
    f.write_text(f'reference: {ref}\nbag_dir: {tmp_path / "bags"}\n')
    cf = 'config:=' + str(f)
    assert lc.plan('grade', [cf])[0] == ['--score', '--reference', str(ref), str(latest)]
    assert lc.plan('grade', [cf, 'B1', 'B2'])[0] == ['B1', 'B2']           # metrics, as before
    assert lc.plan('grade', [cf, '--score', 'B1', '--reference', 'R'])[0] == \
        ['--score', 'B1', '--reference', 'R']
    assert lc.plan('check-bags', [cf, 'car:=B'])[0] == ['--car', 'B', str(latest)]
    assert lc.plan('plot', [cf])[0] == [str(latest)]
    assert lc.plan('plot', [cf, 'some/bag'])[0] == ['some/bag']


def test_show_names_every_source(tmp_path):
    f = tmp_path / 'x.yaml'
    f.write_text('car: A\n')
    out = lc.show(['config:=' + str(f), 'team:=4'])
    assert f'# {f}' in out and '# command line' in out and '# default' in out and '# not set' in out


def _aligned_survey(root, route='lab2_uturn', path_text=None):
    """A survey directory as `just lab2 survey-align` leaves it."""
    import shutil
    src = lc.COURSES / route
    d = root / 'surveys' / '2026-10-16_10-00-00'
    d.mkdir(parents=True)
    t = yaml.safe_load((src / 'course.yaml').read_text())
    t['alignment'] = {'route': route}
    t['markers'] = [{'id': 24, 'x': 1.0, 'y': 2.0, 'yaw': 0.1}]
    (d / 'course.yaml').write_text(yaml.safe_dump(t))
    shutil.copyfile(src / 'path.csv', d / 'path.csv')
    if path_text is not None:
        (d / 'path.csv').write_text(path_text)
    (root / 'surveys' / 'latest').symlink_to(d.name)
    return d


def test_vehicle_runs_the_aligned_survey(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'surveys' / 'latest')
    f = tmp_path / 'x.yaml'
    f.write_text('car: A\nteam_dir: /t/team3\n')
    cf = 'config:=' + str(f)
    # no survey: the packaged course, said out loud
    assert lc.plan('vehicle', [cf])[0][0] == 'route:=lab2_uturn'
    assert 'PACKAGED' in capsys.readouterr().err
    d = _aligned_survey(tmp_path)
    args, line = lc.plan('vehicle', [cf])
    assert args[0] == f'course_dir:={d.resolve()}' and 'course_dir' in line
    # the bench, `package`, and another route never take it
    assert lc.plan('vehicle', [cf], bench=True)[0][0] == 'route:=lab2_uturn'
    assert lc.plan('vehicle', [cf, 'course_dir:=package'])[0][0] == 'route:=lab2_uturn'
    assert lc.plan('vehicle', [cf, 'route:=hall_demo'])[0][0] == 'route:=hall_demo'
    assert "aligned for route 'lab2_uturn'" in capsys.readouterr().err


def test_a_stale_or_unaligned_survey_is_not_run(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'surveys' / 'latest')
    d = _aligned_survey(tmp_path, path_text='0,0\n1,0\n')
    ok, why = lc.aligned_for(d, 'lab2_uturn')
    assert not ok and 'older' in why
    assert lc.plan('vehicle', ['team_dir:=/t', 'car:=A'])[0][0] == 'route:=lab2_uturn'
    assert 'run `just lab2 survey-align` again' in capsys.readouterr().err
    t = yaml.safe_load((d / 'course.yaml').read_text())
    del t['alignment']
    (d / 'course.yaml').write_text(yaml.safe_dump(t))
    ok, why = lc.aligned_for(d, 'lab2_uturn')
    assert not ok and 'not aligned' in why


def test_sim_reads_part1_submission_not_the_gate_submission(tmp_path):
    f = tmp_path / 'lab2.yaml'
    f.write_text('submission: /s/gate\npart1_submission: /s/part1\n')
    cf = f'config:={f}'
    assert lc.plan('sim', [cf, '--final'])[0] == ['--final', 'submission:=/s/part1']
    assert lc.plan('sim', [cf, 'submission:=/x'])[0] == ['submission:=/x']
    assert lc.plan('sim', [cf, '--class', 'a.b:C'])[0] == ['--class', 'a.b:C']
    f.write_text('submission: /s/gate\n')
    assert lc.plan('sim', [cf])[0] == []           # no part1_submission: the runner's stub
    assert lc.plan('gate', [cf])[0] == ['submission:=/s/gate']


# ---- survey reuse: an aligned survey of the U-turn's field serves a straight on that field

def _real_aligned_survey(root, route='lab2_uturn'):
    """A survey aligned for `route` by the real solver, from a synthetic stage 1 and a
    synthetic straight drive (as autosdv_marker_survey/test/test_align.py makes them),
    with surveys/latest pointing at it."""
    import math
    import random
    import numpy as np
    for pkg in ('autosdv_marker_survey', 'autosdv_waypoint_lab'):
        p = str(lc.REPO / 'src/lab' / pkg)
        if p not in sys.path:
            sys.path.insert(0, p)
    from autosdv_marker_survey import align as al
    from autosdv_marker_survey.simulate import stage1_map
    t = yaml.safe_load((lc.COURSES / 'lab2_uturn' / 'course.yaml').read_text())
    truth = {m['id']: (m['x'] + 0.03, m['y'] - 0.02, 0.25, m['yaw'] + 0.01, 0.0, 0.0)
             for m in t['markers']}
    stage1, T_w_s1 = stage1_map(truth, 24)
    d = root / 'surveys' / '2026-10-16_10-00-00'
    d.mkdir(parents=True)
    (d / 'course.yaml').write_text(yaml.safe_dump(
        {'path': 'path.csv', 'closed': True, 'laps': 1, 'v_ref': 0.8,
         'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'marker_size': 0.38,
         'dictionary': 'DICT_5X5_1000', 'survey': {'origin_marker': 24}, 'markers': stage1}))
    al.prepare_stage1(str(d))
    T = np.linalg.inv(T_w_s1)
    x0, y0, yaw0 = T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])
    rng = random.Random(1)
    samples, tt = [], 0.0
    for k in range(100):            # 2 s at rest, then 3 m at 0.3 m/s, then at rest
        samples.append(al.Sample(tt, x0 + rng.gauss(0, 0.004), y0 + rng.gauss(0, 0.004), 0.0, yaw0,
                                 0.0, 0.0, 0.0))
        tt += 0.02
    dd = 0.0
    while dd < 3.0:
        dd = min(3.0, dd + 0.006)
        samples.append(al.Sample(tt, x0 + math.cos(yaw0) * dd + rng.gauss(0, 0.004),
                                 y0 + math.sin(yaw0) * dd + rng.gauss(0, 0.004), 0.0, yaw0, 0.0, 0.0, 0.3))
        tt += 0.02
    al.write_samples(str(d / al.POSES_FILE), samples)
    a, _ = al.solve_dir(str(d), str(lc.COURSES / route), samples, log=lambda *x: None)
    assert a.ok, a.problems
    (root / 'surveys' / 'latest').symlink_to(d.name)
    return d


def _tree(d: Path) -> dict:
    return {str(p.relative_to(d)): p.read_bytes() for p in sorted(d.rglob('*')) if p.is_file()}


def test_same_field_follows_the_field_key():
    assert lc.same_field('lab2_uturn', 'lab2_straight_3m')
    assert lc.same_field('lab2_straight_2m', 'lab2_straight_3m')
    assert not lc.same_field('lab2_uturn', 'hall_demo')


def test_a_straight_reuses_the_uturn_survey_in_a_sibling(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'surveys' / 'latest')
    d = _real_aligned_survey(tmp_path)
    before = _tree(d)
    args, _ = lc.plan('vehicle', ['team_dir:=/t', 'car:=A', 'route:=lab2_straight_3m'])
    sib = tmp_path / 'surveys' / f'{d.name}.lab2_straight_3m'
    assert args[0] == f'course_dir:={sib}'
    err = capsys.readouterr().err
    assert f'survey: {sib}' in err and 're-solved' in err
    ok, why = lc.aligned_for(sib, 'lab2_straight_3m')
    assert ok, why
    # the same drive, the same frame: the boards stand where latest has them
    mine = yaml.safe_load((sib / 'course.yaml').read_text())['markers']
    theirs = yaml.safe_load((d / 'course.yaml').read_text())['markers']
    assert mine == theirs
    # latest itself is never touched
    assert _tree(d) == before
    # cached: the second run re-solves nothing
    stamp = (sib / 'course.yaml').stat().st_mtime_ns
    args2, _ = lc.plan('vehicle', ['team_dir:=/t', 'car:=A', 'route:=lab2_straight_3m'])
    assert args2 == args and (sib / 'course.yaml').stat().st_mtime_ns == stamp
    assert 'cached' in capsys.readouterr().err
    # the U-turn still runs latest itself
    assert lc.plan('vehicle', ['team_dir:=/t', 'car:=A'])[0][0] == f'course_dir:={d.resolve()}'


def test_the_sibling_follows_a_new_alignment_of_latest(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'surveys' / 'latest')
    d = _real_aligned_survey(tmp_path)
    lc.plan('vehicle', ['team_dir:=/t', 'car:=A', 'route:=lab2_straight_2m'])
    sib = tmp_path / 'surveys' / f'{d.name}.lab2_straight_2m'
    old = (sib / 'course.yaml').read_text()
    # survey-align again: another drive (shifted 2 cm) -> other alignment inputs
    from autosdv_marker_survey import align as al
    ss = al.read_samples(str(d / al.POSES_FILE))
    for s in ss:
        s.y += 0.02
    al.write_samples(str(d / al.POSES_FILE), ss)
    capsys.readouterr()
    lc.plan('vehicle', ['team_dir:=/t', 'car:=A', 'route:=lab2_straight_2m'])
    assert 're-solved' in capsys.readouterr().err
    assert (sib / 'course.yaml').read_text() != old


def test_reuse_falls_back_to_the_packaged_course(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'surveys' / 'latest')
    d = _real_aligned_survey(tmp_path)
    # another field: never reused
    assert lc.plan('vehicle', ['team_dir:=/t', 'car:=A', 'route:=hall_demo'])[0][0] == 'route:=hall_demo'
    assert 'PACKAGED' in capsys.readouterr().err
    # no recorded drive: nothing to re-solve from
    (d / 'align_poses.csv').unlink()
    assert lc.plan('vehicle', ['team_dir:=/t', 'car:=A', 'route:=lab2_straight_3m'])[0][0] == \
        'route:=lab2_straight_3m'
    err = capsys.readouterr().err
    assert 'PACKAGED' in err and 'align_poses.csv' in err
    assert not (tmp_path / 'surveys' / f'{d.name}.lab2_straight_3m').exists()


def test_lab2_home_moves_the_settings_file_and_the_reference(tmp_path, monkeypatch):
    home = tmp_path / 'elsewhere'
    home.mkdir()
    (home / 'lab2.yaml').write_text('car: C\n')
    monkeypatch.setenv('LAB2_HOME', str(home))
    c = lc.Config()
    assert c.file == home / 'lab2.yaml' and c.get('car') == 'C'
    assert c.get('reference') == str(home / 'reference' / 'uturn')
