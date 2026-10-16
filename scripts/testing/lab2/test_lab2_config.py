"""lab2_config.py: python3 -m pytest scripts/testing/lab2/test_lab2_config.py"""
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lab2_config as lc  # noqa: E402


@pytest.fixture(autouse=True)
def no_real_home(tmp_path, monkeypatch):
    """Point ~ at an empty directory so the user's real ~/lab2/lab2.yaml never leaks in."""
    home = tmp_path / 'home'
    home.mkdir()
    monkeypatch.setenv('HOME', str(home))
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


def test_vehicle_and_gate_keep_the_explicit_arguments(tmp_path):
    f = tmp_path / 'x.yaml'
    f.write_text('team_dir: /t/team3\nsubmission: /s/sub\n')
    cf = 'config:=' + str(f)
    assert lc.plan('vehicle', [cf])[0] == ['route:=lab2_uturn', 'team:=/t/team3']
    assert lc.plan('vehicle', [cf, 'route:=hall_demo', 'team:=/o/t'])[0] == \
        ['route:=hall_demo', 'team:=/o/t']
    assert lc.plan('vehicle', [cf, 'course_dir:=/c', 'submission:=/x'])[0] == \
        ['course_dir:=/c', 'submission:=/x']
    assert lc.plan('vehicle', [cf, 'speed_controller_class:=a:B'])[0] == \
        ['speed_controller_class:=a:B', 'route:=lab2_uturn']
    assert lc.plan('gate', [cf])[0] == ['submission:=/s/sub']
    assert lc.plan('gate', [cf, '--team', '/o/t', '--only', 'hall_demo'])[0] == \
        ['--only', 'hall_demo', 'team:=/o/t']
    assert lc.plan('gate', ['speed_controller_class:=a:B'])[0] == ['speed_controller_class:=a:B']


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
