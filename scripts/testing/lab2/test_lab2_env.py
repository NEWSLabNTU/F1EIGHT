"""lab2_env.py: python3 -m pytest scripts/testing/lab2/test_lab2_env.py

The machine-side decisions of `just lab2 sim` / `just lab2 run`, with the environment and
the filesystem mocked: where ~/lab2 is, which X display works, tmux over SSH, car or bench.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lab2_env as le  # noqa: E402


# ---- ~/lab2

def test_lab2_home_defaults_to_home_lab2(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    assert le.lab2_home({}) == tmp_path / 'lab2'


def test_lab2_home_follows_the_override():
    assert le.lab2_home({'LAB2_HOME': '/x/y'}) == Path('/x/y')


# ---- X displays

def x11(tmp_path, *numbers):
    d = tmp_path / 'X11'
    d.mkdir(parents=True)
    for n in numbers:
        (d / f'X{n}').touch()
    return d


def test_display_works_asks_the_probe():
    assert le.display_works(':0', probe=lambda d: True)
    assert not le.display_works(':0', probe=lambda d: False)
    assert not le.display_works('', probe=lambda d: True)
    assert not le.display_works(None, probe=lambda d: True)


def test_pick_display_keeps_a_working_display(tmp_path):
    d, msg = le.pick_display({'DISPLAY': ':0'}, x11(tmp_path, 0, 1), probe=lambda d: True)
    assert d == ':0' and ':0' in msg


def test_pick_display_falls_back_to_the_desktop_on_1(tmp_path):
    ok = {':1'}
    d, msg = le.pick_display({'DISPLAY': ':7'}, x11(tmp_path, 1), probe=lambda d: d in ok)
    assert d == ':1' and ':7' in msg and ':1' in msg
    d, _ = le.pick_display({}, x11(tmp_path / 'b', 1), probe=lambda d: d in ok)
    assert d == ':1'


def test_pick_display_is_headless_without_a_desktop(tmp_path):
    d, msg = le.pick_display({}, x11(tmp_path), probe=lambda d: True)   # no X1 socket
    assert d is None and 'headless' in msg
    d, msg = le.pick_display({}, x11(tmp_path / 'b', 1), probe=lambda d: False)  # X1 dead
    assert d is None and 'headless' in msg


# ---- tmux over SSH

def test_tmux_only_over_ssh_and_outside_tmux():
    base = dict(have_tmux=True, tty=True)
    assert le.tmux_plan({'SSH_CONNECTION': '1 2 3 4'}, **base) == 'reexec'
    assert le.tmux_plan({'SSH_TTY': '/dev/pts/3'}, **base) == 'reexec'
    assert le.tmux_plan({}, **base) is None                                  # at the desk
    assert le.tmux_plan({'SSH_TTY': 'x', 'TMUX': '/tmp/tmux-1/default,1,0'}, **base) is None
    assert le.tmux_plan({'SSH_TTY': 'x'}, have_tmux=False, tty=True) == 'warn'
    assert le.tmux_plan({'SSH_TTY': 'x'}, have_tmux=True, tty=False) is None  # scripted
    assert le.tmux_plan({'SSH_TTY': 'x', 'LAB2_NO_TMUX': '1'}, **base) is None


def test_tmux_session_name_avoids_a_running_one():
    assert le.tmux_session_name(lambda n: False) == 'lab2-run'
    taken = {'lab2-run', 'lab2-run-2'}
    assert le.tmux_session_name(lambda n: n in taken) == 'lab2-run-3'


def test_tmux_command_reruns_the_same_command_and_keeps_the_window():
    cmd = le.tmux_command('lab2-run', ['just', 'lab2', 'run', 'team:=3'], '/repo',
                          {'LAB2_HOME': '/h', 'ROS_DOMAIN_ID': '5', 'PATH': '/bin', 'HOME': '/x'})
    assert cmd[:5] == ['tmux', 'new-session', '-s', 'lab2-run', '-c'] and cmd[5] == '/repo'
    shell = cmd[-1]
    assert "just lab2 run 'team:=3'" in shell or 'just lab2 run team:=3' in shell
    assert 'LAB2_HOME=/h' in shell and 'ROS_DOMAIN_ID=5' in shell and 'LAB2_NO_TMUX=1' in shell
    assert 'PATH=' not in shell                     # only the lab's own variables travel
    assert 'read' in shell                          # the window stays until Enter


# ---- car or bench

def test_car_hardware_is_a_jetson(tmp_path):
    assert not le.car_hardware(tmp_path / 'nv_tegra_release')
    (tmp_path / 'nv_tegra_release').write_text('# R36 (release), REVISION: 5.0\n')
    assert le.car_hardware(tmp_path / 'nv_tegra_release')


@pytest.mark.parametrize('hw, car, bench, force_car, mode', [
    (True, 'A', False, False, 'car'),
    (False, 'A', False, False, 'bench'),     # a workstation, even with a car letter set
    (False, None, False, False, 'bench'),
    (True, 'A', True, False, 'bench'),       # --bench wins
    (False, 'B', False, True, 'car'),        # --car wins
])
def test_decide_mode(hw, car, bench, force_car, mode):
    assert le.decide_mode(hw, car, bench, force_car) == mode


def test_a_car_without_its_letter_stops_with_the_ta_step():
    with pytest.raises(le.Lab2EnvError, match=r'just lab2 init car:=X'):
        le.decide_mode(True, None, False, False)
    with pytest.raises(le.Lab2EnvError, match=r'just lab2 init car:=X'):
        le.decide_mode(False, None, False, True)
    with pytest.raises(le.Lab2EnvError, match='both'):
        le.decide_mode(True, 'A', True, True)
