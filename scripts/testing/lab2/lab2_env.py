"""The machine-side decisions `just lab2 sim` and `just lab2 run` make for the student.

Pure functions over an environment mapping and the filesystem, so the unit tests
(test_lab2_env.py) can mock both:

    lab2_home(env)          ~/lab2, or $LAB2_HOME: the settings file, the Part 1
                            submission, the team directories and the reference bag
    display_works(d)        does an X server answer on d (xdpyinfo, else xset, else
                            the socket): the probe `just lab2 loc-check` uses
    pick_display(env)       Part 2: $DISPLAY if it answers, else :1 (the car's desktop,
                            seen over RDP) if it answers, else headless
    tmux_plan(env)          started over SSH and not inside tmux: re-run inside tmux
    car_hardware()          a Jetson (/etc/nv_tegra_release): this is a car
    decide_mode(...)        car or bench (the rule is in its docstring)
"""
import os
import shlex
import shutil
import subprocess
from pathlib import Path
from typing import Callable, Mapping, Optional, Tuple

X11_DIR = Path('/tmp/.X11-unix')
TEGRA_RELEASE = Path('/etc/nv_tegra_release')
DESKTOP = ':1'
TMUX_SESSION = 'lab2-run'
# what travels into the tmux session: tmux takes the server's environment, not ours
TMUX_ENV_PREFIXES = ('LAB2_', 'AUTOSDV_')
TMUX_ENV_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'DISPLAY')
CAR_LETTER_HINT = "this car has no car letter: the TA sets it once per car with `just lab2 init car:=X`"


class Lab2EnvError(Exception):
    pass


def lab2_home(env: Mapping[str, str] = os.environ) -> Path:
    """~/lab2, or $LAB2_HOME when set (tests, and a TA grading from another directory)."""
    v = env.get('LAB2_HOME')
    return Path(os.path.expanduser(v)) if v else Path(os.path.expanduser('~/lab2'))


# ---- X displays

def x_probe(display: str, x11_dir: Path = X11_DIR) -> bool:
    """True when an X server answers on `display`: xdpyinfo, else xset, else its socket."""
    for tool, args in (('xdpyinfo', ['-display', display]), ('xset', ['-display', display, 'q'])):
        if shutil.which(tool):
            try:
                return subprocess.run([tool, *args], capture_output=True, timeout=5).returncode == 0
            except (OSError, subprocess.SubprocessError):
                return False
    if display.startswith(':'):
        return (Path(x11_dir) / f'X{display[1:].split(".")[0]}').exists()
    return False


def display_works(display: Optional[str], probe: Callable[[str], bool] = x_probe) -> bool:
    return bool(display) and bool(probe(display))


def pick_display(env: Mapping[str, str], x11_dir: Path = X11_DIR,
                 probe: Callable[[str], bool] = x_probe) -> Tuple[Optional[str], str]:
    """(the display for the viewers or None, one line saying which and why)."""
    cur = env.get('DISPLAY') or ''
    if display_works(cur, probe):
        return cur, f'display {cur}'
    was = f'DISPLAY={cur!r} does not answer' if cur else 'no DISPLAY'
    if (Path(x11_dir) / f'X{DESKTOP[1:]}').exists() and display_works(DESKTOP, probe):
        return DESKTOP, f'{was}: using {DESKTOP} (the desktop; view it over RDP)'
    return None, f'{was} and no desktop on {DESKTOP}: headless (plots are saved, not shown)'


# ---- tmux

def tmux_plan(env: Mapping[str, str], have_tmux: bool, tty: bool) -> Optional[str]:
    """'reexec' (re-run inside tmux), 'warn' (over SSH without tmux) or None."""
    over_ssh = bool(env.get('SSH_CONNECTION') or env.get('SSH_TTY'))
    if not over_ssh or env.get('TMUX') or env.get('LAB2_NO_TMUX') or not tty:
        return None
    return 'reexec' if have_tmux else 'warn'


def tmux_session_name(exists: Callable[[str], bool]) -> str:
    """lab2-run, or lab2-run-2, -3, ... while a session of that name is running."""
    name, n = TMUX_SESSION, 1
    while exists(name):
        n += 1
        name = f'{TMUX_SESSION}-{n}'
    return name


def tmux_exists(name: str) -> bool:
    return subprocess.run(['tmux', 'has-session', '-t', f'={name}'],
                          capture_output=True).returncode == 0


def tmux_command(session: str, argv, cwd, env: Mapping[str, str]):
    """tmux new-session running `argv` in `cwd`, with the lab's variables, then holding the
    window open until Enter so the result stays on screen."""
    keep = {k: v for k, v in env.items()
            if k.startswith(TMUX_ENV_PREFIXES) or k in TMUX_ENV_KEYS}
    keep['LAB2_NO_TMUX'] = '1'
    assigns = ' '.join(f'{k}={shlex.quote(v)}' for k, v in sorted(keep.items()))
    run = ' '.join(shlex.quote(a) for a in argv)
    shell = (f'env {assigns} {run}; echo; '
             f'read -r -p "lab2 run is over; press Enter to close this tmux window. " _')
    return ['tmux', 'new-session', '-s', session, '-c', str(cwd), 'bash', '-c', shell]


# ---- car or bench

def car_hardware(tegra_release: Path = TEGRA_RELEASE) -> bool:
    """A Jetson: the car's computer. The PCA9685 is not probed: reading the I2C bus needs
    the i2c group and races the actuator, and every car is a Jetson."""
    return Path(tegra_release).is_file()


def decide_mode(hardware: bool, car: Optional[str], force_bench: bool, force_car: bool) -> str:
    """'car' or 'bench'.

    --bench or --car decide when given (not both). Otherwise a Jetson is a car and
    anything else is a workstation, which runs the vehicle bench. A car needs its car
    letter (`car:` in ~/lab2/lab2.yaml, written by the TA's `just lab2 init car:=X`):
    without it, stop rather than record a bag with no car."""
    if force_bench and force_car:
        raise Lab2EnvError('--bench and --car both given: pick one')
    mode = 'bench' if force_bench else 'car' if (force_car or hardware) else 'bench'
    if mode == 'car' and not car:
        raise Lab2EnvError(CAR_LETTER_HINT)
    return mode
