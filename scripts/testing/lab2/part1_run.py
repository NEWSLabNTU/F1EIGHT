#!/usr/bin/env python3
"""Lab 2 Part 1 runner: the simulation vehicle model on the simulated car, live, graded.

    just lab2 sim                           the graded set (nominal, low_batt, low_mu), mark / 25
    just lab2 sim --no-gui                  the same without windows (SSH, batch)
    just lab2 plot-sim                      redraw the latest run's curves.png

    TA:
    just lab2 sim submission:=DIR           a student's model source, built here first (below)
    just lab2 sim variant:=low_mu           one car only (no mark out of 25)
    just lab2 sim model:=NAME               another model than lab2_sim_vehicle
    python3 scripts/testing/lab2/part1_run.py --class pkg.module:Class --params '{kp: 30.0}'
        [--out DIR] [--no-rviz] [--no-gui] [--timeout S] [--stall S] [key:=value ...]

Part 1 runs ONE simulation vehicle model, the same name for everyone: lab2_sim_vehicle
(packages lab2_sim_vehicle_{description,interface,launch}; docs/design/lab2-team-vehicle-
models.md), as `vehicle_model:=lab2_sim_vehicle vehicle_id:=sim` in
lab2_part1_sim.launch.xml: the model's own SpeedPID (lab2_sim_vehicle_interface/speed_pid.py)
with its own tuning (lab2_sim_vehicle_launch/config/sim/speed_controller.param.yaml). The
car models, teamN_vehicle, are Part 2's. team:= is not needed (accepted and ignored). The
grader reads topics only.

The model resolves from the sourced install's ament index: the student copies the handout's
lab2_sim_vehicle_launch/ into this repository's src/lab2_vehicles/ (gitignored) and builds
it with

    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-up-to lab2_sim_vehicle_launch

Not installed: stop with that fix. The source is found through the installed package.xml,
which --symlink-install makes a symlink into src/lab2_vehicles/; an install without it is
refused with the same fix (it cannot be traced to a source, and edits would not run).

Before anything starts, the model is checked (scripts/testing/lab2/submission.py --part1:
the three packages, class SpeedPID in speed_pid.py, the sim tuning's keys, types and ranges,
no use of the TA reference), and its speed_pid.py must be the one the install imports. With
--symlink-install an edit to speed_pid.py or to config/sim/speed_controller.param.yaml runs
without a rebuild; a new file or package needs the colcon build again.

    submission:=DIR    TA: DIR is a student's model source (lab2_sim_vehicle_launch/, or a
                       directory holding it, such as <student_id>_lab_2_1/ with a report
                       beside it): checked as above, then built on its own (colcon build of
                       that directory alone, no --symlink-install, the sourced AutoSDV
                       install as the underlay) into <out>/ws/, and that install is sourced
                       on top for the runs. Every submission has the same package names, so
                       each gets its own workspace; nothing in src/lab2_vehicles/ is used
                       or changed, and the mark is for the code as handed in. The run directory is
                       labelled with DIR's name (<date>_<time>_<DIR name>) unless --name is
                       given. A build failure exits 2 with the log's tail (<out>/ws/build.log).
    --class/--params   a controller by import path on the reference model (autosdv_vehicle),
                       with its parameters
    --final            accepted, a no-op: the graded set is the default

Each car starts `play_launch launch autosdv_launch lab2_part1_sim.launch.xml` in its own
session (setsid), output to <out>/<variant>/stack.log, waits for the grader's marks.json
in <out>/<variant>/, prints the checks, and stops the whole process group (SIGINT,
SIGTERM, SIGKILL). Nothing it started is left running, whatever happens (Ctrl-C included).

Windows: the live dashboard and RViz for every car. After the LAST car, from a terminal,
the windows stay up with the final curves, violations and marks until the dashboard
window is closed, Enter or Ctrl-C (--no-hold skips it; --hold forces it). The dashboard
saves its window to <out>/<variant>/dashboard.png when it closes. --no-gui (or
rviz:=false gui:=false) opens none; with no working X display the run is headless on its
own, with one line saying so.

Timeouts follow the simulation, not the wall clock, because the clock handshake slows
simulated time under load:

    startup  no grader status within --startup-timeout s of wall time (default 120)
    stall    the grader's simulated time `t` has not advanced for --stall s of wall time
             (default 30): the simulation is stuck, not slow
    cap      startup + 1.2 x (the grader's 300 s engage + 150 s run timeouts) / rtf, rtf from
             /rc_sim/lockstep/status (else the real_time_factor:= argument, else 1),
             never below 0.2; --timeout S replaces it

While simulated time advances the runner waits: the grader ends every run itself (finish,
no progress, MRM, its own timeout) and writes the reason into marks.json.

Outputs: <out> = tmp/lab2/part1/<model>/<YYYY-mm-dd_HH-MM-SS>[_<--name>]/ (--class: the class
path instead of the model), a new directory per run, with tmp/lab2/part1/<model>/latest
pointing at the newest. An explicit --out DIR writes DIR/<variant>/ instead (no date, no
latest: the TA's calibration and tolerance tools read it back). Per car: <variant>/trace.csv and
<variant>/marks.json (the grader's), <variant>/run.json (the command, the play_log, the
timings), <variant>/curves.png (plot_part1.py, drawn when the run ends, no display needed),
<variant>/dashboard.png (with the dashboard). The graded set adds final.json:
marks = 25 * sum(passed) / sum(total), an invalid run counting 0 of its checks.

Exit status: 0 when every run was valid (passed or not), 1 when a run was invalid or never
produced marks, 2 on a usage error (bad arguments, an unknown variant, a model that is not
found, fails its check or is not built, a submission that does not build).

ROS_DOMAIN_ID: an exported non-zero value is used as it is. Unset or 0, the runner uses a
per-user domain, 1 + uid % 100 (never 0), so two users' runs on one host do not cross-talk
and a laptop's `export ROS_DOMAIN_ID=0` does not collide with other stacks; it prints the
value to export for `ros2 topic echo` beside the run. The web UI (play_launch's) is likewise
per user, 127.0.0.1:(8081 + uid % 100), unless LAB2_WEB_ADDR (or --web-addr) moves it, so
two users on one workstation run side by side without colliding.
"""
import argparse
import json
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import sys
import time

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lab2_env as le  # noqa: E402
import lab2_ws  # noqa: E402
import submission as sub  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
PART1_TMP = REPO / 'tmp/lab2/part1'
VEHICLE_YAML = REPO / 'src/sim/autosdv_rc_sim/autosdv_rc_sim/vehicle.yaml'
VARIANTS_FINAL = ('nominal', 'low_batt', 'low_mu')
METRICS = REPO / 'src/lab/autosdv_waypoint_lab/autosdv_waypoint_lab/part1/metrics.py'


def _check_names():
    """metrics.CHECK_NAMES, read from the source (metrics.py imports no ROS)."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('lab2_part1_metrics', METRICS)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.CHECK_NAMES


CHECKS_PER_RUN = len(_check_names())   # the final marks' denominator: never hard-coded
FINAL_POINTS = 25.0
GRADER_ENGAGE_TIMEOUT = 300.0   # [s sim] lab2_part1_grader: launch to engage
GRADER_RUN_TIMEOUT = 150.0      # [s sim] lab2_part1_grader: engage to the end of the run
MIN_RTF = 0.2
STOP_WAITS = ((signal.SIGINT, 20.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0))

_running = []           # the stack's Popen, for the signal handler
_holding = []          # non-empty while the finished run is held open (Ctrl-C ends the hold)


class UsageError(ValueError):
    """Exit 2."""


class HoldEnded(Exception):
    """Ctrl-C (or SIGTERM) while a finished run is held open: stop the stack, carry on."""


def info(msg):
    print(msg, flush=True)


def dim(msg):
    print(f'\033[2m{msg}\033[0m' if sys.stdout.isatty() else msg, flush=True)


def loud(msg):
    print(f'\033[1;33m{msg}\033[0m' if sys.stdout.isatty() else msg, flush=True)


# ------------------------------------------------------------------ arguments
def parse(argv):
    ap = argparse.ArgumentParser(
        description='Run a SpeedPID on the simulated car, live, and grade the run.')
    ap.add_argument('--team', default=None, metavar='N',
                    help='accepted and ignored: Part 1 has one simulation model')
    ap.add_argument('--model', default=None, metavar='NAME',
                    help=f'TA: another model than {sub.SIM_MODEL}, from the sourced install')
    ap.add_argument('--submission', metavar='DIR',
                    help="TA: a team's model source directory, built into the run's ws/ first")
    ap.add_argument('--class', dest='cls', metavar='PKG.MOD:CLASS',
                    help='TA: a controller by import path, on the reference model')
    ap.add_argument('--params', default=None, help='with --class: its parameters, a YAML mapping')
    ap.add_argument('--name', default=None,
                    help='a label for the run directory (<date>_<time>_LABEL); not needed')
    ap.add_argument('--variant', default=None,
                    help='TA: one vehicle model variant only (default: the graded set, '
                         + ', '.join(VARIANTS_FINAL) + ')')
    ap.add_argument('--final', action='store_true',
                    help='accepted for old habits: the graded set is the default')
    ap.add_argument('--out', default=None,
                    help='output directory, <out>/<variant>/ (default tmp/lab2/part1/<submission>/'
                         '<date>_<time>/, with a latest link beside it)')
    ap.add_argument('--rviz', dest='rviz', action='store_true', default=True)
    ap.add_argument('--no-rviz', dest='rviz', action='store_false')
    ap.add_argument('--gui', dest='gui', action='store_true', default=True, help='the dashboard')
    ap.add_argument('--no-gui', dest='nogui', action='store_true',
                    help='no windows at all: neither RViz nor the dashboard (as rviz:=false '
                         'gui:=false)')
    ap.add_argument('--hold', dest='hold', action='store_true', default=None,
                    help='after the last car, keep the dashboard and RViz open until the '
                         'dashboard window is closed or Enter is pressed (default: from a '
                         'terminal, with the dashboard)')
    ap.add_argument('--no-hold', dest='hold', action='store_false')
    ap.add_argument('--timeout', type=float, default=0.0,
                    help='[s wall] hard cap per run; 0 (default): from the achieved rtf')
    ap.add_argument('--stall', type=float, default=30.0,
                    help='[s wall] simulated time frozen this long ends the run (default 30)')
    ap.add_argument('--startup-timeout', type=float, default=120.0,
                    help='[s wall] until the grader reports (default 120)')
    ap.add_argument('--web-addr', default=os.environ.get('LAB2_WEB_ADDR') or default_web_addr(),
                    help='play_launch web UI (default: $LAB2_WEB_ADDR, else 127.0.0.1:'
                         '8081 + uid %% 100, per user)')
    ap.add_argument('launch_args', nargs='*', metavar='key:=value',
                    help='passed to the launch file; team:=, model:= (vehicle_model:=), '
                         'submission:=, variant:=, rviz:=, gui:=, out:= are the options above')
    args = ap.parse_intermixed_args(argv)   # key:=value before and after the options
    rest = []
    for a in args.launch_args:
        key, sep, val = a.partition(':=')
        if not sep:
            ap.error(f'unexpected argument {a!r} (launch arguments are key:=value)')
        if key == 'submission':
            args.submission = val
        elif key == 'team':
            args.team = val
        elif key in ('model', 'vehicle_model'):
            args.model = val
        elif key == 'part1_submission':
            ap.error('part1_submission:= is gone: Part 1 runs the simulation model, '
                     'lab2_sim_vehicle (docs/guides/lab2-part1-sim.md)')
        elif key == 'variant':
            args.variant = val
        elif key in ('rviz', 'gui'):
            if val.lower() not in ('true', 'false'):
                ap.error(f'{key}:= takes true or false, not {val!r}')
            setattr(args, key, val.lower() == 'true')
        elif key == 'out':
            args.out = val
        elif key in ('speed_controller_class', 'speed_controller_params', 'out_dir', 'vehicle_id'):
            ap.error(f'{key}:= is set by the runner: use team:=/model:=, --class/--params, --out '
                     '(Part 1 is always vehicle_id sim)')
        else:
            rest.append(a)
    args.launch_args = rest
    chosen = [n for n, v in (('submission:=', args.submission), ('--class', args.cls)) if v]
    if args.cls and args.model:
        chosen.append('model:=')
    if len(chosen) > 1:
        ap.error(f'{" and ".join(chosen)} both name the controller: give one')
    if args.params is not None and not args.cls:
        ap.error('--params needs --class')
    if args.final and args.variant is not None:
        ap.error('--final runs ' + ', '.join(VARIANTS_FINAL) + '; drop variant:=/--variant')
    if args.nogui:
        args.rviz = args.gui = False
    args.final = args.variant is None          # the graded set, the 25-point mark
    if args.hold is None:
        args.hold = sys.stdin.isatty()
    return args


def variants_of(args):
    """The cars this run drives: the graded set, or the one variant:= a TA named."""
    return VARIANTS_FINAL if args.final else (args.variant,)


def holds_after(args, variant):
    """Hold the windows open after this car? Only after the last one, with the dashboard."""
    return bool(args.hold and args.gui and variant == variants_of(args)[-1])


SIM_MODEL = sub.SIM_MODEL      # Part 1's one simulation model (lab2_ws.SIM_MODEL)
STUDENT_SRC = 'src/lab2_vehicles'   # where a student copies the handout's packages (gitignored)


def build_hint(model=SIM_MODEL):
    """The fix for a model that is missing, not built or built from another copy."""
    return (f'copy {model}_launch from the handout into {STUDENT_SRC}/ and run, in '
            f'{REPO}:\n    colcon build --base-paths src --symlink-install --cmake-args '
            f'-DCMAKE_BUILD_TYPE=Release --packages-up-to {model}_launch\n'
            '  then open a new shell (or `source install/setup.bash`) and run `just lab2 sim` '
            'again')


def _share(pkg):
    """The installed share directory of pkg in this process's ament index, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(pkg))
    except Exception:
        return None


def model_source(model=SIM_MODEL, share=None):
    """The source root of `model` as the sourced install has it (this repository's install:
    src/lab2_vehicles/<model>_launch/ built with --symlink-install), else UsageError naming
    the fix. The root is found through the installed package.xml, a symlink into the source
    under --symlink-install; a copy (a build without it) cannot say which source it came
    from, and its edits would not run, so it is refused with the same fix."""
    share = _share(f'{model}_launch') if share is None else share
    if share is None or not (Path(share) / 'package.xml').exists():
        raise UsageError(f'{model} is not built (no {model}_launch in the sourced install): '
                         + build_hint(model))
    xml = Path(share) / 'package.xml'
    if not xml.is_symlink():
        raise UsageError(f'{model} is installed without --symlink-install ({share}), so it '
                         'cannot be traced to its source and your edits would not run: '
                         + build_hint(model))
    root = xml.resolve().parent.parent
    if not sub._has_model(root, model):
        raise UsageError(f'{model}: the installed {xml} points into {root}, which does not hold '
                         f'{model}_launch/; ' + build_hint(model))
    return root


def check_model(src, model=SIM_MODEL):
    """(model, source root) after submission.py --part1's checks, warnings printed; UsageError
    with the first problem."""
    try:
        model, root, _, warnings = sub.resolve_part1(src, model=model)
    except sub.SubmissionError as e:
        raise UsageError(f'the model does not pass its Part 1 check: {e}') from None
    for w in warnings:
        info(f'warning: {w}')
    return model, root


def installed_from(model):
    """The speed_pid.py the sourced workspaces import as <model>_interface.speed_pid, or None."""
    import importlib.util
    try:
        spec = importlib.util.find_spec(f'{model}_interface')
    except (ImportError, ValueError):
        return None
    if spec is None or not spec.submodule_search_locations:
        return None
    for loc in spec.submodule_search_locations:
        f = Path(loc) / 'speed_pid.py'
        if f.exists():
            return f
    return None


def check_built(model, root, installed=None, imported=None):
    """UsageError unless the sourced install runs THIS source of the model: installed (the
    ament index has its three packages) and its speed_pid.py imported from root (the
    --symlink-install build: edits run without a rebuild)."""
    installed = lab2_ws.model_installed(model) if installed is None else installed
    fix = build_hint(model)
    if not installed:
        raise UsageError(f'{model} is not built (no {model}_launch, _interface, _description in '
                         f'the sourced workspaces): {fix}')
    imported = installed_from(model) if imported is None else imported
    want = (Path(root) / f'{model}_interface' / f'{model}_interface' / 'speed_pid.py').resolve()
    if imported is None or Path(imported).resolve() != want:
        raise UsageError(f'{model} is built from another copy (speed_pid.py imports from '
                         f'{imported}, not {want}), so your edits would not run: {fix}')


def run_dir(base, name, label, now):
    """base/<submission name>/<YYYY-mm-dd_HH-MM-SS>[_label]: one directory per run."""
    stamp = now.strftime('%Y-%m-%d_%H-%M-%S')
    return Path(base) / name / (f'{stamp}_{label}' if label else stamp)


def point_latest(run):
    """<run's parent>/latest -> run (relative, so the tree can move)."""
    run = Path(run)
    link = run.parent / 'latest'
    tmp = run.parent / '.latest.tmp'
    if tmp.is_symlink() or tmp.exists():
        tmp.unlink()
    os.symlink(run.name, tmp)
    os.replace(tmp, link)
    return link


def out_root_for(args, name, base=PART1_TMP, now=None):
    """(the run's output root, dated). An explicit --out keeps <out>/<variant>/, for the
    TA's calibration and tolerance tools that read it back."""
    if args.out:
        return Path(args.out).expanduser(), False
    import datetime
    label = re.sub(r'[^A-Za-z0-9_.-]+', '_', args.name) if args.name else None
    return run_dir(base, name, label, now or datetime.datetime.now()), True


def known_variants():
    """The vehicle model's variants (rc_sim vehicle.yaml), nominal included."""
    try:
        from autosdv_rc_sim.params import variant_names
        names = list(variant_names())
    except Exception:   # not sourced: read the file
        data = yaml.safe_load(VEHICLE_YAML.read_text()) if VEHICLE_YAML.is_file() else {}
        names = list((data or {}).get('variants') or {})
    return ['nominal'] + [n for n in names if n != 'nominal']


def resolve_controller(args):
    """(launch args, name, model source root or None): --class; submission:= (TA: checked
    here, built by build_submission); else lab2_sim_vehicle (model:= names another) from the
    sourced install.
    Sets args.model_name and args.model_root for run.json."""
    args.model_name, args.model_root = None, None
    if args.cls:
        out = [f'speed_controller_class:={args.cls}']
        if args.params is not None:
            out.append(f'speed_controller_params:={args.params}')
        name = re.sub(r'[^A-Za-z0-9_.-]+', '_', args.cls.replace(':', '.'))
        return out, name, None
    model = args.model or SIM_MODEL
    if model == lab2_ws.REFERENCE_MODEL:
        raise UsageError(f"{model} is the reference car model: Part 1 runs it only with a class "
                         "(--class PKG.MOD:CLASS, the TA's)")
    if args.team:
        info(f'team:={args.team} is not needed: Part 1 has one simulation model, {SIM_MODEL} '
             '(ignored)')
    if args.submission:
        d = Path(args.submission).expanduser()
        if not d.is_dir():
            raise UsageError(f'no submission directory {d}')
        model, root = check_model(d, model)
        info(f'submission: {model} from {root} (built for this run)')
        if not args.name:       # every submission is the same model: label the run by the dir
            args.name = d.resolve().name
    else:
        model, root = check_model(model_source(model), model)
        check_built(model, root)
    args.model_name, args.model_root = model, str(root)
    return [f'vehicle_model:={model}'], model, root


def build_submission(root, ws, env):
    """colcon build of the model source root alone into ws (no --symlink-install: the run
    grades the copy as built), over the sourced environment; returns the environment with
    ws/install sourced on top. UsageError with the log's tail when the build fails."""
    ws = Path(ws)
    ws.mkdir(parents=True, exist_ok=True)
    log = ws / 'build.log'
    cmd = ['colcon', 'build', '--base-paths', str(root),
           '--cmake-args', '-DCMAKE_BUILD_TYPE=Release']
    info(f'building {root} into {ws} ...')
    dim(' '.join(cmd))
    with open(log, 'w') as f:
        r = subprocess.run(cmd, cwd=ws, env=env, stdout=f, stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL)
    if r.returncode != 0:
        tail = ''.join(log.read_text(errors='replace').splitlines(True)[-15:])
        raise UsageError(f'the submission does not build ({log}):\n{tail}')
    setup = ws / 'install' / 'local_setup.bash'
    out = subprocess.run(['bash', '-c', f'set +u; . "{setup}" >/dev/null && env -0'],
                         env=env, capture_output=True, check=True).stdout
    return dict(kv.split('=', 1) for kv in out.decode().split('\0') if '=' in kv)


def check_display(args, env=None):
    """No working X display: headless (no RViz, no dashboard), with one line saying so."""
    env = os.environ if env is None else env
    if (args.rviz or args.gui) and not le.display_works(env.get('DISPLAY')):
        cur = env.get('DISPLAY')
        info(f'no working X display ({"DISPLAY=" + repr(cur) + " does not answer" if cur else "no DISPLAY"}): '
             'running headless, no RViz and no dashboard (curves.png is still drawn)')
        args.rviz = args.gui = False


def domain_id(env):
    """(ROS_DOMAIN_ID to use, why)."""
    v = env.get('ROS_DOMAIN_ID', '').strip()
    if v and v != '0':
        return v, 'exported'
    return str(1 + os.getuid() % 100), 'per-user default, 1 + uid % 100'


def default_web_addr(uid=None):
    """The per-user web UI address: 127.0.0.1:(8081 + uid % 100), as the domain id."""
    uid = os.getuid() if uid is None else uid
    return f'127.0.0.1:{8081 + uid % 100}'


def launch_rtf(launch_args):
    for a in launch_args:
        key, _, val = a.partition(':=')
        if key == 'real_time_factor':
            try:
                return float(val)
            except ValueError:
                pass
    return 1.0


# ------------------------------------------------------------------ process group
def stop_group(proc):
    if proc is None:
        return True
    for sig, wait in STOP_WAITS:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            proc.poll()
            if not group_alive(proc.pid):
                break
            time.sleep(0.2)
        else:
            continue
        break
    proc.poll()
    if group_alive(proc.pid):
        info(f'warning: process group {proc.pid} still has members')
        return False
    return True


def group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def on_signal(signum, frame):
    if _holding:
        raise HoldEnded()
    for proc in list(_running):
        info('\nstopping the stack ...')
        stop_group(proc)
    sys.exit(130)


# ------------------------------------------------------------------ live progress
class Watch:
    """The scenario's events, the grader's state and simulated time, the plant's rtf.

    Without rclpy (an unsourced shell) it only sleeps, and the runner falls back to the
    wall-clock cap: no stall detection."""

    def __init__(self):
        self.node = None
        self.events = []
        self.status = None
        self.last_state = None
        self.sim_t = None            # the grader's latest `t`
        self.sim_t_wall = None       # wall time `t` last advanced
        self.rtf = None
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
        except ImportError:
            return
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=[])   # not sys.argv: its key:=value would read as remaps
        self.node = rclpy.create_node('lab2_part1_runner')

        def latched(depth):
            return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription
        self.subs = [sub(String, '/lab2/part1/event', self.on_event, latched(10)),
                     sub(String, '/lab2/part1/status', self.on_status, latched(1)),
                     sub(String, '/rc_sim/lockstep/status', self.on_lockstep, latched(1))]

    def on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        if ev in self.events:
            return
        self.events.append(ev)
        where = f" at s = {ev['s']:.1f} m" if 's' in ev else ''
        why = f": {ev['reason']}" if ev.get('reason') else ''
        info(f"  t {ev.get('t', 0.0):6.1f} s  {ev.get('event')}{where}{why}")

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            return
        t = self.status.get('t')
        if isinstance(t, (int, float)) and (self.sim_t is None or t > self.sim_t):
            self.sim_t, self.sim_t_wall = float(t), time.monotonic()
        state = self.status.get('state')
        if state != self.last_state:
            self.last_state = state
            reason = self.status.get('invalid_reason')
            dim(f'  grader: {state}' + (f' ({reason})' if reason else ''))

    def on_lockstep(self, msg):
        try:
            rtf = json.loads(msg.data).get('rtf')
        except (ValueError, AttributeError):
            return
        if isinstance(rtf, (int, float)) and rtf > 0:
            self.rtf = float(rtf)

    def spin(self, seconds):
        if self.node is None:
            time.sleep(seconds)
            return
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.1, end - time.monotonic())))

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None


def why_stop(args, watch, t0, now, rtf_arg):
    """The reason to give up on this run now, or None to keep waiting."""
    if watch.sim_t_wall is None:
        if now - t0 > args.startup_timeout:
            return f'the grader did not report within {args.startup_timeout:.0f} s of the launch'
        return None
    if watch.node is not None and now - watch.sim_t_wall > args.stall:
        return (f'simulated time stuck at {watch.sim_t:.1f} s for {args.stall:.0f} s of wall time '
                '(the simulation stalled)')
    rtf = max(MIN_RTF, watch.rtf if watch.rtf else rtf_arg)
    cap = args.timeout or (args.startup_timeout
                           + 1.2 * (GRADER_ENGAGE_TIMEOUT + GRADER_RUN_TIMEOUT) / rtf)
    if now - t0 > cap:
        return f'no marks.json within {cap:.0f} s (rtf {rtf:.2f})'
    return None


def session_has(sid, needle):
    """True while a process of session sid has needle in its command line (Linux /proc).

    A session, not a process group: the stack starts in its own session (setsid), and
    play_launch moves the nodes into a process group of their own inside it."""
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            if os.getsid(int(d.name)) != sid:
                continue
            if needle in (d / 'cmdline').read_bytes():
                return True
        except (OSError, ProcessLookupError):
            continue
    return False


def hold_open(proc, out_dir):
    """Keep the finished run's stack (the dashboard and RViz) up for the student to read.

    Returns when the dashboard window is closed, Enter is pressed, Ctrl-C, or the stack
    exits on its own. The grader stopped recording at the end of the run, so the curves
    and the marks on screen are final while the car sits still behind them."""
    if not session_has(proc.pid, b'lab2_part1_dashboard'):
        return
    info('\n  The run is over: the dashboard and RViz stay open so you can read them.\n'
         '  Close the dashboard window or press Enter to stop the simulation '
         f'(the window is saved to {out_dir / "dashboard.png"}).')
    _holding.append(True)
    watch_stdin = sys.stdin is not None and sys.stdin.isatty()
    try:
        while proc.poll() is None and session_has(proc.pid, b'lab2_part1_dashboard'):
            if not watch_stdin:
                time.sleep(0.5)
                continue
            ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            if ready:
                sys.stdin.readline()
                break
    except HoldEnded:
        pass
    finally:
        _holding.clear()


# ------------------------------------------------------------------ one run
def run_one(args, variant, ctrl_args, out_root, env, before_hold=None):
    out_dir = out_root / variant
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in ('marks.json', 'trace.csv', 'run.json'):
        (out_dir / f).unlink(missing_ok=True)
    play_log = out_dir / 'play_log'
    cmd = ['play_launch', 'launch', '--web-addr', args.web_addr, '--log-dir', str(play_log),
           'autosdv_launch', 'lab2_part1_sim.launch.xml',
           f'variant:={variant}', f'out_dir:={out_dir.resolve()}',
           f'rviz:={str(args.rviz).lower()}', f'gui:={str(args.gui).lower()}']
    cmd += ctrl_args + args.launch_args
    run_env = dict(env)
    info(f'\n== {variant}: {out_dir}')
    dim(' '.join(cmd))
    log = open(out_dir / 'stack.log', 'w')
    t0 = time.monotonic()
    proc = subprocess.Popen(cmd, cwd=REPO, env=run_env, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    _running.append(proc)
    watch = Watch()
    rtf_arg = launch_rtf(args.launch_args)
    marks, why, t_engaged, t_marks = None, None, None, None
    printed = False
    try:
        while True:
            watch.spin(0.5)
            now = time.monotonic()
            if t_engaged is None and any(e.get('event') == 'engaged' for e in watch.events):
                t_engaged = now - t0
            mpath = out_dir / 'marks.json'
            if mpath.exists():
                try:
                    marks = json.loads(mpath.read_text())
                    t_marks = now - t0
                    break
                except ValueError:
                    pass    # being written
            if proc.poll() is not None:
                why = f'the stack exited ({proc.returncode}) before the grader wrote marks.json'
                break
            why = why_stop(args, watch, t0, now, rtf_arg)
            if why:
                break
        if marks is not None and holds_after(args, variant):
            print_checks(marks)
            printed = True
            if before_hold is not None:
                before_hold(marks)       # the final marks and the plots, before the wait
            hold_open(proc, out_dir)
    finally:
        watch.spin(0.5)
        watch.close()
        clean = stop_group(proc)
        _running.remove(proc)
        log.close()
    if marks is None:
        aborted = next((e for e in watch.events if e.get('event') == 'aborted'), None)
        if aborted:
            why = f"{why}; the scenario aborted: {aborted.get('reason')}"
        marks = {'variant': variant, 'valid': False, 'invalid_reason': why, 'passed': 0,
                 'total': CHECKS_PER_RUN, 'checks': {},
                 'source': 'part1_run.py (no grader result)'}
        (out_dir / 'marks.json').write_text(json.dumps(marks, indent=2) + '\n')
    marks = enforce_own_controller(marks, args, play_log, out_dir)
    run = {'variant': variant, 'command': cmd, 'play_log': str(play_log.resolve()),
           'vehicle_model': getattr(args, 'model_name', None) or lab2_ws.REFERENCE_MODEL,
           'model_source': getattr(args, 'model_root', None),
           'stack_log': str((out_dir / 'stack.log').resolve()),
           'startup_s': None if t_engaged is None else round(t_engaged, 1),
           'wall_s': round(time.monotonic() - t0, 1),
           'marks_after_s': None if t_marks is None else round(t_marks, 1),
           'sim_s': watch.sim_t, 'rtf': watch.rtf,
           'events': watch.events, 'stopped_cleanly': clean}
    (out_dir / 'run.json').write_text(json.dumps(run, indent=2) + '\n')
    print_marks(marks, run, table=not printed)
    return marks



_CONTROLLER_LINE = re.compile(r'Speed controller: (\S+)')


def loaded_controller(play_log: Path):
    """The class the actuator loaded in this run ("Speed controller: X" in its log), or None."""
    for f in sorted(Path(play_log).glob('*/node/actuator_node/*'), reverse=True):
        if f.is_file():
            m = _CONTROLLER_LINE.search(f.read_text(errors='replace'))
            if m:
                return m.group(1)
    return None


def enforce_own_controller(marks: dict, args, play_log: Path, out_dir: Path) -> dict:
    """The grader rule: a model run counts only if the actuator ran the model's own SpeedPID
    (speed_pid.py), not AutoSDV's built-in controller. --class runs (the TA) are exempt."""
    model = getattr(args, 'model_name', None)
    if args.cls or not model:
        return marks
    want, used = sub.expected_controller(model), loaded_controller(play_log)
    marks['speed_controller'] = used
    if used != want:
        marks.update(valid=False, passed=0, invalid_reason=(
            f'the actuator ran {used or "an unknown controller"}, not {want}: only your own '
            'SpeedPID in speed_pid.py is graded'))
    (out_dir / 'marks.json').write_text(json.dumps(marks, indent=2) + '\n')
    return marks

def fmt(v):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.3f}'
    return str(v)


def print_checks(marks):
    checks = marks.get('checks') or {}
    if checks:
        w = max(len(k) for k in checks)
        info(f"  {'check':<{w}}  {'value':>9}  {'limit':>9}")
        for name, c in checks.items():
            ok = c.get('pass')
            tag = 'PASS' if ok else ('FAIL' if ok is False else ' -- ')
            info(f"  {name:<{w}}  {fmt(c.get('value')):>9}  {fmt(c.get('limit')):>9}  {tag}")


def print_marks(marks, run, table=True):
    if table:
        print_checks(marks)
    if marks.get('message'):
        loud(f"  {marks['message']}")
    if marks.get('valid', False):
        info(f"  {marks['variant']}: {marks.get('passed', 0)}/"
             f"{marks.get('total', CHECKS_PER_RUN)} passed")
    else:
        info(f"  {marks.get('variant')}: INVALID run, scores 0: {marks.get('invalid_reason')}")
    dim(f"  outputs {Path(run['play_log']).parent}  (startup {run['startup_s']} s, "
        f"wall {run['wall_s']} s)")


def keep_hint(out_root, dated):
    """The closing line: where the outputs are."""
    if not dated:
        return f'\noutputs in {out_root}'
    return (f'\noutputs in {out_root}\n  (also {out_root.parent / "latest"}; every run keeps its '
            'own directory)')


def write_plots(out_root, variants):
    """{variant: curves.png or None}: plot_part1 (Agg) into each variant's directory."""
    try:
        import plot_part1
    except ImportError:
        return {v: None for v in variants}
    out = {}
    for v in variants:
        d = Path(out_root) / v
        try:
            out[v] = plot_part1.plot(d, d / 'curves.png') if (d / 'trace.csv').is_file() else None
        except (Exception, SystemExit) as e:     # a plot never fails the run
            info(f'  (no plot for {v}: {e})')
            out[v] = None
    return out


def summary_table(results, plots):
    """One line per car: checks passed (or INVALID and why) and its curves.png."""
    w = max(len(str(m.get('variant'))) for m in results)
    lines = []
    for m in results:
        v = m.get('variant')
        if m.get('valid'):
            res = f"{m.get('passed', 0)}/{m.get('total', CHECKS_PER_RUN)} passed"
        else:
            res = f"INVALID (scores 0): {m.get('invalid_reason')}"
        png = plots.get(v)
        lines.append(f'  {v:<{w}}  {res:<16}  {png if png else "(no plot)"}')
    return '\n'.join(lines)


def final_marks(name, results):
    passed = sum(int(m.get('passed', 0)) if m.get('valid') else 0 for m in results)
    total = sum(int(m.get('total', CHECKS_PER_RUN)) if m.get('valid') else CHECKS_PER_RUN
                for m in results)
    total = total or CHECKS_PER_RUN * len(results)
    return {'name': name, 'marks': round(FINAL_POINTS * passed / total, 2),
            'out_of': FINAL_POINTS, 'passed': passed, 'total': total,
            'runs': {m.get('variant'): {'valid': bool(m.get('valid')),
                                        'invalid_reason': m.get('invalid_reason'),
                                        'passed': m.get('passed', 0),
                                        'total': m.get('total', CHECKS_PER_RUN)}
                     for m in results}}


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    try:
        variants = variants_of(args)
        known = known_variants()
        bad = [v for v in variants if v not in known]
        if bad:
            raise UsageError(f'unknown variant {bad[0]!r}; the model has {", ".join(known)}')
        ctrl_args, name, root = resolve_controller(args)
    except UsageError as e:
        info(f'error: {e}')
        return 2
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'], why_domain = domain_id(env)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']     # the progress watcher's too
    check_display(args, env)
    out_root, dated = out_root_for(args, name)
    out_root.mkdir(parents=True, exist_ok=True)
    if dated:
        point_latest(out_root)
    if args.submission:
        try:
            env = build_submission(root, out_root / 'ws', env)
        except UsageError as e:
            info(f'error: {e}')
            return 2
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    dim(f'ROS_DOMAIN_ID {env["ROS_DOMAIN_ID"]} ({why_domain}; export it to watch with ros2), '
        f'web UI {args.web_addr}, controller: {" ".join(ctrl_args)}')
    info(f'run directory: {out_root}' + (f'  ({out_root.parent / "latest"})' if dated else ''))

    results = []
    shown = []

    def report(last_marks=None):
        """The marks table, the plots and (for the graded set) the mark out of 25."""
        done = results + ([last_marks] if last_marks is not None else [])
        plots = write_plots(out_root, [m.get('variant') for m in done])
        info('\n' + summary_table(done, plots))
        if args.final:
            final = final_marks(name, done)
            (out_root / 'final.json').write_text(json.dumps(final, indent=2) + '\n')
            info(f"\nFINAL  {final['marks']:.2f} / {FINAL_POINTS:g}  "
                 f"({final['passed']}/{final['total']} checks)  {out_root / 'final.json'}")
        shown.append(True)

    for v in variants:
        results.append(run_one(args, v, ctrl_args, out_root, env,
                               before_hold=report if v == variants[-1] else None))
    if not shown:
        report()
    valid = all(m.get('valid', False) for m in results)
    info(keep_hint(out_root, dated))
    return 0 if valid else 1


if __name__ == '__main__':
    sys.exit(main())
