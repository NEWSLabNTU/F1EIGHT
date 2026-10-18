#!/usr/bin/env python3
"""Lab 2 Part 1 runner: one live run (or the graded set) of a SpeedPID on the simulated car.

    just lab2 sim submission:=~/lab2/B12345678_lab_2_1            one run, nominal car
    just lab2 sim submission:=DIR --final                          nominal, low_batt, low_mu
    just lab2 sim submission:=DIR variant:=low_mu rviz:=false gui:=false

    python3 scripts/testing/lab2/part1_run.py --submission DIR [--variant V | --final]
    python3 scripts/testing/lab2/part1_run.py --class pkg.module:Class --params '{kp: 30.0}'
        [--out DIR] [--no-rviz] [--no-gui] [--timeout S] [--stall S] [key:=value ...]

Each run starts `play_launch launch autosdv_launch lab2_part1_sim.launch.xml` in its own
session (setsid), output to <out>/<variant>/stack.log, waits for the grader's marks.json
in <out>/<variant>/, prints the marks, and stops the whole process group (SIGINT, SIGTERM,
SIGKILL). Nothing it started is left running, whatever happens (Ctrl-C included).

A single run from a terminal holds the finished run open first: the dashboard and RViz
stay up with the final curves, violations and marks until the dashboard window is closed,
Enter or Ctrl-C (--no-hold skips it; --final and a non-terminal stdin never hold; --hold
forces it). The dashboard saves its window to <out>/<variant>/dashboard.png when it closes.

    --submission DIR   a Part 1 submission (<id>_lab_2_1/ holding lab2_speed/, or the
                       package): scripts/testing/lab2/submission.py --part1 gives the class,
                       the parameters and the PYTHONPATH entry. `just lab2 sim` also takes
                       `part1_submission` from ~/lab2/lab2.yaml
                       (scripts/testing/lab2/lab2_config.py)
    --class/--params   a controller by import path, with its parameters
    neither            THE STUB (autosdv_waypoint_lab.stub.speed_pid_part1, which holds
                       neutral), with a loud warning: the car does not move

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

Outputs (default <out> = tmp/lab2/part1/<submission name>): <variant>/trace.csv and
<variant>/marks.json (the grader's), <variant>/run.json (the command, the play_log, the
timings), <variant>/dashboard.png (with the dashboard), and with --final, final.json: marks = 25 * sum(passed) / sum(total), an invalid
run counting 0 of its checks.

Exit status: 0 when every run was valid (passed or not), 1 when a run was invalid or never
produced marks, 2 on a usage error (bad arguments, an unknown variant, a submission that
does not resolve).

ROS_DOMAIN_ID: an exported non-zero value is used as it is. Unset or 0, the runner uses a
per-user domain, 1 + uid % 100 (never 0), so two users' runs on one host do not cross-talk
and a laptop's `export ROS_DOMAIN_ID=0` does not collide with other stacks; it prints the
value to export for `ros2 topic echo` beside the run. LAB2_WEB_ADDR (or --web-addr) moves
the web UI.
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

REPO = Path(__file__).resolve().parents[3]
SUBMISSION = REPO / 'scripts/testing/lab2/submission.py'
VEHICLE_YAML = REPO / 'src/sim/autosdv_rc_sim/autosdv_rc_sim/vehicle.yaml'
STUB_CLASS = 'autosdv_waypoint_lab.stub.speed_pid_part1:SpeedPID'
STUB_PARAMS = REPO / 'src/lab/autosdv_waypoint_lab/config/speed_pid_part1.yaml'
VARIANTS_FINAL = ('nominal', 'low_batt', 'low_mu')
CHECKS_PER_RUN = 16          # autosdv_waypoint_lab.part1.metrics.CHECK_NAMES
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
    who = ap.add_mutually_exclusive_group()
    who.add_argument('--submission', metavar='DIR', help='a Part 1 submission directory')
    who.add_argument('--class', dest='cls', metavar='PKG.MOD:CLASS',
                     help='a controller by import path')
    ap.add_argument('--params', default=None, help='with --class: its parameters, a YAML mapping')
    ap.add_argument('--name', default=None, help='output name (default: the submission directory)')
    ap.add_argument('--variant', default=None,
                    help='vehicle model variant, one run (default nominal)')
    ap.add_argument('--final', action='store_true',
                    help='the graded set: ' + ', '.join(VARIANTS_FINAL))
    ap.add_argument('--out', default=None, help='output directory (default tmp/lab2/part1/<name>)')
    ap.add_argument('--rviz', dest='rviz', action='store_true', default=True)
    ap.add_argument('--no-rviz', dest='rviz', action='store_false')
    ap.add_argument('--gui', dest='gui', action='store_true', default=True, help='the dashboard')
    ap.add_argument('--no-gui', dest='gui', action='store_false')
    ap.add_argument('--hold', dest='hold', action='store_true', default=None,
                    help='after a run, keep the dashboard and RViz open until the dashboard '
                         'window is closed or Enter is pressed (default: a single run from a '
                         'terminal, with the dashboard)')
    ap.add_argument('--no-hold', dest='hold', action='store_false')
    ap.add_argument('--timeout', type=float, default=0.0,
                    help='[s wall] hard cap per run; 0 (default): from the achieved rtf')
    ap.add_argument('--stall', type=float, default=30.0,
                    help='[s wall] simulated time frozen this long ends the run (default 30)')
    ap.add_argument('--startup-timeout', type=float, default=120.0,
                    help='[s wall] until the grader reports (default 120)')
    ap.add_argument('--web-addr', default=os.environ.get('LAB2_WEB_ADDR', '127.0.0.1:8081'))
    ap.add_argument('launch_args', nargs='*', metavar='key:=value',
                    help='passed to the launch file; submission:=, variant:=, rviz:=, gui:=, '
                         'out:= are the options above')
    args = ap.parse_intermixed_args(argv)   # key:=value before and after the options
    rest = []
    for a in args.launch_args:
        key, sep, val = a.partition(':=')
        if not sep:
            ap.error(f'unexpected argument {a!r} (launch arguments are key:=value)')
        if key == 'submission':
            if args.cls:
                ap.error('submission:= and --class are exclusive')
            args.submission = val
        elif key == 'variant':
            args.variant = val
        elif key in ('rviz', 'gui'):
            if val.lower() not in ('true', 'false'):
                ap.error(f'{key}:= takes true or false, not {val!r}')
            setattr(args, key, val.lower() == 'true')
        elif key == 'out':
            args.out = val
        elif key in ('speed_controller_class', 'speed_controller_params', 'out_dir'):
            ap.error(f'{key}:= is set by the runner: use --class/--params, --out')
        else:
            rest.append(a)
    args.launch_args = rest
    if args.params is not None and not args.cls:
        ap.error('--params needs --class')
    if args.final and args.variant is not None:
        ap.error('--final runs ' + ', '.join(VARIANTS_FINAL) + '; drop variant:=/--variant')
    if args.variant is None:
        args.variant = 'nominal'
    if args.hold is None:
        args.hold = not args.final and sys.stdin.isatty()
    return args


def known_variants():
    """The vehicle model's variants (rc_sim vehicle.yaml), nominal included."""
    try:
        from autosdv_rc_sim.params import variant_names
        names = list(variant_names())
    except Exception:   # not sourced: read the file
        data = yaml.safe_load(VEHICLE_YAML.read_text()) if VEHICLE_YAML.is_file() else {}
        names = list((data or {}).get('variants') or {})
    return ['nominal'] + [n for n in names if n != 'nominal']


def yaml_mapping_arg(path):
    """A YAML file's top-level mapping as one-line `{k: v, ...}`, as submission.py writes it."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return '{' + ', '.join(f'{k}: {str(v).lower() if isinstance(v, bool) else v}'
                           for k, v in data.items()) + '}'


def resolve_controller(args):
    """(launch args, PYTHONPATH entry or None, name, is_stub) for the chosen controller."""
    if args.submission:
        sub = Path(args.submission).expanduser()
        if not sub.exists():
            raise UsageError(f'no submission directory {sub}')
        base = [sys.executable, str(SUBMISSION), str(sub), '--part1']
        try:
            pkg = subprocess.run(base + ['--pythonpath'], check=True, text=True,
                                 capture_output=True).stdout.strip()
            lines = subprocess.run(base + ['--launch-args'], check=True, text=True,
                                   capture_output=True).stdout.splitlines()
        except subprocess.CalledProcessError as e:
            msg = (e.stderr or e.stdout or '').strip()
            raise UsageError(f'submission {sub}: {msg or "submission.py failed"}') from None
        name = args.name or sub.resolve().name
        return [x for x in lines if x.strip()], pkg or None, name, False
    if args.cls:
        out = [f'speed_controller_class:={args.cls}']
        if args.params is not None:
            out.append(f'speed_controller_params:={args.params}')
        name = args.name or re.sub(r'[^A-Za-z0-9_.-]+', '_', args.cls.replace(':', '.'))
        return out, None, name, False
    out = [f'speed_controller_class:={STUB_CLASS}']
    if STUB_PARAMS.is_file():
        out.append(f'speed_controller_params:={yaml_mapping_arg(STUB_PARAMS)}')
    return out, None, args.name or 'stub', True


def check_display(args):
    if (args.rviz or args.gui) and not (os.environ.get('DISPLAY')
                                        or os.environ.get('WAYLAND_DISPLAY')):
        info('warning: no DISPLAY: running without RViz and the dashboard '
             '(rviz:=false gui:=false)')
        args.rviz = args.gui = False


def domain_id(env):
    """(ROS_DOMAIN_ID to use, why)."""
    v = env.get('ROS_DOMAIN_ID', '').strip()
    if v and v != '0':
        return v, 'exported'
    return str(1 + os.getuid() % 100), 'per-user default, 1 + uid % 100'


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
def run_one(args, variant, ctrl_args, pythonpath, out_root, env):
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
    if pythonpath:
        run_env['PYTHONPATH'] = pythonpath + (os.pathsep + env['PYTHONPATH']
                                              if env.get('PYTHONPATH') else '')
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
        if marks is not None and args.hold and args.gui:
            print_checks(marks)
            printed = True
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
    run = {'variant': variant, 'command': cmd, 'play_log': str(play_log.resolve()),
           'stack_log': str((out_dir / 'stack.log').resolve()),
           'startup_s': None if t_engaged is None else round(t_engaged, 1),
           'wall_s': round(time.monotonic() - t0, 1),
           'marks_after_s': None if t_marks is None else round(t_marks, 1),
           'sim_s': watch.sim_t, 'rtf': watch.rtf,
           'events': watch.events, 'stopped_cleanly': clean}
    (out_dir / 'run.json').write_text(json.dumps(run, indent=2) + '\n')
    print_marks(marks, run, table=not printed)
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
    if marks.get('valid', False):
        info(f"  {marks['variant']}: {marks.get('passed', 0)}/"
             f"{marks.get('total', CHECKS_PER_RUN)} passed")
    else:
        info(f"  {marks.get('variant')}: INVALID run, scores 0: {marks.get('invalid_reason')}")
    dim(f"  outputs {Path(run['play_log']).parent}  (startup {run['startup_s']} s, "
        f"wall {run['wall_s']} s)")


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    try:
        variants = VARIANTS_FINAL if args.final else (args.variant,)
        known = known_variants()
        bad = [v for v in variants if v not in known]
        if bad:
            raise UsageError(f'unknown variant {bad[0]!r}; the model has {", ".join(known)}')
        ctrl_args, pythonpath, name, stub = resolve_controller(args)
    except UsageError as e:
        info(f'error: {e}')
        return 2
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'], why_domain = domain_id(env)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']     # the progress watcher's too
    check_display(args)
    out_root = Path(args.out).expanduser() if args.out else REPO / 'tmp/lab2/part1' / name
    out_root.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if stub:
        loud('WARNING: no submission given: running the STUB, which holds neutral (the car will '
             'not move). Pass submission:=DIR, or set `part1_submission:` in ~/lab2/lab2.yaml.')
    dim(f'ROS_DOMAIN_ID {env["ROS_DOMAIN_ID"]} ({why_domain}; export it to watch with ros2), '
        f'controller: {" ".join(ctrl_args)}')
    results = [run_one(args, v, ctrl_args, pythonpath, out_root, env) for v in variants]

    valid = all(m.get('valid', False) for m in results)
    if args.final:
        passed = sum(int(m.get('passed', 0)) if m.get('valid') else 0 for m in results)
        total = sum(int(m.get('total', CHECKS_PER_RUN)) if m.get('valid') else CHECKS_PER_RUN
                    for m in results)
        total = total or CHECKS_PER_RUN * len(results)
        final = {'name': name, 'marks': round(FINAL_POINTS * passed / total, 2),
                 'out_of': FINAL_POINTS, 'passed': passed, 'total': total,
                 'runs': {m.get('variant'): {'valid': bool(m.get('valid')),
                                             'invalid_reason': m.get('invalid_reason'),
                                             'passed': m.get('passed', 0),
                                             'total': m.get('total', CHECKS_PER_RUN)}
                          for m in results}}
        (out_root / 'final.json').write_text(json.dumps(final, indent=2) + '\n')
        info(f"\nFINAL  {final['marks']:.2f} / {FINAL_POINTS:g}  ({passed}/{total} checks)  "
             f"{out_root / 'final.json'}")
    if stub:
        loud('(that was the stub: pass submission:=DIR to run your SpeedPID)')
    return 0 if valid else 1


if __name__ == '__main__':
    sys.exit(main())
