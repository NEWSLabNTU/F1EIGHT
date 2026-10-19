#!/usr/bin/env python3
"""`just coach record-part1`: the Part 1 speed-step bag from one terminal.

    just coach record-part1 vehicle_id:=car_b                 rosbags/lab2cal_<time>, the default steps
    just coach record-part1 vehicle_id:=car_b --name steps1   rosbags/lab2cal_steps1
    just coach record-part1 vehicle_id:=car_b --steps 0,0.4,0.7,0 --hold 2.5 --max-distance 3
    just coach record-part1 --bench --yes           the same flow on the vehicle bench (no car)

vehicle_id:= is required on the car (the stack refuses to start without one,
and the fit measures deadband and gain from that car's init_pwm); the bench
defaults to sim, the simulated nominal car.

In order, with short status lines here and the rest in BAG/run_logs/:

  1. start the stack without the pursuit planner (`just coach vehicle
     launch_planner:=false`, or `just coach bench ...` with --bench); any
     other KEY:=VALUE words are passed to it
  2. once /autosdv/actuator_node is up, start the speed step planner and the
     recording (`just coach record-cal`, which first saves the actuator's
     parameters as BAG/actuator_params.yaml; no engage without that file)
  3. wait until autonomous mode is available, then ask before engaging (the
     spotter must hold the transmitter)
  4. engage; the planner drives the schedule and stops; the run ends when the
     car has been at rest for a second after the schedule (or the distance
     limit) ended. An MRM, a disengage, the timeout, a dead stack or Ctrl-C
     end it early and disengage at once
  5. record 2 s more, disengage, stop the recording, the planner and the stack
  6. fit the motor and export the students' CSV into the bag directory:
     BAG/motor_plant.yaml (`just coach fit-motor`), BAG/speed_step.csv
     (`just coach export-speed-csv`)

Every process this starts runs in a session of its own and is stopped by
process group (SIGINT, then SIGTERM, then SIGKILL). Exit status: 0 when the
schedule finished and both files were written, 1 otherwise, 2 for usage.
"""

import argparse
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from lab2_run import AT_REST, Child  # noqa: E402

REPO = HERE.parents[2]
DEFAULT_STEPS, DEFAULT_HOLD, DEFAULT_DISTANCE = '0,0.3,0.6,0', 4.0, 2.5   # just/coach.just speed-steps
READY_TIMEOUT = 180.0     # s for the stack to make autonomous mode available
READY_STEADY = 3.0        # s of continuous availability before asking
REST_HOLD = 1.0           # s at rest after the schedule ended
SETTLE_AFTER_END = 2.0    # s recorded after the end
RUN_MARGIN = 20.0         # s beyond the schedule's own length before giving up
ACTUATOR = ('actuator_node', '/autosdv')
PARAMS_FILE = 'actuator_params.yaml'   # motor_fit.ACTUATOR_PARAMS_FILE, written by record-cal
PARAMS_TIMEOUT = 40.0     # s for record-cal to save it (its own wait is 30 s)


def launch_vehicle_id(launch_args):
    """The value of the last vehicle_id:= among the launch arguments, or None."""
    found = None
    for w in launch_args:
        key, _, value = w.partition(':=')
        if key == 'vehicle_id':
            found = value
    return found


def parse_args(argv):
    ap = argparse.ArgumentParser(prog='just coach record-part1', description=__doc__.split('\n')[0])
    ap.add_argument('--name', default='', help='bag name: rosbags/lab2cal_NAME (default: the time)')
    ap.add_argument('--steps', default=DEFAULT_STEPS, help=f'speeds [m/s] (default {DEFAULT_STEPS})')
    ap.add_argument('--hold', type=float, default=DEFAULT_HOLD, help=f'[s] per step (default {DEFAULT_HOLD})')
    ap.add_argument('--max-distance', type=float, default=DEFAULT_DISTANCE,
                    help=f'[m] brake once this far (default {DEFAULT_DISTANCE}; clear about 0.5 m more)')
    ap.add_argument('--bench', action='store_true', help='the vehicle bench instead of the car')
    ap.add_argument('--yes', action='store_true', help='engage without asking (--bench only)')
    ap.add_argument('--no-fit', action='store_true', help='skip the motor fit and the CSV export')
    opts, rest = ap.parse_known_args(argv)
    bad = [w for w in rest if ':=' not in w]
    if bad:
        ap.error(f'unknown arguments: {" ".join(bad)} (launch arguments are KEY:=VALUE)')
    if opts.yes and not opts.bench:
        ap.error('--yes is for --bench only: on the car, a person engages')
    opts.launch_args = rest
    car = launch_vehicle_id(rest)
    if car is None and not opts.bench:
        ap.error('vehicle_id:= is required on the car, e.g. `just coach record-part1 vehicle_id:=car_b`: '
                 'the fit measures the deadband and gain from that car\'s init_pwm')
    if car is not None:
        from autosdv_vehicle_interface import car_profile
        if car not in car_profile.names():
            ap.error(f'vehicle_id:={car}: one of {", ".join(car_profile.names())}')
    opts.vehicle_id = car or 'sim'      # coach_pursuit_bench.launch.xml's default
    try:
        opts.speeds = [float(x) for x in opts.steps.split(',') if x.strip()]
    except ValueError:
        ap.error(f'--steps: not a comma-separated list of speeds: {opts.steps}')
    return opts


def make_monitor():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from autoware_adapi_v1_msgs.srv import ChangeOperationMode
    from autoware_vehicle_msgs.msg import VelocityReport
    from autosdv_lab_msgs.msg import PursuitDebug
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tier4_system_msgs.msg import OperationModeAvailability

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    class Monitor(Node):
        def __init__(self):
            super().__init__('lab2_record_part1')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.AUTONOMOUS, self.MRM_NORMAL = OperationModeState.AUTONOMOUS, MrmState.NORMAL
            self.mode = self.available = self.limit = None
            self.mrm = False
            self.speed, self.rest_since, self.v_cmd = 0.0, None, 0.0
            self.create_subscription(OperationModeState, '/api/operation_mode/state', self.on_mode, latched)
            self.create_subscription(MrmState, '/system/fail_safe/mrm_state', self.on_mrm, 10)
            self.create_subscription(OperationModeAvailability, '/system/operation_mode/availability',
                                     self.on_avail, 10)
            self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.on_vel, 10)
            self.create_subscription(PursuitDebug, '/planning/coach_pursuit/debug', self.on_dbg, 10)
            self.cli_auto = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
            self.cli_stop = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')
            self.Req = ChangeOperationMode.Request

        def on_mode(self, m):
            self.mode = int(m.mode)

        def on_mrm(self, m):
            self.mrm = m.state != self.MRM_NORMAL

        def on_avail(self, m):
            self.available = bool(m.autonomous)

        def on_vel(self, m):
            self.speed = float(m.longitudinal_velocity)
            if abs(self.speed) < AT_REST:
                self.rest_since = self.rest_since or time.monotonic()
            else:
                self.rest_since = None

        def on_dbg(self, m):
            self.limit, self.v_cmd = m.limit, float(m.v_command)

        @property
        def engaged(self):
            return self.mode == self.AUTONOMOUS

        def call(self, client, timeout=5.0):
            if not client.wait_for_service(timeout_sec=timeout):
                return False, 'service not available'
            fut = client.call_async(self.Req())
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not fut.done():
                return False, 'no answer'
            st = fut.result().status
            return bool(st.success), st.message

    return rclpy, Monitor()


def say(msg):
    print(msg, flush=True)


class Stop(Exception):
    pass


def main(argv=None) -> int:
    opts = parse_args(sys.argv[1:] if argv is None else argv)
    name = opts.name or time.strftime('%Y%m%d_%H%M%S')
    bag = REPO / 'rosbags' / f'lab2cal_{name}'
    if bag.exists():
        say(f'{bag.relative_to(REPO)} exists: pick another --name')
        return 2
    logs = REPO / 'rosbags' / 'run_logs' / bag.name
    just = ['just', 'coach']
    stack = Child('stack', just + ['bench' if opts.bench else 'vehicle', 'launch_planner:=false']
                  + opts.launch_args, logs / 'stack.log')
    planner = Child('planner', just + ['speed-steps', ','.join(str(v) for v in opts.speeds),
                                       str(opts.hold), str(opts.max_distance)], logs / 'planner.log')
    recorder = Child('recorder', just + ['record-cal', name], logs / 'record.log')
    children = [recorder, planner, stack]     # stopped in this order
    interrupted = []
    signal.signal(signal.SIGINT, lambda *_: interrupted.append(1))
    signal.signal(signal.SIGTERM, lambda *_: interrupted.append(1))
    signal.signal(signal.SIGHUP, lambda *_: interrupted.append(1))

    rclpy = mon = None
    finished = False

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(mon, timeout_sec=0.05)
            if interrupted:
                raise Stop('Ctrl-C')
            if not stack.alive():
                raise Stop(f'the stack exited (see {stack.log})')

    def disengage():
        if mon is not None and mon.engaged:
            ok, msg = mon.call(mon.cli_stop)
            say(f'disengage: {"ok" if ok else "FAILED: " + msg}')

    try:
        say(f'1. stack: {" ".join(stack.cmd)}   (log {stack.log.relative_to(REPO)})')
        stack.start()
        rclpy, mon = make_monitor()
        say(f'   vehicle_id {opts.vehicle_id}; waiting for /autosdv/actuator_node ...')
        t0 = time.monotonic()
        while ACTUATOR not in mon.get_node_names_and_namespaces():
            spin(0.5)
            if time.monotonic() - t0 > READY_TIMEOUT:
                raise Stop(f'/autosdv/actuator_node not up after {READY_TIMEOUT:.0f} s (see {stack.log})')
        say(f'2. planner: {opts.steps} m/s, {opts.hold:g} s each, brake at {opts.max_distance:g} m; '
            f'recording to {bag.relative_to(REPO)}')
        planner.start()
        recorder.start()
        say('3. waiting for autonomous mode to become available ...')
        t0, since = time.monotonic(), None
        while True:
            spin(0.5)
            if not recorder.alive():
                raise Stop(f'the recording exited (see {recorder.log})')
            if not (bag / PARAMS_FILE).is_file():
                if time.monotonic() - t0 > PARAMS_TIMEOUT:
                    raise Stop(f'no {bag.relative_to(REPO)}/{PARAMS_FILE}: the fit could not use '
                               f'this bag (see {recorder.log})')
                continue
            if mon.available:
                since = since or time.monotonic()
                if time.monotonic() - since >= READY_STEADY:
                    break
            else:
                since = None
            if time.monotonic() - t0 > READY_TIMEOUT:
                raise Stop(f'not available after {READY_TIMEOUT:.0f} s (see {stack.log})')
        if opts.yes:
            say('4. engaging (--yes)')
        else:
            say(f'4. Ready. The car will drive about {opts.max_distance:g} m straight ahead. '
                'Spotter holds the transmitter. Press Enter to engage, Ctrl-C to quit.')
            while not select.select([sys.stdin], [], [], 0.1)[0]:
                spin(0.1)
            sys.stdin.readline()
        ok, msg = mon.call(mon.cli_auto)
        if not ok:
            raise Stop(f'engage refused: {msg}')
        t_engage = time.monotonic()
        timeout = len(opts.speeds) * opts.hold + RUN_MARGIN
        last_line = 0.0
        while True:
            spin(0.1)
            now = time.monotonic()
            if mon.mrm:
                raise Stop('MRM')
            if not mon.engaged and now - t_engage > 2.0:
                raise Stop('disengaged (transmitter or another terminal)')
            if now - t_engage > timeout:
                raise Stop(f'timeout ({timeout:.0f} s)')
            ended = mon.limit in ('done', 'distance')
            if ended and mon.rest_since and now - mon.rest_since >= REST_HOLD:
                finished = True
                say(f'   schedule {mon.limit}; car at rest')
                break
            if now - last_line >= 1.0:
                last_line = now
                say(f'   t {now - t_engage:5.1f} s  {mon.limit or "-":>9}  '
                    f'target {mon.v_cmd:4.2f}  speed {mon.speed:5.2f} m/s')
        spin(SETTLE_AFTER_END)
    except Stop as e:
        say(f'STOPPED: {e}')
    finally:
        try:
            disengage()
        except Exception as e:  # noqa: BLE001 -- a failed call must not skip the teardown
            say(f'disengage failed: {e}')
        say('5. stopping the recording, the planner and the stack ...')
        for c in children:
            if not c.stop(say=say):
                say(f'  {c.name}: processes left in its session')
        if rclpy is not None:
            mon.destroy_node()
            rclpy.try_shutdown()

    if not finished:
        return 1
    if opts.no_fit:
        say(f'bag: {bag.relative_to(REPO)}')
        return 0
    say('6. fitting the motor and exporting the CSV ...')
    fit = subprocess.run(just + ['fit-motor', str(bag), '--vehicle-id', opts.vehicle_id,
                                 '-o', str(bag / 'motor_plant.yaml')], cwd=REPO,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (logs / 'fit.log').write_text(fit.stdout)
    for line in fit.stdout.splitlines():
        if line.startswith(('prior ', 'lab2_fit_motor:', 'excitation', 'steady levels',
                            '# NOT identified', '# WEAKLY',
                            'deadband:', 'gain:', 'time_constant:', 'coast_time_constant:')):
            say('   ' + line)
    csv = subprocess.run(just + ['export-speed-csv', str(bag), '-o', str(bag / 'speed_step.csv')], cwd=REPO,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (logs / 'export.log').write_text(csv.stdout)
    ok = fit.returncode == 0 and csv.returncode == 0
    say(f'bag:   {bag.relative_to(REPO)}')
    say(f'fit:   {bag.relative_to(REPO)}/motor_plant.yaml' if fit.returncode == 0
        else f'fit FAILED (see {logs / "fit.log"})')
    say(f'csv:   {bag.relative_to(REPO)}/speed_step.csv' if csv.returncode == 0
        else f'export FAILED (see {logs / "export.log"})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
