"""The Lab 2 vehicle gate's safety probe: does a vehicle model stop the car?

A team owns its whole vehicle interface (docs/design/lab2-team-vehicle-models.md), stop
logic included, so before a model goes on a car it runs here, in the RC simulator, against
one car's ESC and servo points (individual_params/config/<vehicle_id>/autosdv_vehicle/):

    lab2_vehicle_gate_probe -> /control/command/control_cmd, /control/command/emergency_cmd,
                               /system/fail_safe/mrm_state
      -> <vehicle_model>_launch vehicle interface (hardware:=sim)
      -> /rc_sim/pwm -> rc_sim_plant (that car's ESC) -> hall edges -> velocity_report
    rc_sim_plant -> /localization/kinematic_state (the true speed)

The probe commands a fixed sequence (PHASES) and reads back what the model WROTE to the
motor channel, not what it says it did. Checks (judge()):

    neutral     at rest it holds the car's init_pwm
    drives      it moves the car when asked (else the stop checks prove nothing)
    stop        commanded 0: the car stops and the PWM settles on init_pwm
    watchdog    control_cmd stops arriving: no drive within control_cmd_timeout + a margin,
                the car stops, and the PWM settles on init_pwm
    mrm         an emergency-stop MRM while still commanded to drive: the same
    emergency   vehicle_cmd_gate's emergency verdict while commanded to drive: the same
    no reverse  never a command at or below the car's reverse_arm_pwm (that arms the ESC's
                reverse; nothing here ever asks for reverse)

"No drive" is a PWM at or below forward_hold_pwm (init_pwm when not measured): above it a
moving car's ESC keeps driving.
"""
import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

DRIVE_SPEED = 0.6          # [m/s] asked in the drive phases
MOVING = 0.3               # [m/s] a drive phase that never reaches this: the model does not drive
AT_REST = 0.05             # [m/s]
SETTLE_WINDOW = 1.0        # [s] the end of a stop phase over which the PWM must be init_pwm
REACT_MARGIN = 0.5         # [s] beyond control_cmd_timeout (watchdog) or 0 (MRM, emergency)


@dataclass(frozen=True)
class Phase:
    name: str
    duration: float
    speed: Optional[float]      # None: no control_cmd at all (the watchdog phase)
    mrm: bool = False
    emergency: bool = False


PHASES: Tuple[Phase, ...] = (
    Phase('rest', 3.0, 0.0),
    Phase('drive', 4.0, DRIVE_SPEED),
    Phase('stop', 5.0, 0.0),
    Phase('drive_watchdog', 4.0, DRIVE_SPEED),
    Phase('watchdog', 5.0, None),
    Phase('drive_mrm', 4.0, DRIVE_SPEED),
    Phase('mrm', 5.0, DRIVE_SPEED, mrm=True),
    Phase('clear', 2.0, 0.0),
    Phase('drive_emergency', 4.0, DRIVE_SPEED),
    Phase('emergency', 5.0, DRIVE_SPEED, emergency=True),
)


@dataclass(frozen=True)
class CarPoints:
    """One car's motor points, from its individual_params actuator.param.yaml."""
    init_pwm: int
    forward_hold_pwm: int
    reverse_arm_pwm: int
    control_cmd_timeout: float = 0.3

    @property
    def no_drive(self) -> int:
        return self.forward_hold_pwm or self.init_pwm


@dataclass(frozen=True)
class Sample:
    t: float          # [s] since the first phase began
    phase: str
    pwm: int          # the motor channel as the model last wrote it
    speed: float      # [m/s] the plant's true longitudinal speed


def phase_times(phases: Sequence[Phase] = PHASES) -> Dict[str, Tuple[float, float]]:
    out, t = {}, 0.0
    for p in phases:
        out[p.name] = (t, t + p.duration)
        t += p.duration
    return out


def _in(samples, name):
    return [s for s in samples if s.phase == name]


def _settled(samples, name, car: CarPoints, times) -> Optional[str]:
    """Why the end of phase `name` is not a stop on init_pwm, or None."""
    t1 = times[name][1]
    tail = [s for s in _in(samples, name) if s.t >= t1 - SETTLE_WINDOW]
    if not tail:
        return f'no samples in the last {SETTLE_WINDOW:g} s of {name}'
    if tail[-1].speed > AT_REST:
        return f'still moving at {tail[-1].speed:.2f} m/s at the end of {name}'
    wrong = sorted({s.pwm for s in tail if s.pwm != car.init_pwm})
    if wrong:
        return (f'at rest at the end of {name} it writes {", ".join(map(str, wrong))}, not the car\'s '
                f'neutral {car.init_pwm}')
    return None


def _peak(samples, name) -> float:
    return max((s.speed for s in _in(samples, name)), default=0.0)


def _reacts(samples, name, car: CarPoints, times, allow: float) -> Optional[str]:
    """Why phase `name` shows a drive command `allow` s after it began, or None."""
    t0 = times[name][0] + allow
    late = [s for s in _in(samples, name) if s.t >= t0 and s.pwm > car.no_drive]
    if late:
        s = late[0]
        return (f'still driving {s.t - times[name][0]:.2f} s into {name} (PWM {s.pwm}, above '
                f'{car.no_drive})')
    return None


def judge(samples: Sequence[Sample], car: CarPoints,
          phases: Sequence[Phase] = PHASES) -> List[dict]:
    """[{check, ok, detail}] in the module docstring's order."""
    times = phase_times(phases)
    out = []

    def add(check, problem, ok_detail=''):
        out.append({'check': check, 'ok': problem is None, 'detail': problem or ok_detail})

    add('neutral', _settled(samples, 'rest', car, times), f'holds {car.init_pwm} at rest')
    peak = _peak(samples, 'drive')
    drives = peak >= MOVING
    add('drives', None if drives else
        f'{DRIVE_SPEED:g} m/s asked, the car peaked at {peak:.2f} m/s: the model does not drive '
        'it, so the stop checks below prove nothing',
        f'peak {peak:.2f} m/s for {DRIVE_SPEED:g} asked')
    add('stop', _settled(samples, 'stop', car, times), f'stops and holds {car.init_pwm}')
    for check, before, allow in (('watchdog', 'drive_watchdog', car.control_cmd_timeout + REACT_MARGIN),
                                 ('mrm', 'drive_mrm', REACT_MARGIN),
                                 ('emergency', 'drive_emergency', REACT_MARGIN)):
        if _peak(samples, before) < MOVING:
            add(check, f'the car was not moving before {check} (peak '
                f'{_peak(samples, before):.2f} m/s), so it was not tested')
            continue
        add(check, _reacts(samples, check, car, times, allow) or _settled(samples, check, car, times),
            f'no drive after {allow:.1f} s, stopped on {car.init_pwm}')
    armed = [s for s in samples if s.pwm <= car.reverse_arm_pwm]
    add('no reverse', None if not armed else
        f'wrote {armed[0].pwm} at {armed[0].t:.2f} s ({armed[0].phase}), at or below the car\'s '
        f'reverse_arm_pwm {car.reverse_arm_pwm}: that arms the ESC\'s reverse',
        f'never at or below {car.reverse_arm_pwm}')
    return out


def car_points(param_files: Sequence[str]) -> CarPoints:
    """The motor points the actuator would load from these files, later files winning."""
    import yaml
    v = {'init_pwm': 370, 'forward_hold_pwm': 0, 'reverse_arm_pwm': 0, 'control_cmd_timeout': 0.3}
    for f in param_files:
        if not f:
            continue
        with open(f) as fh:
            d = yaml.safe_load(fh) or {}
        params = (d.get('/**') or {}).get('ros__parameters') or {}
        for k in v:
            if params.get(k) not in (None,):
                v[k] = params[k]
    return CarPoints(int(v['init_pwm']), int(v['forward_hold_pwm']), int(v['reverse_arm_pwm']),
                     float(v['control_cmd_timeout']))


# ---- the node --------------------------------------------------------------------------

def main(argv=None):
    import rclpy
    from autoware_adapi_v1_msgs.msg import MrmState
    from autoware_control_msgs.msg import Control
    from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int32MultiArray

    from autosdv_vehicle_interface.hal import PWM_TOPIC

    class Probe(Node):
        def __init__(self):
            super().__init__('lab2_vehicle_gate_probe')
            self.declare_parameter('param_files', [''])
            self.declare_parameter('out_file', '')
            self.declare_parameter('vehicle_model', '')
            self.declare_parameter('vehicle_id', '')
            self.declare_parameter('ready_timeout', 40.0)
            gp = lambda k: self.get_parameter(k).value  # noqa: E731
            self.car = car_points(gp('param_files'))
            self.out_file = gp('out_file')
            self.meta = {'vehicle_model': gp('vehicle_model'), 'vehicle_id': gp('vehicle_id'),
                         'car': self.car.__dict__}
            self.pwm = None
            self.speed = 0.0
            self.t0 = None
            self.samples: List[Sample] = []
            self.done = False
            reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Int32MultiArray, PWM_TOPIC, self.on_pwm, reliable)
            self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, 10)
            self.pub_cmd = self.create_publisher(Control, '/control/command/control_cmd', 1)
            self.pub_emg = self.create_publisher(VehicleEmergencyStamped,
                                                 '/control/command/emergency_cmd', 1)
            self.pub_mrm = self.create_publisher(MrmState, '/system/fail_safe/mrm_state', 1)
            self.MrmState = MrmState
            self.Control, self.Emergency = Control, VehicleEmergencyStamped
            self.started_wall = time.monotonic()
            self.ready_timeout = float(gp('ready_timeout'))
            self.create_timer(1.0 / 30.0, self.tick)

        def on_pwm(self, m):
            if len(m.data) >= 2 and m.data[0] == 0:
                self.pwm = int(m.data[1])

        def on_odom(self, m):
            self.speed = float(m.twist.twist.linear.x)

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def phase_at(self, t):
            for p, (a, b) in zip(PHASES, phase_times().values()):
                if a <= t < b:
                    return p
            return None

        def tick(self):
            if self.done:
                return
            now = self.now()
            if self.t0 is None:
                self.publish(Phase('wait', 0.0, 0.0), now)
                if self.pwm is not None and now > 0.0:
                    self.t0 = now
                    self.get_logger().info(f'ready; car {self.meta["vehicle_id"]}: {self.car}')
                elif time.monotonic() - self.started_wall > self.ready_timeout:
                    self.finish(error='the simulator clock never started (no /clock)'
                                if now == 0.0 else
                                f'the model never wrote the motor channel ({PWM_TOPIC}) in '
                                f'{self.ready_timeout:g} s: its actuator did not start, or does '
                                'not drive the simulator\'s PCA9685 (see its log)')
                return
            t = now - self.t0
            p = self.phase_at(t)
            if p is None:
                self.finish()
                return
            self.publish(p, now)
            if self.pwm is not None:
                self.samples.append(Sample(round(t, 4), p.name, self.pwm, self.speed))

        def publish(self, p: Phase, now: float):
            stamp = self.get_clock().now().to_msg()
            if p.speed is not None:
                c = self.Control()
                c.stamp = stamp
                c.longitudinal.stamp = stamp
                c.lateral.stamp = stamp
                c.longitudinal.velocity = float(p.speed)
                self.pub_cmd.publish(c)
            e = self.Emergency()
            e.stamp = stamp
            e.emergency = bool(p.emergency)
            self.pub_emg.publish(e)
            m = self.MrmState()
            m.stamp = stamp
            if p.mrm:
                m.state, m.behavior = self.MrmState.MRM_OPERATING, self.MrmState.EMERGENCY_STOP
            else:
                m.state, m.behavior = self.MrmState.NORMAL, self.MrmState.NONE
            self.pub_mrm.publish(m)

        def finish(self, error: str = ''):
            self.done = True
            checks = [] if error else judge(self.samples, self.car)
            result = dict(self.meta, ok=not error and all(c['ok'] for c in checks),
                          error=error, checks=checks,
                          samples=[s.__dict__ for s in self.samples])
            if self.out_file:
                with open(self.out_file + '.tmp', 'w') as f:
                    json.dump(result, f)
                import os
                os.replace(self.out_file + '.tmp', self.out_file)
            for c in checks:
                self.get_logger().info(f"{'PASS' if c['ok'] else 'FAIL'} {c['check']}: {c['detail']}")
            raise SystemExit(0)

    rclpy.init(args=argv)
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
