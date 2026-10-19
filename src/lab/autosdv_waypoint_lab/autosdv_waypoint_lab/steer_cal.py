"""Steering calibration: hold a fixed steering PWM, fit the ratio from the circles.

The actuator turns a commanded tire angle into a servo count with

    pwm = init_steer + int(average_angle * tire_angle_to_steer_ratio)

where ``average_angle`` is the mean of the Ackermann inner and outer wheel
angles for the commanded bicycle angle.  Nothing on the car measures the
steering angle, so the ratio has to be measured by driving circles
(docs/known-config-defects.md section 8):

    lab2_steer_cal hold  OFFSET        drive with the servo held at init_steer + OFFSET
    lab2_steer_cal fit   --vehicle-id car_b OFFSET:RADIUS ...    fit ratio and lock

``hold`` reads init_steer, min_steer, max_steer, tire_angle_to_steer_ratio,
max_steer_angle and the wheel geometry from the RUNNING actuator
(/autosdv/actuator_node), so it matches whatever car's PWM drives the servo,
and publishes the tire angle that makes the actuator write exactly that offset:
the offset is the experiment's independent variable.  ``fit`` is offline: it
takes the measured circle radii and the car's actuator file
(individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml).
"""
import argparse
import math
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

WHEEL_BASE = 0.319             # vehicle_info.param.yaml, for the offline fit
WHEEL_TREAD = 0.263
MAX_COMMAND_ANGLE = 0.349      # vehicle_info max_steer_angle: the actuator clamps here
ACTUATOR_NODE = '/autosdv/actuator_node'
SERVO_PARAMS = ('init_steer', 'min_steer', 'max_steer', 'tire_angle_to_steer_ratio')
GEOMETRY_PARAMS = ('max_steer_angle', 'wheel_base', 'wheel_tread')


@dataclass
class Servo:
    """The steering numbers the actuator runs: one car's actuator file (+ vehicle_info)."""
    init_steer: int
    min_steer: int
    max_steer: int
    ratio: float                         # tire_angle_to_steer_ratio, counts/rad
    max_steer_angle: float = MAX_COMMAND_ANGLE
    wheel_base: float = WHEEL_BASE
    wheel_tread: float = WHEEL_TREAD
    source: str = ''                     # where the numbers came from, for messages

    @classmethod
    def from_params(cls, p: Mapping[str, Any], source: str = '') -> 'Servo':
        """Build from actuator parameter names; ``p`` may be a car_profile.load(vehicle_id) dict
        or what the running node answered.  Missing servo keys raise ValueError."""
        missing = [k for k in SERVO_PARAMS if p.get(k) is None]
        if missing:
            raise ValueError(f'{source or "parameters"}: missing {", ".join(missing)}')
        s = cls(int(p['init_steer']), int(p['min_steer']), int(p['max_steer']),
                float(p['tire_angle_to_steer_ratio']), source=source)
        for k in GEOMETRY_PARAMS:
            if p.get(k) is not None:
                setattr(s, k, float(p[k]))
        if not s.min_steer <= s.init_steer <= s.max_steer:
            raise ValueError(f'{source}: init_steer {s.init_steer} outside '
                             f'{s.min_steer}..{s.max_steer}')
        if s.ratio == 0:
            raise ValueError(f'{source}: tire_angle_to_steer_ratio is 0')
        return s

    @property
    def window(self) -> Tuple[int, int]:
        """Offsets the actuator can write without clamping: (min - init, max - init)."""
        return self.min_steer - self.init_steer, self.max_steer - self.init_steer

    @property
    def half_range(self) -> int:
        """Counts to the nearer servo limit: the lock both directions reach."""
        lo, hi = self.window
        return min(-lo, hi)

    def describe(self) -> str:
        return (f'init_steer {self.init_steer}, window {self.min_steer}..{self.max_steer}, '
                f'ratio {self.ratio:+g}, clamp {self.max_steer_angle:g} rad')


def graded_radius() -> float:
    """The tightest Lab 2 circle: the U-turn radius the graded course lab2_uturn was
    generated with (course.yaml `uturn.radius`, written by make-uturn)."""
    import os
    import yaml
    from autosdv_waypoint_lab import route_grade
    d = route_grade.template_dir('uturn')
    if d:
        try:
            with open(os.path.join(d, 'course.yaml')) as f:
                return float(yaml.safe_load(f)['uturn']['radius'])
        except (OSError, KeyError, TypeError, ValueError):
            pass
    from autosdv_waypoint_lab import uturn
    return uturn.DEFAULT_RADIUS


R_FIRST_CIRCLE = graded_radius()
RADIUS_MARGIN = 0.2            # m: the graded circle sits this far outside the car's tightest
MAX_MIN_RADIUS = R_FIRST_CIRCLE - RADIUS_MARGIN   # the gate: measured min radius at most this


def average_wheel_angle(delta: float, wheel_base: float = WHEEL_BASE,
                        tread: float = WHEEL_TREAD) -> float:
    """The actuator's ``average_angle`` for a commanded bicycle angle ``delta``."""
    if abs(delta) < 0.001:
        return 0.0
    radius = wheel_base / math.tan(abs(delta))
    inner = math.atan(wheel_base / (radius - tread / 2.0))
    outer = math.atan(wheel_base / (radius + tread / 2.0))
    return math.copysign((inner + outer) / 2.0, delta)


def command_for_offset(offset: float, ratio: float,
                       wheel_base: float = WHEEL_BASE, tread: float = WHEEL_TREAD) -> float:
    """The commanded tire angle for which the actuator writes ``init_steer + offset``.

    ``ratio`` is the tire_angle_to_steer_ratio the actuator is running.

    The actuator truncates ``average_angle * ratio`` toward zero, so the
    target is nudged half a count past the integer to land inside it.
    """
    want = (abs(offset) + 0.5) / abs(ratio) * math.copysign(1.0, offset * ratio)
    if offset == 0:
        return 0.0
    lo, hi = 0.0, math.pi / 4
    sign = 1.0 if want > 0 else -1.0
    target = abs(want)
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if average_wheel_angle(mid, wheel_base, tread) < target:
            lo = mid
        else:
            hi = mid
    return sign * (lo + hi) / 2.0


@dataclass
class SteerFit:
    ratio: float                 # counts per rad: tire_angle_to_steer_ratio
    centre: float                # counts: where the fit crosses zero angle (0 = init_steer straight)
    rms_counts: float            # residual of the fit, in servo counts
    max_angle: float             # rad at +-servo.half_range counts
    min_radius: float            # m at that lock
    angle_needed: float          # rad for the tightest circle (information; the gate is the radius)
    margin_ok: bool
    n: int
    servo: Servo
    car: str = ''                # the vehicle_id, for the "where it goes" lines

    def lines(self) -> List[str]:
        out = [
            f'tire_angle_to_steer_ratio  {self.ratio:+.1f} counts/rad   '
            f'({self.car or "profile"} has {self.servo.ratio:+.0f})',
            f'centre error               {self.centre:+.1f} counts  '
            f'(init_steer {self.servo.init_steer} is straight if ~0)',
            f'fit residual               {self.rms_counts:.1f} counts rms over {self.n} circles',
            f'max_steer_angle            {self.max_angle:.3f} rad at +-{self.servo.half_range} counts'
            f'  (vehicle_info.param.yaml)',
            f'minimum radius at lock     {self.min_radius:.2f} m  '
            f'-> {"PASS" if self.margin_ok else "FAIL"} (at most {MAX_MIN_RADIUS:.2f} m: '
            f'R {R_FIRST_CIRCLE:g} m less {RADIUS_MARGIN:g} m)',
            f'R {R_FIRST_CIRCLE:g} m needs          {self.angle_needed:.3f} rad = '
            f'{100 * self.angle_needed / self.max_angle:.0f} % of lock',
        ]
        if self.ratio > 0:
            out.append(f'NOTE: positive ratio: PWM above {self.servo.init_steer} turned the car '
                       'LEFT; the shipped sign (negative) is the opposite of this car')
        if self.max_angle > MAX_COMMAND_ANGLE:
            out.append(f'NOTE: max angle exceeds the {MAX_COMMAND_ANGLE} rad clamp the actuator '
                       'applies from vehicle_info; set max_steer_angle there to the fitted value')
        return out


def delta_from_radius(radius: float, turn: float = 1.0, wheel_base: float = WHEEL_BASE) -> float:
    """Bicycle tire angle of a circle of rear-axle radius ``radius`` (``turn``: +1 left, -1 right)."""
    return math.copysign(math.atan(wheel_base / radius), turn)


def fit(pairs: Sequence[Tuple[float, float]], servo: Servo, car: str = '') -> SteerFit:
    """Fit ``offset = ratio * delta + centre`` to (offset counts, signed radius m) pairs.

    ``offset`` is counts from ``servo.init_steer``; ``radius`` is the
    rear-axle-centre radius; its sign is the turn: positive left
    (counter-clockwise from above), negative right.  Straight driving is not a
    circle, so zero offsets are for the caller to leave out.  An offset outside
    the servo window was clamped while it was held, so it is refused.
    """
    wheel_base = servo.wheel_base
    lo, hi = servo.window
    out = [o for o, _ in pairs if not lo <= o <= hi]
    if out:
        raise ValueError(f'offsets {", ".join(f"{o:+g}" for o in out)} are outside the servo '
                         f'window {lo:+d}..{hi:+d} ({servo.describe()})')
    pts = [(off, delta_from_radius(abs(r), math.copysign(1.0, r), wheel_base))
           for off, r in pairs]
    if len(pts) < 3:
        raise ValueError('need at least 3 circles to fit a ratio')
    if len({round(d, 6) for _, d in pts}) < 2:
        raise ValueError('the circles are all the same size')
    n = len(pts)
    mx = sum(d for _, d in pts) / n
    my = sum(o for o, _ in pts) / n
    sxx = sum((d - mx) ** 2 for _, d in pts)
    ratio = sum((d - mx) * (o - my) for o, d in pts) / sxx
    centre = my - ratio * mx
    rms = math.sqrt(sum((o - (ratio * d + centre)) ** 2 for o, d in pts) / n)
    max_angle = servo.half_range / abs(ratio)
    needed = math.atan(wheel_base / R_FIRST_CIRCLE)
    min_radius = wheel_base / math.tan(max_angle)
    return SteerFit(ratio, centre, rms, max_angle, min_radius,
                    needed, min_radius <= MAX_MIN_RADIUS, n, servo, car)


def radius_at_axle(measured: float, side: str, tread: float = WHEEL_TREAD) -> float:
    """Convert a measured radius to the rear-axle-centre radius.

    ``side``: ``axle`` (already the centre), ``inner`` or ``outer`` (the track
    of the rear tyre on the inside or outside of the turn).
    """
    return {'axle': measured, 'inner': measured + tread / 2.0,
            'outer': measured - tread / 2.0}[side]


def parse_pair(text: str) -> Tuple[float, float]:
    off, _, rad = text.partition(':')
    if not rad:
        raise argparse.ArgumentTypeError(f'{text!r}: expected OFFSET:RADIUS, e.g. -25:+3.1')
    return float(off), float(rad)


def hold_check(servo: Servo, offset: int) -> Tuple[Optional[float], str]:
    """(tire angle to command, '') for an offset the actuator can write, else (None, why)."""
    lo, hi = servo.window
    if not lo <= offset <= hi:
        return None, (f'offset {offset:+d} is outside the servo window {lo:+d}..{hi:+d} '
                      f'(PWM {servo.min_steer}..{servo.max_steer}); the actuator would clamp it')
    delta = command_for_offset(offset, servo.ratio, servo.wheel_base, servo.wheel_tread)
    if abs(delta) > servo.max_steer_angle:
        return None, (f'offset {offset:+d} needs {delta:.3f} rad, beyond the actuator clamp '
                      f'max_steer_angle {servo.max_steer_angle:g}; the actuator would write less')
    return delta, ''


def _param_value(v) -> Any:
    """rcl_interfaces/ParameterValue to Python; None when the parameter is not set."""
    from rcl_interfaces.msg import ParameterType as T
    return {T.PARAMETER_BOOL: lambda: v.bool_value, T.PARAMETER_INTEGER: lambda: v.integer_value,
            T.PARAMETER_DOUBLE: lambda: v.double_value,
            T.PARAMETER_STRING: lambda: v.string_value}.get(v.type, lambda: None)()


def read_node_params(node, target: str, names: Sequence[str], timeout: float) -> Dict[str, Any]:
    """Ask ``target``'s get_parameters service; raises RuntimeError naming the node."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f'{target}/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{target} is not running (no {target}/get_parameters within '
                               f'{timeout:g} s): start `just control basic` first')
        fut = cli.call_async(GetParameters.Request(names=list(names)))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f'{target} did not answer get_parameters within {timeout:g} s')
        return {n: _param_value(v) for n, v in zip(names, fut.result().values)}
    finally:
        node.destroy_client(cli)


def _hold(args) -> int:
    import rclpy
    from autoware_control_msgs.msg import Control
    from rclpy.executors import ExternalShutdownException
    off = args.offset
    rclpy.init()
    node = rclpy.create_node('lab2_steer_hold')
    try:
        params = read_node_params(node, args.node, SERVO_PARAMS + GEOMETRY_PARAMS, args.timeout)
        servo = Servo.from_params(params, args.node)
    except (RuntimeError, ValueError) as e:
        print(f'steer-hold: {e}', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return 2
    delta, why = hold_check(servo, off)
    if delta is None:
        print(f'steer-hold: {why} ({args.node}: {servo.describe()})', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return 2
    print(f'holding steer PWM {servo.init_steer + off} (offset {off:+d}) with tire angle '
          f'{delta:+.4f} rad, {args.speed} m/s; {args.node}: {servo.describe()}; Ctrl-C to stop')
    pub = node.create_publisher(Control, '/control/command/control_cmd', 1)

    def tick():
        msg = Control()
        msg.stamp = node.get_clock().now().to_msg()
        msg.longitudinal.velocity = float(args.speed)
        msg.lateral.steering_tire_angle = float(delta)
        pub.publish(msg)
    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:                    # the actuator also stops on its own command timeout
            stop = Control()
            stop.stamp = node.get_clock().now().to_msg()
            pub.publish(stop)   # speed 0, straight
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def _fit(args) -> int:
    from autosdv_vehicle_interface import car_profile
    try:
        p = dict(car_profile.load(args.vehicle_id))
        p.update(wheel_base=args.wheel_base, wheel_tread=args.wheel_tread)
        servo = Servo.from_params(p, f'individual_params/config/{args.vehicle_id}/autosdv_vehicle/actuator.param.yaml')
    except ValueError as e:
        print(f'cannot fit: {e}', file=sys.stderr)
        return 2
    pairs = [(o, radius_at_axle(abs(r), args.measured, servo.wheel_tread) * math.copysign(1, r))
             for o, r in args.pairs]
    try:
        result = fit(pairs, servo, args.vehicle_id)
    except ValueError as e:
        print(f'cannot fit: {e}', file=sys.stderr)
        return 2
    print('\n'.join(result.lines()))
    print(f'\ntire_angle_to_steer_ratio: {result.ratio:.1f}   '
          f'# individual_params/config/{args.vehicle_id}/autosdv_vehicle/actuator.param.yaml')
    print(f'max_steer_angle: {result.max_angle:.3f}            # vehicle_info.param.yaml')
    return 0 if result.margin_ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    h = sub.add_parser('hold', help='publish the tire angle that holds a steer PWM offset')
    h.add_argument('offset', type=int, help='servo counts from the running init_steer, inside '
                                            'its min_steer..max_steer (sign: PWM direction)')
    h.add_argument('--speed', type=float, default=0.4, help='m/s (default 0.4)')
    h.add_argument('--rate', type=float, default=20.0)
    h.add_argument('--node', default=ACTUATOR_NODE,
                   help=f'the actuator whose parameters to read (default {ACTUATOR_NODE})')
    h.add_argument('--timeout', type=float, default=3.0, help='s to wait for it (default 3)')
    f = sub.add_parser('fit', help='fit the ratio from OFFSET:RADIUS pairs')
    f.add_argument('pairs', nargs='*', metavar='OFFSET:RADIUS',
                   help='PWM offset and measured radius [m], negative radius = right turn')
    f.add_argument('--measured', choices=('axle', 'inner', 'outer'), default='axle',
                   help='what the radius is the radius of: rear-axle centre, or the inside / '
                        'outside rear tyre track')
    from autosdv_vehicle_interface import car_profile
    car_profile.add_argument(f)
    f.add_argument('--wheel-base', type=float, default=WHEEL_BASE)
    f.add_argument('--wheel-tread', type=float, default=WHEEL_TREAD)
    args, rest = ap.parse_known_args(argv)     # '-25:+3.1' looks like an option to argparse
    if args.cmd == 'fit':
        try:
            args.pairs = [parse_pair(t) for t in list(args.pairs) + rest]
        except argparse.ArgumentTypeError as e:
            ap.error(str(e))
    elif rest:
        ap.error(f'unrecognized arguments: {" ".join(rest)}')
    return _hold(args) if args.cmd == 'hold' else _fit(args)


if __name__ == '__main__':
    sys.exit(main())
