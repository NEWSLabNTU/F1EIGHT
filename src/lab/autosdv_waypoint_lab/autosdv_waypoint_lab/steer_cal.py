"""Steering calibration: hold a fixed steering PWM, fit the ratio from the circles.

The actuator turns a commanded tire angle into a servo count with

    pwm = init_steer + int(average_angle * tire_angle_to_steer_ratio)

where ``average_angle`` is the mean of the Ackermann inner and outer wheel
angles for the commanded bicycle angle.  Nothing on the car measures the
steering angle, so the ratio has to be measured by driving circles
(docs/known-config-defects.md section 8):

    lab2_steer_cal hold  OFFSET        drive with the servo held at 489 + OFFSET
    lab2_steer_cal fit   OFFSET:RADIUS ...   fit ratio and max steering angle

``hold`` publishes the tire angle that makes the actuator write exactly that
offset, whatever ratio is configured, so the offset is the experiment's
independent variable.  ``fit`` takes the measured circle radii.
"""
import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Sequence, Tuple

WHEEL_BASE = 0.319
WHEEL_TREAD = 0.263
INIT_STEER = 489
SERVO_HALF_RANGE = 50          # min_steer 439 .. max_steer 539 around 489
SHIPPED_RATIO = -1000.0
MAX_COMMAND_ANGLE = 0.349      # vehicle_info max_steer_angle: the actuator clamps here
R_FIRST_CIRCLE = 1.5           # the tightest Lab 2 circle
MARGIN = 0.8                   # the tightest circle may use at most this share of lock


def average_wheel_angle(delta: float, wheel_base: float = WHEEL_BASE,
                        tread: float = WHEEL_TREAD) -> float:
    """The actuator's ``average_angle`` for a commanded bicycle angle ``delta``."""
    if abs(delta) < 0.001:
        return 0.0
    radius = wheel_base / math.tan(abs(delta))
    inner = math.atan(wheel_base / (radius - tread / 2.0))
    outer = math.atan(wheel_base / (radius + tread / 2.0))
    return math.copysign((inner + outer) / 2.0, delta)


def command_for_offset(offset: float, ratio: float = SHIPPED_RATIO,
                       wheel_base: float = WHEEL_BASE, tread: float = WHEEL_TREAD) -> float:
    """The commanded tire angle for which the actuator writes ``init_steer + offset``.

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
    centre: float                # counts: where the fit crosses zero angle (0 = 489 is straight)
    rms_counts: float            # residual of the fit, in servo counts
    max_angle: float             # rad at +-SERVO_HALF_RANGE counts
    min_radius: float            # m at that lock
    angle_needed: float          # rad for the tightest circle
    margin_ok: bool
    n: int

    def lines(self) -> List[str]:
        out = [
            f'tire_angle_to_steer_ratio  {self.ratio:+.1f} counts/rad   '
            f'(shipped {SHIPPED_RATIO:+.0f})',
            f'centre error               {self.centre:+.1f} counts  '
            f'(init_steer {INIT_STEER} is straight if ~0)',
            f'fit residual               {self.rms_counts:.1f} counts rms over {self.n} circles',
            f'max_steer_angle            {self.max_angle:.3f} rad at +-{SERVO_HALF_RANGE} counts'
            f'  (vehicle_info.param.yaml)',
            f'minimum radius at lock     {self.min_radius:.2f} m',
            f'R {R_FIRST_CIRCLE} m needs          {self.angle_needed:.3f} rad = '
            f'{100 * self.angle_needed / self.max_angle:.0f} % of lock  '
            f'-> {"PASS" if self.margin_ok else "FAIL"} (limit {100 * MARGIN:.0f} %)',
        ]
        if self.ratio > 0:
            out.append('NOTE: positive ratio: PWM above 489 turned the car LEFT; the shipped '
                       'sign (negative) is the opposite of this car')
        if self.max_angle > MAX_COMMAND_ANGLE:
            out.append(f'NOTE: max angle exceeds the {MAX_COMMAND_ANGLE} rad clamp the actuator '
                       'applies from vehicle_info; set max_steer_angle there to the fitted value')
        return out


def delta_from_radius(radius: float, turn: float = 1.0, wheel_base: float = WHEEL_BASE) -> float:
    """Bicycle tire angle of a circle of rear-axle radius ``radius`` (``turn``: +1 left, -1 right)."""
    return math.copysign(math.atan(wheel_base / radius), turn)


def fit(pairs: Sequence[Tuple[float, float]], wheel_base: float = WHEEL_BASE) -> SteerFit:
    """Fit ``offset = ratio * delta + centre`` to (offset counts, signed radius m) pairs.

    ``radius`` is the rear-axle-centre radius; its sign is the turn: positive
    left (counter-clockwise from above), negative right.  Straight driving is
    not a circle, so zero offsets are for the caller to leave out.
    """
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
    max_angle = SERVO_HALF_RANGE / abs(ratio)
    needed = math.atan(wheel_base / R_FIRST_CIRCLE)
    return SteerFit(ratio, centre, rms, max_angle, wheel_base / math.tan(max_angle),
                    needed, needed <= MARGIN * max_angle, n)


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


def _hold(args) -> int:
    import rclpy
    from autoware_control_msgs.msg import Control
    from rclpy.executors import ExternalShutdownException
    ratio, off = args.ratio, args.offset
    if abs(off) > SERVO_HALF_RANGE:
        print(f'offset {off} is beyond the servo window +-{SERVO_HALF_RANGE}', file=sys.stderr)
        return 2
    delta = command_for_offset(off, ratio, args.wheel_base, args.wheel_tread)
    if abs(delta) > MAX_COMMAND_ANGLE:
        print(f'needs {delta:.3f} rad, beyond the actuator clamp {MAX_COMMAND_ANGLE}; '
              'raise max_steer_angle or pass the ratio the actuator is really using',
              file=sys.stderr)
        return 2
    print(f'holding steer PWM {INIT_STEER + off} (offset {off:+d}) with tire angle '
          f'{delta:+.4f} rad, {args.speed} m/s; Ctrl-C to stop')
    rclpy.init()
    node = rclpy.create_node('lab2_steer_hold')
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
    pairs = [(o, radius_at_axle(abs(r), args.measured, args.wheel_tread) * math.copysign(1, r))
             for o, r in args.pairs]
    try:
        result = fit(pairs, args.wheel_base)
    except ValueError as e:
        print(f'cannot fit: {e}', file=sys.stderr)
        return 2
    print('\n'.join(result.lines()))
    print(f'\ntire_angle_to_steer_ratio: {result.ratio:.1f}   # actuator.yaml / actuator_param_overrides')
    print(f'max_steer_angle: {result.max_angle:.3f}            # vehicle_info.param.yaml')
    return 0 if result.margin_ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    h = sub.add_parser('hold', help='publish the tire angle that holds a steer PWM offset')
    h.add_argument('offset', type=int, help='servo counts from 489, within +-50 (sign: PWM direction)')
    h.add_argument('--speed', type=float, default=0.4, help='m/s (default 0.4)')
    h.add_argument('--ratio', type=float, default=SHIPPED_RATIO,
                   help='the tire_angle_to_steer_ratio the actuator is running with')
    h.add_argument('--rate', type=float, default=20.0)
    h.add_argument('--wheel-base', type=float, default=WHEEL_BASE)
    h.add_argument('--wheel-tread', type=float, default=WHEEL_TREAD)
    f = sub.add_parser('fit', help='fit the ratio from OFFSET:RADIUS pairs')
    f.add_argument('pairs', nargs='*', metavar='OFFSET:RADIUS',
                   help='PWM offset and measured radius [m], negative radius = right turn')
    f.add_argument('--measured', choices=('axle', 'inner', 'outer'), default='axle',
                   help='what the radius is the radius of: rear-axle centre, or the inside / '
                        'outside rear tyre track')
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
