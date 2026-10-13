"""Export a speed-step recording as the CSV the Lab 2 students fit from. ROS-free apart from the bag.

    t,pwm,measured_speed
    0.05,370,0.0000

One row per hall-sensor speed report (20 Hz): ``t`` seconds from the first PWM
sample, ``pwm`` the count the actuator last wrote at that time, ``measured_speed``
what velocity_report published, in m/s. The same two signals lab2_fit_motor
fits, so a student's steady-state fit and the TA's motor fit read one recording.

    just coach export-speed-csv rosbags/lab2cal_X -o tmp/car_A.csv
    just coach export-speed-csv rosbags/lab2cal_X -o tmp/car_A.csv --start 5 --end 120

The course repository's ``ta/cars/car_tool.py install-fit`` ships the file to
the handout together with the fit.
"""
import argparse
import csv
from typing import Optional

from .motor_fit import PWM_TOPIC, SPEED_TOPIC, Recording, pwm_at, read_csv, recording_from_bag


def rows(rec: Recording):
    """(t, pwm, speed) per speed report, t from the first PWM sample."""
    t0 = float(rec.pwm_t[0])
    pwm = pwm_at(rec, rec.v_t)
    for t, p, v in zip(rec.v_t, pwm, rec.v):
        if t >= t0:
            yield float(t) - t0, int(round(float(p))), float(v)


def write(path: str, rec: Recording) -> int:
    n = 0
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'pwm', 'measured_speed'])
        for t, p, v in rows(rec):
            w.writerow([f'{t:.2f}', p, f'{v:.4f}'])
            n += 1
    return n


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog='lab2_export_speed_csv', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('recording', help='rosbag2 directory (or a t,pwm,speed CSV from lab2_fit_motor)')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--pwm-topic', default=PWM_TOPIC)
    ap.add_argument('--speed-topic', default=SPEED_TOPIC)
    ap.add_argument('--start', type=float, help='ignore the first START s')
    ap.add_argument('--end', type=float, help='ignore everything after END s')
    args = ap.parse_args(argv)
    if args.recording.endswith('.csv'):
        rec = read_csv(args.recording)
    else:
        rec = recording_from_bag(args.recording, args.pwm_topic, args.speed_topic)
    if args.start is not None or args.end is not None:
        rec = rec.window(args.start, args.end)
    n = write(args.out, rec)
    print(f'wrote {args.out}: {n} rows, {rec.duration:.1f} s, '
          f'max speed {float(rec.v.max()):.2f} m/s')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
