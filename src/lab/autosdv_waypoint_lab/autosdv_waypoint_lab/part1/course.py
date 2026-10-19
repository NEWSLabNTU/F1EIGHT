"""The Lab 2 Part 1 course: one straight whose speed column is the speed profile. No ROS.

    lab2_make_part1_course [--out DIR] [--speeds 4,11] [--holds 4,4]
                           [--accel 2.5] [--decel-brake 2.5] [--runout 10]
                           [--estop-hold K --estop-offset M]

`just lab2 make-part1-course` runs it; the default output is the package's
config/courses/lab2_part1 (committed, so `just lab2 sim` needs no generation step).
Segments, in order (docs/design/lab2-part1-sim-design.md, "The course and the run"):

    accel 0 -> 4, hold 4 m/s 4 s       launch from rest, low-speed hold
    accel 4 -> 11, hold 4 s            the speed change: full throttle, hold at 40 km/h
    brake 11 -> 0 onto the stop line, run-out 10 m

About 17 s of driving (nominal_duration 16.8 s). Accelerations are planned at `accel`
(2.5 m/s^2), braking (the final stop, and any step down a custom --speeds asks for) at
`decel_brake` (2.5 m/s^2, under every variant's full rear brake of 2.98-3.36 m/s^2); each
segment carries its own `rate`. The written files:

    path.csv              x, y, v every 0.1 m: the waypoint path publisher's input
    course.yaml           the waypoint lab's course header (open, one lap, no markers)
    lanelet2_map.osm      the lane (a lanelet per segment, with its speed limit) and the
                          stop line, for RViz: rc_sim's lanelet writer
    map_projector_info.yaml   projector_type: local
    track.yaml            the start and goal poses, which rc_sim_plant reads (track_dir)
    part1_profile.yaml    THE CONTRACT with the scenario node and the grader: the holds,
                          every segment's [s0, s1] and speeds, the stop line (and the e-stop
                          trigger, when one is asked for). Positions are metres along the
                          path from the start pose.

The first waypoint's speed is floored at ENGAGE_VELOCITY: a profile that starts at
exactly 0 m/s would command 0 to a car at rest forever.

The course has no e-stop. One is optional: `--estop-hold K --estop-offset M` puts a trigger
M metres into hold K, and the profile then carries `estop` (and estop_trigger_s,
estop_hold_index); lab2_part1_scenario raises the emergency there, clears it 1 s after the
car is at rest and ramps the target back up at `resume_accel` (2.5 m/s^2). Without the
block the scenario never raises one. The graded checks (metrics.py) do not use it.
"""
import argparse
import math
import os
import sys
from types import SimpleNamespace

import yaml

ENGAGE_VELOCITY = 0.25    # [m/s] rc_sim's floor (track.make_profile engage_velocity)
SPACING = 0.1             # [m] path.csv; the path publisher resamples to its own `spacing`
DEFAULT_SPEEDS = (4.0, 11.0)
DEFAULT_HOLDS = (4.0, 4.0)
RESUME_ACCEL = 2.5      # [m/s^2] the scenario's target ramp after an (optional) e-stop clears
COURSE_NAME = 'lab2_part1'


class CourseError(ValueError):
    """A profile that cannot be written."""


def make_segments(speeds, holds, accel, decel, runout):
    """[{kind, s0, s1, v0, v1, rate, group, t0, t1[, hold_index]}]; decel: the braking rate."""
    speeds = [float(v) for v in speeds]
    holds = [float(h) for h in holds]
    if not speeds or len(speeds) != len(holds):
        raise CourseError(f'need one hold time per speed (speeds {speeds}, holds {holds})')
    if any(v <= 0.0 or not math.isfinite(v) for v in speeds):
        raise CourseError(f'hold speeds must be positive: {speeds}')
    if any(h <= 0.0 or not math.isfinite(h) for h in holds):
        raise CourseError(f'hold times must be positive: {holds}')
    if accel <= 0.0 or decel <= 0.0 or runout < 0.0:
        raise CourseError('accel and decel must be positive and runout not negative')
    for a, b in zip(speeds, speeds[1:]):
        if a == b:
            raise CourseError(f'two consecutive holds at {a} m/s: merge them')
    segs, s, t, v0 = [], 0.0, 0.0, 0.0

    def add(kind, length, v_from, v_to, group, duration, rate=0.0, **extra):
        nonlocal s, t
        seg = {'kind': kind, 's0': round(s, 4), 's1': round(s + length, 4),
               'v0': v_from, 'v1': v_to, 'rate': rate, 'group': group,
               't0': round(t, 3), 't1': round(t + duration, 3)}
        seg.update(extra)
        segs.append(seg)
        s += length
        t += duration

    for i, (v, hold) in enumerate(zip(speeds, holds)):
        if v > v0:
            add('accel', (v * v - v0 * v0) / (2.0 * accel), v0, v, i + 1, (v - v0) / accel, accel)
        else:
            add('decel', (v0 * v0 - v * v) / (2.0 * decel), v0, v, i + 1, (v0 - v) / decel, decel)
        add('hold', v * hold, v, v, i + 1, hold, hold_index=i)
        v0 = v
    add('brake', v0 * v0 / (2.0 * decel), v0, 0.0, len(speeds) + 1, v0 / decel, decel)
    add('runout', runout, 0.0, 0.0, len(speeds) + 1, 0.0)
    return segs


def v_profile(segs, s):
    """The planned speed at s: what the path publisher's v column says."""
    for seg in segs:
        if s < seg['s1'] or seg is segs[-1]:
            ds = max(0.0, s - seg['s0'])
            kind, v0, v1, rate = seg['kind'], seg['v0'], seg['v1'], seg['rate']
            if kind == 'accel':
                return max(min(math.sqrt(v0 * v0 + 2.0 * rate * ds), v1), ENGAGE_VELOCITY)
            if kind == 'decel':
                return max(math.sqrt(max(0.0, v0 * v0 - 2.0 * rate * ds)), v1)
            if kind == 'hold':
                return v0
            if kind == 'brake':
                return math.sqrt(max(0.0, 2.0 * rate * (seg['s1'] - s)))
            return 0.0
    return 0.0


def make_profile(speeds=DEFAULT_SPEEDS, holds=DEFAULT_HOLDS, accel=2.5, decel_brake=2.5,
                 runout=10.0, estop_hold_index=None, estop_offset=None,
                 resume_accel=RESUME_ACCEL):
    """The part1_profile.yaml mapping; an e-stop only when estop_hold_index is given."""
    segs = make_segments(speeds, holds, accel, decel_brake, runout)
    brake = segs[-2]
    out = {
        'speeds': [float(v) for v in speeds],
        'holds': [float(x) for x in holds],
        'accel': float(accel),
        'decel_brake': float(decel_brake),
        'runout': float(runout),
        'engage_velocity': ENGAGE_VELOCITY,
        'segments': segs,
        'stop_line_s': brake['s1'],
        'length': segs[-1]['s1'],
        'nominal_duration': brake['t1'],   # [s] from the engage to the stop line
    }
    if estop_hold_index is None:
        return out
    held = [seg for seg in segs if seg['kind'] == 'hold']
    if not 0 <= estop_hold_index < len(held):
        raise CourseError(f'estop_hold_index {estop_hold_index}: there are {len(held)} holds')
    h = held[estop_hold_index]
    if estop_offset is None or not 0.0 < estop_offset < h['s1'] - h['s0']:
        raise CourseError(f'--estop-offset {estop_offset} m: must fall inside the '
                          f'{h["s1"] - h["s0"]:g} m hold')
    trigger = round(h['s0'] + estop_offset, 4)
    out.update({
        'estop_hold_index': estop_hold_index,
        'estop_trigger_s': trigger,
        'estop': {'hold_index': estop_hold_index, 'hold_s0': h['s0'], 'hold_s1': h['s1'],
                  'speed': h['v0'], 'trigger_s': trigger, 'rest_time': 1.0,
                  'resume_accel': float(resume_accel)},
    })
    return out


def check_feasible(profile, variant='nominal'):
    """Warnings for a profile the vehicle model cannot drive (empty without rc_sim)."""
    try:
        from autosdv_rc_sim.drivetrain import brake_decel, grip_accel, top_speed
        from autosdv_rc_sim.params import load_vehicle
    except ImportError:
        return ['autosdv_rc_sim not importable: the profile was not checked against the model']
    p = load_vehicle(variant=variant)
    out = []
    top = top_speed(p)
    for v in profile['speeds']:
        if v > 0.95 * top:
            out.append(f'{variant}: {v:g} m/s leaves under 5 % headroom (top speed {top:.2f} m/s)')
    if profile['accel'] > 0.9 * grip_accel(p):
        out.append(f'{variant}: {profile["accel"]:g} m/s^2 is beyond the rear grip '
                   f'({grip_accel(p):.2f} m/s^2)')
    v_top = max(profile['speeds'])
    if brake_decel(p, v_top) < profile['decel_brake']:
        out.append(f'{variant}: the rear brake gives {brake_decel(p, v_top):.2f} m/s^2 at '
                   f'{v_top:g} m/s, under the {profile["decel_brake"]:g} m/s^2 ramp')
    return out


def path_rows(profile):
    segs = profile['segments']
    n = int(round(profile['length'] / SPACING))
    rows = []
    for i in range(n + 1):
        s = min(i * SPACING, profile['length'])
        rows.append((s, 0.0, v_profile(segs, s)))
    return rows


def _osm(profile):
    """rc_sim's lanelet writer, on this profile (it reads speeds, segments, stop_line_s)."""
    from autosdv_rc_sim.track import _osm as rc_sim_osm
    view = SimpleNamespace(
        speeds=(profile['speeds'][0], max(profile['speeds'])),
        segments=[SimpleNamespace(**{k: seg[k] for k in ('kind', 's0', 's1', 'v0', 'v1')})
                  for seg in profile['segments']],
        stop_line_s=profile['stop_line_s'], length=profile['length'])
    return rc_sim_osm(view).replace('autosdv_rc_sim make-track',
                                    'autosdv_waypoint_lab lab2_make_part1_course')


def _dump(data):
    return yaml.safe_dump(data, sort_keys=False, default_flow_style=None, width=100)


def write_course(profile, out_dir, header=''):
    os.makedirs(out_dir, exist_ok=True)
    gen = f'# GENERATED by lab2_make_part1_course{header}; regenerate, do not edit.\n'
    with open(os.path.join(out_dir, 'path.csv'), 'w') as f:
        f.write(gen + '# x [m], y [m], v [m/s]: the speed profile, every '
                f'{SPACING:g} m along the straight\n')
        for x, y, v in path_rows(profile):
            f.write(f'{x:.3f},{y:.3f},{v:.4f}\n')
    course = {
        'path': 'path.csv', 'closed': False, 'laps': 1, 'v_ref': ENGAGE_VELOCITY,
        'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'marker_size': 0.25, 'markers': [],
    }
    with open(os.path.join(out_dir, 'course.yaml'), 'w') as f:
        f.write(gen + '# The Lab 2 Part 1 straight. Speeds come from path.csv\'s v column;\n'
                '# v_ref is used only for rows without one (none here). No markers: the\n'
                '# simulated car is localized by its true pose.\n' + _dump(course))
    with open(os.path.join(out_dir, 'lanelet2_map.osm'), 'w') as f:
        f.write(_osm(profile))
    with open(os.path.join(out_dir, 'map_projector_info.yaml'), 'w') as f:
        f.write('projector_type: local\n')
    track = {
        'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'goal_pose': {'x': profile['stop_line_s'], 'y': 0.0, 'yaw': 0.0},
        'stop_line_s': profile['stop_line_s'], 'length': profile['length'],
    }
    with open(os.path.join(out_dir, 'track.yaml'), 'w') as f:
        f.write(gen + '# For rc_sim_plant (track_dir): where the car starts. Not an rc_sim\n'
                '# track: the profile is part1_profile.yaml.\n' + _dump(track))
    with open(os.path.join(out_dir, 'part1_profile.yaml'), 'w') as f:
        f.write(gen + '# The contract between the course, lab2_part1_scenario and '
                'lab2_part1_grader.\n'
                '# s: metres along the path from the start pose; t0/t1: the nominal plan [s]\n'
                '# from the engage; hold_index: 0.. over the holds; group: the spec\'s row.\n'
                + _dump(profile))


def default_out_dir():
    here = os.path.realpath(__file__)   # a symlink install points into the source tree
    pkg = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    src_courses = os.path.join(pkg, 'config', 'courses')
    if os.path.isdir(src_courses) and os.path.isfile(os.path.join(pkg, 'setup.py')):
        return os.path.join(src_courses, COURSE_NAME)
    return os.path.join('src', 'lab', 'autosdv_waypoint_lab', 'config', 'courses', COURSE_NAME)


def table(profile):
    rows = ['| # | segment | s0 [m] | s1 [m] | t [s] |', '|---|---|---|---|---|']
    for seg in profile['segments']:
        k = seg['kind']
        if k in ('accel', 'decel'):
            name = f'{k} {seg["v0"]:g} -> {seg["v1"]:g} m/s'
        elif k == 'hold':
            name = f'hold {seg["v0"]:g} m/s'
        elif k == 'brake':
            name = f'brake {seg["v0"]:g} -> 0 onto the stop line'
        else:
            name = 'run-out'
        rows.append(f'| {seg["group"]} | {name} | {seg["s0"]:.2f} | {seg["s1"]:.2f} | '
                    f'{seg["t1"] - seg["t0"]:.1f} |')
    estop = (f'e-stop at {profile["estop_trigger_s"]:.2f} m (hold '
             f'{profile["estop_hold_index"]}), ' if 'estop' in profile else 'no e-stop, ')
    rows.append(f'stop line {profile["stop_line_s"]:.2f} m, {estop}'
                f'nominal {profile["nominal_duration"]:.1f} s')
    return '\n'.join(rows)


def _floats(text):
    return [float(x) for x in text.split(',') if x.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(description='Write the Lab 2 Part 1 straight.')
    ap.add_argument('--out', default='',
                    help=f'output directory (default config/courses/{COURSE_NAME})')
    ap.add_argument('--speeds', default=','.join(f'{v:g}' for v in DEFAULT_SPEEDS))
    ap.add_argument('--holds', default=','.join(f'{v:g}' for v in DEFAULT_HOLDS))
    ap.add_argument('--accel', type=float, default=2.5)
    ap.add_argument('--decel-brake', type=float, default=2.5,
                    help='the step down and the final stop [m/s^2]')
    ap.add_argument('--resume-accel', type=float, default=RESUME_ACCEL,
                    help='the target ramp after an e-stop clears [m/s^2]')
    ap.add_argument('--runout', type=float, default=10.0)
    ap.add_argument('--estop-hold', type=int, default=None,
                    help='optional: the hold (0..) an e-stop fires in (default: no e-stop)')
    ap.add_argument('--estop-offset', type=float, default=None,
                    help='with --estop-hold: where in that hold, metres after its start')
    ap.add_argument('--variant', default='nominal', help='vehicle model variant to check against')
    args = ap.parse_args(argv)
    try:
        profile = make_profile(_floats(args.speeds), _floats(args.holds), args.accel,
                               args.decel_brake, args.runout, args.estop_hold, args.estop_offset,
                               args.resume_accel)
    except (CourseError, ValueError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    print(table(profile))
    for w in check_feasible(profile, args.variant):
        print(f'warning: {w}', file=sys.stderr)
    out = args.out or default_out_dir()
    write_course(profile, out)
    print(f'wrote {out}: path.csv, course.yaml, lanelet2_map.osm, map_projector_info.yaml, '
          'track.yaml, part1_profile.yaml')
    return 0


if __name__ == '__main__':
    sys.exit(main())
