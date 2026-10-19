"""Straight test routes on the U-turn's field (lab2_straight_*). No ROS.

    lab2_make_straight LENGTH [--speed 0.8] [--name NAME] [--write] [--out DIR] [--force]
    just lab2 make-straight LENGTH [--speed V] [--name NAME] [--write]

A TA/testing route, not graded: from the start box, straight LENGTH metres
driving +x along the U-turn's first leg, at v_ref, an open path, one lap; the
path publisher's end ramp brings the car to rest on the last waypoint, as on the
U-turn. It stands on the SAME field as lab2_uturn: the same start-box frame
(origin at the start box, x along the first leg), the same boards, read from
lab2_uturn/course.yaml when this runs. So the U-turn's tape and boards, and an
aligned survey of its start box (an alignment has no route), serve unchanged.

Writes path.csv, course.yaml (generated header), lanelet2_map.osm,
ar_tag.param.yaml and map_projector_info.yaml into
config/courses/NAME (default lab2_straight_<LENGTH>m). Without --write it is a
dry run: the summary with the marker coverage, and a diff. A layout that brings
the car's nose within NOSE_GAP of a board, or loses the two-board cover, is
refused unless --force.
"""

import argparse
import difflib
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from autosdv_waypoint_lab import course as course_mod
from autosdv_waypoint_lab import uturn
from autosdv_waypoint_lab.path import CoursePath, Path

FIELD = uturn.COURSE_NAME       # the field (start box + boards) these routes stand on
V_REF = uturn.V_REF
SPACING = uturn.SPACING_LEG
CAR_FRONT = uturn.CAR_FRONT
CAR_REAR = uturn.CAR_REAR
NOSE_GAP = 1.0                  # m, board centre to the car's track (its nose, at the stop)
START_BOX_MIN = 2
COVER_MIN = 1.0                 # fraction of the path with a board in view (goal)
TWO_BOARD_WANT = 1.0            # ... with two or more (reported, a warning below it)


@dataclass
class Layout:
    name: str
    length: float
    v_ref: float
    xs: List[float]
    ys: List[float]
    boards: List[uturn.Board]
    marker_size: float
    dictionary: str
    marker_height: float
    start: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    start_box_min: int = 0
    fraction: float = 0.0
    two_fraction: float = 0.0
    longest_gap: float = 0.0
    seen_on: Dict[int, List[Tuple[float, float]]] = field(default_factory=dict)
    problems: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems

    @property
    def stop(self) -> Tuple[float, float]:
        return (self.xs[-1], self.ys[-1])


def default_name(length: float) -> str:
    return f'lab2_straight_{length:g}m'


def field_dir() -> str:
    return uturn.default_out_dir()


def default_out_dir(name: str) -> str:
    return os.path.join(os.path.dirname(field_dir()), name)


def make_layout(length: float, v_ref: float = V_REF, name: str = None) -> Layout:
    length, v_ref = float(length), float(v_ref)
    if length <= 0 or v_ref <= 0:
        raise ValueError('length and speed must be positive')
    n = max(1, int(math.ceil(length / SPACING - 1e-9)))
    xs = [round(length * k / n, 3) + 0.0 for k in range(n + 1)]
    ys = [0.0] * len(xs)
    fc = course_mod.load_course(field_dir())
    with open(os.path.join(field_dir(), 'course.yaml')) as f:
        import yaml
        height = float((yaml.safe_load(f) or {}).get('marker_height', 0.25))
    boards = [uturn.Board(m.id, m.x, m.y, m.yaw, origin=(i == 0)) for i, m in enumerate(fc.markers)]
    lay = Layout(name or default_name(length), length, v_ref, xs, ys, boards, fc.marker_size,
                 fc.dictionary, height)
    evaluate(lay)
    return lay


def to_course(lay: Layout, directory: str = '.') -> course_mod.Course:
    markers = [course_mod.Marker(b.id, b.x, b.y, lay.marker_height, b.yaw) for b in lay.boards]
    return course_mod.Course(directory, os.path.join(directory, 'path.csv'), False, 1, lay.v_ref,
                             course_mod.Pose2D(*lay.start), lay.marker_size, markers, lay.dictionary)


def evaluate(lay: Layout) -> None:
    cov, _turn, two, _gap_at, per = uturn.course_coverage(to_course(lay), (lay.xs, lay.ys))
    lay.start_box_min, lay.fraction, lay.longest_gap = cov.start_box_min, cov.fraction, cov.longest_gap
    lay.two_fraction = two
    lay.seen_on = {int(k): v for k, v in per.items()}
    lay.problems = []
    track = ([-CAR_REAR] + lay.xs + [lay.xs[-1] + CAR_FRONT], [0.0] * (len(lay.xs) + 2))
    d = uturn._point_path_distance([b.x for b in lay.boards], [b.y for b in lay.boards], *track)
    for b, di in zip(lay.boards, d):
        if di < NOSE_GAP - 1e-6:
            lay.problems.append(f'board {b.id} is {di:.2f} m from the car\'s track, under {NOSE_GAP} m')
    if lay.start_box_min < START_BOX_MIN:
        lay.problems.append(f'the start box sees {lay.start_box_min} board(s), {START_BOX_MIN} wanted')
    if lay.fraction < COVER_MIN - 1e-9:
        lay.problems.append(f'a board in view on {lay.fraction:.0%} of the path, {COVER_MIN:.0%} wanted')
    lay.warnings = []
    if lay.two_fraction < TWO_BOARD_WANT - 1e-9:
        lay.warnings.append(f'two or more boards in view on {lay.two_fraction:.0%} of the path, '
                            f'not all: one board alone on {_single_spans(lay)}')


def _single_spans(lay: Layout) -> str:
    """Where on the path (arc length) fewer than two boards are in view."""
    cam = uturn.CameraModel(marker_size=lay.marker_size)
    lms = [lm for v in course_mod.parse_osm_landmarks(course_mod.make_osm(to_course(lay))).values()
           for lm in v]
    cp = CoursePath(lay.xs, lay.ys, [lay.v_ref] * len(lay.xs), closed=False)
    path = Path(cp.x, cp.y, cp.v)
    spans: List[List[float]] = []
    prev = None
    for x, y, yaw, s in zip(path.x, path.y, path.yaw, path.s):
        n = sum(1 for lm in lms if uturn.visible(cam, x, y, yaw, lm))
        if n < 2:
            if spans and prev is not None and abs(spans[-1][1] - prev) < 1e-9:
                spans[-1][1] = s
            else:
                spans.append([s, s])
        prev = s
    return ', '.join(f's = {a:.2f}-{b:.2f} m' for a, b in spans) or 'nowhere'


def _command(lay: Layout) -> str:
    cmd = f'just lab2 make-straight {lay.length:g}'
    if abs(lay.v_ref - V_REF) > 1e-9:
        cmd += f' --speed {lay.v_ref:g}'
    if lay.name != default_name(lay.length):
        cmd += f' --name {lay.name}'
    return cmd + ' --write'


def _board_lines(lay: Layout, prefix: str) -> List[str]:
    out = []
    for b in lay.boards:
        seen = ', '.join(f'{a:.1f}-{c:.1f}' for a, c in lay.seen_on.get(b.id, [])) or 'not in view'
        out.append(f'{prefix}id {b.id}  ({b.x:6.2f}, {b.y:5.2f})  facing {uturn._facing(b.yaw):<8} '
                   f'in view at s = {seen} m')
    return out


def _coverage_line(lay: Layout) -> str:
    return (f'start box sees >= {lay.start_box_min}; a board in view on {lay.fraction:.0%}, '
            f'two or more on {lay.two_fraction:.0%}; longest gap {lay.longest_gap:.2f} m')


def path_csv(lay: Layout) -> str:
    out = [f'# GENERATED by lab2_make_straight (length {lay.length:g} m, field {FIELD}); do not edit.',
           '# x [m], y [m]   (no v column: every waypoint uses v_ref)']
    out += [f'{x:.3f},{y:.3f}' for x, y in zip(lay.xs, lay.ys)]
    return '\n'.join(out) + '\n'


def course_yaml(lay: Layout) -> str:
    ex, ey = lay.stop
    nearest = min(lay.boards, key=lambda b: math.hypot(b.x - ex, b.y - ey))
    h = [
        f'# Lab 2 TEST route {lay.name}: a straight, NOT graded (audit reports metrics only).',
        '#',
        f'# GENERATED by `{_command(lay)}`',
        '# (scripts/testing/lab2/make_straight.py, autosdv_waypoint_lab/straight.py);',
        '# do not edit this file, path.csv or the map by hand: run that again.',
        '#',
        f'# Route: from the start box, straight {lay.length:g} m driving +x along {FIELD}\'s',
        f'# first leg, one run at v_ref {lay.v_ref:g} m/s, open path; the path publisher\'s end',
        f'# ramp (decel {uturn.DECEL} m/s^2) brings the car to rest on the last waypoint,',
        f'# ({ex:.2f}, {ey:.2f}), {math.hypot(nearest.x - ex, nearest.y - ey):.2f} m short of board {nearest.id}.',
        '#',
        f'# Field: {FIELD}\'s, unchanged: its START-BOX frame (origin at the start box, rear',
        '# axle; x along the first leg; y to the car\'s left), its start box and its boards,',
        '# copied from its course.yaml when this was generated. Its tape and boards serve as',
        '# they stand, and so does an aligned survey of that start box (alignments have no',
        '# route; a run combines the survey with this path: surveys/<latest>.' + lay.name + '/).',
        '#',
        '# Boards (from the start box):',
    ]
    h += _board_lines(lay, '#   ')
    h += [f'# Coverage (marker_sim): {_coverage_line(lay)}.']
    h += [f'# Note: {w}.' for w in lay.warnings]
    body = [
        'path: path.csv',
        'closed: false',
        'laps: 1',
        f'v_ref: {lay.v_ref:g}',
        f'start_pose: {{x: {lay.start[0]:.3f}, y: {lay.start[1]:.3f}, yaw: {lay.start[2]:.4f}}}',
        f'marker_size: {lay.marker_size:g}',
        f'dictionary: {lay.dictionary}',
        f'marker_height: {lay.marker_height:g}',
        '# what make-straight generated this course from (the tests read it; the stack does not)',
        f'straight: {{length: {lay.length:g}}}',
        'markers:',
    ]
    body += [f'  - {{id: {b.id}, x: {b.x:.3f}, y: {b.y:.3f}, yaw: {b.yaw:.4f}}}' for b in lay.boards]
    return '\n'.join(h + body) + '\n'


def files(lay: Layout, directory: str) -> Dict[str, str]:
    texts = {'path.csv': path_csv(lay), 'course.yaml': course_yaml(lay)}
    c = to_course(lay, directory)
    texts['lanelet2_map.osm'] = course_mod.make_osm(c)
    texts['ar_tag.param.yaml'] = course_mod.make_ar_tag_params(c)
    texts['map_projector_info.yaml'] = 'projector_type: local\n'
    return texts


def summary(lay: Layout) -> str:
    ex, ey = lay.stop
    out = [f'{lay.name}: straight {lay.length:g} m at v_ref {lay.v_ref:g} m/s, 1 run, on the {FIELD} field '
           '(not graded)',
           '  start box     (0.00, 0.00) yaw 0 (driving +x; the frame origin)',
           f'  stop point    ({ex:.2f}, {ey:.2f})',
           f'  boards        {len(lay.boards)}, {FIELD}\'s']
    out += _board_lines(lay, '    ')
    out.append(f'  coverage      {_coverage_line(lay)}')
    return '\n'.join(out)


def main(argv=None):
    """lab2_make_straight: generate a straight test route on the U-turn's field."""
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('length', type=float, help='straight length [m]')
    ap.add_argument('--speed', type=float, default=V_REF, help='v_ref [m/s]')
    ap.add_argument('--name', default=None, help='course name (default lab2_straight_<LENGTH>m)')
    ap.add_argument('--out', default=None, help='course directory (default: the package\'s NAME)')
    ap.add_argument('--write', action='store_true', help='write the files (default: dry run)')
    ap.add_argument('--force', action='store_true', help='write even if a goal fails')
    args = ap.parse_args(argv)
    lay = make_layout(args.length, args.speed, args.name)
    out = args.out or default_out_dir(lay.name)
    print(summary(lay))
    for w in lay.warnings:
        print(f'  note          {w}')
    for p in lay.problems:
        print(f'!! GOAL MISSED: {p}')
    texts = files(lay, out)
    changed = []
    for name, text in texts.items():
        p = os.path.join(out, name)
        old = open(p).read() if os.path.exists(p) else None
        if old == text:
            continue
        changed.append(name)
        if not args.write:
            diff = list(difflib.unified_diff((old or '').splitlines(), text.splitlines(),
                                             f'a/{name}', f'b/{name}', lineterm='', n=1))
            print(f'\n{name}: {"new" if old is None else "changes"} ({len(diff)} diff lines)')
            print('\n'.join(diff[:40]) + ('\n  ...' if len(diff) > 40 else ''))
    if not changed:
        print(f'\n{out}: up to date')
        return 0 if lay.ok else 1
    if not args.write:
        print(f'\ndry run: {len(changed)} file(s) would change in {out}; add --write')
        return 0 if lay.ok else 1
    if not lay.ok and not args.force:
        print('\nREFUSED: the layout misses a goal (above); nothing written. --force writes it anyway.')
        return 1
    os.makedirs(out, exist_ok=True)
    for name in changed:
        with open(os.path.join(out, name), 'w') as f:
            f.write(texts[name])
        print(f'wrote {os.path.join(out, name)}')
    return 0 if lay.ok else 1


if __name__ == '__main__':
    sys.exit(main())
