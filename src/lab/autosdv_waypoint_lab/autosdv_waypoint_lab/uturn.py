"""The Lab 2 graded route, lab2_uturn, generated from its radius and leg length. No ROS.

    lab2_make_uturn [--radius 1.1] [--leg 2] [--write] [--out DIR] [--force]
    just lab2 make-uturn RADIUS [--leg 2] [--write]
    python3 scripts/testing/lab2/make_uturn.py --radius 1.1 --leg 2 [--write]

The route: from the start box, straight `leg` metres, a RIGHT U-turn of radius
R (the two legs 2R apart, centre to centre), straight `leg` metres back in the
other lane. One run at v_ref, an open path; the path publisher's end ramp
brings the car to rest on the last waypoint, level with the start box, which is
the graded stop point.

Everything else follows from (R, leg), and this module derives all of it:

    path.csv              the waypoints (0.25 m on the legs, about 0.1 m on the arc)
    course.yaml           start_pose, the boards, and a header whose numbers are
                          generated here (so it cannot go stale)
    lanelet2_map.osm      course.make_osm
    ar_tag.param.yaml     course.make_ar_tag_params
    map_projector_info.yaml

The boards come from a coverage search with marker_sim's camera (the bench's
detector model: 100 deg hfov, 5 m = ar_tag distance_threshold, 60 deg
incidence, 0.38 m marker). Candidates stand on a GRID-metre grid, facing every
YAW_STEP, at least CLEARANCE from the path. The goals, in order:

  1. every start-box placement (+-0.3 m, +-10 deg) sees >= START_BOX_MIN boards,
  2. no blind stretch longer than GAP_MAX (localization_monitor's error_age is
     5 s; GAP_MAX at v_ref leaves margin),
  3. a board in view through the whole U-turn,
  4. a car at rest near the stop line (+-0.3 m, +-10 deg) sees >= STOP_BOX_MIN
     board: the stop error is read from the localized pose after the stop, and
     without a fix there the EKF only dead-reckons (and the monitor's error_age
     runs out while the car stands),

with the fewest boards that achieve them. One board stands on the first leg's
line beyond the turn, facing the start box: the search anchors on it (it is
also a convenient origin for the stage-1 survey, though any board will do).

The frame is the START-BOX frame, the one `just aruco-map align` writes:
origin on the floor at the start box (base_link, the rear axle), x along the
first leg (the way the car drives off), y to the car's left, z up. The route is
taped on the floor and the boards are placed from the table this writes,
measured from the start box; the survey then measures where they really are,
and survey-align puts that map into this frame, so path.csv never moves.

Without --write nothing is written: the summary, the board table and a diff
against the files on disk are printed. With --write a layout that misses a goal
is refused unless --force.
"""

import argparse
import difflib
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from autosdv_waypoint_lab import course as course_mod
from autosdv_waypoint_lab.marker_sim import CameraModel, coverage, visible
from autosdv_waypoint_lab.path import CoursePath, Path, turn_spans

COURSE_NAME = 'lab2_uturn'
DEFAULT_RADIUS = 1.1
DEFAULT_LEG = 2.0
V_REF = 0.8
DECEL = 0.3                 # m/s^2, waypoint_path_publisher's end ramp (for the header)
SPACING_LEG = 0.25          # m between waypoints on the legs
SPACING_ARC = 0.10          # m (at most) between waypoints on the arc
MARKER_SIZE = 0.38
MARKER_HEIGHT = 0.25
DICTIONARY = 'DICT_5X5_1000'
FIRST_ID = 24               # placeholder ids; the survey writes the real ones

# board placement
CLEARANCE = 1.4             # m, board centre to the car's track: 1 m clear + car half width 0.15 + half plate 0.25
CAR_FRONT = 0.5             # m, rear axle to nose (the track runs on past the stop point by this)
CAR_REAR = 0.15             # m, rear axle to tail (and starts this far behind the start box)
SEPARATION = 1.0            # m, between two boards
GRID = 0.5                  # m
YAW_STEP = 15.0             # deg
START_BOX_MIN = 2
STOP_BOX_MIN = 1           # a car at rest on the stop line keeps a fix (the stop error is read from it)
GAP_MAX = 3.0               # m
ERROR_AGE = 5.0             # s, localization_monitor
MAX_BOARDS = 9
ORIGIN_EXTRA = 3.0          # m: the origin board is tried up to this far beyond its nearest legal spot
RESTARTS = 8               # per board count; a count that works stops after FEASIBLE_RUNS
FEASIBLE_RUNS = 2
ITERATIONS = 8000
SEED = 7


def default_out_dir() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'config', 'courses', COURSE_NAME))


# ------------------------------------------------------------ the route

def route_polyline(radius: float, leg: float) -> Tuple[List[float], List[float]]:
    """The waypoints in the TURN frame: turn centre at (0, radius), the out leg
    on y = 0 driving -x from (leg, 0), the return leg on y = 2 radius driving +x."""
    if radius <= 0 or leg <= 0:
        raise ValueError('radius and leg must be positive')
    n_leg = max(1, int(math.ceil(leg / SPACING_LEG - 1e-9)))
    n_arc = max(4, int(math.ceil(math.pi * radius / SPACING_ARC - 1e-9)))
    xs, ys = [], []
    for k in range(n_leg + 1):                       # out: (leg, 0) -> (0, 0)
        xs.append(leg * (1.0 - k / n_leg))
        ys.append(0.0)
    for j in range(1, n_arc):                        # clockwise about (0, R): a right turn
        th = -math.pi / 2.0 - math.pi * j / n_arc
        xs.append(radius * math.cos(th))
        ys.append(radius + radius * math.sin(th))
    for k in range(n_leg + 1):                       # back: (0, 2R) -> (leg, 2R)
        xs.append(leg * k / n_leg)
        ys.append(2.0 * radius)
    return xs, ys


def car_track(xs, ys) -> Tuple[List[float], List[float]]:
    """The path, run on by CAR_REAR behind the start and CAR_FRONT past the stop:
    where the car's body goes, to keep the boards clear of."""
    def ext(i, j, d):
        dx, dy = xs[i] - xs[j], ys[i] - ys[j]
        n = math.hypot(dx, dy) or 1.0
        return xs[i] + d * dx / n, ys[i] + d * dy / n
    (bx, by), (fx, fy) = ext(0, 1, CAR_REAR), ext(-1, -2, CAR_FRONT)
    return [bx] + list(xs) + [fx], [by] + list(ys) + [fy]


def _point_path_distance(px, py, xs, ys) -> np.ndarray:
    """Distance from each (px, py) to the polyline."""
    px = np.asarray(px, float)[:, None]
    py = np.asarray(py, float)[:, None]
    x0, y0 = np.asarray(xs[:-1])[None, :], np.asarray(ys[:-1])[None, :]
    dx, dy = np.asarray(xs[1:])[None, :] - x0, np.asarray(ys[1:])[None, :] - y0
    seg2 = np.maximum(dx * dx + dy * dy, 1e-12)
    t = np.clip(((px - x0) * dx + (py - y0) * dy) / seg2, 0.0, 1.0)
    return np.hypot(px - x0 - t * dx, py - y0 - t * dy).min(axis=1)


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _visibility(cam: CameraModel, bx, by, byaw, poses) -> np.ndarray:
    """marker_sim.visible, vectorised: [board, pose] -> detected."""
    bx, by, byaw = (np.asarray(a, float)[:, None] for a in (bx, by, byaw))
    px, py, ph = (np.asarray(a, float)[None, :] for a in zip(*poses))
    c, s = np.cos(ph), np.sin(ph)
    cx, cy = px + c * cam.x - s * cam.y, py + s * cam.x + c * cam.y
    dx, dy = bx - cx, by - cy
    rng = np.hypot(dx, dy)
    ok = (rng >= cam.range_min) & (rng <= cam.range_max)
    rx, ry = -np.sin(byaw), np.cos(byaw)            # the marker's x axis: viewer's right
    h = cam.marker_size / 2.0
    lim = cam.hfov / 2.0 - cam.fov_margin
    for sgn in (-1.0, 1.0):
        ex, ey = bx + sgn * h * rx - cx, by + sgn * h * ry - cy
        ok &= np.abs(_wrap(np.arctan2(ey, ex) - ph)) <= lim
    nx, ny = np.cos(byaw), np.sin(byaw)
    cos_inc = (-dx * nx - dy * ny) / np.maximum(rng, 1e-9)
    ok &= cos_inc >= math.cos(cam.max_incidence)
    return ok


def _start_box(start, box=0.3, box_yaw=math.radians(10.0)):
    """The placements marker_sim.coverage tries for the start box."""
    sx, sy, syaw = start
    c, s = math.cos(syaw), math.sin(syaw)
    out = []
    for dx in (-box, 0.0, box):
        for dy in (-box, 0.0, box):
            for dyaw in (-box_yaw, 0.0, box_yaw):
                out.append((sx + c * dx - s * dy, sy + s * dx + c * dy, syaw + dyaw))
    return out


# ------------------------------------------------------------ the search

@dataclass
class Board:
    id: int
    x: float
    y: float
    yaw: float          # facing direction, rad
    origin: bool = False    # the ANCHOR: the search's first board, on the first leg's line


@dataclass
class Layout:
    radius: float
    leg: float
    xs: List[float]                 # path, course (start-box) frame
    ys: List[float]
    boards: List[Board]
    start: Tuple[float, float, float]
    stop: Tuple[float, float]
    turn: Tuple[float, float]       # arc length of the turn on path.csv
    path_length: float
    # coverage, from marker_sim.coverage and marker_sim.visible
    start_box_min: int = 0
    stop_box_min: int = 0
    fraction: float = 0.0
    two_fraction: float = 0.0
    longest_gap: float = 0.0
    gap_at: Tuple[float, float] = (0.0, 0.0)
    turn_fraction: float = 0.0
    seen_on: Dict[int, List[Tuple[float, float]]] = field(default_factory=dict)
    fewer_failed: Optional[int] = None   # the board count the search tried and could not meet
    problems: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems

    @property
    def centre(self) -> Tuple[float, float]:
        return (self.leg, -self.radius)

    @property
    def apex(self) -> Tuple[float, float]:
        return (self.leg + self.radius, -self.radius)


class _Problem:
    """The candidate boards and what each sees, for one route geometry."""

    def __init__(self, radius: float, leg: float, cam: CameraModel):
        self.radius, self.leg, self.cam = radius, leg, cam
        xs, ys = route_polyline(radius, leg)
        self.xs, self.ys = xs, ys
        cp = CoursePath(xs, ys, [V_REF] * len(xs), closed=False)
        self.path = Path(cp.x, cp.y, cp.v)
        self.poses = list(zip(self.path.x, self.path.y, self.path.yaw))
        s = np.asarray(self.path.s)
        self.ds = np.concatenate([[0.0], np.diff(s)])
        (t0, t1), = turn_spans(xs, ys)
        self.turn_mask = (s >= t0 - 1e-6) & (s <= t1 + 1e-6)
        self.start = (leg, 0.0, math.pi)
        self.box = _start_box(self.start)
        self.stop_box = _start_box((leg, 2.0 * radius, 0.0))

        # the origin board: facing +x on the out leg's line, beyond the apex
        d_min = math.sqrt((radius + CLEARANCE) ** 2 - radius ** 2)
        d0 = math.ceil(d_min / GRID - 1e-9) * GRID
        origin = [(-d, 0.0, 0.0) for d in np.arange(d0, d0 + ORIGIN_EXTRA + 1e-9, GRID)]
        # the others: a grid around the route, every YAW_STEP
        gx = np.arange(math.floor((-radius - 5.5) / GRID) * GRID, leg + 5.5 + 1e-9, GRID)
        gy = np.arange(math.floor(-5.5 / GRID) * GRID, 2 * radius + 5.5 + 1e-9, GRID)
        X, Y = np.meshgrid(gx, gy)
        X, Y = X.ravel(), Y.ravel()
        dist = _point_path_distance(X, Y, *car_track(xs, ys))
        keep = dist >= CLEARANCE - 1e-9
        X, Y = X[keep], Y[keep]
        yaws = [math.radians(a) for a in np.arange(-180.0, 180.0, YAW_STEP)]
        others = [(x, y, a) for a in yaws for x, y in zip(X, Y)]
        cands = origin + others
        bx, by, byaw = zip(*cands)
        V = _visibility(cam, bx, by, byaw, self.poses)
        B = _visibility(cam, bx, by, byaw, self.box)
        S = _visibility(cam, bx, by, byaw, self.stop_box)
        useful = V.any(axis=1) | B.any(axis=1) | S.any(axis=1)
        useful[:len(origin)] = True
        idx = np.nonzero(useful)[0]
        self.cands = [cands[i] for i in idx]
        self.V, self.B, self.S = V[idx], B[idx], S[idx]
        self.n_origin = len(origin)
        self.key = {(round(x / GRID), round(y / GRID), round(math.degrees(a) / YAW_STEP) % int(360 / YAW_STEP)): i
                    for i, (x, y, a) in enumerate(self.cands) if i >= self.n_origin}

    def stats(self, sel):
        cnt = self.V[sel].sum(axis=0)
        unc = cnt == 0
        csum = np.cumsum(self.ds * unc)
        last = np.maximum.accumulate(np.where(unc, 0.0, csum))
        gap = float((csum - last).max())
        sb = int(self.B[sel].sum(axis=0).min())
        sb_def = int(np.maximum(0, START_BOX_MIN - self.B[sel].sum(axis=0)).sum())
        turn_unc = float((self.ds * (unc & self.turn_mask)).sum())
        two = float((cnt >= 2).mean())
        sb_def += int(np.maximum(0, STOP_BOX_MIN - self.S[sel].sum(axis=0)).sum())
        return gap, sb, sb_def, turn_unc, two

    def cost(self, sel):
        gap, sb, sb_def, turn_unc, two = self.stats(sel)
        hard = sb_def + 5.0 * turn_unc + 5.0 * max(0.0, gap - GAP_MAX)
        # ... then the shortest gap, more double cover, and the origin board (so the
        # whole route) as near the turn as it can stand
        soft = 0.003 * gap - 0.003 * two - 0.002 * min(sb, 3) - 0.02 * self.cands[sel[0]][0]
        return hard * 100.0 + soft, hard == 0.0

    def neighbour(self, i, rng):
        x, y, a = self.cands[i]
        k = (round(x / GRID), round(y / GRID), round(math.degrees(a) / YAW_STEP) % int(360 / YAW_STEP))
        for _ in range(8):
            d = rng.integers(0, 3)
            step = rng.choice([-1, 1])
            kk = list(k)
            if d == 2:
                kk[2] = (kk[2] + step) % int(360 / YAW_STEP)
            else:
                kk[d] += step
            j = self.key.get((kk[0], kk[1], kk[2]))
            if j is not None:
                return j
        return None

    def separated(self, sel):
        pts = [self.cands[i][:2] for i in sel]
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                if math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) < SEPARATION - 1e-9:
                    return False
        return True

    def search(self, k, rng):
        """Simulated annealing over k boards, the first the origin. (cost, feasible, selection)."""
        n = len(self.cands)
        best = None
        feasible_runs = 0
        for _ in range(RESTARTS):
            if feasible_runs >= FEASIBLE_RUNS:
                break
            while True:
                sel = [int(rng.integers(0, self.n_origin))] + \
                      [int(rng.integers(self.n_origin, n)) for _ in range(k - 1)]
                if self.separated(sel):
                    break
            cur, feas = self.cost(sel)
            run_best = (cur, feas, list(sel))
            for it in range(ITERATIONS):
                temp = 200.0 * (0.0005 / 200.0) ** (it / ITERATIONS)
                slot = int(rng.integers(0, k))
                trial = list(sel)
                if slot == 0:
                    trial[0] = int(rng.integers(0, self.n_origin))
                elif rng.random() < 0.6:
                    j = self.neighbour(sel[slot], rng)
                    if j is None:
                        continue
                    trial[slot] = j
                else:
                    trial[slot] = int(rng.integers(self.n_origin, n))
                if not self.separated(trial):
                    continue
                c, f = self.cost(trial)
                if c <= cur or rng.random() < math.exp(-(c - cur) / temp):
                    sel, cur, feas = trial, c, f
                    if c < run_best[0]:
                        run_best = (c, f, list(sel))
            feasible_runs += bool(run_best[1])
            if best is None or run_best[0] < best[0]:
                best = run_best
        assert best is not None     # RESTARTS >= 1
        return best


def place_boards(radius: float, leg: float, seed: int = SEED, cam: Optional[CameraModel] = None):
    """(boards in the turn frame as (x, y, yaw) with the origin first, the
    count below it that failed or None)."""
    cam = cam or CameraModel(marker_size=MARKER_SIZE)
    prob = _Problem(radius, leg, cam)
    rng = np.random.default_rng(seed)
    failed = None
    found: List[int] = []
    for k in range(START_BOX_MIN + STOP_BOX_MIN + 1, MAX_BOARDS + 1):
        cost, feas, sel = prob.search(k, rng)
        if feas:
            found = sel
            break
        failed = k
        found = sel
    return [prob.cands[i] for i in found], failed


# ------------------------------------------------------------ the layout

def turn_to_course(x: float, y: float, leg: float) -> Tuple[float, float]:
    """A turn-frame point in the course (start-box) frame: the turn frame turned
    through 180 degrees about the start box (leg, 0), which the car leaves driving
    -x there. A rotation, not a mirror: a right turn stays a right turn."""
    return round(leg - x, 3) + 0.0, round(-y, 3) + 0.0


def _wrap_yaw(a: float) -> float:
    a = math.atan2(math.sin(a), math.cos(a))
    return math.pi if abs(a + math.pi) < 1e-9 else a


def make_layout(radius: float = DEFAULT_RADIUS, leg: float = DEFAULT_LEG, seed: int = SEED) -> Layout:
    radius, leg = float(radius), float(leg)
    boards_t, failed = place_boards(radius, leg, seed)
    xs_t, ys_t = route_polyline(radius, leg)
    xs, ys = (list(v) for v in zip(*(turn_to_course(x, y, leg) for x, y in zip(xs_t, ys_t))))
    # ids: the origin first, the rest in the order the car first sees them
    cam = CameraModel(marker_size=MARKER_SIZE)
    cp = CoursePath(xs, ys, [V_REF] * len(xs), closed=False)
    path = Path(cp.x, cp.y, cp.v)
    start = (xs[0], ys[0], 0.0)
    placed = [(*turn_to_course(x, y, leg), round(_wrap_yaw(a + math.pi), 4)) for x, y, a in boards_t]

    def landmark(x, y, a):
        return course_mod.Landmark('b', (x, y, MARKER_HEIGHT), (-math.sin(a), math.cos(a), 0.0),
                                   (0.0, 0.0, 1.0), (math.cos(a), math.sin(a), 0.0))

    box = _start_box(start)

    def first_seen(b):
        lm = landmark(*b)
        if any(visible(cam, p[0], p[1], p[2], lm) for p in box):
            return -1
        return next((i for i, p in enumerate(zip(path.x, path.y, path.yaw)) if visible(cam, *p, lm)),
                    len(path.x))
    rest = sorted(placed[1:], key=first_seen)
    boards = [Board(FIRST_ID, *placed[0], origin=True)] + \
        [Board(FIRST_ID + 1 + i, *b) for i, b in enumerate(rest)]
    (t0, t1), = turn_spans(xs, ys)
    lay = Layout(radius, leg, xs, ys, boards, start, (xs[-1], ys[-1]), (t0, t1),
                 Path(xs, ys, [V_REF] * len(xs)).length, fewer_failed=failed)
    evaluate(lay)
    return lay


def to_course(lay: Layout, directory: str = '.') -> course_mod.Course:
    markers = [course_mod.Marker(b.id, b.x, b.y, MARKER_HEIGHT, b.yaw) for b in lay.boards]
    return course_mod.Course(directory, os.path.join(directory, 'path.csv'), False, 1, V_REF,
                             course_mod.Pose2D(*lay.start), MARKER_SIZE, markers, DICTIONARY)


def course_coverage(course: course_mod.Course, xy=None):
    """(marker_sim.Coverage, turn fraction, two-board fraction, gap interval,
    per-marker arc-length intervals) for a course: its path.csv, or the
    waypoints `xy` = (xs, ys) of one not written yet."""
    if xy is None:
        from autosdv_waypoint_lab.path import load_csv
        xs, ys, vs = load_csv(course.path_file, course.v_ref)
    else:
        xs, ys = list(xy[0]), list(xy[1])
        vs = [course.v_ref] * len(xs)
    cp = CoursePath(xs, ys, vs, closed=course.closed)
    path = Path(cp.x, cp.y, cp.v)
    landmarks = course_mod.parse_osm_landmarks(course_mod.make_osm(course))
    cam = CameraModel(marker_size=course.marker_size)
    sp = course.start_pose
    cov = coverage(path, landmarks, (sp.x, sp.y, sp.yaw), cam)
    spans = turn_spans(xs, ys)
    flat = [lm for lms in landmarks.values() for lm in lms]
    seen = []
    per = {lm.id: [] for lm in flat}
    for i in range(len(path.x)):
        ids = [lm.id for lm in flat if visible(cam, path.x[i], path.y[i], path.yaw[i], lm)]
        seen.append(len(ids))
        for mid in ids:
            iv = per[mid]
            if iv and abs(iv[-1][1] - path.s[i - 1]) < 1e-9:
                iv[-1] = (iv[-1][0], path.s[i])
            else:
                iv.append((path.s[i], path.s[i]))
    in_turn = [i for i, s in enumerate(path.s) if any(a - 1e-6 <= s <= b + 1e-6 for a, b in spans)]
    turn_frac = (sum(1 for i in in_turn if seen[i]) / len(in_turn)) if in_turn else math.nan
    two = sum(1 for n in seen if n >= 2) / len(seen)
    # where the longest gap is
    best, cur0, gap, gap_at = 0.0, None, 0.0, (0.0, 0.0)
    for i, n in enumerate(seen):
        if n:
            gap, cur0 = 0.0, None
        else:
            if cur0 is None:
                cur0 = path.s[i - 1] if i else 0.0
            gap += (path.s[i] - path.s[i - 1]) if i else 0.0
            if gap > best:
                best, gap_at = gap, (cur0, path.s[i])
    return cov, turn_frac, two, gap_at, per


def evaluate(lay: Layout) -> None:
    c = to_course(lay)
    cov, turn_frac, two, gap_at, per = course_coverage(c, (lay.xs, lay.ys))
    lay.start_box_min, lay.fraction, lay.longest_gap = cov.start_box_min, cov.fraction, cov.longest_gap
    lay.turn_fraction, lay.two_fraction, lay.gap_at = turn_frac, two, gap_at
    lay.seen_on = {int(k): v for k, v in per.items()}
    cam = CameraModel(marker_size=MARKER_SIZE)
    lms = [lm for v in course_mod.parse_osm_landmarks(course_mod.make_osm(c)).values() for lm in v]
    lay.stop_box_min = min(sum(1 for lm in lms if visible(cam, p[0], p[1], p[2], lm))
                           for p in _start_box((lay.stop[0], lay.stop[1], math.pi)))
    lay.problems = []
    if lay.start_box_min < START_BOX_MIN:
        lay.problems.append(f'the start box sees {lay.start_box_min} board(s), {START_BOX_MIN} wanted')
    if lay.longest_gap > GAP_MAX + 1e-6:
        lay.problems.append(f'a blind stretch of {lay.longest_gap:.2f} m, over {GAP_MAX} m')
    if lay.stop_box_min < STOP_BOX_MIN:
        lay.problems.append(f'a car at rest near the stop line may see {lay.stop_box_min} boards, '
                            f'{STOP_BOX_MIN} wanted')
    if not lay.turn_fraction >= 1.0:
        lay.problems.append(f'the turn has a board in view on {lay.turn_fraction:.0%} of it, not all')
    d = _point_path_distance([b.x for b in lay.boards], [b.y for b in lay.boards], *car_track(lay.xs, lay.ys))
    for b, di in zip(lay.boards, d):
        if di < CLEARANCE - 1e-6:
            lay.problems.append(f'board {b.id} is {di:.2f} m from the car\'s track, under {CLEARANCE} m')
    for i, a in enumerate(lay.boards):
        for b in lay.boards[i + 1:]:
            if math.hypot(a.x - b.x, a.y - b.y) < SEPARATION - 1e-6:
                lay.problems.append(f'boards {a.id} and {b.id} are under {SEPARATION} m apart')


# ------------------------------------------------------------ the files

def _deg(a):
    return f'{math.degrees(a):+.0f} deg'


def _facing(a):
    names = {0: '+x', 90: '+y', 180: '-x', -180: '-x', -90: '-y'}
    a = _wrap_yaw(a)
    d = round(math.degrees(a))
    return names.get(d, _deg(a))


def path_csv(lay: Layout) -> str:
    out = ['# GENERATED by lab2_make_uturn (radius %.3g m, leg %.3g m); do not edit.' % (lay.radius, lay.leg),
           '# x [m], y [m]   (no v column: every waypoint uses v_ref)']
    out += [f'{x:.3f},{y:.3f}' for x, y in zip(lay.xs, lay.ys)]
    return '\n'.join(out) + '\n'


def _command(lay: Layout) -> str:
    return f'just lab2 make-uturn {lay.radius:g} --leg {lay.leg:g} --write'


def board_lines(lay: Layout, prefix: str = '') -> List[str]:
    out = []
    for b in lay.boards:
        seen = ', '.join(f'{a:.1f}-{c:.1f}' for a, c in lay.seen_on.get(b.id, [])) or 'start box only'
        role = 'ANCHOR' if b.origin else ''
        out.append(f'{prefix}id {b.id}  ({b.x:6.2f}, {b.y:5.2f})  facing {_facing(b.yaw):<8} '
                   f'{role:<6}  in view at s = {seen} m')
    return out


def floor_extent(lay: Layout):
    xs = lay.xs + [b.x for b in lay.boards]
    ys = lay.ys + [b.y for b in lay.boards]
    return min(xs) - 0.5, max(xs) + 0.5, min(ys) - 0.5, max(ys) + 0.5


def course_yaml(lay: Layout) -> str:
    r, L = lay.radius, lay.leg
    (cx, cy), (ax, ay) = lay.centre, lay.apex
    sx, sy, _ = lay.start
    ex, ey = lay.stop
    x0, x1, y0, y1 = floor_extent(lay)
    n = len(lay.boards)
    anchor = lay.boards[0]
    why_n = (f'The search found no layout of {lay.fewer_failed} that meets all four'
             if lay.fewer_failed else f'{n} is the fewest the search tries')
    gap_s = lay.longest_gap / V_REF
    h = [
        '# Lab 2 graded route: the U-TURN (one bag, 45 of the team\'s 60 points: speed',
        '# error on the straight legs and in the turn, overshoot, stop error).',
        '#',
        f'# GENERATED by `{_command(lay)}`',
        '# (scripts/testing/lab2/make_uturn.py, autosdv_waypoint_lab/uturn.py). To change',
        '# the radius or the legs, run that again with the new numbers; do not edit',
        '# this file, path.csv or the map by hand.',
        '#',
        f'# Route: from the start box, straight {L:g} m driving +x, a RIGHT U-turn of',
        f'# radius {r:g} m, straight {L:g} m back driving -x in the other lane; the legs',
        f'# are {2 * r:g} m apart, centre to centre. One run at v_ref {V_REF} m/s, open',
        f'# path; the path publisher\'s end ramp (decel {DECEL} m/s^2) brings the car to',
        '# rest on the last waypoint, which is the graded stop point. Path length',
        f'# {lay.path_length:.2f} m; the turn is s = {lay.turn[0]:.2f} to {lay.turn[1]:.2f} m along it',
        '# (the grader splits straight from turn by arc length, read from path.csv).',
        '#',
        '# Frame: the START-BOX frame `just aruco-map align` writes -- origin on the',
        '# floor at the start box (base_link: the rear axle centre), x along the first',
        '# leg (the way the car drives off), y to the car\'s left, z up. A right turn',
        '# from +x goes to -y, so the second leg runs at y = -2R.',
        '#',
        '# Tape marks (rear axle for the car):',
        f'#   start box    ({sx:.2f}, {sy:.2f}), car pointing +x along the first leg',
        f'#   turn centre  ({cx:.2f}, {cy:.2f}); apex ({ax:.2f}, {ay:.2f})',
        f'#   stop line    ({ex:.2f}, {ey:.2f}): level with the start box, in the other lane',
        f'# Floor: x {x0:.1f} to {x1:.1f}, y {y0:.1f} to {y1:.1f} (route and boards), clear.',
        '#',
        '# Boards (DICT_5X5_1000, 0.38 m black square on a 500 mm plate standing on',
        f'# the floor, so the marker centre is 0.25 m up), measured from the start box. {n} boards:',
    ]
    h += board_lines(lay, '#   ')
    h += [
        f'# Why {n}: a coverage search (marker_sim.visible: hfov 100 deg, range 5 m =',
        '# ar_tag distance_threshold, 60 deg incidence limit, 0.38 m marker) over',
        f'# boards on a {GRID} m grid facing every {YAW_STEP:g} deg, each at least {CLEARANCE} m from',
        '# the car\'s track (1 m clear of the car), for the fewest that give: the start box',
        f'# (+-0.3 m, +-10 deg) sees >= {START_BOX_MIN} boards; no blind stretch over {GAP_MAX:g} m;',
        '# a board in view through the whole turn; a car at rest near the stop line',
        f'# (+-0.3 m, +-10 deg) sees >= {STOP_BOX_MIN}, so the stop is measured on a fix.',
        f'# {why_n}.',
        f'# Result: the start box sees >= {lay.start_box_min} boards, the stop >= {lay.stop_box_min}; a board in view on',
        f'# {lay.fraction:.0%} of the path (two or more on {lay.two_fraction:.0%}); the turn covered on',
        f'# {lay.turn_fraction:.0%}; the longest blind stretch {lay.longest_gap:.2f} m (s = {lay.gap_at[0]:.1f}',
        f'# to {lay.gap_at[1]:.1f} m), {gap_s:.1f} s at {V_REF} m/s, under localization_monitor\'s',
        f'# {ERROR_AGE:g} s error_age.',
        '#',
        '# Setting up (docs/guides/lab2-waypoint-stack.md, "On the field"): tape the',
        '# route, place the boards by the table above, measured from the start box (a',
        '# few cm off is fine: the survey measures them), survey them with any board as',
        f'# the origin (`just aruco-map scan ID`; {anchor.id}, the ANCHOR, faces the first leg),',
        '# then `just aruco-map align` with the car in the start box, driven straight',
        '# along the taped first leg. That writes the measured boards in THIS frame beside',
        '# this path.csv and start_pose, so the route lies where the tape is. The ids are',
        f'# placeholders ({FIRST_ID} is a real board id); the survey writes the ids the',
        '# boards actually carry.',
    ]
    body = [
        'path: path.csv',
        'closed: false',
        'laps: 1',
        f'v_ref: {V_REF}',
        f'start_pose: {{x: {sx:.3f}, y: {sy:.3f}, yaw: {lay.start[2]:.4f}}}',
        f'marker_size: {MARKER_SIZE}',
        f'dictionary: {DICTIONARY}',
        '# 500 mm plate standing on the floor, the 0.38 m black square centred on it',
        f'marker_height: {MARKER_HEIGHT}',
        '# what make-uturn generated this course from (the tests read it; the stack does not)',
        f'uturn: {{radius: {r:g}, leg: {L:g}}}',
        'markers:',
    ]
    body += [f'  - {{id: {b.id}, x: {b.x:.3f}, y: {b.y:.3f}, yaw: {b.yaw:.4f}}}' for b in lay.boards]
    return '\n'.join(h + body) + '\n'


def files(lay: Layout, directory: str) -> Dict[str, str]:
    """{file name: text} of the whole course directory."""
    texts = {'path.csv': path_csv(lay), 'course.yaml': course_yaml(lay)}
    c = to_course(lay, directory)
    texts['lanelet2_map.osm'] = course_mod.make_osm(c)
    texts['ar_tag.param.yaml'] = course_mod.make_ar_tag_params(c)
    texts['map_projector_info.yaml'] = 'projector_type: local\n'
    return texts


def summary(lay: Layout) -> str:
    sx, sy, _ = lay.start
    ex, ey = lay.stop
    (cx, cy), (ax, ay) = lay.centre, lay.apex
    out = [
        f'lab2_uturn: radius {lay.radius:g} m, legs {lay.leg:g} m, {2 * lay.radius:g} m apart, v_ref {V_REF} m/s, 1 run',
        f'  start box     ({sx:.2f}, {sy:.2f}) yaw 0 (driving +x; the frame origin)',
        f'  turn          centre ({cx:.2f}, {cy:.2f}), apex ({ax:.2f}, {ay:.2f}), right turn',
        f'  stop point    ({ex:.2f}, {ey:.2f}) (driving -x)',
        f'  path length   {lay.path_length:.2f} m; turn at s = {lay.turn[0]:.2f} .. {lay.turn[1]:.2f} m',
        f'  boards        {len(lay.boards)} (anchor id {lay.boards[0].id} on the first leg\'s line, '
        'facing the start box)',
    ]
    out += board_lines(lay, '    ')
    out += [
        f'  coverage      start box sees >= {lay.start_box_min}; stop box >= {lay.stop_box_min}; in view on {lay.fraction:.0%} '
        f'(>= 2 on {lay.two_fraction:.0%}); longest gap {lay.longest_gap:.2f} m = '
        f'{lay.longest_gap / V_REF:.1f} s at s = {lay.gap_at[0]:.1f}..{lay.gap_at[1]:.1f}; '
        f'turn covered {lay.turn_fraction:.0%}',
        f'  fewest        {len(lay.boards)}' + (f' (no layout of {lay.fewer_failed} found)' if lay.fewer_failed else ''),
    ]
    return '\n'.join(out)


def spec_block(lay: Layout) -> str:
    sx, sy, _ = lay.start
    ex, ey = lay.stop
    (cx, cy), (ax, ay) = lay.centre, lay.apex
    rows = ['| id | x [m] | y [m] | faces | role |', '|---|---|---|---|---|']
    for b in lay.boards:
        deg = math.degrees(b.yaw)
        deg = 180.0 if abs(deg + 180.0) < 0.5 else deg
        rows.append(f'| {b.id} | {b.x:.2f} | {b.y:.2f} | {deg:.0f} deg | '
                    f'{"anchor" if b.origin else ""} |')
    return '\n'.join([
        f'Generated with radius {lay.radius:g} m, leg {lay.leg:g} m ({_command(lay)}).',
        f'Route lab2_uturn: straight {lay.leg:g} m, right U-turn R = {lay.radius:g} m '
        f'(legs {2 * lay.radius:g} m apart, centre to centre), straight {lay.leg:g} m back; '
        f'path length {lay.path_length:.2f} m, one run at {V_REF} m/s.',
        'Course frame: origin at the start box (rear axle), x along the first leg, y to the car\'s left.',
        f'Start box (rear axle): ({sx:.2f}, {sy:.2f}), car facing +x. '
        f'Stop line (rear axle): ({ex:.2f}, {ey:.2f}). '
        f'Turn centre ({cx:.2f}, {cy:.2f}), apex ({ax:.2f}, {ay:.2f}).',
        f'Boards: {len(lay.boards)} (0.38 m DICT_5X5_1000 on 500 mm floor plates; ids are placeholders):',
        *rows,
    ])


def spec_tex(lay: Layout) -> str:
    """The route's geometry as LaTeX macros, for the Lab 2 spec to \\input.

    The spec draws its map and states its numbers from these, so a new radius or
    leg length is: regenerate with --tex, rebuild the PDF.
    """
    sx, sy, _ = lay.start
    ex, ey = lay.stop
    cx, cy = lay.centre

    def num(v):
        return f'{v:.2f}'.rstrip('0').rstrip('.')

    boards = ', '.join(
        f'{num(b.x)}/{num(b.y)}/{num(math.degrees(b.yaw))}/{b.id}/{int(b.origin)}'
        for b in lay.boards)
    lines = [
        f'% Generated by `{_command(lay)} --tex FILE` (AutoSDV autosdv_waypoint_lab/uturn.py).',
        '% Do not edit by hand: change the radius or leg length there and regenerate.',
        f'\\def\\uturnR{{{num(lay.radius)}}}%           U-turn radius [m]',
        f'\\def\\uturnLeg{{{num(lay.leg)}}}%         each straight leg [m]',
        f'\\def\\uturnSpacing{{{num(2 * lay.radius)}}}%     the legs, centre to centre [m]',
        f'\\def\\uturnLength{{{num(lay.path_length)}}}%      path length [m]',
        f'\\def\\uturnStartX{{{num(sx)}}}\\def\\uturnStartY{{{num(sy)}}}%  start box, rear axle',
        f'\\def\\uturnStopX{{{num(ex)}}}\\def\\uturnStopY{{{num(ey)}}}%   stop line, rear axle',
        f'\\def\\uturnCx{{{num(cx)}}}\\def\\uturnCy{{{num(cy)}}}%       turn centre',
        f'\\def\\uturnBoardCount{{{len(lay.boards)}}}',
        f'\\def\\uturnOriginId{{{lay.boards[0].id}}}',
        f'\\def\\uturnStartBoards{{{lay.start_box_min}}}%  fewest boards seen from the start box',
        f'\\def\\uturnStopBoards{{{lay.stop_box_min}}}%   fewest boards seen at the stop line',
        f'\\def\\uturnCovered{{{lay.fraction * 100:.0f}}}%    percent of the path with a board in view',
        f'\\def\\uturnGapLongest{{{lay.longest_gap:.1f}}}% longest stretch with no board [m]',
        f'\\def\\uturnGapSeconds{{{lay.longest_gap / V_REF:.1f}}}% ... at the reference speed [s]',
        '% x/y/facing in degrees/id/is-anchor, for \\foreach. The start-box frame: origin',
        '% at the start box (rear axle), x along the first leg, y to the car\'s left.',
        f'\\def\\uturnBoards{{{boards}}}',
    ]
    return '\n'.join(lines) + '\n'


def main(argv=None):
    """lab2_make_uturn: regenerate the lab2_uturn course from its radius and leg length."""
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('radius', nargs='?', type=float, default=None, help='U-turn radius [m]')
    ap.add_argument('--radius', dest='radius_opt', type=float, default=None)
    ap.add_argument('--leg', type=float, default=DEFAULT_LEG, help='each straight leg [m]')
    ap.add_argument('--out', default=None, help=f'course directory (default: the package\'s {COURSE_NAME})')
    ap.add_argument('--write', action='store_true', help='write the files (default: dry run)')
    ap.add_argument('--force', action='store_true', help='write even if a coverage goal fails')
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--tex', default=None,
                    help='also write the geometry as LaTeX macros to this file (the Lab 2 spec inputs it)')
    args = ap.parse_args(argv)
    radius = args.radius_opt if args.radius_opt is not None else args.radius
    radius = DEFAULT_RADIUS if radius is None else radius
    out = args.out or default_out_dir()
    print(f'searching board layouts for R = {radius:g} m, legs {args.leg:g} m ...', flush=True)
    lay = make_layout(radius, args.leg, args.seed)
    print(summary(lay))
    if lay.problems:
        print('\n' + '!' * 72)
        for p in lay.problems:
            print(f'!! GOAL MISSED: {p}')
        print('!' * 72)
    print('\n--- for the spec ---\n' + spec_block(lay) + '\n---')
    if args.tex:
        if lay.ok or args.force:
            with open(args.tex, 'w') as f:
                f.write(spec_tex(lay))
            print(f'wrote {args.tex}')
        else:
            print(f'not writing {args.tex}: the layout misses a goal (--force writes it anyway)')
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
            print('\n'.join(diff[:60]) + ('\n  ...' if len(diff) > 60 else ''))
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
