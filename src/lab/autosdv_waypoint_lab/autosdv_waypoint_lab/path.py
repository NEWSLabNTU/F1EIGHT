"""Waypoint paths: loading, geometry and the speed profile. No ROS.

A path is a polyline of waypoints in the map frame. `Path` is what the student
tracker receives (the window of the course ahead of the car, as an open
polyline); `Course path` handling -- loading the TA's CSV, resampling, closing
the loop, counting laps -- lives in `CoursePath`, which only the path publisher
and the offline benches use.

Conventions: x forward/east, y left/north, yaw counter-clockwise from +x,
curvature positive turning left. Lengths in metres, speeds in m/s.
"""

from dataclasses import dataclass, field
import csv
import math
from typing import List, Optional, Sequence, Tuple


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Projection:
    """Where a point falls on a path."""

    index: int          # segment start index: the point lies on [index, index + 1]
    t: float            # fraction along that segment, 0..1
    s: float            # arc length of the foot point from the path's start
    x: float            # foot point
    y: float
    yaw: float          # path heading at the foot point
    cross_track: float  # signed distance, positive when the point is LEFT of the path


@dataclass
class Path:
    """An open polyline with per-point heading, speed, arc length and curvature.

    All arrays have the same length (>= 2). `v` is the TA's speed for that
    waypoint: the most the car may drive there.
    """

    x: List[float]
    y: List[float]
    v: List[float]
    yaw: List[float] = field(default_factory=list)
    s: List[float] = field(default_factory=list)
    kappa: List[float] = field(default_factory=list)

    def __post_init__(self):
        n = len(self.x)
        if n < 2 or len(self.y) != n or len(self.v) != n:
            raise ValueError(f'a path needs >= 2 points and equal-length x, y, v (got {n})')
        if not self.yaw:
            self.yaw = headings(self.x, self.y)
        if not self.s:
            self.s = arc_lengths(self.x, self.y)
        if not self.kappa:
            self.kappa = curvatures(self.x, self.y)

    def __len__(self):
        return len(self.x)

    @property
    def length(self) -> float:
        return self.s[-1]

    def nearest(self, x: float, y: float) -> int:
        """Index of the waypoint closest to (x, y)."""
        best, best_d = 0, math.inf
        for i, (px, py) in enumerate(zip(self.x, self.y)):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best, best_d = i, d
        return best

    def project(self, x: float, y: float) -> Projection:
        """Closest point on the polyline (not just the closest waypoint)."""
        best = None
        best_d = math.inf
        for i in range(len(self.x) - 1):
            x0, y0, x1, y1 = self.x[i], self.y[i], self.x[i + 1], self.y[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0.0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
            fx, fy = x0 + t * dx, y0 + t * dy
            d = (x - fx) ** 2 + (y - fy) ** 2
            if d < best_d:
                best_d, best = d, (i, t, fx, fy, dx, dy)
        i, t, fx, fy, dx, dy = best
        seg_yaw = math.atan2(dy, dx) if (dx or dy) else self.yaw[i]
        # left of the path is positive: z of (segment direction x offset)
        cross = math.cos(seg_yaw) * (y - fy) - math.sin(seg_yaw) * (x - fx)
        s = self.s[i] + t * (self.s[i + 1] - self.s[i])
        return Projection(i, t, s, fx, fy, seg_yaw, cross)

    def point_at(self, s: float) -> Tuple[float, float, float]:
        """(x, y, yaw) at arc length s, clamped to the path's ends."""
        if s <= 0.0:
            return self.x[0], self.y[0], self.yaw[0]
        if s >= self.s[-1]:
            return self.x[-1], self.y[-1], self.yaw[-1]
        lo, hi = 0, len(self.s) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.s[mid] <= s:
                lo = mid
            else:
                hi = mid
        ds = self.s[hi] - self.s[lo]
        t = 0.0 if ds == 0.0 else (s - self.s[lo]) / ds
        x = self.x[lo] + t * (self.x[hi] - self.x[lo])
        y = self.y[lo] + t * (self.y[hi] - self.y[lo])
        return x, y, math.atan2(self.y[hi] - self.y[lo], self.x[hi] - self.x[lo])

    def speed_at(self, s: float) -> float:
        """The waypoint speed at arc length s (linear between waypoints)."""
        if s <= 0.0:
            return self.v[0]
        if s >= self.s[-1]:
            return self.v[-1]
        for i in range(len(self.s) - 1):
            if self.s[i + 1] >= s:
                ds = self.s[i + 1] - self.s[i]
                t = 0.0 if ds == 0.0 else (s - self.s[i]) / ds
                return self.v[i] + t * (self.v[i + 1] - self.v[i])
        return self.v[-1]


# ---------------------------------------------------------------- geometry

def arc_lengths(x: Sequence[float], y: Sequence[float]) -> List[float]:
    s = [0.0]
    for i in range(1, len(x)):
        s.append(s[-1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]))
    return s


def headings(x: Sequence[float], y: Sequence[float]) -> List[float]:
    """Heading of the segment leaving each point (the last repeats the previous)."""
    n = len(x)
    yaw = []
    for i in range(n):
        j0, j1 = (i, i + 1) if i < n - 1 else (i - 1, i)
        yaw.append(math.atan2(y[j1] - y[j0], x[j1] - x[j0]))
    return yaw


def curvatures(x: Sequence[float], y: Sequence[float]) -> List[float]:
    """Signed curvature at each point from its neighbours (Menger curvature)."""
    n = len(x)
    k = [0.0] * n
    for i in range(1, n - 1):
        ax, ay = x[i - 1], y[i - 1]
        bx, by = x[i], y[i]
        cx, cy = x[i + 1], y[i + 1]
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        d = math.hypot(bx - ax, by - ay) * math.hypot(cx - bx, cy - by) * math.hypot(cx - ax, cy - ay)
        k[i] = 0.0 if d == 0.0 else 2.0 * cross / d
    if n > 2:
        k[0], k[-1] = k[1], k[-2]
    return k


def resample(x: Sequence[float], y: Sequence[float], v: Sequence[float],
             spacing: float) -> Tuple[List[float], List[float], List[float]]:
    """Points every `spacing` metres along the polyline; v interpolated."""
    s = arc_lengths(x, y)
    total = s[-1]
    if total == 0.0:
        raise ValueError('path has zero length')
    n = max(2, int(math.ceil(total / spacing)) + 1)
    out_x, out_y, out_v = [], [], []
    j = 0
    for k in range(n):
        sk = total * k / (n - 1)
        while j < len(s) - 2 and s[j + 1] < sk:
            j += 1
        ds = s[j + 1] - s[j]
        t = 0.0 if ds == 0.0 else (sk - s[j]) / ds
        out_x.append(x[j] + t * (x[j + 1] - x[j]))
        out_y.append(y[j] + t * (y[j + 1] - y[j]))
        out_v.append(v[j] + t * (v[j + 1] - v[j]))
    return out_x, out_y, out_v


# ---------------------------------------------------------------- the course

def load_csv(path: str, v_default: float) -> Tuple[List[float], List[float], List[float]]:
    """Read `x, y[, v]` rows. A header row is allowed; `#` lines are comments."""
    xs, ys, vs = [], [], []
    with open(path, newline='') as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                vals = [float(c) for c in row if c.strip() != '']
            except ValueError:
                if not xs:
                    continue    # header
                raise
            if len(vals) < 2:
                raise ValueError(f'{path}: row {row} has fewer than two numbers')
            xs.append(vals[0])
            ys.append(vals[1])
            vs.append(vals[2] if len(vals) > 2 else v_default)
    if len(xs) < 2:
        raise ValueError(f'{path}: fewer than two waypoints')
    return xs, ys, vs


class CoursePath:
    """The TA's path, resampled, with lap counting and the end-of-run stop.

    closed: the last waypoint joins the first. The course is driven `laps`
    times (closed) or once (open), and the speed ramps down with `decel` to
    stop exactly at the end of the run.
    """

    def __init__(self, x, y, v, closed: bool, spacing: float = 0.1,
                 v_max: float = 1.0, decel: float = 0.3, laps: int = 1):
        x, y, v = list(x), list(y), list(v)
        if closed and math.hypot(x[0] - x[-1], y[0] - y[-1]) > 1e-6:
            x.append(x[0])
            y.append(y[0])
            v.append(v[0])
        x, y, v = resample(x, y, v, spacing)
        v = [max(0.0, min(v_max, vi)) for vi in v]
        self.closed = closed
        self.laps = max(1, int(laps)) if closed else 1
        self.decel = decel
        if closed:
            # drop the duplicated closing point so index arithmetic wraps cleanly
            x, y, v = x[:-1], y[:-1], v[:-1]
        self.x, self.y, self.v = x, y, v
        self.n = len(x)
        self.lap_length = arc_lengths(x + ([x[0]] if closed else []),
                                      y + ([y[0]] if closed else []))[-1]
        self.spacing = self.lap_length / (self.n if closed else self.n - 1)
        self.run_length = self.lap_length * self.laps
        self.progress: Optional[float] = None   # unwrapped arc length driven

    # -- progress -------------------------------------------------------------
    def _local_projection(self, x, y, s_guess: Optional[float], search: float):
        """Project onto the course near s_guess (or anywhere when None)."""
        n = self.n
        if s_guess is None:
            idx = range(n if self.closed else n - 1)
        else:
            i0 = int(math.floor(s_guess / self.spacing))
            k = int(math.ceil(search / self.spacing))
            idx = [(i0 + d) for d in range(-k, k + 1)]
            if not self.closed:
                idx = [i for i in idx if 0 <= i < n - 1]
        best = None
        for i in idx:
            a = i % n if self.closed else i
            b = (a + 1) % n if self.closed else a + 1
            x0, y0, x1, y1 = self.x[a], self.y[a], self.x[b], self.y[b]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0.0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
            d2 = (x - x0 - t * dx) ** 2 + (y - y0 - t * dy) ** 2
            if best is None or d2 < best[0]:
                best = (d2, i, t)
        d2, i, t = best
        return (i + t) * self.spacing, math.sqrt(d2)

    def update_progress(self, x: float, y: float, search: float = 1.5) -> Tuple[float, float]:
        """Track the car along the course. Returns (progress, distance from path).

        Progress is unwrapped: on a closed course it grows by lap_length each
        lap, which is what counts laps. The search stays within `search`
        metres of the previous progress so a path that crosses itself, or a
        car near the start/finish, does not jump.
        """
        if self.progress is None:
            s, dist = self._local_projection(x, y, None, 0.0)
            # start of the run: the car sits at or just before the start point
            if self.closed and s > 0.5 * self.lap_length:
                s -= self.lap_length
            self.progress = s
            return self.progress, dist
        s, dist = self._local_projection(x, y, self.progress, search)
        if not self.closed:
            self.progress = max(self.progress, min(s, self.run_length))
        else:
            # s is unwrapped around the guess already (index i was unwrapped)
            if s > self.progress:
                self.progress = min(s, self.run_length)
        return self.progress, dist

    @property
    def laps_done(self) -> int:
        if self.progress is None:
            return 0
        return int(max(0.0, self.progress + 1e-6) // self.lap_length) if self.closed else (
            1 if self.progress >= self.run_length - 1e-6 else 0)

    @property
    def finished(self) -> bool:
        return self.progress is not None and self.progress >= self.run_length - 0.05

    # -- the window -------------------------------------------------------------
    def window(self, behind: float, ahead: float) -> Path:
        """The course from progress - behind to progress + ahead, as an open Path.

        Each point's speed is the waypoint speed, capped so the car can stop by
        the end of the run at `decel`: v <= sqrt(2 * decel * remaining).
        """
        p = self.progress if self.progress is not None else 0.0
        s0 = max(p - behind, 0.0 if not self.closed else -math.inf)
        s1 = min(p + ahead, self.run_length)
        if not self.closed:
            s0 = max(0.0, min(s0, self.lap_length - self.spacing))
        i0 = int(math.floor(s0 / self.spacing))
        i1 = int(math.ceil(s1 / self.spacing))
        if i1 <= i0:
            i1 = i0 + 1
        if not self.closed:
            i1 = min(i1, self.n - 1)
            i0 = min(i0, i1 - 1)
        xs, ys, vs = [], [], []
        for i in range(i0, i1 + 1):
            a = i % self.n if self.closed else i
            remaining = max(0.0, self.run_length - i * self.spacing)
            v_stop = math.sqrt(2.0 * self.decel * remaining) if self.decel > 0 else math.inf
            v = min(self.v[a], v_stop)
            if remaining < 0.05:
                v = 0.0
            xs.append(self.x[a])
            ys.append(self.y[a])
            vs.append(v)
        return Path(xs, ys, vs)
