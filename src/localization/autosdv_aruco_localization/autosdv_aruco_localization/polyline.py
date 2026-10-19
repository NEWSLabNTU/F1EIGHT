"""A reference path to compare a localization trail with. No ROS.

A marker map may carry a path (course.yaml `path:`, default path.csv; `closed`):
a Lab 2 route does, a bare survey does not. loc_check draws it and reports how
far the trail runs from it. The CSV is `x, y[, v]` rows; a header row and `#`
comment lines are allowed, and any column after y is ignored.
"""

import csv
import math
import os
from typing import List, Optional, Tuple

import yaml


def load_xy(path: str) -> Tuple[List[float], List[float]]:
    xs, ys = [], []
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
    if len(xs) < 2:
        raise ValueError(f'{path}: fewer than two waypoints')
    return xs, ys


class Polyline:
    def __init__(self, xs, ys, closed: bool = False):
        xs, ys = list(xs), list(ys)
        if closed and math.hypot(xs[0] - xs[-1], ys[0] - ys[-1]) > 1e-9:
            xs.append(xs[0])
            ys.append(ys[0])
        self.x, self.y = xs, ys

    def distance(self, px: float, py: float) -> float:
        """The shortest distance from (px, py) to the polyline."""
        best = math.inf
        for i in range(len(self.x) - 1):
            ax, ay, bx, by = self.x[i], self.y[i], self.x[i + 1], self.y[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
        return best


def map_path(directory: str) -> Tuple[Optional[Polyline], str]:
    """(the map's path or None, the path file it names)."""
    with open(os.path.join(directory, 'course.yaml')) as f:
        d = yaml.safe_load(f) or {}
    path_file = os.path.join(directory, str(d.get('path', 'path.csv')))
    if not os.path.exists(path_file):
        return None, path_file
    xs, ys = load_xy(path_file)
    return Polyline(xs, ys, closed=bool(d.get('closed', True))), path_file
