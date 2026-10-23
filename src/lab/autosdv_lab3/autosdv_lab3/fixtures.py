"""A synthetic classroom map folder, for the lanelet check's tests and the simulator lap.

    python3 -m autosdv_lab3.fixtures OUT [--variant good|ccw|gap|desk|narrow|tight]

Writes OUT/pointcloud_map.pcd (x y z intensity: the walls of a 8 x 12 m room, a desk
island in the middle and desks along the walls, a sparse floor, the board on the
x = 0 wall), OUT/map_projector_info.yaml (projector_type: Local) and
OUT/lanelet2_map.osm in Vector Map Builder's local export: every node carries
local_x / local_y / ele tags (lat/lon are placeholders Autoware's Local projector
ignores), each lanelet has its own left and right line_thin ways, and consecutive
lanelets share their end nodes, which is what makes one the successor of the other.

The loop is a rounded rectangle around the desk island, 1.0 m wide with 1.6 m
corners, four lanelets joined on the long straights. Tighter corners pass the
static radius check but not the simulator lap: Autoware 1.5.0's default planning
(2 m behavior path interval, a 1 m validator radius) could not drive 1.2 m corners.
Map frame as the anchor makes it: origin on the floor below the board, x the way
the board faces, z up.

Variants: good (clockwise, passes), ccw (counter-clockwise), gap (one lanelet starts
0.3 m after the previous ends, on its own nodes), desk (a desk on the lane),
narrow (0.4 m wide), tight (0.4 m corner radius, below the car's turning radius).
"""

import argparse
import math
import os
import sys
from typing import List

import numpy as np

from autosdv_lab3 import mapdir

ROOM = (0.0, 8.0, -6.0, 6.0)          # x0, x1, y0, y1
WALL_HEIGHT = 2.5
ISLAND = (3.4, 4.6, -2.5, 2.5)        # the desk island the loop goes round
WALL_DESKS = [(0.3, 0.9, -5.0, -3.0), (0.3, 0.9, 3.0, 5.0), (7.1, 7.7, -1.0, 1.0)]
DESK_HEIGHT = 0.75
LOOP_CENTRE = (4.0, 0.0)
LOOP_HALF = (2.0, 4.0)                # half extents in x and y of the centreline
SPEED_LIMIT_KMH = 5.0                 # what a student sets in VMB for the RC car

VARIANTS = {
    'good': dict(),
    'ccw': dict(clockwise=False),
    'gap': dict(gap=True),
    'desk': dict(desk_on_lane=True),
    'narrow': dict(width=0.4),
    'tight': dict(radius=0.4),
}


def rounded_rect(cx, cy, hx, hy, r, step=0.25, clockwise=True) -> np.ndarray:
    """Closed centreline (last point != first), starting mid-way up the x = cx - hx side."""
    pts = []

    def line(a, b):
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        pts.extend((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n))

    def arc(c, a0, a1):
        n = max(2, int(math.ceil(abs(a1 - a0) * r / step)))
        pts.extend((c[0] + r * math.cos(a0 + (a1 - a0) * k / n),
                    c[1] + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n))

    x0, x1, y0, y1 = cx - hx, cx + hx, cy - hy, cy + hy
    # counter-clockwise round the rectangle (z up), then reversed for clockwise
    line((x1, cy), (x1, y1 - r))
    arc((x1 - r, y1 - r), 0.0, math.pi / 2)
    line((x1 - r, y1), (x0 + r, y1))
    arc((x0 + r, y1 - r), math.pi / 2, math.pi)
    line((x0, y1 - r), (x0, y0 + r))
    arc((x0 + r, y0 + r), math.pi, 1.5 * math.pi)
    line((x0 + r, y0), (x1 - r, y0))
    arc((x1 - r, y0 + r), 1.5 * math.pi, 2.0 * math.pi)
    line((x1, y0 + r), (x1, cy))
    p = np.array(pts)
    if clockwise:
        p = p[::-1]
    # start on the x0 side at cy, so the first lanelet is the same in both directions
    i = int(np.argmin(np.hypot(p[:, 0] - x0, p[:, 1] - cy)))
    return np.roll(p, -i, axis=0)


def offsets(p: np.ndarray, d: float) -> np.ndarray:
    """The closed polyline p moved d to the left of its direction of travel."""
    t = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    t /= np.linalg.norm(t, axis=1)[:, None]
    return p + d * np.column_stack([-t[:, 1], t[:, 0]])


class _Osm:
    def __init__(self):
        self.next_id = 1
        self.nodes: List[str] = []
        self.ways: List[str] = []
        self.relations: List[str] = []

    def _id(self):
        self.next_id += 1
        return self.next_id - 1

    def node(self, x, y, z=0.0):
        nid = self._id()
        # VMB writes a lat/lon for every node even on a local map; Local ignores it
        lat, lon = 35.0 + y / 111_000.0, 139.0 + x / 91_000.0
        self.nodes.append(
            f'  <node id="{nid}" lat="{lat:.9f}" lon="{lon:.9f}">\n'
            f'    <tag k="local_x" v="{x:.4f}"/>\n    <tag k="local_y" v="{y:.4f}"/>\n'
            f'    <tag k="ele" v="{z:.4f}"/>\n  </node>')
        return nid

    def way(self, refs):
        wid = self._id()
        body = ''.join(f'    <nd ref="{r}"/>\n' for r in refs)
        self.ways.append(f'  <way id="{wid}">\n{body}    <tag k="type" v="line_thin"/>\n'
                         f'    <tag k="subtype" v="solid"/>\n  </way>')
        return wid

    def lanelet(self, left, right, speed_kmh):
        rid = self._id()
        tags = (('type', 'lanelet'), ('subtype', 'road'), ('location', 'urban'),
                ('one_way', 'yes'), ('participant:vehicle', 'yes'),
                ('speed_limit', f'{speed_kmh:g}'), ('turn_direction', 'straight'))
        body = (f'    <member type="way" role="left" ref="{left}"/>\n'
                f'    <member type="way" role="right" ref="{right}"/>\n'
                + ''.join(f'    <tag k="{k}" v="{v}"/>\n' for k, v in tags))
        self.relations.append(f'  <relation id="{rid}">\n{body}  </relation>')
        return rid

    def text(self):
        return ('<?xml version="1.0" encoding="UTF-8"?>\n'
                '<osm generator="VMB">\n  <MetaInfo format_version="1" map_version="2"/>\n'
                + '\n'.join(self.nodes + self.ways + self.relations) + '\n</osm>\n')


def lanelet_osm(clockwise=True, width=1.0, radius=1.6, gap=False) -> str:
    centre = rounded_rect(*LOOP_CENTRE, *LOOP_HALF, radius, clockwise=clockwise)
    left, right = offsets(centre, width / 2), offsets(centre, -width / 2)
    osm = _Osm()
    m = len(centre)
    lid = [osm.node(*p) for p in left]
    rid = [osm.node(*p) for p in right]
    # junctions on the long straights only, the way a student would split them; a
    # junction in the short straight between two corners (1.2-1.6 m corners on a 4 m
    # side) put a curvature spike in Autoware's trajectory and stopped the lap
    (cx, cy), (hx, hy) = LOOP_CENTRE, LOOP_HALF
    d = 1.0 if clockwise else -1.0          # the direction of travel up the x0 side
    targets = [(cx - hx, cy), (cx - hx, cy + 0.6 * d * hy), (cx + hx, cy), (cx + hx, cy - 0.6 * d * hy)]
    cuts = sorted(int(np.argmin(np.hypot(*(centre - t).T))) for t in targets) + [m]
    for k in range(len(cuts) - 1):
        idx = [i % m for i in range(cuts[k], cuts[k + 1] + 1)]
        lrefs, rrefs = [lid[i] for i in idx], [rid[i] for i in idx]
        if gap and k == 2:
            # starts 0.3 m on, on nodes of its own: no successor relation from lanelet 2
            skip = 2
            lrefs = [osm.node(*left[idx[skip]])] + lrefs[skip + 1:]
            rrefs = [osm.node(*right[idx[skip]])] + rrefs[skip + 1:]
        osm.lanelet(osm.way(lrefs), osm.way(rrefs), SPEED_LIMIT_KMH)
    return osm.text()


def _box(x0, x1, y0, y1, z0, z1, step, rng):
    """Surface points of a box."""
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    zs = np.arange(z0, z1 + 1e-9, step)
    out = []
    for x in (x0, x1):
        Y, Z = np.meshgrid(ys, zs)
        out.append(np.column_stack([np.full(Y.size, x), Y.ravel(), Z.ravel()]))
    for y in (y0, y1):
        X, Z = np.meshgrid(xs, zs)
        out.append(np.column_stack([X.ravel(), np.full(X.size, y), Z.ravel()]))
    X, Y = np.meshgrid(xs, ys)
    out.append(np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, z1)]))
    p = np.vstack(out)
    return p + rng.normal(0.0, 0.005, p.shape)


def room_cloud(desk_on_lane=False, seed=0):
    rng = np.random.default_rng(seed)
    x0, x1, y0, y1 = ROOM
    parts = []
    step = 0.1
    zs = np.arange(0.0, WALL_HEIGHT + 1e-9, step)
    for x in (x0, x1):
        Y, Z = np.meshgrid(np.arange(y0, y1 + 1e-9, step), zs)
        parts.append(np.column_stack([np.full(Y.size, x), Y.ravel(), Z.ravel()]))
    for y in (y0, y1):
        X, Z = np.meshgrid(np.arange(x0, x1 + 1e-9, step), zs)
        parts.append(np.column_stack([X.ravel(), np.full(X.size, y), Z.ravel()]))
    X, Y = np.meshgrid(np.arange(x0, x1, 0.3), np.arange(y0, y1, 0.3))   # sparse floor
    parts.append(np.column_stack([X.ravel(), Y.ravel(), rng.normal(0.0, 0.01, X.size)]))
    desks = [ISLAND] + WALL_DESKS
    if desk_on_lane:
        cx = LOOP_CENTRE[0] - LOOP_HALF[0]          # on the first lanelet's straight
        desks.append((cx - 0.3, cx + 0.3, 0.5, 1.1))
    for d in desks:
        parts.append(_box(*d, 0.0, DESK_HEIGHT, 0.08, rng))
    p = np.vstack(parts)
    intensity = rng.uniform(10.0, 60.0, len(p))
    # the reflective board: 0.6 m square on the x = 0 wall, centred above the origin
    board = (np.abs(p[:, 0] - x0) < 0.01) & (np.abs(p[:, 1]) <= 0.3) & (p[:, 2] >= 0.2) & (p[:, 2] <= 0.8)
    intensity[board] = 220.0
    return p, intensity


def write_map(out_dir: str, variant: str = 'good') -> str:
    """Write the map folder for `variant`; returns out_dir."""
    if variant not in VARIANTS:
        raise ValueError(f'unknown variant {variant!r}; one of {sorted(VARIANTS)}')
    from reflective_pose_core.pointcloud_io import PointCloud, write_pcd
    opts = dict(VARIANTS[variant])
    os.makedirs(out_dir, exist_ok=True)
    points, intensity = room_cloud(desk_on_lane=opts.pop('desk_on_lane', False))
    write_pcd(os.path.join(out_dir, mapdir.PCD), PointCloud(points, intensity))
    with open(os.path.join(out_dir, mapdir.LANELET), 'w') as f:
        f.write(lanelet_osm(**opts))
    with open(os.path.join(out_dir, mapdir.PROJECTOR), 'w') as f:
        f.write('projector_type: Local\n')
    return out_dir


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Write a synthetic classroom map folder.')
    ap.add_argument('out')
    ap.add_argument('--variant', default='good', choices=sorted(VARIANTS))
    args = ap.parse_args(argv)
    write_map(args.out, args.variant)
    print(f'wrote {args.out} ({args.variant})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
