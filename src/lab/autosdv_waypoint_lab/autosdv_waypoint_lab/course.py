"""A hall course: its markers, its path and its start box. No ROS.

A course is a directory under config/courses/<name>/:

    course.yaml           the TA's description (below)
    path.csv              x, y[, v] waypoints in the hall frame
    lanelet2_map.osm      GENERATED from course.yaml by `make_course_map`
    ar_tag.param.yaml     GENERATED likewise: the localizer's marker settings
    map_projector_info.yaml   projector_type: local (hall metres, no GNSS)

course.yaml:

    path: path.csv
    closed: true          # the last waypoint joins the first
    laps: 3
    v_ref: 0.8            # [m/s] speed for rows without a v column
    start_pose: {x: 0.0, y: 0.0, yaw: 0.0}   # the taped start box, base_link
    marker_size: 0.25     # [m] black edge, what the detector is told
    dictionary: ALL_DICTS # the printed markers' family (below)
    marker_height: 0.25   # [m] marker centre above the floor (default for all)
    markers:              # yaw: the direction the marker FACES (its normal)
      - {id: 0, x: 6.0, y: 0.0, yaw: 3.1416}
      - {id: 1, x: 3.0, y: 2.5, yaw: -1.5708, z: 0.30}
      - {id: 2, x: 0.0, y: 4.0, yaw: 0.0, pitch: 0.05, roll: -0.02}

`pitch` and `roll` (radians, default 0) describe a board that does not stand
exactly upright, as `just lab2 survey` measures it: pitch is its top leaning
back, away from the viewer; roll is the board turned in its own plane,
counter-clockwise as seen from the front. The localizer reads a marker's full
orientation from its corners, so a map that leaves a measured tilt out puts
every fix from that marker off by about range x tilt (mostly in height).

The map is what Autoware's ar_tag_based_localizer reads: one lanelet2
`pose_marker` polygon per marker (subtype apriltag_16h5, the subtype the stock
node asks the landmark manager for), corners counter-clockwise as seen from
the front -- bottom-left, bottom-right, top-right, top-left -- so that the
landmark manager's axes come out as the ArUco marker's: x to the viewer's
right, y up, z out of the marker towards the viewer.

`dictionary` is what the localizer decodes with. The aruco library's own
families go by name: ALL_DICTS (the default; covers ARUCO_MIP_36h12) or one of
them, e.g. ARUCO_MIP_36h12. An OpenCV family, e.g. DICT_5X5_1000 (the LCTK
calibration boards), is not in that library; it is decoded from a file under
config/dictionaries/, and only the families shipped there are accepted.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET

import yaml

LANDMARK_TYPE = 'pose_marker'
LANDMARK_SUBTYPE = 'apriltag_16h5'

# The aruco library's predefined dictionaries (aruco/dictionary.h).
ARUCO_DICTIONARIES = (
    'ALL_DICTS', 'ARUCO_MIP_36h12', 'ARUCO', 'ARUCO_MIP_25h7', 'ARUCO_MIP_16h3',
    'ARTAG', 'ARTOOLKITPLUS', 'ARTOOLKITPLUSBCH', 'TAG16h5', 'TAG25h7', 'TAG25h9',
    'TAG36h11', 'TAG36h10', 'CHILITAGS')
# OpenCV dictionaries exported to config/dictionaries/<name>.dict.
SHIPPED_DICTIONARIES = ('DICT_5X5_1000',)


@dataclass
class Marker:
    id: int
    x: float
    y: float
    z: float
    yaw: float          # facing direction
    pitch: float = 0.0  # top leaning back, away from the viewer
    roll: float = 0.0   # turned in its own plane, counter-clockwise from the front

    def axes(self):
        """The marker's x (viewer's right), y (up) and z (facing) axes in the course frame:
        upright at `yaw`, then leaned back by `pitch` about its x axis, then turned by `roll`
        about its normal."""
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        right, up, facing = (-sy, cy, 0.0), (0.0, 0.0, 1.0), (cy, sy, 0.0)
        a = -self.pitch                    # about x: up turns toward -facing for a backward lean
        ca, sa = math.cos(a), math.sin(a)
        y1 = tuple(ca * u + sa * f for u, f in zip(up, facing))
        z1 = tuple(-sa * u + ca * f for u, f in zip(up, facing))
        cb, sb = math.cos(self.roll), math.sin(self.roll)
        x2 = tuple(cb * r + sb * yy for r, yy in zip(right, y1))
        y2 = tuple(-sb * r + cb * yy for r, yy in zip(right, y1))
        return x2, y2, z1

    def corners(self, size: float) -> List[Tuple[float, float, float]]:
        """bottom-left, bottom-right, top-right, top-left, seen from the front."""
        h = size / 2.0
        xa, ya, _ = self.axes()
        out = []
        for u, w in ((-h, -h), (h, -h), (h, h), (-h, h)):
            out.append(tuple(c + u * xx + w * yy for c, xx, yy in zip((self.x, self.y, self.z), xa, ya)))
        return out


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class Course:
    directory: str
    path_file: str
    closed: bool
    laps: int
    v_ref: float
    start_pose: Pose2D
    marker_size: float
    markers: List[Marker] = field(default_factory=list)
    dictionary: str = 'ALL_DICTS'

    @property
    def map_file(self) -> str:
        return os.path.join(self.directory, 'lanelet2_map.osm')

    @property
    def ar_tag_param_file(self) -> str:
        return os.path.join(self.directory, 'ar_tag.param.yaml')


def load_course(directory: str) -> Course:
    with open(os.path.join(directory, 'course.yaml')) as f:
        d = yaml.safe_load(f) or {}
    height = float(d.get('marker_height', 0.25))
    markers = []
    seen = set()
    for m in d.get('markers', []) or []:
        mid = int(m['id'])
        if mid in seen:
            raise ValueError(f'{directory}: marker id {mid} appears twice; ids must be unique')
        seen.add(mid)
        markers.append(Marker(mid, float(m['x']), float(m['y']),
                              float(m.get('z', height)), float(m['yaw']),
                              float(m.get('pitch', 0.0)), float(m.get('roll', 0.0))))
    dictionary = str(d.get('dictionary', 'ALL_DICTS'))
    if dictionary not in ARUCO_DICTIONARIES + SHIPPED_DICTIONARIES:
        raise ValueError(f'{directory}: unknown dictionary {dictionary!r}; one of '
                         f'{", ".join(ARUCO_DICTIONARIES + SHIPPED_DICTIONARIES)}')
    sp = d.get('start_pose') or {}
    return Course(
        directory=directory,
        path_file=os.path.join(directory, d.get('path', 'path.csv')),
        closed=bool(d.get('closed', True)),
        laps=int(d.get('laps', 1)),
        v_ref=float(d.get('v_ref', 0.8)),
        start_pose=Pose2D(float(sp.get('x', 0.0)), float(sp.get('y', 0.0)),
                          float(sp.get('yaw', 0.0))),
        marker_size=float(d.get('marker_size', 0.25)),
        dictionary=dictionary,
        markers=markers)


# ------------------------------------------------- the localizer's markers

def make_ar_tag_params(course: Course) -> str:
    """The marker settings ar_tag_based_localizer takes from the course.

    Loaded after config/ar_tag_based_localizer.param.yaml with allow_substs,
    which resolves the $(find-pkg-share) of a shipped dictionary file.
    """
    if course.dictionary in SHIPPED_DICTIONARIES:
        dictionary = ('$(find-pkg-share autosdv_waypoint_lab)/config/dictionaries/'
                      f'{course.dictionary}.dict')
    else:
        dictionary = course.dictionary
    ids = ', '.join(f"'{m.id}'" for m in sorted(course.markers, key=lambda m: m.id))
    return ('# GENERATED from course.yaml by make_course_map; edit that, not this.\n'
            '/**:\n'
            '  ros__parameters:\n'
            f'    dictionary: "{dictionary}"\n'
            f'    marker_size: {course.marker_size}\n'
            f'    target_tag_ids: [{ids}]\n')


# ---------------------------------------------------------------- the OSM map

def make_osm(course: Course) -> str:
    """The lanelet2 map: marker polygons only, in local (hall) coordinates."""
    root = ET.Element('osm', {'version': '0.6', 'generator': 'autosdv_waypoint_lab make_course_map'})
    ET.SubElement(root, 'MetaInfo', {'format_version': '1', 'map_version': '1'})
    nid = 1
    ways = []
    for m in course.markers:
        refs = []
        for (x, y, z) in m.corners(course.marker_size):
            node = ET.SubElement(root, 'node', {'id': str(nid), 'lat': '0', 'lon': '0'})
            ET.SubElement(node, 'tag', {'k': 'local_x', 'v': f'{x:.4f}'})
            ET.SubElement(node, 'tag', {'k': 'local_y', 'v': f'{y:.4f}'})
            ET.SubElement(node, 'tag', {'k': 'ele', 'v': f'{z:.4f}'})
            refs.append(nid)
            nid += 1
        ways.append((m, refs))
    wid = 1000
    for m, refs in ways:
        way = ET.SubElement(root, 'way', {'id': str(wid)})
        for r in refs:
            ET.SubElement(way, 'nd', {'ref': str(r)})
        for k, v in (('type', LANDMARK_TYPE), ('subtype', LANDMARK_SUBTYPE),
                     ('marker_id', str(m.id)), ('area', 'yes')):
            ET.SubElement(way, 'tag', {'k': k, 'v': v})
        wid += 1
    ET.indent(root)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


@dataclass
class Landmark:
    """A marker as the landmark manager sees it: centre and rotation matrix columns."""

    id: str
    position: Tuple[float, float, float]
    x_axis: Tuple[float, float, float]
    y_axis: Tuple[float, float, float]
    z_axis: Tuple[float, float, float]

    @property
    def facing_yaw(self) -> float:
        return math.atan2(self.z_axis[1], self.z_axis[0])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(a):
    n = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    return (a[0] / n, a[1] / n, a[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def landmark_from_corners(mid: str, v) -> Landmark:
    """landmark_manager.cpp's construction: centre, x = v0->v1, y = v1->v2, z = x cross y."""
    cx = sum(p[0] for p in v) / 4.0
    cy = sum(p[1] for p in v) / 4.0
    cz = sum(p[2] for p in v) / 4.0
    xa = _norm(_sub(v[1], v[0]))
    ya = _norm(_sub(v[2], v[1]))
    za = _norm(_cross(xa, ya))
    return Landmark(mid, (cx, cy, cz), xa, ya, za)


def parse_osm_landmarks(text: str) -> Dict[str, List[Landmark]]:
    """The pose_marker polygons of a lanelet2 OSM in local coordinates, by marker id."""
    root = ET.fromstring(text)
    nodes = {}
    for n in root.iter('node'):
        tags = {t.get('k'): t.get('v') for t in n.iter('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            nodes[n.get('id')] = (float(tags['local_x']), float(tags['local_y']),
                                  float(tags.get('ele', 0.0)))
    out: Dict[str, List[Landmark]] = {}
    for w in root.iter('way'):
        tags = {t.get('k'): t.get('v') for t in w.iter('tag')}
        if tags.get('type') != LANDMARK_TYPE or tags.get('subtype') != LANDMARK_SUBTYPE:
            continue
        refs = [nd.get('ref') for nd in w.iter('nd')]
        if len(refs) != 4 or any(r not in nodes for r in refs):
            continue
        mid = tags.get('marker_id', 'none')
        out.setdefault(mid, []).append(landmark_from_corners(mid, [nodes[r] for r in refs]))
    return out


def print_coverage(course: Course) -> None:
    """Marker coverage of the path for the default camera, and of the start box."""
    from autosdv_waypoint_lab.marker_sim import CameraModel, coverage
    from autosdv_waypoint_lab.path import CoursePath, Path, load_csv
    x, y, v = load_csv(course.path_file, course.v_ref)
    cp = CoursePath(x, y, v, closed=course.closed)
    path = Path(cp.x, cp.y, cp.v)
    landmarks = parse_osm_landmarks(make_osm(course))
    sp = course.start_pose
    cov = coverage(path, landmarks, (sp.x, sp.y, sp.yaw),
                   CameraModel(marker_size=course.marker_size))
    print(f'  coverage (100 deg, 5 m camera): a marker in view on {cov.fraction:.0%} of the '
          f'path, longest gap {cov.longest_gap:.1f} m; start box (+-0.3 m, +-10 deg): '
          f'at least {cov.start_box_min} marker(s) in view')
    if cov.start_box_min == 0:
        print('  WARNING: a car placed in the start box may see no marker: engage needs a fix')
    if cov.longest_gap > 3.0:
        print('  WARNING: a gap over 3 m; the EKF coasts on odometry through it')


def main(argv=None):
    """make_course_map <course_dir> [...]: write lanelet2_map.osm and ar_tag.param.yaml from course.yaml."""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('course_dirs', nargs='+')
    ap.add_argument('--check', action='store_true',
                    help='do not write; exit 1 if a generated file is missing or out of date')
    args = ap.parse_args(argv)
    stale = False
    for d in args.course_dirs:
        course = load_course(d)
        for path, text in ((course.map_file, make_osm(course)),
                           (course.ar_tag_param_file, make_ar_tag_params(course))):
            old = None
            if os.path.exists(path):
                with open(path) as f:
                    old = f.read()
            if args.check:
                if old != text:
                    print(f'{path}: out of date with course.yaml')
                    stale = True
            elif old != text:
                with open(path, 'w') as f:
                    f.write(text)
                print(f'wrote {path}')
            else:
                print(f'{path}: up to date')
        if args.check:
            continue
        print(f'  {len(course.markers)} markers, {course.dictionary}, {course.marker_size} m')
        print_coverage(course)
        proj = os.path.join(d, 'map_projector_info.yaml')
        if not os.path.exists(proj):
            with open(proj, 'w') as f:
                f.write('projector_type: local\n')
            print(f'wrote {proj}')
    return 1 if stale else 0


if __name__ == '__main__':
    raise SystemExit(main())
