"""A hall course: its markers, its path and its start box. No ROS.

A course is a directory under config/courses/<name>/:

    course.yaml           the TA's description of the route (below)
    path.csv              x, y[, v] waypoints in the hall frame
    marker_map.yaml       the markers: marker_survey's map file, written by
                          `just lab2 survey`, read as it is by marker_localizer
    lanelet2_map.osm      GENERATED from marker_map.yaml by `make_course_map`
    ar_tag.param.yaml     GENERATED likewise: the marker settings of the
                          fallback localizer (localizer:=ar_tag)
    map_projector_info.yaml   projector_type: local (hall metres, no GNSS)

course.yaml:

    path: path.csv
    closed: true          # the last waypoint joins the first
    laps: 3
    v_ref: 0.8            # [m/s] speed for rows without a v column
    start_pose: {x: 0.0, y: 0.0, yaw: 0.0}   # the taped start box, base_link

marker_map.yaml (marker_survey's README, "The marker map file"):

    version: 1
    frame_id: map
    dictionary: DICT_5X5_1000   # the printed family
    marker_size: 0.38           # [m] edge of the black square, measured
    mount: wall
    markers:                    # facing_deg: the direction the board FACES
      - {id: 24, x: 0.000, y: 0.000, z: 0.250, facing_deg: 0.0}
      - {id: 20, x: 6.800, y: -1.000, z: 0.250, facing_deg: 30.0, lean_deg: 2.1}

Here a marker is kept as `Marker` (yaw, pitch, roll in radians): yaw is the
direction it faces, pitch its top leaning back, away from the viewer, roll
the board turned in its own plane, counter-clockwise as seen from the front;
for a wall board these are the map file's facing_deg, lean_deg and turn_deg.
The localizer reads a marker's full orientation from its corners, so a map
that leaves a measured tilt out puts every fix from that marker off by about
range x tilt (mostly in height).

A course directory from before marker_map.yaml, with its markers in
course.yaml, converts once with
`ros2 run marker_survey marker_map_import course course.yaml marker_map.yaml`.

The OSM is what Autoware's ar_tag_based_localizer reads: one lanelet2
`pose_marker` polygon per marker (subtype apriltag_16h5, the subtype the stock
node asks the landmark manager for), corners counter-clockwise as seen from
the front -- bottom-left, bottom-right, top-right, top-left -- so that the
landmark manager's axes come out as the ArUco marker's: x to the viewer's
right, y up, z out of the marker towards the viewer.

The fallback localizer decodes with the aruco library, whose own families go
by name (ARUCO_MIP_36h12, TAG36h11, ...); the map's dictionary is translated
to one of them. An OpenCV family, e.g. DICT_5X5_1000 (the lab's boards), is
not in that library; it is decoded from a file under config/dictionaries/,
and only the families shipped there work with the fallback.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET

import yaml

from marker_survey.core import marker_map as mm

MARKER_MAP_FILE = 'marker_map.yaml'
LANDMARK_TYPE = 'pose_marker'
LANDMARK_SUBTYPE = 'apriltag_16h5'

# The aruco library's predefined dictionaries (aruco/dictionary.h).
ARUCO_DICTIONARIES = (
    'ALL_DICTS', 'ARUCO_MIP_36h12', 'ARUCO', 'ARUCO_MIP_25h7', 'ARUCO_MIP_16h3',
    'ARTAG', 'ARTOOLKITPLUS', 'ARTOOLKITPLUSBCH', 'TAG16h5', 'TAG25h7', 'TAG25h9',
    'TAG36h11', 'TAG36h10', 'CHILITAGS')
# OpenCV dictionaries exported to config/dictionaries/<name>.dict.
SHIPPED_DICTIONARIES = ('DICT_5X5_1000',)
# A marker map's OpenCV dictionary as the aruco library names it.
ARUCO_NAMES = {
    'DICT_ARUCO_MIP_36h12': 'ARUCO_MIP_36h12',
    'DICT_ARUCO_ORIGINAL': 'ARUCO',
    'DICT_APRILTAG_16h5': 'TAG16h5',
    'DICT_APRILTAG_25h9': 'TAG25h9',
    'DICT_APRILTAG_36h10': 'TAG36h10',
    'DICT_APRILTAG_36h11': 'TAG36h11',
}


@dataclass
class Marker:
    id: int
    x: float
    y: float
    z: float
    yaw: float          # facing direction
    pitch: float = 0.0  # top leaning back, away from the viewer
    roll: float = 0.0   # turned in its own plane, counter-clockwise from the front
    size: float = 0.0   # [m] black square, if not the course's marker_size

    @classmethod
    def from_map(cls, m: 'mm.MapMarker', size: float = 0.0) -> 'Marker':
        """A marker of marker_map.yaml, its rotation read back as yaw, pitch and roll
        (exact for any board that is not lying flat)."""
        R = m.rotation()
        z = R[:, 2]
        pitch = math.asin(max(-1.0, min(1.0, float(z[2]))))
        yaw = math.atan2(float(z[1]), float(z[0]))
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        right = (-sy, cy, 0.0)
        y1 = (-sp * cy, -sp * sy, cp)          # the upright board's up, leaned back
        x = R[:, 0]
        roll = math.atan2(float(sum(a * b for a, b in zip(x, y1))),
                          float(sum(a * b for a, b in zip(x, right))))
        return cls(m.id, m.x, m.y, m.z, yaw, pitch, roll, size)

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
        h = (self.size or size) / 2.0
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
    dictionary: str = 'DICT_5X5_1000'

    @property
    def marker_map_file(self) -> str:
        return os.path.join(self.directory, MARKER_MAP_FILE)

    @property
    def map_file(self) -> str:
        return os.path.join(self.directory, 'lanelet2_map.osm')

    @property
    def ar_tag_param_file(self) -> str:
        return os.path.join(self.directory, 'ar_tag.param.yaml')


def load_markers(directory: str):
    """(markers, marker_size, dictionary) of a course's marker_map.yaml; no markers if
    the course has none yet. The dictionary is OpenCV's name (DICT_*)."""
    path = os.path.join(directory, MARKER_MAP_FILE)
    if not os.path.exists(path):
        return [], 0.25, 'DICT_5X5_1000'
    m = mm.load(path)     # MarkerMapError (a ValueError) names the line and field
    if m.frame_id != 'map':
        raise ValueError(f'{path}: frame_id {m.frame_id!r}; the lab localizes in map')
    size = float(m.marker_size)
    markers = [Marker.from_map(k, m.size_of(k.id) if k.size is not None else 0.0)
               for k in m.markers]
    return markers, size, mm.opencv_dictionary_name(m.dictionary)


def load_course(directory: str) -> Course:
    with open(os.path.join(directory, 'course.yaml')) as f:
        d = yaml.safe_load(f) or {}
    if 'markers' in d:
        raise ValueError(
            f'{directory}/course.yaml lists markers: they now live in {MARKER_MAP_FILE}. '
            f'Convert them once with `ros2 run marker_survey marker_map_import course '
            f'{directory}/course.yaml {directory}/{MARKER_MAP_FILE} [--dictionary DICT_5X5_1000]`, '
            f'then delete marker_size, dictionary, marker_height and markers from course.yaml')
    markers, marker_size, dictionary = load_markers(directory)
    sp = d.get('start_pose') or {}
    return Course(
        directory=directory,
        path_file=os.path.join(directory, d.get('path', 'path.csv')),
        closed=bool(d.get('closed', True)),
        laps=int(d.get('laps', 1)),
        v_ref=float(d.get('v_ref', 0.8)),
        start_pose=Pose2D(float(sp.get('x', 0.0)), float(sp.get('y', 0.0)),
                          float(sp.get('yaw', 0.0))),
        marker_size=marker_size,
        dictionary=dictionary,
        markers=markers)


# ------------------------------------------------- the localizer's markers

def ar_tag_dictionary(dictionary: str) -> str:
    """The fallback localizer's `dictionary` for an OpenCV dictionary name."""
    if dictionary in SHIPPED_DICTIONARIES:
        return ('$(find-pkg-share autosdv_waypoint_lab)/config/dictionaries/'
                f'{dictionary}.dict')
    if dictionary in ARUCO_NAMES:
        return ARUCO_NAMES[dictionary]
    if dictionary in ARUCO_DICTIONARIES:
        return dictionary
    raise ValueError(f'{dictionary}: the ar_tag fallback cannot decode it (it knows '
                     f'{", ".join(SHIPPED_DICTIONARIES + tuple(ARUCO_NAMES))})')


def make_ar_tag_params(course: Course) -> str:
    """The marker settings of the fallback localizer, ar_tag_based_localizer.

    Loaded after config/ar_tag_based_localizer.param.yaml with allow_substs,
    which resolves the $(find-pkg-share) of a shipped dictionary file.
    """
    dictionary = ar_tag_dictionary(course.dictionary)
    ids = ', '.join(f"'{m.id}'" for m in sorted(course.markers, key=lambda m: m.id))
    return ('# GENERATED from marker_map.yaml by make_course_map; edit that, not this.\n'
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
    """make_course_map <course_dir> [...]: write lanelet2_map.osm and ar_tag.param.yaml from marker_map.yaml."""
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
                    print(f'{path}: out of date with {MARKER_MAP_FILE}')
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
