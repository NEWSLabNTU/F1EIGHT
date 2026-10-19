"""An ArUco marker map: where the boards stand, and the files the localizer reads. No ROS.

A marker map is a directory holding

    course.yaml           the boards (below); `just aruco-map survey` writes it
    lanelet2_map.osm      GENERATED from course.yaml (`make_marker_map DIR`)
    ar_tag.param.yaml     GENERATED likewise: the localizer's marker settings
    map_projector_info.yaml   projector_type: local (metres, no GNSS)

The file is called course.yaml because a Lab 2 course is a marker map plus a
route (path, laps, speed); this module reads only the marker part and ignores
the rest:

    start_pose: {x: 0.0, y: 0.0, yaw: 0.0}   # where start_pose_init seeds the EKF (base_link)
    marker_size: 0.4667   # [m] black edge, what the detector is told
    dictionary: DICT_5X5_1000   # the printed markers' family (below)
    marker_height: 0.25   # [m] marker centre above the floor (default for all)
    markers:              # yaw: the direction the marker FACES (its normal)
      - {id: 0, x: 6.0, y: 0.0, yaw: 3.1416}
      - {id: 1, x: 3.0, y: 2.5, yaw: -1.5708, z: 0.30}
      - {id: 2, x: 0.0, y: 4.0, yaw: 0.0, pitch: 0.05, roll: -0.02}

`pitch` and `roll` (radians, default 0) describe a board that does not stand
exactly upright, as `just aruco-map scan` measures it: pitch is its top leaning
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
them, e.g. ARUCO_MIP_36h12. An OpenCV family, e.g. DICT_5X5_1000 (the lab's
boards and the LCTK calibration boards), is not in that library; it is decoded
from a file under this package's config/dictionaries/, and only the families
shipped there are accepted.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET

import yaml

PACKAGE = 'autosdv_aruco_localization'
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
        """The marker's x (viewer's right), y (up) and z (facing) axes in the map frame:
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
class MarkerMap:
    directory: str
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


def read_yaml(directory: str) -> dict:
    with open(os.path.join(directory, 'course.yaml')) as f:
        return yaml.safe_load(f) or {}


def parse_markers(d: dict, directory: str = '') -> List[Marker]:
    """course.yaml's `markers`, with `marker_height` as the default z."""
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
    return markers


def parse_dictionary(d: dict, directory: str = '') -> str:
    dictionary = str(d.get('dictionary', 'ALL_DICTS'))
    if dictionary not in ARUCO_DICTIONARIES + SHIPPED_DICTIONARIES:
        raise ValueError(f'{directory}: unknown dictionary {dictionary!r}; one of '
                         f'{", ".join(ARUCO_DICTIONARIES + SHIPPED_DICTIONARIES)}')
    return dictionary


def parse_start_pose(d: dict) -> Pose2D:
    sp = d.get('start_pose') or {}
    return Pose2D(float(sp.get('x', 0.0)), float(sp.get('y', 0.0)), float(sp.get('yaw', 0.0)))


def load_marker_map(directory: str) -> MarkerMap:
    d = read_yaml(directory)
    return MarkerMap(
        directory=directory,
        start_pose=parse_start_pose(d),
        marker_size=float(d.get('marker_size', 0.25)),
        markers=parse_markers(d, directory),
        dictionary=parse_dictionary(d, directory))


# ------------------------------------------------- the localizer's markers

def dictionary_file(name: str) -> str:
    """The localizer's `dictionary` value for a shipped OpenCV family (resolved by the
    launch's allow_substs)."""
    return f'$(find-pkg-share {PACKAGE})/config/dictionaries/{name}.dict'


def make_ar_tag_params(m) -> str:
    """The marker settings ar_tag_based_localizer takes from the map (anything with
    `dictionary`, `marker_size` and `markers`: a MarkerMap or a Lab 2 Course).

    Loaded after config/ar_tag_based_localizer.param.yaml with allow_substs,
    which resolves the $(find-pkg-share) of a shipped dictionary file.
    """
    if m.dictionary in SHIPPED_DICTIONARIES:
        dictionary = dictionary_file(m.dictionary)
    else:
        dictionary = m.dictionary
    ids = ', '.join(f"'{k.id}'" for k in sorted(m.markers, key=lambda k: k.id))
    return ('# GENERATED from course.yaml by make_marker_map; edit that, not this.\n'
            '/**:\n'
            '  ros__parameters:\n'
            f'    dictionary: "{dictionary}"\n'
            f'    marker_size: {m.marker_size}\n'
            f'    target_tag_ids: [{ids}]\n')


# ---------------------------------------------------------------- the OSM map

def make_osm(m) -> str:
    """The lanelet2 map: marker polygons only, in local (map-frame) coordinates."""
    root = ET.Element('osm', {'version': '0.6', 'generator': 'autosdv_aruco_localization make_marker_map'})
    ET.SubElement(root, 'MetaInfo', {'format_version': '1', 'map_version': '1'})
    nid = 1
    ways = []
    for k in m.markers:
        refs = []
        for (x, y, z) in k.corners(m.marker_size):
            node = ET.SubElement(root, 'node', {'id': str(nid), 'lat': '0', 'lon': '0'})
            ET.SubElement(node, 'tag', {'k': 'local_x', 'v': f'{x:.4f}'})
            ET.SubElement(node, 'tag', {'k': 'local_y', 'v': f'{y:.4f}'})
            ET.SubElement(node, 'tag', {'k': 'ele', 'v': f'{z:.4f}'})
            refs.append(nid)
            nid += 1
        ways.append((k, refs))
    wid = 1000
    for k, refs in ways:
        way = ET.SubElement(root, 'way', {'id': str(wid)})
        for r in refs:
            ET.SubElement(way, 'nd', {'ref': str(r)})
        for key, v in (('type', LANDMARK_TYPE), ('subtype', LANDMARK_SUBTYPE),
                       ('marker_id', str(k.id)), ('area', 'yes')):
            ET.SubElement(way, 'tag', {'k': key, 'v': v})
        wid += 1
    ET.indent(root)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


PROJECTOR_INFO = 'projector_type: local\n'


def generated_files(m) -> List[Tuple[str, str]]:
    """(path, text) of the two files generated from course.yaml."""
    return [(m.map_file, make_osm(m)), (m.ar_tag_param_file, make_ar_tag_params(m))]


def write_map_files(directory: str, projector: bool = True) -> List[str]:
    """Regenerate DIR's lanelet2_map.osm and ar_tag.param.yaml from its course.yaml, and
    write map_projector_info.yaml if it is missing (projector=True). Returns the files."""
    m = load_marker_map(directory)
    written = []
    for p, text in generated_files(m):
        with open(p, 'w') as f:
            f.write(text)
        written.append(p)
    proj = os.path.join(directory, 'map_projector_info.yaml')
    if projector and not os.path.exists(proj):
        with open(proj, 'w') as f:
            f.write(PROJECTOR_INFO)
        written.append(proj)
    return written


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


def check_or_write(m, check: bool, log=print) -> bool:
    """Write m's generated files where they differ (or, check=True, only report them).
    True when something was (or would be) out of date."""
    stale = False
    for path, text in generated_files(m):
        old = None
        if os.path.exists(path):
            with open(path) as f:
                old = f.read()
        if check:
            if old != text:
                log(f'{path}: out of date with course.yaml')
                stale = True
        elif old != text:
            with open(path, 'w') as f:
                f.write(text)
            log(f'wrote {path}')
        else:
            log(f'{path}: up to date')
    return stale


def main(argv=None):
    """make_marker_map <map_dir> [...]: write lanelet2_map.osm and ar_tag.param.yaml from course.yaml."""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('map_dirs', nargs='+')
    ap.add_argument('--check', action='store_true',
                    help='do not write; exit 1 if a generated file is missing or out of date')
    args = ap.parse_args(argv)
    stale = False
    for d in args.map_dirs:
        m = load_marker_map(d)
        stale |= check_or_write(m, args.check)
        if args.check:
            continue
        print(f'  {len(m.markers)} markers, {m.dictionary}, {m.marker_size} m')
        proj = os.path.join(d, 'map_projector_info.yaml')
        if not os.path.exists(proj):
            with open(proj, 'w') as f:
                f.write(PROJECTOR_INFO)
            print(f'wrote {proj}')
    return 1 if stale else 0


if __name__ == '__main__':
    raise SystemExit(main())
