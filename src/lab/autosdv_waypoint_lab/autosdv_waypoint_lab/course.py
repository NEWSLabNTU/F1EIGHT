"""A Lab 2 course: a marker map plus a route (path, laps, speed). No ROS.

A course is a directory under config/courses/<name>/:

    course.yaml           the TA's description (below)
    path.csv              x, y[, v] waypoints in the course frame
    lanelet2_map.osm      GENERATED from course.yaml by `make_course_map`
    ar_tag.param.yaml     GENERATED likewise: the localizer's marker settings
    map_projector_info.yaml   projector_type: local (hall metres, no GNSS)

course.yaml is a marker map's (autosdv_aruco_localization.marker_map: the
boards, start_pose, marker_size, dictionary) with the route's keys added:

    path: path.csv
    closed: true          # the last waypoint joins the first
    laps: 3
    v_ref: 0.8            # [m/s] speed for rows without a v column
    start_pose: {x: 0.0, y: 0.0, yaw: 0.0}   # the taped start box, base_link
    marker_size: 0.25     # [m] black edge, what the detector is told
    dictionary: ALL_DICTS # the printed markers' family
    markers:              # yaw: the direction the marker FACES (its normal)
      - {id: 0, x: 6.0, y: 0.0, yaw: 3.1416}

The marker side (Marker, the lanelet2 map, ar_tag.param.yaml, the dictionaries)
lives in autosdv_aruco_localization.marker_map and is re-exported here; this
module keeps what is Lab 2's: the route fields and the path's marker coverage.
"""

from dataclasses import dataclass, field
import os
from typing import List

from autosdv_aruco_localization.marker_map import (  # noqa: F401  (re-exported)
    ARUCO_DICTIONARIES, LANDMARK_SUBTYPE, LANDMARK_TYPE, SHIPPED_DICTIONARIES, Landmark,
    Marker, Pose2D, check_or_write, landmark_from_corners, make_ar_tag_params, make_osm,
    parse_dictionary, parse_markers, parse_osm_landmarks, parse_start_pose, read_yaml)


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
    d = read_yaml(directory)
    return Course(
        directory=directory,
        path_file=os.path.join(directory, d.get('path', 'path.csv')),
        closed=bool(d.get('closed', True)),
        laps=int(d.get('laps', 1)),
        v_ref=float(d.get('v_ref', 0.8)),
        start_pose=parse_start_pose(d),
        marker_size=float(d.get('marker_size', 0.25)),
        dictionary=parse_dictionary(d, directory),
        markers=parse_markers(d, directory))


def print_coverage(course: Course) -> None:
    """Marker coverage of the path for the default camera, and of the start box."""
    from autosdv_aruco_localization.marker_sim import CameraModel, coverage
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
    """make_course_map <course_dir> [...]: write lanelet2_map.osm and ar_tag.param.yaml from
    course.yaml, and print the path's marker coverage."""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('course_dirs', nargs='+')
    ap.add_argument('--check', action='store_true',
                    help='do not write; exit 1 if a generated file is missing or out of date')
    args = ap.parse_args(argv)
    stale = False
    for d in args.course_dirs:
        course = load_course(d)
        stale |= check_or_write(course, args.check)
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
