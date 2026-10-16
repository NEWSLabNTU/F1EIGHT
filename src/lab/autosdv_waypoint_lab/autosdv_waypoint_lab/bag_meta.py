"""What a Lab 2 submission bag says about itself, and which topics it holds.

No ROS imports at module level, so the grader and the checker unit-test
without a ROS environment.

`lab2_record` writes the metadata twice: as one latched std_msgs/String
(JSON) on META_TOPIC, which is inside the bag, and as `lab2_meta.json` beside
the bag's metadata.yaml (rosbag2 on Humble has no custom-data field). Readers
take the topic first and fall back to the file.

Course hashes. A surveyed course.yaml differs from the template in its
markers (the survey measures them), so a hash of the whole file cannot tell a
surveyed course from a doctored one. Two hashes are stored:

    route_hash    path.csv bytes + the fields that define the ROUTE (closed,
                  laps, v_ref, start_pose, path name). A survey leaves these
                  alone; editing the path, the speed or the lap count changes
                  it. This is what the checker compares.
    course_hash   course.yaml bytes + path.csv bytes, for the record.
"""

import hashlib
import json
import math
import os
from typing import Dict, List, Optional, Sequence

import yaml

META_TOPIC = '/lab2/record_meta'
CAMERA_TOPIC = '/lab2/camera/image/compressed'     # 960 x 600 at 10 Hz, from the ZED's compressed stream
CAMERA_SOURCE = '/sensing/camera/zedxm/rgb/color/rect/image/compressed'
SIDECAR = 'lab2_meta.json'
CAMERA_SIZE = (960, 600)
CAMERA_RATE = 10.0

PROGRESS = '/planning/waypoint_path_publisher/progress'
TRACKER_DEBUG = '/control/path_tracker/debug'
TRACKER_CMD = '/control/trajectory_follower/control_cmd'
GATE_CMD = '/control/command/control_cmd'
VELOCITY = '/vehicle/status/velocity_status'
STEERING = '/vehicle/status/steering_status'
KINEMATIC = '/localization/kinematic_state'
MARKER_FIXES = '/localization/pose_estimator/pose_with_covariance'
TWIST = '/localization/twist_estimator/twist_with_covariance'
MODE = '/api/operation_mode/state'
MRM = '/system/fail_safe/mrm_state'
DIAG_GRAPH = '/diagnostics_graph/status'
IMU = '/sensing/imu/imu_data'
TRAJECTORY = '/planning/trajectory'
PWM = '/autosdv/actuator_node/debug/pwm_values'
PID = '/autosdv/actuator_node/debug/pid_values'
ACT_CONTROL = '/autosdv/actuator_node/debug/control_values'
TRUE_POSE = '/vehicle_bench_plant/odometry'      # bench only

# Without these a bag cannot be graded or used for the report.
REQUIRED_TOPICS = [
    KINEMATIC, MARKER_FIXES, VELOCITY, GATE_CMD, TRACKER_CMD, PWM, PID,
    PROGRESS, MRM, MODE, '/tf', '/tf_static', CAMERA_TOPIC, META_TOPIC,
]
# Nice to have; a missing one is a warning.
OPTIONAL_TOPICS = [
    TRACKER_DEBUG, STEERING, TWIST, DIAG_GRAPH, IMU, TRAJECTORY, ACT_CONTROL,
    '/vehicle/status/control_mode', '/vehicle/status/gear_status',
]
# Recorded, never required or warned about: published on change only.
ON_CHANGE_TOPICS = ['/autosdv/actuator_node/debug/mode']
RECORD_TOPICS = ([t for t in REQUIRED_TOPICS if t != CAMERA_TOPIC and t != META_TOPIC]
                 + OPTIONAL_TOPICS + ON_CHANGE_TOPICS)


def _read(path: str) -> bytes:
    with open(path, 'rb') as f:
        return f.read()


def course_hashes(course_dir: str) -> Dict[str, str]:
    """{route_hash, course_hash, route_name} of a course directory."""
    yaml_bytes = _read(os.path.join(course_dir, 'course.yaml'))
    d = yaml.safe_load(yaml_bytes) or {}
    path_name = str(d.get('path', 'path.csv'))
    path_bytes = _read(os.path.join(course_dir, path_name))
    sp = d.get('start_pose') or {}
    route = {
        'path': path_name,
        'closed': bool(d.get('closed', True)),
        'laps': int(d.get('laps', 1)),
        'v_ref': round(float(d.get('v_ref', 0.8)), 6),
        'start_pose': [round(float(sp.get(k, 0.0)), 6) for k in ('x', 'y', 'yaw')],
    }
    h = hashlib.sha256()
    h.update(path_bytes)
    h.update(json.dumps(route, sort_keys=True).encode())
    return {
        'route_hash': h.hexdigest(),
        'course_hash': hashlib.sha256(yaml_bytes + path_bytes).hexdigest(),
        'route': route,
    }


def build_meta(course_dir: str, car_id: str, team_id: str, git_commit: str,
               started: float = 0.0) -> dict:
    h = course_hashes(course_dir)
    return {
        'schema': 1,
        'course_name': os.path.basename(os.path.normpath(course_dir)),
        'course_hash': h['course_hash'],
        'route_hash': h['route_hash'],
        'route': h['route'],
        'route_kind': classify_route(course_dir),
        'car_id': car_id,
        'team_id': team_id,
        'git_commit': git_commit,
        'recorded_at': started,
    }


def read_sidecar(bag_dir: str) -> Optional[dict]:
    p = os.path.join(bag_dir, SIDECAR)
    if os.path.isfile(p):
        with open(p) as f:
            return json.load(f)
    return None


def meta_from_messages(msgs: Dict[str, list], bag_dir: Optional[str] = None) -> Optional[dict]:
    """The bag's metadata: the topic's last message, else the sidecar."""
    for _, m in reversed(msgs.get(META_TOPIC, [])):
        try:
            return json.loads(m.data)
        except (ValueError, AttributeError):
            continue
    return read_sidecar(bag_dir) if bag_dir else None


# --------------------------------------------------------------- the route

# The graded route. (The straight and the two circles of the first draft are gone;
# a bag of one of those is reported as an ungraded route.)
ROUTES = ('uturn',)
ROUTE_BY_NAME = {'lab2_uturn': 'uturn'}


def load_xy(course_dir: str):
    with open(os.path.join(course_dir, 'course.yaml')) as f:
        d = yaml.safe_load(f) or {}
    xs, ys = [], []
    with open(os.path.join(course_dir, str(d.get('path', 'path.csv')))) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line:
                continue
            a = line.split(',')
            xs.append(float(a[0]))
            ys.append(float(a[1]))
    return xs, ys, bool(d.get('closed', True))


def classify_route(course_dir: str) -> Optional[str]:
    """'uturn' for an open path that turns once through about 180 degrees, from
    the path's geometry, so a surveyed copy under any name is still recognised;
    None for anything else."""
    from autosdv_waypoint_lab.path import turn_spans
    xs, ys, closed = load_xy(course_dir)
    if closed or len(xs) < 3:
        return None
    if len(turn_spans(xs, ys)) != 1:
        return None
    h0 = math.atan2(ys[1] - ys[0], xs[1] - xs[0])
    h1 = math.atan2(ys[-1] - ys[-2], xs[-1] - xs[-2])
    turned = abs(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0)))
    return 'uturn' if turned > math.radians(170.0) else None


def route_label(meta: Optional[dict]) -> Optional[str]:
    """What the bag says it drove, graded or not: route_kind, else the course name."""
    if not meta:
        return None
    return meta.get('route_kind') or meta.get('course_name') or None


def route_from_meta(meta: Optional[dict]) -> Optional[str]:
    """The graded route a bag drove, or None (no metadata, or not a graded route)."""
    if not meta:
        return None
    if meta.get('route_kind') in ROUTES:
        return meta['route_kind']
    name = str(meta.get('course_name', ''))
    if name in ROUTE_BY_NAME and not meta.get('route_kind'):
        return ROUTE_BY_NAME[name]
    return None
