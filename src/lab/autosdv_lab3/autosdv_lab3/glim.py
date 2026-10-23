"""`just lab3 map BAG`: build the raw point cloud map of a mapping bag with GLIM.

    BAG --(topics, /tf_static)--> OUT/glim_config/ --glim_rosbag--> OUT/glim_dump/
        --export--> OUT/raw_map.pcd (x y z intensity, GLIM world frame)
                    OUT/glim_report.yaml

The config is generated per run: GLIM's installed defaults, then the classroom
tuning in config/glim/common/, then the sensor family's files in config/glim/<family>/,
then what only this bag knows -- its topics and its own LiDAR -> IMU transform.
config/glim/README.md says what each override does.

The exporter reads GLIM's dump directly (GLIM 1.2.2, glim/src/glim/mapping/sub_map.cpp
and gtsam_points PointCloud::save_compact): one directory per submap, `%06d/`, with
`data.txt` (T_world_origin, the optimised submap pose, as a 4x4 after its label) and
`points_compact.bin` (float32 xyz) plus `intensities_compact.bin` (float32), the points
in the submap's origin frame. GlobalMapping::export_points does the same product,
T_world_origin * p; doing it here keeps the intensities and needs no viewer.
"""

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import mapdir

# The raw LiDAR topic the AutoSDV drivers publish, per sensor family (LIDAR_TOPICS in
# autosdv_sensor_kit_launch/launch/pointcloud_preprocessor.launch.py). The second
# names are launch-argument defaults the sensor kit overrides; accepted so a bag
# recorded from a hand-launched driver still resolves.
LIDAR_TOPICS: List[Tuple[str, str]] = [
    ('/sensing/lidar/velodyne_points', 'vlp16'),
    ('/sensing/lidar/iv_points', 'robin_w'),
    ('/sensing/lidar/top/pointcloud_raw_ex', 'vlp16'),
    ('/sensing/lidar/robin_w/pointcloud_raw', 'robin_w'),
]
# Raw ZED IMU first: the corrected one has had a bias removed that GLIM estimates
# itself, and it is in base_link's frame only if the corrector ran.
IMU_TOPICS = ['/sensing/camera/zedxm/imu/data', '/sensing/imu/imu_data']
FAMILIES = ('vlp16', 'robin_w')

POINTS_TYPE = 'sensor_msgs/msg/PointCloud2'
IMU_TYPE = 'sensor_msgs/msg/Imu'

RAW_MAP = 'raw_map.pcd'
REPORT = 'glim_report.yaml'
CONFIG_DIR = 'glim_config'
DUMP_DIR = 'glim_dump'
LOG = 'glim_run.log'

# The intensity a retroreflector returns above; the board anchor (F4) uses the same
# order of magnitude, so the report says how much of the map is that bright.
BRIGHT = 100.0


class Lab3MapError(RuntimeError):
    """A failure the user can act on; printed without a traceback."""


# ---------------------------------------------------------------- bag metadata

def bag_dir(bag: str) -> str:
    """The bag directory; a .db3/.mcap path inside one is accepted too."""
    bag = os.path.abspath(bag)
    if os.path.isfile(bag):
        bag = os.path.dirname(bag)
    if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        raise Lab3MapError(f'{bag} is not a rosbag2 directory (no metadata.yaml)')
    return bag


def read_metadata(bag: str) -> dict:
    """metadata.yaml as {'topics': {name: (type, count)}, 'storage', 'duration', 'files'}."""
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    topics = {}
    for t in info.get('topics_with_message_count', []):
        md = t['topic_metadata']
        topics[md['name']] = (md['type'], int(t.get('message_count', 0)))
    files = info.get('relative_file_paths') or [f['path'] for f in info.get('files', [])]
    return {
        'topics': topics,
        'storage': info.get('storage_identifier', ''),
        'duration': info.get('duration', {}).get('nanoseconds', 0) / 1e9,
        'files': [os.path.join(bag, os.path.basename(p)) for p in files],
    }


def detect_topics(topics: Dict[str, Tuple[str, int]], sensor: str = 'auto',
                  lidar_topic: Optional[str] = None,
                  imu_topic: Optional[str] = None) -> Tuple[str, str, str]:
    """(lidar_topic, imu_topic, family) for this bag.

    A topic given by hand must be in the bag with messages; a hand-given LiDAR topic
    that is not one of the known names needs --sensor, since the family decides the
    preprocessing.
    """
    def present(name, typ):
        t = topics.get(name)
        return t is not None and t[0] == typ and t[1] > 0

    if lidar_topic:
        if not present(lidar_topic, POINTS_TYPE):
            raise Lab3MapError(
                f'{lidar_topic} is not a PointCloud2 topic with messages in the bag')
        family = dict(LIDAR_TOPICS).get(lidar_topic)
    else:
        found = [(t, f) for t, f in LIDAR_TOPICS if present(t, POINTS_TYPE)]
        if not found:
            known = ', '.join(t for t, _ in LIDAR_TOPICS)
            raise Lab3MapError(f'no LiDAR topic in the bag (looked for {known}); '
                               'name one with --lidar-topic')
        lidar_topic, family = found[0]

    if sensor != 'auto':
        family = sensor
    if family is None:
        raise Lab3MapError(f'{lidar_topic} is not a known AutoSDV LiDAR topic; '
                           f'say which sensor it is with --sensor {"|".join(FAMILIES)}')

    if imu_topic:
        if not present(imu_topic, IMU_TYPE):
            raise Lab3MapError(f'{imu_topic} is not an Imu topic with messages in the bag')
    else:
        found = [t for t in IMU_TOPICS if present(t, IMU_TYPE)]
        if not found:
            raise Lab3MapError(f'no IMU topic in the bag (looked for {", ".join(IMU_TOPICS)}); '
                               'name one with --imu-topic')
        imu_topic = found[0]
    return lidar_topic, imu_topic, family


# ---------------------------------------------------------------- transforms

def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    """(qx, qy, qz, qw) -> 3x3; normalised, since bags carry rounded quaternions."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(r: np.ndarray) -> np.ndarray:
    """3x3 -> (qx, qy, qz, qw), qw >= 0."""
    m = np.asarray(r, float)
    tr = np.trace(m)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0] * 4
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
        q[3] = (m[k, j] - m[j, k]) / s
    q = np.array(q)
    return -q if q[3] < 0 else q


def make_T(xyz: Sequence[float], q: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q)
    T[:3, 3] = xyz
    return T


def T_to_tum(T: np.ndarray) -> List[float]:
    """4x4 -> [x, y, z, qx, qy, qz, qw], GLIM's format for T_lidar_imu."""
    return [float(v) for v in T[:3, 3]] + [float(v) for v in rot_to_quat(T[:3, :3])]


def _frame(name: str) -> str:
    return name.lstrip('/')


def chain_transform(transforms: Sequence[Tuple[str, str, Sequence[float], Sequence[float]]],
                    target: str, source: str) -> np.ndarray:
    """T_target_source from a set of (parent, child, xyz, quat) static transforms.

    Each entry is the child's pose in the parent, as tf2 stores it. The answer maps a
    point in `source` into `target` -- for target=LiDAR, source=IMU that is GLIM's
    T_lidar_imu (p_lidar = T_lidar_imu * p_imu). Raises when the two frames are not
    in one tree.
    """
    parent: Dict[str, Tuple[str, np.ndarray]] = {}
    for p, c, xyz, q in transforms:
        parent[_frame(c)] = (_frame(p), make_T(xyz, q))   # later messages win, as in tf2

    def to_root(frame):
        T, seen = np.eye(4), [frame]
        while frame in parent:
            frame, T_pc = parent[frame][0], parent[frame][1]
            T = T_pc @ T
            if frame in seen:
                raise Lab3MapError(f'/tf_static has a cycle through {frame}')
            seen.append(frame)
        return frame, T

    target, source = _frame(target), _frame(source)
    root_t, T_root_target = to_root(target)
    root_s, T_root_source = to_root(source)
    if root_t != root_s:
        raise Lab3MapError(f'/tf_static does not connect {target} (tree {root_t}) and '
                           f'{source} (tree {root_s})')
    return np.linalg.inv(T_root_target) @ T_root_source


def read_bag_frames(bag: str, storage: str, lidar_topic: str, imu_topic: str,
                    has_tf_static: bool, window_s: float = 5.0):
    """(lidar frame_id, imu frame_id, /tf_static transforms) from the bag itself."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    topics = [lidar_topic, imu_topic] + (['/tf_static'] if has_tf_static else [])
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    lidar_frame = imu_frame = None
    transforms = []
    t0 = None
    # /tf_static is latched, so every publisher's copy lands in the first moments of
    # the recording; reading a few seconds collects them all without reading the bag.
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t0 = stamp if t0 is None else t0
        if topic == '/tf_static':
            for tf in deserialize_message(data, TFMessage).transforms:
                t, r = tf.transform.translation, tf.transform.rotation
                transforms.append((tf.header.frame_id, tf.child_frame_id,
                                   (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)))
        elif topic == lidar_topic and lidar_frame is None:
            lidar_frame = deserialize_message(data, PointCloud2).header.frame_id
        elif topic == imu_topic and imu_frame is None:
            imu_frame = deserialize_message(data, Imu).header.frame_id
        if lidar_frame and imu_frame and (not has_tf_static or (stamp - t0) / 1e9 > window_s):
            break
    return lidar_frame, imu_frame, transforms


def parse_tum(text: str) -> np.ndarray:
    v = [float(x) for x in text.replace(',', ' ').split()]
    if len(v) != 7:
        raise Lab3MapError('--T-lidar-imu takes "x y z qx qy qz qw"')
    return make_T(v[:3], v[3:])


# ---------------------------------------------------------------- config

def load_glim_json(path: str) -> dict:
    """GLIM's config files are JSON with // and /* */ comments and the odd trailing
    comma; strip those outside strings, then parse."""
    with open(path) as f:
        text = f.read()
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == '\\' else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith('//', i):
            i = text.find('\n', i)
            i = n if i < 0 else i
        elif text.startswith('/*', i):
            i = text.find('*/', i + 2)
            i = n if i < 0 else i + 2
        else:
            out.append(c)
            i += 1
    return json.loads(re.sub(r',(\s*[}\]])', r'\1', ''.join(out)))


def deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        both = isinstance(v, dict) and isinstance(out.get(k), dict)
        out[k] = deep_merge(out[k], v) if both else v
    return out


def glim_defaults_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('glim'), 'config')
    except Exception:  # noqa: BLE001 -- ament absent or GLIM not installed: try the usual prefix
        return '/opt/ros/humble/share/glim/config'


def templates_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('autosdv_lab3'), 'config', 'glim')
        if os.path.isdir(d):
            return d
    except Exception:  # noqa: BLE001 -- running from the source tree
        pass
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'glim')


def generate_config(out_dir: str, family: str, lidar_topic: str, imu_topic: str,
                    T_lidar_imu: np.ndarray, cpu: bool, log_dir: str,
                    defaults: Optional[str] = None, templates: Optional[str] = None) -> dict:
    """Write a complete GLIM config directory; returns {file: merged dict}.

    Every default file is copied, merged with common/ and <family>/ where those have
    one, so the directory is self-contained and records exactly what ran.
    """
    defaults = defaults or glim_defaults_dir()
    templates = templates or templates_dir()
    if family not in FAMILIES:
        raise Lab3MapError(f'unknown sensor family {family}')
    if not os.path.isdir(defaults):
        raise Lab3MapError(f'GLIM config not found at {defaults}; is GLIM installed? '
                           '(./setup.sh --run --only glim)')
    kind = 'cpu' if cpu else 'gpu'
    runtime = {
        'config.json': {'global': {
            'config_path': '',
            'config_odometry': f'config_odometry_{kind}.json',
            'config_sub_mapping': f'config_sub_mapping_{kind}.json',
            'config_global_mapping': f'config_global_mapping_{kind}.json',
        }},
        'config_ros.json': {'glim_ros': {
            'imu_topic': imu_topic,
            'points_topic': lidar_topic,
            # no viewer: headless on the car, and a window on a shared display is
            # someone else's screen
            'extension_modules': [],
        }},
        'config_sensors.json': {'sensors': {'T_lidar_imu': T_to_tum(T_lidar_imu)}},
        # GLIM's default is /tmp
        'config_logging.json': {'logging': {'log_dir': log_dir}},
    }
    layers = [os.path.join(templates, 'common'), os.path.join(templates, family)]
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    merged = {}
    for name in sorted(os.listdir(defaults)):
        if not name.endswith('.json'):
            continue
        cfg = load_glim_json(os.path.join(defaults, name))
        for layer in layers:
            path = os.path.join(layer, name)
            if os.path.isfile(path):
                with open(path) as f:
                    cfg = deep_merge(cfg, json.load(f))
        cfg = deep_merge(cfg, runtime.get(name, {}))
        with open(os.path.join(out_dir, name), 'w') as f:
            json.dump(cfg, f, indent=2)
            f.write('\n')
        merged[name] = cfg
    return merged


def gpu_modules_installed() -> bool:
    prefix = os.path.dirname(glim_defaults_dir().rstrip('/'))   # .../share/glim
    lib = os.path.join(os.path.dirname(os.path.dirname(prefix)), 'lib')
    return os.path.isfile(os.path.join(lib, 'libodometry_estimation_gpu.so'))


# ---------------------------------------------------------------- run

# Lines worth showing live; everything goes to the log.
_SHOW = re.compile(r'playback speed|\[error\]|\[critical\]|what\(\)|terminate|'
                   r'waiting for|saving|serializ|cuda|while loading shared', re.IGNORECASE)
_CUDA_FAIL = re.compile(r'no kernel image|cudaError|CUDA error|invalid device function|'
                        r'unsupported (gpu|ptx)', re.IGNORECASE)


def run_glim(bag: str, config_dir: str, dump_dir: str, log_path: str) -> Tuple[int, str]:
    """Run glim_rosbag to the end of the bag; returns (exit code, the log's tail)."""
    env = dict(os.environ)
    for k in ('DISPLAY', 'WAYLAND_DISPLAY'):
        env.pop(k, None)   # headless, whatever the shell had
    # gtsam_points installs to /usr/local/lib and is found only once ldconfig has run
    # after the install; a loader cache rebuilt without it fails glim_rosbag with
    # "libgtsam_points.so.1: cannot open shared object file". Appended, so a
    # library the cache does know still wins.
    env['LD_LIBRARY_PATH'] = ':'.join(
        p for p in (env.get('LD_LIBRARY_PATH', ''), '/usr/local/lib') if p)
    cmd = ['ros2', 'run', 'glim_ros', 'glim_rosbag', bag, '--ros-args',
           '-p', f'config_path:={config_dir}',
           '-p', f'dump_path:={dump_dir}',
           '-p', 'auto_quit:=true',
           # GLIM publishes map -> odom -> base TF; on a car with the stack up that
           # would fight localization's own, so it goes to a topic nothing reads
           '-r', '/tf:=/lab3_glim/tf', '-r', '/tf_static:=/lab3_glim/tf_static']
    tail: List[str] = []
    with open(log_path, 'w') as log:
        log.write(' '.join(cmd) + '\n')
        log.flush()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=env, text=True,
                                errors='replace', start_new_session=True)
        try:
            for line in proc.stdout:
                log.write(line)
                tail.append(line)
                del tail[:-60]
                if _SHOW.search(line):
                    print('  glim | ' + line.rstrip(), flush=True)
            proc.wait()
        except KeyboardInterrupt:
            # glim_rosbag and its children share our new session; stop them all
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            raise
    return proc.returncode, ''.join(tail)


# ---------------------------------------------------------------- export

def _read_bin(path: str, dtype, width: int) -> np.ndarray:
    a = np.fromfile(path, dtype=dtype)
    if a.size % width:
        raise Lab3MapError(f'{path}: {a.size} values is not a multiple of {width}')
    return a.reshape(-1, width)


def read_submap(path: str) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    """(T_world_origin, points Nx3 in the origin frame, intensities N or None)."""
    with open(os.path.join(path, 'data.txt')) as f:
        tokens = f.read().split()
    try:
        i = tokens.index('T_world_origin:')
        T = np.array([float(v) for v in tokens[i + 1:i + 17]]).reshape(4, 4)
    except (ValueError, IndexError) as e:
        raise Lab3MapError(f'{path}/data.txt has no T_world_origin') from e

    # save_compact writes float32; the full save (PointCloud::save) doubles, xyz1
    if os.path.isfile(os.path.join(path, 'points_compact.bin')):
        pts = _read_bin(os.path.join(path, 'points_compact.bin'), np.float32, 3).astype(float)
    elif os.path.isfile(os.path.join(path, 'points.bin')):
        pts = _read_bin(os.path.join(path, 'points.bin'), np.float64, 4)[:, :3]
    else:
        raise Lab3MapError(f'{path} has no points_compact.bin or points.bin')

    inten = None
    for name, dtype in (('intensities_compact.bin', np.float32), ('intensities.bin', np.float64)):
        p = os.path.join(path, name)
        if os.path.isfile(p):
            inten = np.fromfile(p, dtype=dtype).astype(float)
            break
    if inten is not None and len(inten) != len(pts):
        raise Lab3MapError(f'{path}: {len(pts)} points but {len(inten)} intensities')
    return T, pts, inten


def export_dump(dump: str) -> Tuple[np.ndarray, Optional[np.ndarray], int]:
    """All submaps in GLIM's world frame: (points Nx3, intensities N or None, submaps)."""
    subs = sorted(d for d in os.listdir(dump)
                  if re.fullmatch(r'\d{6}', d) and os.path.isdir(os.path.join(dump, d)))
    if not subs:
        raise Lab3MapError(f'{dump} has no submaps; GLIM produced no map (see {LOG})')
    graph = os.path.join(dump, 'graph.txt')
    if os.path.isfile(graph):
        with open(graph) as f:
            m = re.search(r'num_submaps:\s*(\d+)', f.read())
        if m and int(m.group(1)) != len(subs):
            raise Lab3MapError(f'{dump}: graph.txt says {m.group(1)} submaps, found {len(subs)} '
                               '(a stale dump mixed with a new one?)')
    pts_all, int_all, missing = [], [], 0
    for s in subs:
        T, pts, inten = read_submap(os.path.join(dump, s))
        pts_all.append(pts @ T[:3, :3].T + T[:3, 3])
        if inten is None:
            missing += 1
            inten = np.zeros(len(pts))
        int_all.append(inten)
    points = np.concatenate(pts_all)
    if missing == len(subs):
        return points, None, len(subs)
    if missing:
        raise Lab3MapError(f'{missing} of {len(subs)} submaps carry no intensities')
    return points, np.concatenate(int_all), len(subs)


def nn_spacing(points: np.ndarray, sample: int = 20000, seed: int = 0) -> Optional[float]:
    """Median nearest-neighbour distance over a sample: the map's point spacing."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        return None
    if len(points) < 2:
        return None
    idx = np.random.default_rng(seed).choice(len(points), min(sample, len(points)), replace=False)
    d, _ = cKDTree(points).query(points[idx], k=2)
    return float(np.median(d[:, 1]))


# ---------------------------------------------------------------- main

def bag_hash(files: Sequence[str]) -> str:
    """sha256 over the per-file sha256s, so a split bag has one hash."""
    import hashlib
    h = hashlib.sha256()
    for p in files:
        h.update(os.path.basename(p).encode())
        h.update(mapdir.sha256(p).encode())
    return h.hexdigest()


def glim_version() -> str:
    try:
        import xml.etree.ElementTree as ET
        share = os.path.dirname(glim_defaults_dir().rstrip('/'))
        return ET.parse(os.path.join(share, 'package.xml')).findtext('version') or 'unknown'
    except Exception:  # noqa: BLE001
        return 'unknown'


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='lab3_map',
        description='Build the raw point cloud map of a mapping bag with GLIM.')
    ap.add_argument('bag', help='the mapping bag (directory)')
    ap.add_argument('-o', '--out', help='output directory (default: <bag>_glim beside the bag)')
    ap.add_argument('--sensor', choices=('auto',) + FAMILIES, default='auto',
                    help='sensor family; auto reads it from the LiDAR topic')
    ap.add_argument('--lidar-topic', help='LiDAR PointCloud2 topic (default: detected)')
    ap.add_argument('--imu-topic', help='IMU topic (default: detected)')
    ap.add_argument('--T-lidar-imu', dest='T_lidar_imu', metavar='"x y z qx qy qz qw"',
                    help='the IMU pose in the LiDAR frame, when the bag has no /tf_static '
                         'chain between them (overrides the bag)')
    ap.add_argument('--cpu', action='store_true', help="GLIM's CPU modules instead of CUDA")
    ap.add_argument('--keep-dump', action='store_true', help='keep glim_dump/ after export')
    args = ap.parse_args(argv)

    t_start = time.time()
    try:
        bag = bag_dir(args.bag)
        meta = read_metadata(bag)
        lidar_topic, imu_topic, family = detect_topics(
            meta['topics'], args.sensor, args.lidar_topic, args.imu_topic)
        out = os.path.abspath(args.out or bag.rstrip('/') + '_glim')
        os.makedirs(out, exist_ok=True)
        for stale in (RAW_MAP, REPORT):   # a failed rerun must not leave the last map behind
            if os.path.exists(os.path.join(out, stale)):
                os.remove(os.path.join(out, stale))
        print(f'lab3_map: {bag}\n  LiDAR {lidar_topic} ({family}), IMU {imu_topic}', flush=True)

        # bag hash in the background: GLIM reads the same files at the same time
        hashed: Dict[str, str] = {}
        hasher = threading.Thread(target=lambda: hashed.update(sha=bag_hash(meta['files'])),
                                  daemon=True)
        hasher.start()

        lidar_frame, imu_frame, transforms = read_bag_frames(
            bag, meta['storage'], lidar_topic, imu_topic, '/tf_static' in meta['topics'])
        warnings: List[str] = []
        if args.T_lidar_imu:
            T_li = parse_tum(args.T_lidar_imu)
            t_source = 'command line (--T-lidar-imu)'
        else:
            try:
                if not transforms:
                    raise Lab3MapError('the bag has no /tf_static')
                if not lidar_frame or not imu_frame:
                    raise Lab3MapError('no LiDAR or IMU message carries a frame_id')
                T_li = chain_transform(transforms, lidar_frame, imu_frame)
                t_source = f'bag /tf_static ({lidar_frame} <- {imu_frame})'
            except Lab3MapError as e:
                T_li = np.eye(4)
                t_source = f'IDENTITY -- {e}'
                warnings.append(f'T_lidar_imu is identity: {e}. GLIM integrates the IMU in '
                                'the wrong frame if the two are rotated; record /tf_static, '
                                'or pass --T-lidar-imu.')
                bar = '!' * 78
                print(f'\n{bar}\n  WARNING: {warnings[-1]}\n{bar}\n', file=sys.stderr)
        print(f'  T_lidar_imu {np.round(T_to_tum(T_li), 4).tolist()}  [{t_source}]', flush=True)

        cpu = args.cpu
        if not cpu and not gpu_modules_installed():
            cpu = True
            warnings.append('the CUDA build of GLIM is not installed; ran the CPU modules')
            print(f'  {warnings[-1]}')
        cfg_dir = os.path.join(out, CONFIG_DIR)
        dump = os.path.join(out, DUMP_DIR)
        log_dir = os.path.join(out, 'glim_log')
        os.makedirs(log_dir, exist_ok=True)
        generate_config(cfg_dir, family, lidar_topic, imu_topic, T_li, cpu, log_dir)
        if os.path.exists(dump):
            shutil.rmtree(dump)   # GLIM writes into it; old submaps would survive a shorter map

        print(f'  running GLIM ({"CPU" if cpu else "GPU"}); log {os.path.join(out, LOG)}')
        t_glim = time.time()
        code, tail = run_glim(bag, cfg_dir, dump, os.path.join(out, LOG))
        glim_s = time.time() - t_glim
        if not os.path.isdir(dump) or not os.listdir(dump):
            hint = ''
            if not cpu and _CUDA_FAIL.search(tail):
                hint = '\nThe CUDA modules failed on this GPU; try --cpu.'
            elif 'failed to create domain' in tail:
                # glim_rosbag reads the bag itself but is still a ROS node
                hint = ('\nDDS would not start: CYCLONEDDS_URI asks for socket buffers the '
                        'kernel refuses (./setup.sh --run --only cyclonedds-sysctl).')
            raise Lab3MapError(f'GLIM exited ({code}) without a dump.{hint}\n'
                               f'--- log tail ---\n{tail[-3000:]}')

        points, inten, n_sub = export_dump(dump)
        if inten is None:
            warnings.append('GLIM kept no intensities (does the LiDAR topic have an '
                            '"intensity" field?); the board anchor needs them')
            print(f'  WARNING: {warnings[-1]}', file=sys.stderr)
        from reflective_pose_core.pointcloud_io import PointCloud, write_pcd
        write_pcd(os.path.join(out, RAW_MAP), PointCloud(points=points, intensity=inten))
        if not args.keep_dump:
            shutil.rmtree(dump)

        hasher.join()
        lo, hi = points.min(axis=0), points.max(axis=0)
        spacing = nn_spacing(points)
        report = {
            'sensor': family,
            'bag': bag,
            'bag_bytes': int(sum(os.path.getsize(p) for p in meta['files'] if os.path.isfile(p))),
            'bag_sha256': hashed.get('sha', 'unknown'),
            'bag_duration_s': round(meta['duration'], 1),
            'lidar_topic': lidar_topic,
            'imu_topic': imu_topic,
            'lidar_frame': lidar_frame,
            'imu_frame': imu_frame,
            'T_lidar_imu': [round(v, 6) for v in T_to_tum(T_li)],
            'T_lidar_imu_source': t_source,
            'glim': {'version': glim_version(), 'modules': 'cpu' if cpu else 'gpu',
                     'config': cfg_dir, 'exit_code': code},
            'glim_wall_s': round(glim_s, 1),
            'total_wall_s': round(time.time() - t_start, 1),
            'submaps': n_sub,
            'points': int(len(points)),
            'extent_min': [round(float(v), 3) for v in lo],
            'extent_max': [round(float(v), 3) for v in hi],
            'size_m': [round(float(v), 2) for v in hi - lo],
            'median_spacing_m': None if spacing is None else round(spacing, 4),
            'intensity': None if inten is None else {
                'min': float(inten.min()), 'max': float(inten.max()),
                f'fraction_above_{BRIGHT:g}': round(float(np.mean(inten > BRIGHT)), 5)},
            'raw_map': os.path.join(out, RAW_MAP),
            'warnings': warnings,
            'autosdv': mapdir.git_rev(),
            'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        }
        with open(os.path.join(out, REPORT), 'w') as f:
            yaml.safe_dump(report, f, sort_keys=False)
    except Lab3MapError as e:
        print(f'lab3_map: {e}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('lab3_map: interrupted', file=sys.stderr)
        return 130

    size = report['size_m']
    spaced = f', spacing {spacing * 100:.1f} cm' if spacing else ''
    print(f'\nlab3_map: {report["points"]:,} points in {n_sub} submaps, '
          f'{size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} m{spaced}')
    print(f'  GLIM {glim_s:.0f} s, total {report["total_wall_s"]:.0f} s')
    print(f'  {report["raw_map"]}\n  {os.path.join(out, REPORT)}')
    for w in warnings:
        print(f'  WARNING: {w}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
