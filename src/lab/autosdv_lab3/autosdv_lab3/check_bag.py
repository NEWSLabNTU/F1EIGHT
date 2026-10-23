"""`just lab3 check-bag`: can GLIM and the anchor use this mapping bag?

    lab3_check_bag BAG [--max-speed 0.8] [--min-duration 60]

Each requirement prints PASS or FAIL with the measured value. Exit 0 only if all pass.
Only the first LiDAR message is deserialised (for its fields and frame); the rest of
the bag is read at the cost of its small topics.
"""

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from autosdv_lab3 import cars

TIME_FIELDS = ('time_stamp', 'timestamp', 't', 'time', 'time_offset')
INTENSITY_FIELDS = ('intensity', 'reflectivity')
STILL_SPEED = 0.05     # m/s
STILL_SECONDS = 8.0    # of the 10 s asked for, so a slow Ctrl-C does not fail the bag
BOARD_WINDOW = 15.0    # s at each end in which the board must be detected
MIN_IMU_HZ = 100.0


@dataclass
class BagScan:
    topics: Dict[str, str] = field(default_factory=dict)       # name -> type
    counts: Dict[str, int] = field(default_factory=dict)
    start: float = 0.0
    end: float = 0.0
    meta: dict = field(default_factory=dict)
    lidar_topic: Optional[str] = None
    lidar_fields: List[str] = field(default_factory=list)
    lidar_frame: str = ''
    imu_frame: str = ''
    imu_times: List[float] = field(default_factory=list)
    static_edges: List[tuple] = field(default_factory=list)    # (parent, child)
    speeds: List[tuple] = field(default_factory=list)          # (t, speed)
    board_times: List[float] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return self.end - self.start


def _lidar_topic(topics, meta) -> Optional[str]:
    if meta.get('lidar_topic') in topics:
        return meta['lidar_topic']
    for topic in cars.LIDAR_TOPIC.values():
        if topic in topics:
            return topic
    return None


def scan_bag(path: str) -> BagScan:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    s = BagScan()
    s.topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    sidecar = os.path.join(path, 'lab3_meta.json')
    if os.path.isfile(sidecar):
        with open(sidecar) as f:
            s.meta = json.load(f)
    s.lidar_topic = _lidar_topic(s.topics, s.meta)
    wanted = {cars.IMU_RAW, '/tf_static', cars.VELOCITY, cars.BOARD_POSE_MAPPING,
              '/lab3/record_meta'}
    types = {t: get_message(s.topics[t]) for t in wanted | {s.lidar_topic} if t in s.topics}
    first = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        first = t if first is None else first
        s.end = t
        s.counts[topic] = s.counts.get(topic, 0) + 1
        if topic == s.lidar_topic:
            if not s.lidar_fields:
                msg = deserialize_message(raw, types[topic])
                s.lidar_fields = [f.name for f in msg.fields]
                s.lidar_frame = msg.header.frame_id
        elif topic == cars.IMU_RAW:
            s.imu_times.append(t)
            if not s.imu_frame:
                s.imu_frame = deserialize_message(raw, types[topic]).header.frame_id
        elif topic == '/tf_static':
            for tf in deserialize_message(raw, types[topic]).transforms:
                s.static_edges.append((tf.header.frame_id, tf.child_frame_id))
        elif topic == cars.VELOCITY:
            msg = deserialize_message(raw, types[topic])
            s.speeds.append((t, abs(msg.longitudinal_velocity)))
        elif topic == cars.BOARD_POSE_MAPPING:
            s.board_times.append(t)
        elif topic == '/lab3/record_meta' and not s.meta:
            s.meta = json.loads(deserialize_message(raw, types[topic]).data)
    s.start = first or 0.0
    return s


def connected(edges, a: str, b: str) -> bool:
    """Is there a chain of static transforms between frames a and b, either direction?"""
    graph = {}
    for p, c in edges:
        graph.setdefault(p, set()).add(c)
        graph.setdefault(c, set()).add(p)
    seen, todo = {a}, [a]
    while todo:
        f = todo.pop()
        if f == b:
            return True
        for n in graph.get(f, ()):
            if n not in seen:
                seen.add(n)
                todo.append(n)
    return False


def still_seconds(speeds, from_start: bool) -> float:
    """How long the car stood still at the start (or the end) of the bag."""
    seq = speeds if from_start else list(reversed(speeds))
    if not seq:
        return 0.0
    t0 = seq[0][0]
    last = t0
    for t, v in seq:
        if v > STILL_SPEED:
            break
        last = t
    return abs(last - t0)


def check(s: BagScan, max_speed: float, min_duration: float):
    """[(requirement, ok, measured)]"""
    rows = []
    rows.append(('LiDAR topic', s.lidar_topic is not None and s.counts.get(s.lidar_topic, 0) > 0,
                 f'{s.lidar_topic or "none of " + ", ".join(cars.LIDAR_TOPIC.values())}: '
                 f'{s.counts.get(s.lidar_topic, 0)} scans'))
    has_i = any(f in s.lidar_fields for f in INTENSITY_FIELDS)
    has_t = any(f in s.lidar_fields for f in TIME_FIELDS)
    rows.append(('LiDAR intensity and per-point time', has_i and has_t,
                 'fields ' + (' '.join(s.lidar_fields) or 'unknown')))
    imu_hz = 0.0
    if len(s.imu_times) > 1:
        imu_hz = (len(s.imu_times) - 1) / max(s.imu_times[-1] - s.imu_times[0], 1e-9)
    rows.append((f'IMU >= {MIN_IMU_HZ:g} Hz', imu_hz >= MIN_IMU_HZ,
                 f'{cars.IMU_RAW}: {imu_hz:.0f} Hz'))
    chain = bool(s.lidar_frame and s.imu_frame and
                 connected(s.static_edges, s.lidar_frame, s.imu_frame))
    rows.append(('/tf_static links LiDAR and IMU', chain,
                 f'{s.lidar_frame or "?"} <-> {s.imu_frame or "?"}, '
                 f'{len(s.static_edges)} static transforms'))
    rows.append((f'duration >= {min_duration:g} s', s.duration >= min_duration,
                 f'{s.duration:.0f} s'))
    vmax = max((v for _, v in s.speeds), default=None)
    rows.append((f'speed <= {max_speed:g} m/s', vmax is not None and vmax <= max_speed,
                 'no velocity_status' if vmax is None else f'max {vmax:.2f} m/s'))
    head, tail = still_seconds(s.speeds, True), still_seconds(s.speeds, False)
    rows.append((f'still >= {STILL_SECONDS:g} s at start and end',
                 head >= STILL_SECONDS and tail >= STILL_SECONDS,
                 f'start {head:.0f} s, end {tail:.0f} s'))
    at_start = any(t - s.start <= BOARD_WINDOW for t in s.board_times)
    at_end = any(s.end - t <= BOARD_WINDOW for t in s.board_times)
    rows.append((f'board detected in the first and last {BOARD_WINDOW:g} s', at_start and at_end,
                 f'{len(s.board_times)} detections; start {"yes" if at_start else "no"}, '
                 f'end {"yes" if at_end else "no"}'))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(prog='lab3_check_bag',
                                 description='Check a Lab 3 mapping bag.')
    ap.add_argument('bag')
    ap.add_argument('--max-speed', type=float, default=0.8)
    ap.add_argument('--min-duration', type=float, default=60.0)
    args = ap.parse_args(argv)
    if not os.path.isdir(args.bag):
        print(f'lab3_check_bag: no bag at {args.bag}', file=sys.stderr)
        return 2
    rows = check(scan_bag(args.bag), args.max_speed, args.min_duration)
    width = max(len(r[0]) for r in rows)
    for name, ok, measured in rows:
        print(f'{"PASS" if ok else "FAIL"}  {name:<{width}}  {measured}')
    bad = sum(not ok for _, ok, _ in rows)
    print(f'\n{args.bag}: ' + ('usable for mapping' if not bad else f'{bad} requirement(s) failed'))
    return 0 if not bad else 1


if __name__ == '__main__':
    raise SystemExit(main())
