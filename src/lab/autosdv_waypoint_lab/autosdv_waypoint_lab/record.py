"""`just lab2 record`: the submission bag.

    lab2_record --car A --team 3 [--name NAME] [--course-dir DIR] [--bench]

Records bag_meta.RECORD_TOPICS plus two topics this tool makes itself:

    /lab2/camera/image/compressed   the ZED's own compressed stream, thinned to
                                    10 Hz and resized to 960 x 600 (JPEG)
    /lab2/record_meta               a latched JSON String: course name and
                                    hashes, car, team, git commit

and writes the same JSON beside the bag as lab2_meta.json. The course is the
one the RUNNING stack drives (the path publisher's `course_dir` parameter), not
one named on the command line, so a bag cannot describe a course it was not
recorded on. Ctrl-C stops the recording.

The ZED's compressed stream comes from image_transport's plugin on the
wrapper's rectified image. The subscription is raw (serialised bytes), so a frame
that is thinned away is never deserialised; a kept frame is deserialised, decoded,
resized and re-encoded. Measured on the Orin with every frame deserialised (rclpy
building a Python message from each 656 kB frame at 15 Hz): 43.5 % of one core.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from typing import Optional

from autosdv_waypoint_lab import bag_meta as bm

PATH_PUBLISHER = '/planning/waypoint_path_publisher'


def git_info(cwd: Optional[str] = None):
    def run(*a):
        return subprocess.run(['git', *a], cwd=cwd, capture_output=True, text=True, timeout=10)
    try:
        commit = run('rev-parse', 'HEAD').stdout.strip() or 'unknown'
        dirty = bool(run('status', '--porcelain', '--untracked-files=no').stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return 'unknown', False
    return commit, dirty


def keep_slot(now: float, period: float, last_slot: Optional[int]):
    """(keep, slot): one frame per `period` slot, so a 15 Hz stream gives 10 Hz and a
    30 Hz one too. Decided from the arrival time alone, before any deserialising."""
    slot = int(now / period)
    return slot != last_slot, slot


def resize_jpeg(data: bytes, size=bm.CAMERA_SIZE, quality: int = 80) -> Optional[bytes]:
    import cv2
    import numpy as np
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    if (img.shape[1], img.shape[0]) != tuple(size):
        img = cv2.resize(img, tuple(size), interpolation=cv2.INTER_AREA)
    ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return enc.tobytes() if ok else None


def make_node(meta_json: str, source: str, rate: float):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    class Aux(Node):
        def __init__(self):
            super().__init__('lab2_record')
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.pub_meta = self.create_publisher(String, bm.META_TOPIC, latched)
            self.pub_img = self.create_publisher(CompressedImage, bm.CAMERA_TOPIC, 5)
            self.meta = meta_json
            self.period = 1.0 / rate
            self.last_slot = None
            self.n_in = self.n_out = 0
            # raw=True: the callback gets the serialised bytes, so a thinned-away frame
            # costs no Python message construction
            self.create_subscription(CompressedImage, source, self.on_image,
                                     qos_profile_sensor_data, raw=True)
            self.create_timer(1.0, self.on_timer)
            self.on_timer()

        def on_timer(self):
            self.pub_meta.publish(String(data=self.meta))

        def on_image(self, raw):
            self.n_in += 1
            keep, slot = keep_slot(time.monotonic(), self.period, self.last_slot)
            if not keep:
                return
            msg = deserialize_message(raw, CompressedImage)
            data = resize_jpeg(bytes(msg.data))
            if data is None:
                return
            self.last_slot = slot
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = data
            self.pub_img.publish(out)
            self.n_out += 1

    rclpy.init()
    return Aux()


def missing_path_file(course: str) -> Optional[str]:
    """The course's path file name if it is not on disk (a survey-only course), else None."""
    from autosdv_waypoint_lab.course import load_course
    c = load_course(course)
    return None if os.path.isfile(c.path_file) else os.path.basename(c.path_file)


def course_dir_from_stack(timeout: float = 15.0) -> Optional[str]:
    import rclpy
    from rcl_interfaces.srv import GetParameters
    node = rclpy.create_node('lab2_record_probe')
    cli = node.create_client(GetParameters, PATH_PUBLISHER + '/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        req = GetParameters.Request()
        req.names = ['course_dir']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if fut.result() is None or not fut.result().values:
            return None
        return fut.result().values[0].string_value or None
    finally:
        node.destroy_node()


def main(argv=None):
    ap = argparse.ArgumentParser(description='Record the Lab 2 submission bag.')
    ap.add_argument('--car', required=True, help='car ID: A, B or C')
    ap.add_argument('--team', required=True, help='team ID')
    ap.add_argument('--name', default='', help='bag name (default: the time)')
    ap.add_argument('--out-dir', default='rosbags')
    ap.add_argument('--course-dir', default=None,
                    help='only if the stack is not running (tests); default: asked of the path publisher')
    ap.add_argument('--bench', action='store_true', help='also record the bench plant\'s true pose')
    ap.add_argument('--camera-source', default=bm.CAMERA_SOURCE)
    args = ap.parse_args(argv)

    import rclpy
    rclpy.init()
    course = args.course_dir or course_dir_from_stack()
    if not course:
        print(f'lab2_record: no {PATH_PUBLISHER} answered; start the stack first '
              '(just lab2 vehicle ...)', file=sys.stderr)
        return 2
    rclpy.shutdown()

    commit, dirty = git_info()
    if not os.path.isfile(os.path.join(course, 'course.yaml')):
        print(f'lab2_record: {course} has no course.yaml; is it a course directory?',
              file=sys.stderr)
        return 2
    missing = missing_path_file(course)
    if missing:
        print(f'lab2_record: course {course} has no {missing}; write the path first '
              '(the marker survey does not write it), then record', file=sys.stderr)
        return 2
    meta = bm.build_meta(course, args.car, args.team, commit, time.time())
    meta['git_dirty'] = dirty
    meta['course_dir'] = os.path.abspath(course)
    meta_json = json.dumps(meta, sort_keys=True)
    print(f'course {meta["course_name"]} ({meta["route_kind"]})  route_hash {meta["route_hash"][:12]}  '
          f'car {args.car}  team {args.team}  commit {commit[:10]}{"+dirty" if dirty else ""}')

    out = os.path.join(args.out_dir, 'lab2_' + (args.name or time.strftime('%Y%m%d_%H%M%S')))
    os.makedirs(args.out_dir, exist_ok=True)
    node = make_node(meta_json, args.camera_source, bm.CAMERA_RATE)
    topics = list(bm.RECORD_TOPICS) + [bm.CAMERA_TOPIC, bm.META_TOPIC]
    if args.bench:
        topics.append(bm.TRUE_POSE)
    cmd = ['ros2', 'bag', 'record', '-o', out, *topics]
    print('recording to', out)
    proc = subprocess.Popen(cmd, start_new_session=True)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    try:
        while proc.poll() is None and not stop['now']:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        if os.path.isdir(out):
            with open(os.path.join(out, bm.SIDECAR), 'w') as f:
                json.dump(meta, f, indent=2, sort_keys=True)
        print(f'camera frames: {node.n_in} in, {node.n_out} out')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
