#!/usr/bin/env python3
"""`just lab2 survey-align [SURVEY_DIR]`: stage 2 of the Lab 2 survey.

    just lab2 survey-align                     surveys/latest, route from the settings
    just lab2 survey-align surveys/2026-10-16_10-02-11 [--route lab2_uturn]
    just lab2 survey-align --force             write even if a check refuses (with a warning)
    just lab2 survey-align --bench --truth DIR the vehicle bench: no car (below)

Stage 1 (`just lab2 survey ID`) measured where the boards stand relative to one
another, in a frame anchored on one board. This puts that map into the ROUTE's
frame, the one the taped route and path.csv use: origin at the start box, x
along the first leg (autosdv_marker_survey/align.py has the maths).

  1. the stage-1 result is kept in SURVEY_DIR/stage1/ and the localization stack
     starts on it (launch waypoint_survey_align.launch.xml: ZED, AR-tag localizer,
     EKF; the actuator as a dry run, no tracker -- nothing here can drive the
     car). Logs: SURVEY_DIR/align_logs/.
  2. the car stands in the taped start box, pointing along the first leg. The
     tool finds the boards in the camera image and seeds the EKF from them
     (start_pose_init cannot: nothing yet says where the start box is in the
     stage-1 frame), waits for the marker fixes to settle, and prints READY.
  3. the TA drives the car STRAIGHT along the taped first leg, by transmitter or
     by hand, 2 m or more (under 1 m is refused), stops, and presses Ctrl-C.
  4. the stack stops; the poses go to SURVEY_DIR/align_poses.csv; the aligned
     course (course.yaml, path.csv, the map, ar_tag.param.yaml) and
     align_report.txt are written into SURVEY_DIR. A refused alignment writes
     only the report; `lab2_survey_align_solve SURVEY_DIR --force` aligns the
     recorded drive again without the car.

Afterwards `just lab2 loc-check` and `just lab2 run` use the aligned survey
(surveys/latest) with the route's path.

--bench: the same on the vehicle bench. The bench plant starts at the start
pose of --truth DIR (the field as it really is), mock_aruco_camera renders that
course's boards, and bench_straight_driver stands in for the TA (it drives
--distance m at --speed m/s once this says READY, and refuses to run where the
actuator publishes); the run ends by itself when the driver has stopped.

Exit status: 0 aligned, 1 refused or failed, 2 usage.
"""

import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import List, Optional

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lab2_config as lc  # noqa: E402
from lab2_run import Child  # noqa: E402

REPO = lc.REPO
SEED_FRAMES = 5             # frames with a board in view, for the EKF seed
SEED_RANGE = 4.0            # m: boards farther than this are not used for the seed
SEED_POSITION_STD = 0.3     # m
SEED_YAW_STD = math.radians(10.0)
READY_FIXES = 10            # accepted marker fixes after the seed
READY_SETTLE = 2.0          # s after the first accepted fix
READY_STD = 0.2             # m, the EKF's sqrt(var_x + var_y)
PNP_PERIOD = 0.1            # s between multi-board poses while recording
RESEED_AFTER = 10.0         # s with no accepted fix after a seed: seed again
BENCH_END_REST = 1.5        # s recorded after the bench driver has stopped
IMAGE = '/sensing/camera/zedxm/rgb/color/rect/image/compressed'
CAMERA_INFO = '/sensing/camera/zedxm/rgb/color/rect/camera_info'
STATE_TOPIC = '/survey_align/state'

INTERRUPTED: List[str] = []


class Usage(Exception):
    pass


def parse(argv):
    import argparse
    ap = argparse.ArgumentParser(prog='just lab2 survey-align', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('survey_dir', nargs='?', default='surveys/latest')
    ap.add_argument('--route', default=None, help='the route template (default: the settings\' route)')
    ap.add_argument('--force', action='store_true', help='write even if a check refuses')
    ap.add_argument('--min-distance', type=float, default=None)
    ap.add_argument('--max-lateral', type=float, default=None)
    ap.add_argument('--ready-timeout', type=float, default=120.0)
    ap.add_argument('--bench', action='store_true', help='the vehicle bench (no car)')
    ap.add_argument('--truth', default=None, help='--bench: the course the mock camera renders')
    ap.add_argument('--distance', type=float, default=3.0, help='--bench: metres to drive')
    ap.add_argument('--speed', type=float, default=0.3, help='--bench: m/s')
    ap.add_argument('--bench-timeout', type=float, default=90.0)
    ap.add_argument('--config', default=None)
    a = ap.parse_args(argv)
    if a.bench and not a.truth:
        raise Usage('--bench needs --truth DIR: the course the mock camera renders')
    if a.truth and not a.bench:
        raise Usage('--truth is for the bench (--bench)')
    return a


def resolve_route(a) -> str:
    from autosdv_marker_survey import align as al
    route = a.route
    if route is None:
        cli = {}
        c = lc.Config(cli, a.config)
        route = c.require('route')
    return al.route_template(route)


# ------------------------------------------------------------ the ROS side

def make_node(stage1_dir: str):
    import numpy as np
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from autoware_vehicle_msgs.msg import VelocityReport
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool
    import tf2_ros

    from autosdv_marker_survey import align as al
    from autosdv_marker_survey.survey import (
        Detector, inv_T, make_T, marker_pose_in_camera, quat_to_matrix)
    from autosdv_waypoint_lab.course import load_course

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    class AlignNode(Node):
        def __init__(self):
            super().__init__('survey_align')
            course = load_course(stage1_dir)
            self.marker_size = course.marker_size
            self.T_map_marker = {}
            for m in course.markers:
                xa, ya, za = m.axes()
                self.T_map_marker[m.id] = make_T(np.column_stack([xa, ya, za]), [m.x, m.y, m.z])
            self.detector = Detector(course.dictionary)
            self.corners = al.marker_corner_points(course.markers, course.marker_size)
            self.K = None
            self.T_base_cam = None
            self.pnp: List = []
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_init = self.create_publisher(
                PoseWithCovarianceStamped, '/localization/pose_twist_fusion_filter/initialpose3d',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
            self.pub_state = self.create_publisher(String, STATE_TOPIC, latched)
            self.trigger = self.create_client(SetBool, '/localization/pose_twist_fusion_filter/trigger_node')
            self.create_subscription(Odometry, '/localization/kinematic_state', self.on_pose, 50)
            self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.on_vel, 10)
            self.create_subscription(PoseWithCovarianceStamped,
                                     '/localization/pose_estimator/pose_with_covariance', self.on_fix, 10)
            self.create_subscription(CameraInfo, CAMERA_INFO, self.on_info, qos_profile_sensor_data)
            self.img_sub = self.create_subscription(CompressedImage, IMAGE, self.on_image,
                                                    qos_profile_sensor_data)
            self.create_subscription(Bool, '/bench_straight_driver/done', self.on_done, latched)
            # bench only: the plant's TRUE pose, kept beside the localized one for align_bench
            self.truth: List = []
            self.create_subscription(Odometry, '/vehicle_bench_plant/odometry', self.on_truth, 50)
            self.phase = 'seeding'
            self.seeds = []           # (x, y, yaw) per frame
            self.last_img = 0.0
            self.seed_pose = None
            self.seeded_at = None
            self.pending = None
            self.fixes = 0
            self.first_fix = None
            self.speed = math.nan
            self.pose = None
            self.std = math.inf
            self.recording = False
            self.samples: List = []
            self.bench_done_at = None
            self.notes: List[str] = []
            self.set_state('seeding')

        def note(self, text):
            if not self.notes or self.notes[-1] != text:
                self.notes.append(text)

        def set_state(self, s):
            self.pub_state.publish(String(data=s))

        # ---- seeding from the boards in view
        def on_info(self, m):
            self.K = np.array(m.p, float).reshape(3, 4)[:, :3]

        def on_image(self, m):
            """Seeding: the EKF's first pose from the nearest board. Recording: a pose
            from every board in view at once (align.pose_from_markers), for the start."""
            if self.K is None or self.phase not in ('seeding', 'ready') or self.pending is not None:
                return
            now = time.monotonic()
            if now - self.last_img < (0.3 if self.phase == 'seeding' else PNP_PERIOD):
                return
            self.last_img = now
            import cv2
            img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
            if img is None:
                return
            if self.T_base_cam is None:
                try:
                    tf = self.tf_buffer.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time())
                except Exception:  # noqa: BLE001
                    self.note('waiting for TF base_link -> camera')
                    return
                t, q = tf.transform.translation, tf.transform.rotation
                self.T_base_cam = make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
            dets = self.detector.detect(img)
            if self.phase == 'ready':
                if self.pose is None:
                    return
                x, y, yaw = self.pose
                guess = make_T(quat_to_matrix(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), [x, y, 0.0])
                r = al.pose_from_markers(dets, self.K, self.corners, self.T_base_cam, guess)
                if r is not None:
                    T, rms, used, sd = r
                    stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
                    self.pnp.append(al.PnpSample(stamp, float(T[0, 3]), float(T[1, 3]),
                                                 math.atan2(T[1, 0], T[0, 0]), len(used), rms, sd))
                return
            best = None
            for det in dets:
                Tm = self.T_map_marker.get(det.id)
                if Tm is None:
                    continue
                T_cm, err = marker_pose_in_camera(det, self.K, self.marker_size)
                rng = float(np.linalg.norm(T_cm[:3, 3]))
                if rng > SEED_RANGE or err > 2.0:
                    continue
                T_map_base = Tm @ inv_T(T_cm) @ inv_T(self.T_base_cam)
                yaw = math.atan2(T_map_base[1, 0], T_map_base[0, 0])
                if best is None or rng < best[3]:
                    best = (float(T_map_base[0, 3]), float(T_map_base[1, 3]), yaw, rng, det.id)
            if best is None:
                return
            self.seeds.append(best)
            if len(self.seeds) >= SEED_FRAMES:
                xs = np.array([s[0] for s in self.seeds])
                ys = np.array([s[1] for s in self.seeds])
                yaw = al.circular_mean([s[2] for s in self.seeds])
                self.seed_pose = (float(np.median(xs)), float(np.median(ys)), yaw)
                self.seeds = []
                self.do_seed()

        def pose_msg(self):
            x, y, yaw = self.seed_pose
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            cov = [0.0] * 36
            cov[0] = cov[7] = SEED_POSITION_STD ** 2
            cov[14] = cov[21] = cov[28] = 0.01
            cov[35] = SEED_YAW_STD ** 2
            msg.pose.covariance = cov
            return msg

        def do_seed(self):
            if not self.trigger.service_is_ready():
                self.note('waiting for the EKF trigger service')
                return
            self.pub_init.publish(self.pose_msg())
            self.pending = self.trigger.call_async(SetBool.Request(data=True))
            self.pending.add_done_callback(self.on_triggered)

        def on_triggered(self, fut):
            self.pending = None
            try:
                ok = fut.result().success
            except Exception:  # noqa: BLE001
                ok = False
            if not ok:
                return
            self.pub_init.publish(self.pose_msg())   # the EKF resets on activation
            self.seeded_at = time.monotonic()
            self.fixes, self.first_fix = 0, None
            self.phase = 'converging'
            self.set_state('converging')

        # ---- the EKF and the fixes
        def on_fix(self, m):
            if self.phase in ('converging', 'ready'):
                self.fixes += 1
                self.first_fix = self.first_fix or time.monotonic()

        def on_vel(self, m):
            self.speed = float(m.longitudinal_velocity)

        def on_done(self, m):
            if m.data and self.bench_done_at is None:
                self.bench_done_at = time.monotonic()

        def on_truth(self, m):
            if self.recording:
                p, q = m.pose.pose.position, m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
                self.truth.append(al.Sample(t, p.x, p.y, p.z, yaw, 0.0, 0.0, m.twist.twist.linear.x, 0.0))

        def on_pose(self, m):
            p = m.pose.pose
            q = p.orientation
            R = quat_to_matrix(q.x, q.y, q.z, q.w)
            yaw = math.atan2(R[1, 0], R[0, 0])
            pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
            roll = math.atan2(R[2, 1], R[2, 2])
            c = m.pose.covariance
            self.std = math.sqrt(max(0.0, c[0]) + max(0.0, c[7]))
            t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            self.pose = (p.position.x, p.position.y, yaw)
            if self.recording:
                self.samples.append(al.Sample(t, p.position.x, p.position.y, p.position.z, yaw, roll,
                                              pitch, self.speed, self.std))

        def tick(self):
            """Phase changes that are not driven by a message."""
            now = time.monotonic()
            if self.phase == 'converging':
                if self.fixes == 0 and now - self.seeded_at > RESEED_AFTER:
                    self.phase = 'seeding'
                    self.set_state('seeding')
                    self.note('no marker fix after the seed: seeding again')
                elif (self.fixes >= READY_FIXES and self.first_fix is not None
                      and now - self.first_fix >= READY_SETTLE and self.std < READY_STD):
                    self.phase = 'ready'
                    self.recording = True
                    self.set_state('ready')
                    return 'ready'
            return None

    return rclpy, AlignNode()


# ------------------------------------------------------------ the run

def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        a = parse(argv)
    except Usage as e:
        print(f'lab2 survey-align: {e}', file=sys.stderr)
        return 2
    from autosdv_marker_survey import align as al
    survey = Path(a.survey_dir)
    if not survey.is_absolute():
        survey = REPO / survey
    if not survey.is_dir():
        print(f'lab2 survey-align: no survey directory {survey}: run `just lab2 survey ID` first',
              file=sys.stderr)
        return 1
    survey = survey.resolve()
    try:
        route = resolve_route(a)
        stage1 = al.prepare_stage1(str(survey))
    except (ValueError, lc.ConfigError) as e:
        print(f'lab2 survey-align: {e}', file=sys.stderr)
        return 1
    truth = None
    if a.bench:
        truth = Path(a.truth).resolve()
        if not (truth / 'course.yaml').is_file():
            print(f'lab2 survey-align: --truth {truth} has no course.yaml', file=sys.stderr)
            return 2
    tty = sys.stdout.isatty()

    def say(msg=''):
        if tty:
            sys.stdout.write('\r\033[K')
        print(msg, flush=True)

    def status(msg):
        if tty:
            sys.stdout.write('\r\033[K' + msg)
            sys.stdout.flush()

    env = dict(os.environ)
    if a.bench:
        env.setdefault('ROS_DOMAIN_ID', '73')
        if env['ROS_DOMAIN_ID'] == '0':
            print('lab2 survey-align: refusing the bench on ROS_DOMAIN_ID 0 (shared host)', file=sys.stderr)
            return 2
        env.setdefault('LAB2_WEB_ADDR', '127.0.0.1:8093')
        os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    logdir = survey / 'align_logs' / time.strftime('%Y-%m-%d_%H-%M-%S')
    cmd = ['play_launch', 'launch', '--log-dir', str(logdir / 'play_log'),
           '--web-addr', env.get('LAB2_WEB_ADDR', '0.0.0.0:8081'),
           'autosdv_launch', 'waypoint_survey_align.launch.xml', f'course_dir:={stage1}']
    if a.bench:
        cmd += ['bench:=true', f'truth_dir:={truth}', f'distance:={a.distance}', f'speed:={a.speed}']
    say(f'survey-align: {survey}')
    say(f'  stage 1 map  {stage1}')
    say(f'  route        {route}')
    say(f'  stack        {" ".join(cmd[cmd.index("autosdv_launch"):])}')
    say(f'  logs         {logdir}')

    def interrupt(signum, _frame):
        INTERRUPTED.append(signal.Signals(signum).name)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)

    stack = Child('stack', cmd, logdir / 'stack.log', env)
    rclpy = node = None
    ended = None
    try:
        rclpy, node = make_node(stage1)
        stack.start()
        say('[1/3] seeding the EKF from the boards in view: the car in the start box, '
            'pointing along the first leg, at rest')
        t0 = time.monotonic()
        notes_seen = 0
        while not INTERRUPTED:
            rclpy.spin_once(node, timeout_sec=0.05)
            for n in node.notes[notes_seen:]:
                say(f'      {n}')
            notes_seen = len(node.notes)
            if not stack.alive():
                ended = f'the stack exited; see {logdir / "stack.log"}'
                break
            if node.tick() == 'ready':
                break
            if time.monotonic() - t0 > a.ready_timeout:
                ended = (f'not ready after {a.ready_timeout:.0f} s (phase {node.phase}, '
                         f'{node.fixes} fixes): is a board in view of the start box?')
                break
            status(f'      {time.monotonic() - t0:4.0f} s: {node.phase}, {node.fixes} fixes, '
                   f'EKF sd {node.std if node.std < 9 else math.nan:.2f} m')
        if ended is None and not INTERRUPTED:
            status('')
            x, y, yaw = node.pose or (math.nan, math.nan, math.nan)
            say(f'[2/3] READY at ({x:.2f}, {y:.2f}) {math.degrees(yaw):.1f} deg (stage-1 frame), '
                f'{node.fixes} fixes')
            if a.bench:
                say('      the bench driver drives now')
            else:
                say('      ' + '=' * 66)
                say('      Drive the car STRAIGHT along the taped first leg, 2 m or more,')
                say('      stop, then press Ctrl-C.')
                say('      ' + '=' * 66)
            start = node.pose
            while not INTERRUPTED:
                rclpy.spin_once(node, timeout_sec=0.05)
                if not stack.alive():
                    ended = f'the stack exited during the drive; see {logdir / "stack.log"}'
                    break
                if node.pose and start:
                    dx, dy = node.pose[0] - start[0], node.pose[1] - start[1]
                    c, s = math.cos(start[2]), math.sin(start[2])
                    status(f'      along {c * dx + s * dy:6.2f} m  lateral {-s * dx + c * dy:+.3f} m  '
                           f'speed {node.speed:4.2f} m/s  EKF sd {node.std:.3f} m  ({len(node.samples)} poses)'
                           '  Ctrl-C ends')
                if a.bench:
                    if node.bench_done_at and time.monotonic() - node.bench_done_at > BENCH_END_REST:
                        break
                    if time.monotonic() - t0 > a.ready_timeout + a.bench_timeout:
                        ended = 'the bench driver did not finish'
                        break
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda *_: say('      stopping the stack, please wait'))
        status('')
        samples = list(node.samples) if node is not None else []
        truth_samples = list(node.truth) if node is not None else []
        pnp = list(node.pnp) if node is not None else []
        say('[3/3] stopping the stack')
        stack.stop(say=say)
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
    if ended:
        say(f'FAILED: {ended}')
        return 1
    if not samples:
        say('nothing recorded (stopped before READY)')
        return 1
    poses = survey / al.POSES_FILE
    al.write_samples(str(poses), samples)
    say(f'      {len(samples)} poses -> {poses}')
    al.write_pnp(str(survey / al.PNP_FILE), pnp)
    say(f'      {len(pnp)} multi-board poses -> {survey / al.PNP_FILE}')
    if truth_samples:
        al.write_samples(str(survey / 'align_truth.csv'), truth_samples)
        say(f'      bench: {len(truth_samples)} true poses -> {survey / "align_truth.csv"}')
    kw = {}
    if a.min_distance is not None:
        kw['min_distance'] = a.min_distance
    if a.max_lateral is not None:
        kw['max_lateral'] = a.max_lateral
    res, _ = al.solve_dir(str(survey), route, samples, force=a.force, log=say, pnp=pnp, **kw)
    if res.ok:
        say('next: `just lab2 loc-check` (drive the route by hand), then `just lab2 run`')
    return 0 if res.ok else 1


if __name__ == '__main__':
    sys.exit(main())
