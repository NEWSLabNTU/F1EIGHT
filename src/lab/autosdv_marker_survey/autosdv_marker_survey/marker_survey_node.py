"""marker_survey: map the ArUco markers around the car while it is driven, write the course.

Drive or push the car slowly around the room. Every processed frame is
matched to the camera's tracking pose (the ZED's odometry); frames that move
or show a new marker become keyframes, and every marker is placed in the map
the moment it is first seen -- the log names each new id, and ~/markers shows
the live map in RViz. At the end (Ctrl-C, or the ~/save service) the whole
recording is bundle-adjusted and the course is written to `course_dir`, in
the frame of the marker `origin_id` (survey.py's docstring has the
convention).

  in   image (raw or compressed), its camera_info (rectified: no distortion)
       the camera's pose: nav_msgs/Odometry (pose_source: odom, the ZED's
       tracking) or TF world_frame -> image frame (pose_source: tf)
       TF image frame -> base_link (static): where the floor is
  out  ~/markers                      visualization_msgs/MarkerArray, live
       ~/debug/image                  the processed frames with the detections drawn
                                      (only while something subscribes; viewer:=true
                                      in the launch file opens rqt_image_view on it)
       course_dir/course.yaml, lanelet2_map.osm, ar_tag.param.yaml,
       survey_report.yaml, survey_data.npz (re-solve with lab2_marker_survey_solve)
"""

import collections
import math
import os
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from autosdv_marker_survey.finish import finish, save_dump
from autosdv_marker_survey.survey import (
    Detector, Survey, SurveySettings, interpolate_T, make_T, quat_to_matrix)


def tf_to_T(t) -> np.ndarray:
    q, p = t.transform.rotation, t.transform.translation
    return make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def stamp_sec(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


def decode(msg, compressed: bool):
    import cv2
    if compressed:
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_GRAYSCALE)
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    ch = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}.get(enc)
    if ch is None:
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    code = {'bgr8': cv2.COLOR_BGR2GRAY, 'rgb8': cv2.COLOR_RGB2GRAY,
            'bgra8': cv2.COLOR_BGRA2GRAY, 'rgba8': cv2.COLOR_RGBA2GRAY}[enc]
    return cv2.cvtColor(img, code)


class MarkerSurveyNode(Node):
    def __init__(self):
        super().__init__('marker_survey')
        d = SurveySettings(marker_size=0.0)
        P = self.declare_parameter
        P('origin_id', -1)
        P('course_dir', '')
        P('dictionary', 'DICT_5X5_1000')
        P('marker_size', 0.4667)
        P('image_topic', '/sensing/camera/zedxm/rgb/color/rect/image')
        P('camera_info_topic', '/sensing/camera/zedxm/rgb/color/rect/camera_info')
        P('compressed', False)
        P('pose_source', 'odom')              # odom | tf
        P('odom_topic', '/sensing/camera/zedxm/odom')
        P('world_frame', 'map')               # pose_source: tf
        P('base_frame', 'base_link')
        P('rate', 5.0)                        # [Hz] frames processed
        P('pose_tolerance', 0.1)              # [s] image stamp vs the nearest pose
        P('save_on_exit', True)
        P('floor_offset', 0.0)                # [m] the floor lies this far below base_link
        P('keyframe_distance', d.keyframe_distance)
        P('keyframe_angle_deg', math.degrees(d.keyframe_angle))
        P('odom_drift', d.odom_drift)
        P('min_marker_px', d.min_marker_px)
        P('debug_width', 960)                 # [px] width of ~/debug/image
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.origin_id = int(gp('origin_id'))
        self.course_dir = gp('course_dir')
        if self.origin_id < 0 or not self.course_dir:
            raise RuntimeError('set origin_id (the marker that defines the course frame) and course_dir')
        self.dictionary = gp('dictionary')
        self.settings = SurveySettings(
            marker_size=float(gp('marker_size')), keyframe_distance=float(gp('keyframe_distance')),
            keyframe_angle=math.radians(float(gp('keyframe_angle_deg'))),
            odom_drift=float(gp('odom_drift')), min_marker_px=float(gp('min_marker_px')))
        self.detector = Detector(self.dictionary)
        self.compressed = bool(gp('compressed'))
        self.pose_source = gp('pose_source')
        self.odom_topic = gp('odom_topic')
        self.world_frame = gp('world_frame')
        self.base_frame = gp('base_frame')
        self.pose_tol = float(gp('pose_tolerance'))
        self.save_on_exit = bool(gp('save_on_exit'))
        self.floor_offset = float(gp('floor_offset'))

        self.survey = None
        self.lock = threading.Lock()
        self.latest = None
        self.odom = collections.deque(maxlen=2000)
        self.odom_child = None
        self.T_child_cam = None
        self.frames = 0
        self.saved = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        img_type = CompressedImage if self.compressed else Image
        topic = gp('image_topic') + ('/compressed' if self.compressed else '')
        self.create_subscription(img_type, topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        if self.pose_source == 'odom':
            # the ZED publishes odometry BEST_EFFORT; a RELIABLE subscription never matches it
            self.create_subscription(
                Odometry, gp('odom_topic'), self.on_odom,
                QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.pub_dbg = self.create_publisher(Image, '~/debug/image', 1)
        self.debug_width = int(gp('debug_width'))
        self.create_service(Trigger, '~/save', self.on_save)
        self.create_timer(1.0 / float(gp('rate')), self.on_tick)
        self.create_timer(5.0, self.on_status)
        self.get_logger().info(
            f'surveying {self.dictionary} markers; marker_size {self.settings.marker_size} m is the '
            "edge of the BLACK SQUARE, not the board: measure it (the lab's 0.60 m board: 0.4667 m; "
            'an LCTK 600 mm board: 0.48 m). A wrong size scales every distance. '
            f'Origin marker {self.origin_id}; course -> {self.course_dir}. Drive slowly past '
            'every marker; Ctrl-C (or the ~/save service) solves and writes the course.')

    # ------------------------------------------------------------ inputs
    def on_info(self, msg):
        if self.survey is None:
            K = np.array(msg.p, float).reshape(3, 4)[:, :3] if any(msg.p) else np.array(msg.k).reshape(3, 3)
            if any(abs(v) > 1e-6 for v in msg.d):
                self.get_logger().warn('camera_info has distortion; the survey expects a rectified image',
                                       once=True)
            self.survey = Survey(self.settings, K)
            self.get_logger().info(f'camera: {msg.width}x{msg.height}, fx {K[0, 0]:.1f} px')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom.append((stamp_sec(msg.header.stamp),
                          make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])))
        self.odom_child = msg.child_frame_id
        self.world_frame_odom = msg.header.frame_id

    def on_image(self, msg):
        self.latest = msg

    # ------------------------------------------------------------ poses
    def pose_at(self, t: float, cam_frame: str):
        if self.pose_source == 'tf':
            try:
                tr = self.tf_buffer.lookup_transform(self.world_frame, cam_frame, Time(seconds=t))
            except tf2_ros.TransformException:
                return None
            return tf_to_T(tr)
        if len(self.odom) < 2 or self.odom_child is None:
            return None
        if self.T_child_cam is None:
            try:
                self.T_child_cam = tf_to_T(self.tf_buffer.lookup_transform(
                    self.odom_child, cam_frame, Time()))
            except tf2_ros.TransformException:
                self.get_logger().info(f'waiting for TF {self.odom_child} -> {cam_frame}',
                                       throttle_duration_sec=5.0)
                return None
        od = list(self.odom)
        ts = np.array([o[0] for o in od])
        i = int(np.searchsorted(ts, t))
        if i == 0 or i >= len(od):
            j = 0 if i == 0 else len(od) - 1
            if abs(ts[j] - t) > self.pose_tol:
                return None
            T = od[j][1]
        else:
            t0, t1 = ts[i - 1], ts[i]
            if t1 - t0 > 2 * self.pose_tol:
                return None
            T = interpolate_T(od[i - 1][1], od[i][1], (t - t0) / max(t1 - t0, 1e-9))
        return T @ self.T_child_cam

    # ------------------------------------------------------------ work
    def on_tick(self):
        msg, self.latest = self.latest, None
        if msg is None or self.survey is None:
            return
        cam_frame = msg.header.frame_id
        if self.survey.T_cam_base is None:
            try:
                self.survey.T_cam_base = tf_to_T(self.tf_buffer.lookup_transform(
                    cam_frame, self.base_frame, Time()))
            except tf2_ros.TransformException:
                self.get_logger().info(f'waiting for TF {cam_frame} -> {self.base_frame}',
                                       throttle_duration_sec=5.0)
                return
        t = stamp_sec(msg.header.stamp)
        T = self.pose_at(t, cam_frame)
        if T is None:
            self.get_logger().info('no camera pose for this frame yet (is tracking on? is the odometry '
                                   f'topic {self.odom_topic} arriving? a QoS mismatch gives exactly this: '
                                   'the ZED publishes BEST_EFFORT)',
                                   throttle_duration_sec=5.0)
            return
        try:
            gray = decode(msg, self.compressed)
            dets = self.detector.detect(gray)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'image: {e}', throttle_duration_sec=5.0)
            return
        self.frames += 1
        with self.lock:
            known = set(self.survey.markers)
            n_kf = len(self.survey.keyframes)
            new = self.survey.add(t, T, dets)
            keyframe = len(self.survey.keyframes) > n_kf
        if self.pub_dbg.get_subscription_count() > 0:
            self.publish_debug(msg.header, gray, dets, known, keyframe)
        for mid in new:
            m = self.survey.markers[mid]
            dist = np.linalg.norm(m.T_world_marker[:3, 3] - T[:3, 3])
            tag = ' (the origin)' if mid == self.origin_id else ''
            self.get_logger().info(f'new marker {mid}{tag}, {dist:.1f} m away; '
                                   f'{len(self.survey.markers)} so far')
        if new:
            self.publish_markers()

    def on_status(self):
        if self.survey is None:
            self.get_logger().info('waiting for camera_info', throttle_duration_sec=10.0)
            return
        views = ', '.join(f'{k}:{v.views}' for k, v in sorted(self.survey.markers.items()))
        origin = '' if self.origin_id in self.survey.markers else \
            f'; origin marker {self.origin_id} NOT SEEN YET'
        self.get_logger().info(f'{self.frames} frames, {len(self.survey.keyframes)} keyframes; '
                               f'markers (views) {views or "none"}{origin}')
        self.publish_markers()

    def publish_debug(self, header, gray, dets, known, keyframe):
        """The frame with every detection drawn: green a known marker, yellow one seen
        for the first time, red one too small to use; the origin is outlined thicker."""
        import cv2
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        scale = min(1.0, self.debug_width / img.shape[1])
        for d in dets:
            c = d.corners.astype(np.int32).reshape(-1, 1, 2)
            usable = self.survey.usable(d)
            color = (0, 0, 255) if not usable else (0, 220, 0) if d.id in known else (0, 220, 255)
            thick = max(2, int(round(3 / scale))) * (2 if d.id == self.origin_id else 1)
            cv2.polylines(img, [c], True, color, thick)
            cv2.circle(img, tuple(int(v) for v in d.corners[0]), thick * 2, color, -1)   # top-left
            label = f'{d.id}' + (' ORIGIN' if d.id == self.origin_id else '')
            if d.id in self.survey.markers:
                label += f' ({self.survey.markers[d.id].views})'
            org = (int(d.corners[:, 0].min()), max(20, int(d.corners[:, 1].min()) - 10))
            cv2.putText(img, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9 / scale, color,
                        max(1, int(round(2 / scale))), cv2.LINE_AA)
        if scale < 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        origin = 'origin seen' if self.origin_id in self.survey.markers else \
            f'origin {self.origin_id} NOT SEEN'
        status = (f'markers {len(self.survey.markers)}  keyframes {len(self.survey.keyframes)}'
                  f'{"  [KEYFRAME]" if keyframe else ""}  {origin}')
        cv2.rectangle(img, (0, 0), (img.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(img, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding, out.step = 'bgr8', img.shape[1] * 3
        out.data = img.tobytes()
        self.pub_dbg.publish(out)

    def publish_markers(self):
        arr = MarkerArray()
        frame = self.world_frame if self.pose_source == 'tf' else getattr(self, 'world_frame_odom', 'odom')
        for k, (mid, m) in enumerate(sorted(self.survey.markers.items())):
            T = m.T_world_marker
            from scipy.spatial.transform import Rotation
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            box = Marker(type=Marker.CUBE, action=Marker.ADD, id=2 * k, ns='marker')
            box.header.frame_id = frame
            box.pose.position.x, box.pose.position.y, box.pose.position.z = T[:3, 3]
            box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w = q
            s = self.settings.marker_size
            box.scale.x, box.scale.y, box.scale.z = s, s, 0.01
            box.color.g = 1.0 if mid != self.origin_id else 0.3
            box.color.r = 0.0 if mid != self.origin_id else 1.0
            box.color.a = 0.7
            txt = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=2 * k + 1, ns='id')
            txt.header.frame_id = frame
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = T[:3, 3] + [0, 0, s]
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{mid} ({m.views})'
            arr.markers += [box, txt]
        self.pub.publish(arr)

    # ------------------------------------------------------------ the result
    def save(self):
        with self.lock:
            if self.survey is None or not self.survey.keyframes:
                return False, 'nothing recorded'
            os.makedirs(self.course_dir, exist_ok=True)
            dump = os.path.join(self.course_dir, 'survey_data.npz')
            save_dump(dump, self.survey, self.dictionary, self.origin_id)
            log = self.get_logger().info
            log(f'wrote {dump}')
            try:
                res = finish(self.survey, self.origin_id, self.dictionary, self.course_dir,
                             log=lambda s: [log(line) for line in str(s).splitlines()],
                             floor_offset=self.floor_offset)
            except ValueError as e:
                self.get_logger().error(f'not solved: {e}. The recording is in {dump}: '
                                        'lab2_marker_survey_solve can try again.')
                return False, str(e)
        self.saved = True
        if not os.path.isfile(os.path.join(self.course_dir, 'path.csv')):
            self.get_logger().warn(
                f'{self.course_dir} has no path.csv: the survey writes the markers, not the path. '
                'Write path.csv (the route in the origin marker\'s frame) before the course is '
                'followed, loc-checked or recorded (lab2_record needs it).')
        flagged = [m.id for m in res['markers'] if m.flags]
        return True, (f'{len(res["markers"])} markers written to {self.course_dir}'
                      + (f'; check {flagged}' if flagged else ''))

    def on_save(self, request, response):
        response.success, response.message = self.save()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MarkerSurveyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RCLError:
        # Ctrl-C invalidates the context while a timer callback may still be
        # publishing: "publisher's context is invalid". That race is shutdown,
        # not a failure; save below and exit 0. Anything else is real.
        if rclpy.ok():
            raise
    finally:
        if node.save_on_exit and not node.saved:
            ok, msg = node.save()
            print(f'survey: {msg}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
