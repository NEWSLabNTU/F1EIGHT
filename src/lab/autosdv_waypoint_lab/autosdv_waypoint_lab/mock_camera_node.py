"""mock_aruco_camera: the ZED as the vehicle bench sees it -- the course's boards,
rendered from the simulated car's TRUE pose.

    true pose (/vehicle_bench_plant/odometry)
      -> render every board in front of the camera (cv2.aruco, the course's
         dictionary, the surveyed pose; the marker on a white plate)
      -> /sensing/camera/zedxm/rgb/color/rect/image         (mono8)
         /sensing/camera/zedxm/rgb/color/rect/camera_info
         /sensing/camera/zedxm/rgb/color/rect/image/compressed  (JPEG, only
                                                    while someone subscribes)
      -> the REAL ar_tag_based_localizer -> EKF -> tracker

so `just lab2 bench camera:=mock` exercises the localization path the car runs,
where the default bench (virtual_marker_localizer) models its output instead.

The camera's pose on the car is the stack's own TF, base_link -> `frame_id`
(the ZED's left optical frame from the sensor kit's URDF), looked up once; if
it never arrives, the prototype's mount (0.20 m forward, 0.25 m up, level)
is used and published as a static TF under `mock_camera_optical`. The
intrinsics are a pinhole with `focal` px at `width` x `height`, no distortion:
960 x 600 and 403 px is the ZED X Mini's ~100 deg horizontal field of view at
half its native resolution. Each frame is stamped with the true pose's own
stamp, so the localizer's EKF interpolation sees what it would on the car.

Nothing is drawn that the camera could not see: a board behind the camera,
facing away, or beyond `max_range` is skipped; one partly in view is clipped
by the image edge. What the localizer then makes of it (its range limit,
its EKF gate) is the localizer's.

Prototype and its measurements: scripts/testing/lab2/synthetic_camera/.
"""

import array
import math
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np

PLATE_RATIO = 500.0 / 380.0     # the LCTK boards: a 0.38 m marker on a 500 mm plate


# ------------------------------------------------------------ geometry (no ROS)

def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# optical (z forward, x right, y down) in base_link (x forward, y left, z up)
R_BASE_OPT_LEVEL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
DEFAULT_MOUNT = (np.array([0.20, 0.0, 0.25]), R_BASE_OPT_LEVEL)


class Camera:
    """A pinhole camera on the car: project course-frame points for a car pose."""

    def __init__(self, width: int, height: int, focal: float,
                 t_base_opt=DEFAULT_MOUNT[0], r_base_opt=DEFAULT_MOUNT[1]):
        self.w, self.h, self.f = int(width), int(height), float(focal)
        self.t = np.asarray(t_base_opt, float)
        self.r = np.asarray(r_base_opt, float)

    def to_optical(self, pose, pts: np.ndarray) -> np.ndarray:
        """pts (N, 3) in the course frame -> (N, 3) in the optical frame, car at
        pose = (x, y, z, yaw)."""
        x, y, z, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        r_map_base = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        p_base = (pts - np.array([x, y, z])) @ r_map_base          # = R^T (p - t)
        return (p_base - self.t) @ self.r                           # = R_bo^T (p_b - t_bo)

    def camera_position(self, pose) -> np.ndarray:
        x, y, z, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([x + c * self.t[0] - s * self.t[1], y + s * self.t[0] + c * self.t[1],
                         z + self.t[2]])

    def project(self, p_opt: np.ndarray) -> Optional[np.ndarray]:
        """(N, 2) pixels, or None if any point is behind (or nearly at) the camera."""
        if np.any(p_opt[:, 2] <= 0.05):
            return None
        return np.stack([self.w / 2 + self.f * p_opt[:, 0] / p_opt[:, 2],
                         self.h / 2 + self.f * p_opt[:, 1] / p_opt[:, 2]], axis=1)

    def info_k(self) -> List[float]:
        return [self.f, 0.0, self.w / 2, 0.0, self.f, self.h / 2, 0.0, 0.0, 1.0]


def board_in_view(cam: Camera, pose, marker, max_range: float) -> bool:
    """Facing the camera and within range (the image edge clips the rest)."""
    cp = cam.camera_position(pose)
    d = cp - np.array([marker.x, marker.y, marker.z])
    if float(np.linalg.norm(d)) > max_range:
        return False
    _, _, normal = marker.axes()
    return float(np.dot(d, normal)) > 0.0


def render(cam: Camera, pose, markers: Sequence, marker_size: float, patterns: dict,
           background: int = 150, max_range: float = 12.0) -> Tuple[np.ndarray, List[int]]:
    """(the grey image, the ids drawn). `patterns`: id -> the marker's bit image
    (uint8, black border included), from cv2.aruco."""
    import cv2
    img = np.full((cam.h, cam.w), background, np.uint8)
    drawn = []
    # far boards first, so a near one occludes a far one
    cp = cam.camera_position(pose)
    order = sorted(markers, key=lambda m: -float(np.linalg.norm(cp - np.array([m.x, m.y, m.z]))))
    for m in order:
        if m.id not in patterns or not board_in_view(cam, pose, m, max_range):
            continue
        plate = cam.project(cam.to_optical(pose, np.array(m.corners(marker_size * PLATE_RATIO))))
        mark = cam.project(cam.to_optical(pose, np.array(m.corners(marker_size))))
        if plate is None or mark is None:
            continue
        # off the image entirely?
        if (plate[:, 0].max() < 0 or plate[:, 0].min() >= cam.w or plate[:, 1].max() < 0
                or plate[:, 1].min() >= cam.h):
            continue
        cv2.fillConvexPoly(img, np.round(plate * 16).astype(np.int32), 255, cv2.LINE_AA, shift=4)
        pat = patterns[m.id]
        n = pat.shape[0]
        # bl, br, tr, tl (Marker.corners) <- the pattern's BL, BR, TR, TL. Pixel i of
        # the pattern covers [i - 0.5, i + 0.5], so its outer edge is at -0.5 and n - 0.5
        a0, a1 = -0.5, n - 0.5
        src = np.float32([[a0, a1], [a1, a1], [a1, a0], [a0, a0]])
        x0 = int(max(0, math.floor(mark[:, 0].min())))
        y0 = int(max(0, math.floor(mark[:, 1].min())))
        x1 = int(min(cam.w, math.ceil(mark[:, 0].max()) + 1))
        y1 = int(min(cam.h, math.ceil(mark[:, 1].max()) + 1))
        if x1 <= x0 or y1 <= y0:
            continue
        dst = np.float32(mark - np.array([x0, y0]))
        M = cv2.getPerspectiveTransform(src, dst)
        roi = img[y0:y1, x0:x1]
        warped = cv2.warpPerspective(pat, M, (x1 - x0, y1 - y0), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(pat, 255), M, (x1 - x0, y1 - y0),
                                   flags=cv2.INTER_LINEAR, borderValue=0)
        a = mask.astype(np.float32) / 255.0
        roi[:] = (a * warped + (1.0 - a) * roi).astype(np.uint8)
        drawn.append(m.id)
    return img, drawn


def aruco_dictionary(name: str):
    """The cv2.aruco dictionary that prints a course's `dictionary`. ALL_DICTS decodes
    the aruco library's families, of which ARUCO_MIP_36h12 is the one boards use."""
    import cv2
    if name in ('ALL_DICTS', 'ARUCO_MIP_36h12', '', None):
        name = 'DICT_ARUCO_MIP_36h12'
    if not name.startswith('DICT_'):
        name = 'DICT_' + name
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_patterns(dictionary, ids, side: int = 240) -> dict:
    import cv2
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    return {i: gen(dictionary, int(i), side) for i in ids}


# ------------------------------------------------------------ the node

def main(args=None):
    import cv2
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

    from autosdv_waypoint_lab.course import load_course

    class MockCamera(Node):
        def __init__(self):
            super().__init__('mock_aruco_camera')
            p = self.declare_parameter
            p('course_dir', '')
            p('rate', 15.0)
            p('width', 960)
            p('height', 600)
            p('focal', 403.0)
            p('frame_id', 'zedxm_left_camera_optical_frame')
            p('base_frame', 'base_link')
            p('tf_wait', 30.0)
            p('max_range', 12.0)
            p('jpeg_quality', 80)
            p('image_topic', '/sensing/camera/zedxm/rgb/color/rect/image')
            p('info_topic', '/sensing/camera/zedxm/rgb/color/rect/camera_info')
            p('true_pose_topic', '/vehicle_bench_plant/odometry')
            gp = lambda k: self.get_parameter(k).value  # noqa: E731
            course = load_course(gp('course_dir'))
            self.markers = course.markers
            self.marker_size = course.marker_size
            self.patterns = marker_patterns(aruco_dictionary(course.dictionary),
                                            [m.id for m in course.markers])
            self.w, self.h, self.f = int(gp('width')), int(gp('height')), float(gp('focal'))
            self.frame_id = gp('frame_id')
            self.base_frame = gp('base_frame')
            self.max_range = float(gp('max_range'))
            self.quality = int(gp('jpeg_quality'))
            self.cam = None
            self.t_start = time.monotonic()
            self.tf_wait = float(gp('tf_wait'))
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.static = StaticTransformBroadcaster(self)
            self.pose = None          # (x, y, z, yaw, stamp)
            self.n = 0
            self.render_s = 0.0
            self.pub_img = self.create_publisher(Image, gp('image_topic'), 5)
            self.pub_info = self.create_publisher(CameraInfo, gp('info_topic'), 5)
            self.pub_jpg = self.create_publisher(CompressedImage, gp('image_topic') + '/compressed', 5)
            self.create_subscription(Odometry, gp('true_pose_topic'), self.on_pose, 10)
            self.create_timer(1.0 / float(gp('rate')), self.on_frame)
            self.create_timer(10.0, self.on_stats)
            self.get_logger().info(f'{len(self.markers)} boards, {course.dictionary}, '
                                   f'{self.w}x{self.h} f={self.f:.0f} px at {gp("rate")} Hz')

        def mount(self):
            """The camera's pose on the car, from TF; the prototype's after tf_wait."""
            if self.cam is not None:
                return self.cam
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.frame_id, rclpy.time.Time())
                tr, q = tf.transform.translation, tf.transform.rotation
                self.cam = Camera(self.w, self.h, self.f, np.array([tr.x, tr.y, tr.z]),
                                  quat_to_matrix(q.x, q.y, q.z, q.w))
                self.get_logger().info(f'camera mount from TF {self.base_frame} -> {self.frame_id}: '
                                       f'({tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f})')
            except Exception:
                if time.monotonic() - self.t_start < self.tf_wait:
                    return None
                self.frame_id = 'mock_camera_optical'
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id, t.child_frame_id = self.base_frame, self.frame_id
                t.transform.translation.x, t.transform.translation.z = 0.20, 0.25
                (t.transform.rotation.x, t.transform.rotation.y,
                 t.transform.rotation.z, t.transform.rotation.w) = (-0.5, 0.5, -0.5, 0.5)
                self.static.sendTransform(t)
                self.cam = Camera(self.w, self.h, self.f)
                self.get_logger().warn('no TF to the ZED optical frame: using the prototype mount '
                                       '(0.20, 0, 0.25), published as mock_camera_optical')
            return self.cam

        def on_pose(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            p = m.pose.pose.position
            self.pose = (p.x, p.y, 0.0, yaw, m.header.stamp)

        def on_frame(self):
            cam = self.mount()
            if cam is None or self.pose is None:
                return
            x, y, z, yaw, stamp = self.pose
            t0 = time.perf_counter()
            gray, _ = render(cam, (x, y, z, yaw), self.markers, self.marker_size, self.patterns,
                             max_range=self.max_range)
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = self.frame_id
            info.width, info.height = self.w, self.h
            info.k = cam.info_k()
            info.p = [self.f, 0.0, self.w / 2, 0.0, 0.0, self.f, self.h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.d = [0.0] * 5
            info.distortion_model = 'plumb_bob'
            im = Image()
            im.header = info.header
            im.height, im.width = self.h, self.w
            # mono8, not the ZED's colour: 576 kB instead of 1.7 MB. The localizer's
            # subscription is best-effort, and a frame larger than the kernel's socket
            # receive buffer (net.core.rmem_max, 1 MB on the shared host) is lost whole:
            # 21 of 650 bgr8 frames arrived. cv_bridge converts mono8 to the RGB8 the
            # localizer asks for. array('B'): rclpy copies it as a buffer; a bytes object
            # is checked per element (that alone was ~80 ms a frame).
            im.encoding = 'mono8'
            im.step = self.w
            im.data = array.array('B', gray.tobytes())
            self.pub_info.publish(info)
            self.pub_img.publish(im)
            if self.pub_jpg.get_subscription_count() > 0:
                ok, enc = cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    c = CompressedImage()
                    c.header = info.header
                    c.format = 'mono8; jpeg compressed mono8'
                    c.data = array.array('B', enc.tobytes())
                    self.pub_jpg.publish(c)
            self.render_s += time.perf_counter() - t0
            self.n += 1

        def on_stats(self):
            if self.n:
                self.get_logger().info(f'{self.n} frames, {1000 * self.render_s / self.n:.1f} ms each')

    rclpy.init(args=args)
    node = MockCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
