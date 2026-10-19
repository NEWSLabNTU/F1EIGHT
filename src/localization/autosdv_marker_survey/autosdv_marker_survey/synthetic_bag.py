"""marker_survey_synthetic_bag: a survey recording without the car, for testing.

Renders what the ZED would see while the car is pushed around a course's
markers -- real ArUco images warped onto every marker's corners -- and writes a
rosbag2 with the topics the survey node reads:

  /sensing/camera/zedxm/rgb/color/rect/image        mono8, rectified
  /sensing/camera/zedxm/rgb/color/rect/camera_info
  /sensing/camera/zedxm/odom                        tracking with drift
  /tf_static     base_link -> zedxm_camera_link -> zedxm_left_camera_optical_frame

and the truth (truth.yaml: every marker in the frame of the origin marker,
the survey's convention) beside it.

    marker_survey_synthetic_bag OUT_DIR --course COURSE_DIR --origin 10
"""

import argparse
import math
import os

import numpy as np
import yaml

from autosdv_marker_survey.simulate import T_BASE_CAM, drive, marker_T
from autosdv_marker_survey.survey import (
    course_frame, inv_T, make_T, marker_object_points, opencv_dictionary, rotvec_to_matrix)

T_BASE_LINK = make_T(np.eye(3), [0.20, 0.0, 0.25])        # base_link -> zedxm_camera_link
T_LINK_OPT = inv_T(T_BASE_LINK) @ T_BASE_CAM               # zedxm_camera_link -> optical


def quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def render(markers, sizes_px, dictionary, T_world_cam, K, W, H, size):
    import cv2
    img = np.full((H, W), 160, np.uint8)
    T_cw = inv_T(T_world_cam)
    obj = marker_object_points(size)
    for mid, Tm in markers.items():
        Xw = (Tm[:3, :3] @ obj.T).T + Tm[:3, 3]
        Xc = (T_cw[:3, :3] @ Xw.T).T + T_cw[:3, 3]
        if np.any(Xc[:, 2] < 0.3):
            continue
        v = T_world_cam[:3, 3] - Tm[:3, 3]
        if (v / np.linalg.norm(v)) @ Tm[:3, 2] < 0.2:
            continue
        uv = (K @ Xc.T).T
        uv = uv[:, :2] / uv[:, 2:]
        if uv[:, 0].max() < 0 or uv[:, 0].min() > W or uv[:, 1].max() < 0 or uv[:, 1].min() > H:
            continue
        side = 300
        gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
        mk = gen(dictionary, int(mid), side)
        q = 30
        big = np.full((side + 2 * q, side + 2 * q), 255, np.uint8)
        big[q:q + side, q:q + side] = mk
        src = np.float32([[q, q], [q + side, q], [q + side, q + side], [q, q + side]])
        M = cv2.getPerspectiveTransform(src, np.float32(uv))
        warped = cv2.warpPerspective(big, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(big, 255), M, (W, H), borderValue=0)
        img[mask > 128] = warped[mask > 128]
    return img


def main(argv=None):
    import rclpy.serialization as ser
    import rosbag2_py
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    from autosdv_aruco_localization.marker_map import load_marker_map
    from autosdv_aruco_localization.polyline import load_xy, map_path

    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('out')
    ap.add_argument('--course', required=True)
    ap.add_argument('--origin', type=int, required=True)
    ap.add_argument('--dictionary', default='DICT_5X5_1000')
    ap.add_argument('--marker-size', type=float, default=0.38)
    ap.add_argument('--marker-height', type=float, default=0.25)
    ap.add_argument('--rate', type=float, default=3.0, help='frames per second (car at 0.3 m/s)')
    ap.add_argument('--drift', type=float, default=0.01)
    ap.add_argument('--yaw-drift-deg-per-m', type=float, default=0.3)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args(argv)

    rng = np.random.default_rng(args.seed)
    c = load_marker_map(args.course)
    markers = {m.id: marker_T(m.x, m.y, args.marker_height, m.yaw) for m in c.markers}
    if args.origin not in markers:
        raise SystemExit(f'origin {args.origin} is not a marker of {args.course}')
    route, path_file = map_path(args.course)
    if route is None:
        raise SystemExit(f'{args.course} has no {os.path.basename(path_file)}: the car drives along it')
    px, py = load_xy(path_file)
    loop = list(zip(px, py)) + [(px[0], py[0])]
    n = len(loop) - 1
    sweeps = {int(i): math.radians(70) for i in np.linspace(0, n, 5)[:-1]}
    base = drive(loop, step=0.3 / args.rate, sweeps=sweeps)

    W, H = 960, 600
    f = W / 2 / math.tan(math.radians(51))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    dic = opencv_dictionary(args.dictionary)

    os.makedirs(args.out, exist_ok=True)
    bag = os.path.join(args.out, 'bag')
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    topics = {
        '/sensing/camera/zedxm/rgb/color/rect/image': 'sensor_msgs/msg/Image',
        '/sensing/camera/zedxm/rgb/color/rect/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/sensing/camera/zedxm/odom': 'nav_msgs/msg/Odometry',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
    }
    for name, typ in topics.items():
        qos = ''
        if name == '/tf_static':
            qos = ('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n'
                   '  deadline: {sec: 2147483647, nsec: 4294967295}\n'
                   '  lifespan: {sec: 2147483647, nsec: 4294967295}\n  liveliness: 1\n'
                   '  liveliness_lease_duration: {sec: 2147483647, nsec: 4294967295}\n'
                   '  avoid_ros_namespace_conventions: false\n')
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr',
                                                offered_qos_profiles=qos))
    t0 = 1_000_000_000

    def stamp(msg_header, t):
        msg_header.stamp.sec = int(t)
        msg_header.stamp.nanosec = int(round((t - int(t)) * 1e9))

    def tfs(parent, child, T, t):
        m = TransformStamped()
        stamp(m.header, t)
        m.header.frame_id, m.child_frame_id = parent, child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = T[:3, 3]
        q = quat(T[:3, :3])
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        return m

    static = TFMessage(transforms=[tfs('base_link', 'zedxm_camera_link', T_BASE_LINK, t0),
                                   tfs('zedxm_camera_link', 'zedxm_left_camera_optical_frame', T_LINK_OPT, t0)])
    w.write('/tf_static', ser.serialize_message(static), int(t0 * 1e9))

    T_odo = base[0].copy()
    prev = base[0]
    odo_rate_div = 3                        # odometry 3x the camera rate
    dt = 1.0 / args.rate
    for k, Tb in enumerate(base):
        t = t0 + 1.0 + k * dt
        if k > 0:
            rel = inv_T(prev) @ Tb
            d = np.linalg.norm(rel[:3, 3])
            nr = rng.normal(0, math.radians(args.yaw_drift_deg_per_m) * d + 1e-5, 3) * np.array([0.2, 0.2, 1.0])
            T_new = T_odo @ make_T(rel[:3, :3] @ rotvec_to_matrix(nr), rel[:3, 3] + rng.normal(0, args.drift * d, 3))
            for j in range(1, odo_rate_div + 1):     # odometry between frames, interpolated
                from autosdv_marker_survey.survey import interpolate_T
                Tj = interpolate_T(T_odo, T_new, j / odo_rate_div)
                o = Odometry()
                stamp(o.header, t - dt + j * dt / odo_rate_div)
                o.header.frame_id, o.child_frame_id = 'odom', 'zedxm_camera_link'
                TL = Tj @ T_BASE_LINK
                o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = TL[:3, 3]
                q = quat(TL[:3, :3])
                o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
                w.write('/sensing/camera/zedxm/odom', ser.serialize_message(o),
                        int((t - dt + j * dt / odo_rate_div) * 1e9))
            T_odo = T_new
            prev = Tb
        img = render(markers, None, dic, Tb @ T_BASE_CAM, K, W, H, args.marker_size)
        msg = Image()
        stamp(msg.header, t)
        msg.header.frame_id = 'zedxm_left_camera_optical_frame'
        msg.height, msg.width, msg.encoding, msg.step = H, W, 'mono8', W
        msg.data = img.tobytes()
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = W, H
        info.k = [float(v) for v in K.ravel()]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        info.distortion_model = 'plumb_bob'
        w.write('/sensing/camera/zedxm/rgb/color/rect/image', ser.serialize_message(msg), int(t * 1e9))
        w.write('/sensing/camera/zedxm/rgb/color/rect/camera_info', ser.serialize_message(info), int(t * 1e9))
    del w

    fr = course_frame(markers[args.origin], np.zeros(3), np.array([0, 0, 1.0]))
    Ti = inv_T(fr.T_world_course)
    truth = []
    for mid, Tm in sorted(markers.items()):
        Tc = Ti @ Tm
        truth.append({'id': mid, 'x': round(float(Tc[0, 3]), 4), 'y': round(float(Tc[1, 3]), 4),
                      'z': round(float(Tc[2, 3]), 4), 'yaw': round(math.atan2(Tc[1, 2], Tc[0, 2]), 4)})
    with open(os.path.join(args.out, 'truth.yaml'), 'w') as f:
        yaml.safe_dump({'origin_marker': args.origin, 'markers': truth}, f, sort_keys=False)
    print(f'{len(base)} frames, {len(base) * dt:.0f} s, bag {bag}, truth {args.out}/truth.yaml')
    return 0


def compare_main(argv=None):
    """marker_survey_compare COURSE_DIR TRUTH_YAML: survey vs truth (or a tape survey)."""
    ap = argparse.ArgumentParser(description=compare_main.__doc__)
    ap.add_argument('course_dir')
    ap.add_argument('truth')
    args = ap.parse_args(argv)
    with open(os.path.join(args.course_dir, 'course.yaml')) as f:
        got = {m['id']: m for m in yaml.safe_load(f)['markers']}
    with open(args.truth) as f:
        want = {m['id']: m for m in yaml.safe_load(f)['markers']}
    pe, ye = [], []
    print('  id   dx [cm]  dy [cm]  dz [cm]  dyaw [deg]')
    for mid in sorted(want):
        if mid not in got:
            print(f'  {mid:3d}   MISSING from the survey')
            continue
        a, b = got[mid], want[mid]
        dyaw = math.degrees(math.atan2(math.sin(a['yaw'] - b['yaw']), math.cos(a['yaw'] - b['yaw'])))
        print(f'  {mid:3d} {100 * (a["x"] - b["x"]):8.1f} {100 * (a["y"] - b["y"]):8.1f} '
              f'{100 * (a.get("z", 0) - b.get("z", 0)):8.1f} {dyaw:10.2f}')
        pe.append(math.hypot(a['x'] - b['x'], a['y'] - b['y']) * 100)
        ye.append(abs(dyaw))
    if pe:
        print(f'  position: median {np.median(pe):.1f} cm, max {max(pe):.1f} cm; '
              f'yaw: median {np.median(ye):.2f} deg, max {max(ye):.2f} deg')
    return 0
