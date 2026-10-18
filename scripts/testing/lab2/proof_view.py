#!/usr/bin/env python3
"""Side panel for a Lab 2 screencast: the camera with detections, and the run in numbers.

Beside RViz on rviz/lab2_proof.rviz (the map, the route, the trails), this
window shows what RViz cannot lay out legibly at 1920x1080:

  top     ar_tag_based_localizer's debug image: the camera frame with each
          detected marker outlined and its axes drawn (the REAL localizer,
          fed rendered boards on the bench with --camera mock)
  bottom  operation mode, speeds (true / reported / commanded), route
          progress, the EKF's error against the bench's true pose, the marker
          fix rate, and the colour key for the RViz view

    python3 scripts/testing/lab2/proof_view.py [--x 1100 --width 820 --height 1080]
    python3 scripts/testing/lab2/proof_view.py --image BAG/audit.png --seconds 6

The second form shows one image full screen for a few seconds and exits: the
audit plot at the end of a recording. Needs a display; nothing here commands
the car.
"""

import argparse
import math
import time

import cv2
import numpy as np


def show_image(path, seconds, width, height):
    img = cv2.imread(path)
    if img is None:
        raise SystemExit(f'proof_view: cannot read {path}')
    scale = min(width / img.shape[1], height / img.shape[0])
    img = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)),
                     interpolation=cv2.INTER_AREA)
    canvas = np.full((height, width, 3), 255, np.uint8)
    y0 = (height - img.shape[0]) // 2
    x0 = (width - img.shape[1]) // 2
    canvas[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = img
    name = 'audit'
    cv2.namedWindow(name, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow(name, 0, 0)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        cv2.imshow(name, canvas)
        cv2.waitKey(50)
    cv2.destroyAllWindows()


def run_panel(args):
    import rclpy
    from autoware_adapi_v1_msgs.msg import OperationModeState
    from autoware_control_msgs.msg import Control
    from autoware_vehicle_msgs.msg import VelocityReport
    from autosdv_lab_msgs.msg import WaypointProgress
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image

    modes = {0: 'UNKNOWN', 1: 'STOP', 2: 'AUTONOMOUS', 3: 'LOCAL', 4: 'REMOTE'}

    class Panel(Node):
        def __init__(self):
            super().__init__('lab2_proof_view')
            self.bridge = CvBridge()
            self.image = None
            self.image_rx = 0.0
            self.mode = None
            self.v_rep = self.v_cmd = None
            self.true = self.ekf = None
            self.prog = None
            self.fixes = []
            self.trail_err = []
            mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            sub = self.create_subscription
            sub(Image, args.image_topic, self.on_image, qos_profile_sensor_data)
            sub(OperationModeState, '/api/operation_mode/state',
                lambda m: setattr(self, 'mode', m), mode_qos)
            sub(VelocityReport, '/vehicle/status/velocity_status',
                lambda m: setattr(self, 'v_rep', m.longitudinal_velocity), 1)
            sub(Control, '/control/command/control_cmd',
                lambda m: setattr(self, 'v_cmd', m.longitudinal.velocity), 1)
            sub(Odometry, '/vehicle_bench_plant/odometry', lambda m: setattr(self, 'true', m), 1)
            sub(Odometry, '/localization/kinematic_state', lambda m: setattr(self, 'ekf', m), 1)
            sub(WaypointProgress, '/planning/waypoint_path_publisher/progress',
                lambda m: setattr(self, 'prog', m), 1)
            sub(PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance',
                lambda m: self.fixes.append(time.monotonic()), 10)

        def on_image(self, msg):
            try:
                self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.image_rx = time.monotonic()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'image: {e}', throttle_duration_sec=5.0)

    rclpy.init()
    node = Panel()
    W, H = args.width, args.height
    name = 'Lab 2 proof run: camera and status'
    cv2.namedWindow(name, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow(name, args.x, args.y)
    font = cv2.FONT_HERSHEY_SIMPLEX
    white, grey = (240, 240, 240), (150, 150, 150)

    def text(img, s, y, scale=0.8, color=white, x=24, thick=2):
        cv2.putText(img, s, (x, y), font, scale, color, thick, cv2.LINE_AA)

    def fmt(v, unit='m/s'):
        return '  --  ' if v is None else f'{v:5.2f} {unit}'

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            canvas = np.full((H, W, 3), 34, np.uint8)
            text(canvas, 'CAMERA  (ar_tag_based_localizer debug image)', 34, 0.7, grey)
            img_h = int(W * 600 / 960)
            if node.image is not None and time.monotonic() - node.image_rx < 2.0:
                canvas[50:50 + img_h] = cv2.resize(node.image, (W, img_h),
                                                   interpolation=cv2.INTER_AREA)
            else:
                cv2.rectangle(canvas, (0, 50), (W - 1, 50 + img_h), (70, 70, 70), 2)
                text(canvas, 'waiting for the camera...', 50 + img_h // 2, 0.9, grey, W // 4)
            y = 50 + img_h + 50
            m = node.mode
            mode = modes.get(m.mode, '?') if m is not None else '--'
            text(canvas, f'mode    {mode}', y, 1.0,
                 (90, 220, 90) if mode == 'AUTONOMOUS' else white)
            now = time.monotonic()
            node.fixes = [t for t in node.fixes if now - t < 2.0]
            y += 46
            v_true = node.true.twist.twist.linear.x if node.true is not None else None
            text(canvas, f'speed   true {fmt(v_true)}   reported {fmt(node.v_rep)}', y)
            y += 38
            text(canvas, f'        commanded {fmt(node.v_cmd)}', y)
            y += 46
            p = node.prog
            if p is not None:
                done = '   FINISHED' if p.finished else ''
                text(canvas, f'route   {p.progress:5.1f} / {p.run_length:4.1f} m{done}', y)
            y += 46
            if node.true is not None and node.ekf is not None:
                a, b = node.true.pose.pose.position, node.ekf.pose.pose.position
                err = math.hypot(a.x - b.x, a.y - b.y)
                text(canvas, f'EKF vs true pose  {err:4.2f} m', y)
            y += 38
            text(canvas, f'marker fixes      {len(node.fixes) / 2.0:4.1f} /s', y)
            y += 54
            for color, label in (((255, 255, 255), 'white   route (path.csv)'),
                                 ((60, 60, 255), 'red     localized trail (EKF)'),
                                 ((255, 200, 60), 'cyan    true pose (bench)'),
                                 ((60, 220, 60), 'green   marker fixes / survey boards'),
                                 ((0, 200, 255), 'yellow  the car (EKF pose)')):
                cv2.rectangle(canvas, (24, y - 16), (44, y + 2), color, -1)
                text(canvas, label, y, 0.65, white, 58, 1)
                y += 30
            cv2.imshow(name, canvas)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--image', help='show this image instead, then exit')
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--image-topic',
                    default='/localization/pose_estimator/ar_tag_based_localizer/debug/image')
    ap.add_argument('--x', type=int, default=1100)
    ap.add_argument('--y', type=int, default=0)
    ap.add_argument('--width', type=int, default=820)
    ap.add_argument('--height', type=int, default=1080)
    args = ap.parse_args()
    if args.image:
        show_image(args.image, args.seconds, 1920, 1080)
    else:
        run_panel(args)


if __name__ == '__main__':
    main()
