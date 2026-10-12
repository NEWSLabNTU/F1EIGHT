#!/usr/bin/env python3
"""Synthetic camera for the real ar_tag_based_localizer.

Renders the course's markers (corners from the course OSM, the map the
localizer loads) as seen from a known car pose, with a pinhole camera at
base_link (cam_x, 0, cam_z) looking forward, and publishes image +
camera_info + the camera TF + an EKF pose (the true pose plus an offset).
Usage: fake_camera.py OSM DICT X Y YAW [EKF_DX]
"""
import math, os, sys
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../../src/lab/autosdv_waypoint_lab'))
from autosdv_waypoint_lab.course import parse_osm_landmarks


W, H, F = (960, 600, 403.0) if os.environ.get("HALF") else (1920, 1200, 806.0)
CAM_X, CAM_Z = 0.20, 0.25

class Cam(Node):
    def __init__(self, osm, dname, x, y, yaw, dx):
        super().__init__('fake_camera')
        self.lms = parse_osm_landmarks(open(osm).read())
        self.dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dname))
        self.pose = (x, y, yaw); self.dx = dx
        self.pub_img = self.create_publisher(Image, '/fake/image', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/fake/camera_info', 10)
        self.pub_ekf = self.create_publisher(PoseWithCovarianceStamped,
            '/localization/pose_twist_fusion_filter/biased_pose_with_covariance', 10)
        st = StaticTransformBroadcaster(self)
        t = TransformStamped(); t.header.frame_id = 'base_link'; t.child_frame_id = 'fake_cam_optical'
        t.transform.translation.x = CAM_X; t.transform.translation.z = CAM_Z
        # optical (z fwd, x right, y down) in base_link: q for R=[[0,0,1],[-1,0,0],[0,-1,0]]
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5, -0.5, 0.5
        st.sendTransform(t)
        self.create_timer(0.02, self.on_ekf)
        self.create_timer(1.0 / float(os.environ.get("RATE", "10")), self.on_frame)
        self.img = self.render()

    def world_to_pixel(self, p):
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        dxw, dyw = p[0] - x, p[1] - y
        bx = c * dxw + s * dyw - CAM_X
        by = -s * dxw + c * dyw
        bz = p[2] - CAM_Z
        # optical: X = -by, Y = -bz, Z = bx
        X, Y, Z = -by, -bz, bx
        if Z <= 0.05: return None
        return (W / 2 + F * X / Z, H / 2 + F * Y / Z)

    def render(self):
        img = np.full((H, W), 180, np.uint8)
        n = 0
        for mid, lms in self.lms.items():
            for lm in lms:
                # rebuild the 4 corners: bl, br, tr, tl from centre and axes (0.25 m)
                h = 0.125
                cs = [tuple(lm.position[k] + u * lm.x_axis[k] + w * lm.y_axis[k] for k in range(3))
                      for u, w in ((-h, -h), (h, -h), (h, h), (-h, h))]
                px = [self.world_to_pixel(p) for p in cs]
                if any(q is None for q in px): continue
                # normal must face the camera
                x, y, yaw = self.pose
                cam = (x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw))
                if (cam[0] - lm.position[0]) * lm.z_axis[0] + (cam[1] - lm.position[1]) * lm.z_axis[1] <= 0: continue
                side = 400
                gen = getattr(cv2.aruco, "drawMarker", None) or cv2.aruco.generateImageMarker
                mk = gen(self.dict, int(mid), side)
                quiet = 60
                big = np.full((side + 2 * quiet, side + 2 * quiet), 255, np.uint8)
                big[quiet:quiet + side, quiet:quiet + side] = mk
                s = side + 2 * quiet
                # marker black square corners in `big`: TL, TR, BR, BL
                src = np.float32([[quiet, quiet], [quiet + side, quiet], [quiet + side, quiet + side], [quiet, quiet + side]])
                dst = np.float32([px[3], px[2], px[1], px[0]])   # tl, tr, br, bl
                M = cv2.getPerspectiveTransform(src, dst)
                warped = cv2.warpPerspective(big, M, (W, H), borderValue=0)
                mask = cv2.warpPerspective(np.full_like(big, 255), M, (W, H), borderValue=0)
                img[mask > 0] = warped[mask > 0]
                n += 1
        self.get_logger().info(f'rendered {n} markers')
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    def on_ekf(self):
        m = PoseWithCovarianceStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = 'map'
        x, y, yaw = self.pose
        m.pose.pose.position.x = x + self.dx; m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2); m.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pub_ekf.publish(m)

    def on_frame(self):
        stamp = self.get_clock().now().to_msg()
        info = CameraInfo(); info.header.stamp = stamp; info.header.frame_id = 'fake_cam_optical'
        info.width, info.height = W, H
        info.k = [F, 0.0, W / 2, 0.0, F, H / 2, 0.0, 0.0, 1.0]
        info.p = [F, 0.0, W / 2, 0.0, 0.0, F, H / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5; info.distortion_model = 'plumb_bob'
        im = Image(); im.header = info.header; im.height, im.width = H, W
        im.encoding = 'rgb8'; im.step = 3 * W; im.data = self.img.tobytes()
        self.pub_info.publish(info); self.pub_img.publish(im)

def main():
    a = sys.argv[1:]
    rclpy.init(); n = Cam(a[0], a[1], float(a[2]), float(a[3]), float(a[4]), float(a[5]) if len(a) > 5 else 0.3)
    rclpy.spin(n)

main()
