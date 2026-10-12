import time, rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
rclpy.init(); n = Node('probe')
st = {'img': 0, 'det': 0, 'fix': [], 'saved': False}
def on_img(m):
    st['img'] += 1
    if not st['saved']:
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        cv2.imwrite(__import__('os').environ.get('DEBUG_PNG', 'tmp/synthetic_camera_debug.png'), cv2.cvtColor(a, cv2.COLOR_RGB2BGR)); st['saved'] = True
def on_det(m): st['det'] += len(m.poses)
def on_fix(m): st['fix'].append(m.pose.pose)
n.create_subscription(Image, '/localization/pose_estimator/ar_tag_based_localizer/debug/image', on_img, qos_profile_sensor_data)
n.create_subscription(PoseArray, '/localization/pose_estimator/ar_tag_based_localizer/debug/detected_tag', on_det, qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance', on_fix, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < float(__import__("os").environ.get("PROBE", "8")): rclpy.spin_once(n, timeout_sec=0.1)
print('debug images', st['img'], 'detections', st['det'], 'fixes', len(st['fix']))
import math
for p in st["fix"][-2:]:
    yaw = math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y), 1-2*(p.orientation.y**2+p.orientation.z**2))
    print('fix', round(p.position.x,3), round(p.position.y,3), round(p.position.z,3), 'yaw deg', round(math.degrees(yaw),2))
