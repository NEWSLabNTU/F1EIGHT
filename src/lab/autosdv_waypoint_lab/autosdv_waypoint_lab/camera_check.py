"""lab2_camera_check: with the car standing in the start box, is the marker localization sound?

    just lab2 camera-check [course] [seconds]

Run on the car, stack up, NOT engaged, car still. Collects for `--duration`
seconds and reports:

  fixes      the AR-tag localizer's accepted fixes: rate, and their mean and
             spread against the start box pose in course.yaml. A mean far from
             the box means the box or the markers are surveyed wrong; a large
             spread means the markers are too far or too oblique for the camera.

  markers    every detection the localizer drew (its debug/detected_tag, each
             detection placed on the map with the current EKF pose), matched
             to the nearest surveyed marker. A marker whose detections sit
             consistently away from its surveyed position, while the others
             agree, is surveyed wrong: re-measure it.

The localizer only publishes debug/detected_tag while something subscribes,
which this tool does. Nothing here changes the stack.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autosdv_waypoint_lab.course import load_course, parse_osm_landmarks

FIX = '/localization/pose_estimator/pose_with_covariance'
DETECTED = '/localization/pose_estimator/ar_tag_based_localizer/debug/detected_tag'
SURVEY_TOLERANCE = 0.15   # [m] a marker's detections this far from its survey: re-measure


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def mean_std(xs):
    if not xs:
        return math.nan, math.nan
    m = sum(xs) / len(xs)
    return m, math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


class Collector(Node):
    def __init__(self):
        super().__init__('lab2_camera_check')
        self.fixes = []
        self.detections = []
        self.create_subscription(PoseWithCovarianceStamped, FIX, self.on_fix, qos_profile_sensor_data)
        self.create_subscription(PoseArray, DETECTED, self.on_detected, qos_profile_sensor_data)

    def on_fix(self, msg):
        p = msg.pose.pose
        self.fixes.append((p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_detected(self, msg):
        for p in msg.poses:
            self.detections.append((p.position.x, p.position.y, p.position.z))


def report(course, landmarks, fixes, detections, duration):
    sp = course.start_pose
    lines = [f'{len(fixes)} fixes in {duration:.0f} s ({len(fixes) / duration:.1f} Hz)']
    ok = True
    if fixes:
        mx, sx = mean_std([f[0] for f in fixes])
        my, sy = mean_std([f[1] for f in fixes])
        dyaw = [math.atan2(math.sin(f[2] - sp.yaw), math.cos(f[2] - sp.yaw)) for f in fixes]
        myaw, syaw = mean_std(dyaw)
        off = math.hypot(mx - sp.x, my - sp.y)
        lines.append(f'  mean fix ({mx:.3f}, {my:.3f}), {off:.3f} m from the start box; '
                     f'yaw {math.degrees(myaw):+.1f} deg from it')
        lines.append(f'  spread x {sx:.3f} m, y {sy:.3f} m, yaw {math.degrees(syaw):.1f} deg')
        if off > 0.3:
            lines.append('  CHECK: the fixes disagree with the start box by more than 0.3 m')
            ok = False
    else:
        lines.append('  NO FIXES: is a marker in view, within range, and is the EKF seeded '
                     '(start_pose_init)?')
        ok = False
    flat = [(mid, lm) for mid, lms in landmarks.items() for lm in lms]
    per = {}
    for x, y, z in detections:
        mid, lm = min(flat, key=lambda e: math.hypot(e[1].position[0] - x, e[1].position[1] - y))
        per.setdefault(mid, []).append((x - lm.position[0], y - lm.position[1], z - lm.position[2]))
    lines.append(f'{len(detections)} marker detections')
    for mid in sorted(per, key=lambda k: int(k) if k.isdigit() else k):
        d = per[mid]
        dx, sx = mean_std([e[0] for e in d])
        dy, sy = mean_std([e[1] for e in d])
        dz, _ = mean_std([e[2] for e in d])
        flag = ''
        if math.hypot(dx, dy) > SURVEY_TOLERANCE:
            flag = '  <- re-measure this marker'
            ok = False
        lines.append(f'  marker {mid:>3}: {len(d):4d} seen, offset ({dx:+.3f}, {dy:+.3f}, '
                     f'{dz:+.3f}) m, spread ({sx:.3f}, {sy:.3f}){flag}')
    lines.append('OK' if ok else 'NOT OK: fix the above before driving')
    return '\n'.join(lines), ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('course_dir')
    ap.add_argument('--duration', type=float, default=20.0)
    args = ap.parse_args(argv)
    course = load_course(args.course_dir)
    with open(course.map_file) as f:
        landmarks = parse_osm_landmarks(f.read())
    rclpy.init()
    node = Collector()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    text, ok = report(course, landmarks, node.fixes, node.detections,
                      max(time.monotonic() - t0, 1e-3))
    print(text)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
