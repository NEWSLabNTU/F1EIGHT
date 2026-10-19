import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_waypoint_lab'


def course_files():
    out = []
    for d in sorted(glob('config/courses/*')):
        if os.path.isdir(d):
            out.append((os.path.join('share', package_name, d),
                        sorted(f for f in glob(os.path.join(d, '*')) if os.path.isfile(f))))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.reference', package_name + '.stub',
              package_name + '.part1'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # a compatibility link to autosdv_aruco_localization's: ar_tag.param.yaml files
        # generated before the ArUco localization moved there name this path
        (os.path.join('share', package_name, 'config', 'dictionaries'),
         glob('config/dictionaries/*.dict')),
        (os.path.join('share', package_name, 'config', 'part1'), glob('config/part1/*.yaml')),
    ] + course_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description='AutoSDV Lab 2: ArUco localization and waypoint following.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_path_publisher = autosdv_waypoint_lab.waypoint_path_publisher_node:main',
            'path_tracker = autosdv_waypoint_lab.path_tracker_node:main',
            'localization_monitor = autosdv_waypoint_lab.localization_monitor_node:main',
            'make_course_map = autosdv_waypoint_lab.course:main',
            'lab2_grade_run = autosdv_waypoint_lab.grading:main',
            'lab2_record = autosdv_waypoint_lab.record:main',
            'lab2_check_bags = autosdv_waypoint_lab.check_bags:main',
            'lab2_steer_cal = autosdv_waypoint_lab.steer_cal:main',
            'lab2_make_uturn = autosdv_waypoint_lab.uturn:main',
            'lab2_make_straight = autosdv_waypoint_lab.straight:main',
            'lab2_audit = autosdv_waypoint_lab.audit:main',
            'bench_straight_driver = autosdv_waypoint_lab.bench_straight_driver_node:main',
            # Lab 2 Part 1 on the RC simulator (docs/design/lab2-part1-sim-design.md)
            'lab2_make_part1_course = autosdv_waypoint_lab.part1.course:main',
            'lab2_part1_scenario = autosdv_waypoint_lab.part1.scenario_node:main',
            'lab2_part1_grader = autosdv_waypoint_lab.part1.grader_node:main',
            'lab2_part1_dashboard = autosdv_waypoint_lab.part1.dashboard:main',
        ],
    },
)
