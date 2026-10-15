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
    packages=[package_name, package_name + '.reference', package_name + '.stub'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'dictionaries'),
         glob('config/dictionaries/*.dict')),
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
            'start_pose_init = autosdv_waypoint_lab.start_pose_init_node:main',
            'localization_monitor = autosdv_waypoint_lab.localization_monitor_node:main',
            'virtual_marker_localizer = autosdv_waypoint_lab.virtual_marker_localizer_node:main',
            'make_course_map = autosdv_waypoint_lab.course:main',
            'lab2_course_from_survey = autosdv_waypoint_lab.course:main_from_survey',
            'lab2_grade_run = autosdv_waypoint_lab.grading:main',
            'lab2_camera_check = autosdv_waypoint_lab.camera_check:main',
            'loc_check = autosdv_waypoint_lab.loc_check_node:main',
            'lab2_record = autosdv_waypoint_lab.record:main',
            'lab2_check_bags = autosdv_waypoint_lab.check_bags:main',
            'lab2_steer_cal = autosdv_waypoint_lab.steer_cal:main',
        ],
    },
)
