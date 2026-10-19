import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_aruco_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'dictionaries'),
         glob('config/dictionaries/*.dict')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description='ArUco localization on a surveyed marker map.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_marker_map = autosdv_aruco_localization.marker_map:main',
            'start_pose_init = autosdv_aruco_localization.start_pose_init_node:main',
            'loc_check = autosdv_aruco_localization.loc_check_node:main',
            'aruco_camera_check = autosdv_aruco_localization.camera_check:main',
            # stand-ins for the camera, for testing without the car
            'virtual_marker_localizer = autosdv_aruco_localization.virtual_marker_localizer_node:main',
            'mock_aruco_camera = autosdv_aruco_localization.mock_camera_node:main',
        ],
    },
)
