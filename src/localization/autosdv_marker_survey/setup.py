import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_marker_survey'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description='Map ArUco markers with the car camera and write a marker map.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_survey = autosdv_marker_survey.marker_survey_node:main',
            'marker_survey_solve = autosdv_marker_survey.finish:resolve_main',
            'marker_survey_synthetic_bag = autosdv_marker_survey.synthetic_bag:main',
            'marker_survey_compare = autosdv_marker_survey.synthetic_bag:compare_main',
            'marker_survey_align_solve = autosdv_marker_survey.align:solve_main',
            # the names before the survey left Lab 2, kept for old notes and scripts
            'lab2_marker_survey_solve = autosdv_marker_survey.finish:resolve_main',
            'lab2_marker_survey_synthetic_bag = autosdv_marker_survey.synthetic_bag:main',
            'lab2_marker_survey_compare = autosdv_marker_survey.synthetic_bag:compare_main',
            'lab2_survey_align_solve = autosdv_marker_survey.align:solve_main',
        ],
    },
)
