import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_lab3'


def tree(src):
    """data_files for every file under src/, keeping the layout under share/."""
    out = []
    for d, _, files in sorted(os.walk(src)):
        if files:
            out.append((os.path.join('share', package_name, d),
                        sorted(os.path.join(d, f) for f in files)))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ] + tree('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description='AutoSDV Lab 3: indoor map building and NDT localization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab3_record = autosdv_lab3.record:main',
            'lab3_check_bag = autosdv_lab3.check_bag:main',
            'lab3_map = autosdv_lab3.glim:main',
            'lab3_anchor = autosdv_lab3.anchor:main',
            'lab3_downsample = autosdv_lab3.downsample:main',
            'lab3_check_lanelet = autosdv_lab3.lanelet_check:main',
            'lab3_sim_lap = autosdv_lab3.sim_lap:main',
            'lab3_route = autosdv_lab3.route:main',
            'lab3_run_meta = autosdv_lab3.run_meta:main',
            'lab3_grade = autosdv_lab3.grade:main',
        ],
    },
)
