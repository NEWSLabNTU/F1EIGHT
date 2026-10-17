import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_rc_sim'


def share_tree(root):
    """data_files entries installing every file under `root` into share/<pkg>/<root>."""
    entries = []
    for directory in sorted({os.path.dirname(p) for p in glob(f'{root}/**/*', recursive=True)
                             if os.path.isfile(p)}):
        files = sorted(p for p in glob(f'{directory}/*') if os.path.isfile(p))
        entries.append((os.path.join('share', package_name, directory), files))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['vehicle.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ] + share_tree('tracks'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description='AutoSDV RC car simulator: PWM-level plant, lockstep grader, track generator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_sim_plant = autosdv_rc_sim.plant_node:main',
            'rc_sim_replay = autosdv_rc_sim.replay_node:main',
            'rc_sim_grade = autosdv_rc_sim.grade:main',
            'rc_sim_make_track = autosdv_rc_sim.track:main',
            'rc_sim_fit = autosdv_rc_sim.fit:main',
        ],
    },
)
