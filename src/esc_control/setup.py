from setuptools import find_packages, setup

package_name = 'esc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='habby',
    maintainer_email='a0979580915@gmail.com',
    description='esc_control of autoware_manual_control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esc_control = esc_control.esc_control:main',
        ],
    },
)
