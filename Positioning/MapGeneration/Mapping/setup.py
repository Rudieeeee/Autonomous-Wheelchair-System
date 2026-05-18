from glob import glob
from setuptools import find_packages, setup

package_name = 'map_generator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
        (
            'share/' + package_name + '/rviz',
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rudrh',
    maintainer_email='rudrh@todo.todo',
    description='Live map generation package for dual RPLIDAR, Arduino odometry, IMU and SLAM Toolbox.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_sensor_node = map_generator.arduino_sensor_node:main',
            'lidar_scan_merger = map_generator.lidar_scan_merger:main',
        ],
    },
)