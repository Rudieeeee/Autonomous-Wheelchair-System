from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
        (
            'share/' + package_name + '/rviz',
            glob('rviz/*.rviz')
        ),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rudrh',
    maintainer_email='rudrh@todo.todo',
    description='Nav2 navigation package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_joystick = navigation.cmd_vel_to_joystick:main',
            'cmd_vel_to_joystick_mapping = navigation.cmd_vel_to_joystick_mapping:main',
            'cmd_vel_to_joystick_pid = navigation.cmd_vel_to_joystick_pid:main',
            'conditional_cmd_vel_limiter = navigation.conditional_cmd_vel_limiter:main',
            'tof_safety_limiter = navigation.tof_safety_limiter:main',
        ],
    },
)