from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("map_generator")
    params_file = os.path.join(pkg_share, "config", "tof64_scan.yaml")

    serial_port = LaunchConfiguration("serial_port")

    tof64_scan_node = Node(
        package="map_generator",
        executable="tof64_scan_node",
        name="tof64_scan_node",
        output="screen",
        parameters=[
            params_file,
            {
                "serial_port": serial_port,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
        ),
        tof64_scan_node,
    ])