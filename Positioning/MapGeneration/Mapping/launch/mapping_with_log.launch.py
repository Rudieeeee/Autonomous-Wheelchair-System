from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')
    use_rviz = LaunchConfiguration('use_rviz')

    save_map = LaunchConfiguration('save_map')
    auto_save_map = LaunchConfiguration('auto_save_map')
    auto_save_period = LaunchConfiguration('auto_save_period')

    log_file = LaunchConfiguration('log_file')

    normal_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('map_generator'),
            '/launch/mapping.launch2.py',
        ]),
        launch_arguments={
            'left_lidar_port': left_lidar_port,
            'right_lidar_port': right_lidar_port,
            'arduino_port': arduino_port,
            'use_rviz': use_rviz,
            'save_map': save_map,
            'auto_save_map': auto_save_map,
            'auto_save_period': auto_save_period,
        }.items(),
    )

    carmen_logger = Node(
        package='map_generator',
        executable='map_carmen_logger',
        name='map_carmen_logger',
        output='screen',
        parameters=[
            {
                'log_file': log_file,
                'odom_topic': '/odom',
                'scan_topic': '/scan',
                'max_scan_readings': 180,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_lidar_port',
            default_value='/dev/left_lidar',
        ),

        DeclareLaunchArgument(
            'right_lidar_port',
            default_value='/dev/right_lidar',
        ),

        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/arduino_wheelchair',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'save_map',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Maps/test_map'
            ),
        ),

        DeclareLaunchArgument(
            'auto_save_map',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'auto_save_period',
            default_value='60.0',
        ),

        DeclareLaunchArgument(
            'log_file',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/mapping_log.txt'
            ),
        ),

        normal_mapping,

        TimerAction(
            period=3.0,
            actions=[carmen_logger],
        ),
    ])