from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def repeated_map_save(save_map, auto_save_map, auto_save_period):
    return ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            [
                'while true; do ',
                'sleep ', auto_save_period, '; ',
                'ros2 run nav2_map_server map_saver_cli ',
                '-f ', save_map, ' ',
                '--ros-args -p map_subscribe_transient_local:=true; ',
                'done'
            ],
        ],
        output='screen',
        condition=IfCondition(auto_save_map),
    )


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    save_map = LaunchConfiguration('save_map')
    auto_save_map = LaunchConfiguration('auto_save_map')
    auto_save_period = LaunchConfiguration('auto_save_period')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('map_generator'),
            '/launch/sensors_tof64.launch.py',
        ]),
        launch_arguments={
            'left_lidar_port': left_lidar_port,
            'right_lidar_port': right_lidar_port,
            'arduino_port': arduino_port,
        }.items(),
    )

    slam_params = PathJoinSubstitution([
        FindPackageShare('map_generator'),
        'config',
        'slam_toolbox.yaml',
    ])

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_sync_launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': slam_params,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',

            # IMPORTANT:
            # /scan_multi is created by ros2_laser_scan_merger_multi.
            # /tof_cloud is only for obstacle avoidance later, not for SLAM mapping.
            'scan_topic': '/scan_multi',
        }.items(),
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('map_generator'),
        'rviz',
        'mapping.rviz',
    ])

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_lidar_port',
            default_value='/dev/ttyUSB0',
        ),
        DeclareLaunchArgument(
            'right_lidar_port',
            default_value='/dev/ttyUSB1',
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyACM0',
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
            description='Continuously autosave the map.',
        ),
        DeclareLaunchArgument(
            'auto_save_period',
            default_value='60.0',
            description='Seconds between map autosaves.',
        ),

        sensors_launch,

        TimerAction(
            period=5.0,
            actions=[slam_launch],
        ),

        TimerAction(
            period=8.0,
            actions=[rviz],
        ),

        repeated_map_save(save_map, auto_save_map, auto_save_period),
    ])