import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    log_file = LaunchConfiguration('log_file')
    rate_hz = LaunchConfiguration('rate_hz')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')

    laser_frame = LaunchConfiguration('laser_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')

    save_map = LaunchConfiguration('save_map')
    use_rviz = LaunchConfiguration('use_rviz')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_sync_launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'scan_topic': '/scan',
            'transform_timeout': '2.0',
            'tf_buffer_duration': '60.0',
        }.items(),
    )

    replay = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mapping_test', 'replay_carmen_log',
            '--ros-args',
            '-p', ['log_file:=', log_file],
            '-p', ['rate_hz:=', rate_hz],
            '-p', ['angle_min:=', angle_min],
            '-p', ['angle_max:=', angle_max],
            '-p', ['range_min:=', range_min],
            '-p', ['range_max:=', range_max],
            '-p', ['laser_frame:=', laser_frame],
            '-p', ['base_frame:=', base_frame],
            '-p', ['odom_frame:=', odom_frame],
        ],
        output='screen',
    )

    delayed_replay = TimerAction(
        period=2.0,
        actions=[replay],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('mapping_test'),
        'rviz',
        'mapping.rviz',
    ])

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    save_map_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', save_map,
        ],
        output='screen',
    )

    auto_save_after_replay = RegisterEventHandler(
        OnProcessExit(
            target_action=replay,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[save_map_process],
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_file',
            default_value=(
                 '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Logs/mapping_new_log.txt'
            ),
        ),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('angle_min', default_value='-3.06201518195'),
        DeclareLaunchArgument('angle_max', default_value='3.06201518195'),
        DeclareLaunchArgument('range_min', default_value='0.1'),
        DeclareLaunchArgument('range_max', default_value='50.0'),

        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),

        DeclareLaunchArgument(
            'save_map',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Maps/my_map'
            ),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        slam_launch,
        rviz,
        delayed_replay,
        auto_save_after_replay,
    ])