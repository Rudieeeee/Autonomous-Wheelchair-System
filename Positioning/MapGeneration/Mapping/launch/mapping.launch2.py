from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    save_map = LaunchConfiguration('save_map')
    auto_save_map = LaunchConfiguration('auto_save_map')
    auto_save_delay = LaunchConfiguration('auto_save_delay')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('map_generator'),
            '/launch/sensors.launch2.py',
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
            'scan_topic': '/scan',
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

    save_map_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', save_map,
        ],
        output='screen',
        condition=IfCondition(auto_save_map),
    )

    delayed_save_map = TimerAction(
        period=PythonExpression([auto_save_delay]),
        actions=[
            save_map_process,
        ],
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
            description='Automatically save the map after auto_save_delay seconds.',
        ),
        DeclareLaunchArgument(
            'auto_save_delay',
            default_value='120.0',
            description='Seconds to wait before automatically saving the map.',
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

        delayed_save_map,
    ])