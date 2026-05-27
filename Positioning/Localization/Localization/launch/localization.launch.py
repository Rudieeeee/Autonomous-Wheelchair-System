from pathlib import Path

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

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PROJECT_ROOT = Path(__file__).resolve().parents[4]

DEFAULT_MAP = (
    PROJECT_ROOT
    / 'Other-Files'
    / 'GeneralData'
    / 'Maps'
    / 'my_map.yaml'
)


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('map_generator'),
                'launch',
                'sensors.launch2.py',
            ])
        ),
        launch_arguments={
            'left_lidar_port': left_lidar_port,
            'right_lidar_port': right_lidar_port,
            'arduino_port': arduino_port,
        }.items(),
    )

    amcl_config = PathJoinSubstitution([
        FindPackageShare('localization'),
        'config',
        'amcl.yaml',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('localization'),
        'rviz',
        'localization.rviz',
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'yaml_filename': map_file,
            }
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': False,
            },
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                ],
            }
        ],
    )

    rviz = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            rviz_config,
        ],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    global_localization = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/reinitialize_global_localization',
            'std_srvs/srv/Empty',
            '{}',
        ],
        output='screen',
    )

    delayed_localization_nodes = TimerAction(
        period=3.0,
        actions=[
            map_server,
            amcl,
        ],
    )

    delayed_lifecycle_manager = TimerAction(
        period=5.0,
        actions=[
            lifecycle_manager,
        ],
    )

    delayed_rviz = TimerAction(
        period=8.0,
        actions=[
            rviz,
        ],
    )

    delayed_global_localization = TimerAction(
        period=15.0,
        actions=[
            global_localization,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=str(DEFAULT_MAP),
        ),

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

        sensors_launch,

        delayed_localization_nodes,
        delayed_lifecycle_manager,
        delayed_rviz,
        delayed_global_localization,
    ])