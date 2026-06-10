from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    save_map = LaunchConfiguration('save_map')
    auto_save_map = LaunchConfiguration('auto_save_map')
    auto_save_period = LaunchConfiguration('auto_save_period')

    nav2_params = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config',
        'nav2_mapping_params.yaml',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'rviz',
        'navigation.rviz',
    ])

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('map_generator'),
            '/launch/mapping.launch2.py',
        ]),
        launch_arguments={
            'left_lidar_port': left_lidar_port,
            'right_lidar_port': right_lidar_port,
            'arduino_port': arduino_port,

            # Use only one RViz instance.
            # This launch starts navigation RViz after a delay.
            'use_rviz': 'false',

            'save_map': save_map,
            'auto_save_map': auto_save_map,
            'auto_save_period': auto_save_period,
        }.items(),
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_raw'),
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_raw'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    cmd_vel_to_joystick_mapping = Node(
        package='navigation',
        executable='cmd_vel_to_joystick_mapping',
        name='cmd_vel_to_joystick_mapping',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/cmd_vel',
                'joystick_topic': '/joystick_cmd',
                'scan_topic': '/scan',

                # Keep same style as your current mapping launch.
                'max_linear_speed': 1.67,
                'max_angular_speed': 1.1,

                'invert_x': False,
                'invert_y': False,

                'minimum_nonzero_joystick': 52,
                'max_joystick_x': 80,
                'max_joystick_y': 80,
                'pure_rotation_joystick': 60,

                'deadzone_percent': 3,
                'timeout_seconds': 0.5,
                'publish_rate_hz': 20.0,
                'send_nothing_without_cmd_vel_publisher': True,

                'use_obstacle_gate': True,
                'full_scan_stop_distance_m': 0.45,
                'scan_timeout_seconds': 0.5,

                'debug': True,
            }
        ],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'behavior_server',
                    'velocity_smoother',
                ],
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=None,
    )

    delayed_navigation = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='STARTING NAV2 WHILE MAPPING'),
            planner_server,
            controller_server,
            bt_navigator,
            behavior_server,
            velocity_smoother,
            cmd_vel_to_joystick_mapping,
            lifecycle_manager_navigation,
        ],
    )

    delayed_rviz = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='STARTING RVIZ FROM NAVIGATION MAPPING LAUNCH NOW'),
            rviz,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_lidar_port',
            default_value='/dev/left_lidar',
            description='Serial port for the left LiDAR.',
        ),

        DeclareLaunchArgument(
            'right_lidar_port',
            default_value='/dev/right_lidar',
            description='Serial port for the right LiDAR.',
        ),

        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/arduino_wheelchair',
            description='Serial port for the Arduino sensor node and serial joystick output.',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start navigation RViz if true.',
        ),

        DeclareLaunchArgument(
            'save_map',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Maps/live_nav_map'
            ),
            description='Path where map_saver_cli should save the live SLAM map.',
        ),

        DeclareLaunchArgument(
            'auto_save_map',
            default_value='false',
            description='Continuously autosave the SLAM map.',
        ),

        DeclareLaunchArgument(
            'auto_save_period',
            default_value='60.0',
            description='Seconds between map autosaves.',
        ),

        mapping_launch,
        delayed_navigation,
        delayed_rviz,
    ])