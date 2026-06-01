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
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    nav2_params = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config',
        'nav2_params.yaml',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'rviz',
        'navigation.rviz',
    ])

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('localization'),
            '/launch/localization.launch.py',
        ]),
        launch_arguments={
            'map': map_file,
            'left_lidar_port': left_lidar_port,
            'right_lidar_port': right_lidar_port,
            'arduino_port': arduino_port,
            'use_rviz': 'false',
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

    cmd_vel_to_joystick = Node(
        package='navigation',
        executable='cmd_vel_to_joystick',
        name='cmd_vel_to_joystick',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/cmd_vel',
                'joystick_topic': '/joystick_cmd',

                # From your Nav2 YAML:
                # velocity_smoother max_velocity: [0.25, 0.0, 0.5]
                'max_linear_speed': 0.25,
                'max_angular_speed': 0.5,

                # Change if direction is reversed
                'invert_x': False,
                'invert_y': False,

                # Safety/settings
                'deadzone_percent': 3,
                'timeout_seconds': 0.5,
                'publish_rate_hz': 20.0,

                # If no /cmd_vel publisher exists, publish nothing
                'send_nothing_without_cmd_vel_publisher': True,
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
            planner_server,
            controller_server,
            bt_navigator,
            behavior_server,
            velocity_smoother,
            cmd_vel_to_joystick,
            lifecycle_manager_navigation,
        ],
    )

    delayed_rviz = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='STARTING RVIZ FROM NAVIGATION TEST LAUNCH NOW'),
            rviz,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Maps/hall_h_map.yaml'
            ),
            description='Full path to the saved map YAML file.',
        ),

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

        localization_launch,
        delayed_navigation,
        delayed_rviz,
    ])