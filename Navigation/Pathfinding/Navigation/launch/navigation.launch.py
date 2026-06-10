from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
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

                # Real wheelchair calibration:
                # 5 km/h = 1.39 m/s. Joystick Y=100 roughly means max chair speed.
                # Nav2 itself is limited lower in nav2_params.yaml.
                'max_linear_speed': 1.39,

                # Same scale as Nav2 max_vel_theta.
                'max_angular_speed': 0.42,

                # Change only if left/right or forward/backward is reversed.
                'invert_x': True,
                'invert_y': False,

                'deadzone_percent': 3,

                # Your measured breakaway values.
                'min_forward_joystick': 25,
                'min_backward_joystick': 40,

                'max_joystick_x': 100,
                'max_joystick_y': 100,

                # Large pure rotation value.
                'pure_rotation_joystick': 100,

                # With stepped turning, this is the large-turn threshold.
                # Above this angular command, pure rotation becomes X=+-100.
                'pure_rotation_angular_threshold': 0.18,

                # IMPORTANT:
                # Do not keep this at 100, otherwise mixed driving can turn too hard.
                # This is only the max X while also driving forward/backward.
                'mixed_turn_max_joystick': 35,

                # New stepped turning parameters.
                # These stop small heading corrections from becoming full X=100.
                'slow_turn_angular_threshold': 0.08,
                'medium_turn_angular_threshold': 0.18,
                'slow_turn_joystick': 35,
                'medium_turn_joystick': 60,

                'linear_cmd_deadband': 0.01,
                'angular_cmd_deadband': 0.01,

                'timeout_seconds': 0.5,
                'publish_rate_hz': 20.0,

                'send_nothing_without_cmd_vel_publisher': True,

                # AMCL safety gate.
                'require_accurate_amcl': True,
                'amcl_pose_topic': '/amcl_pose',

                'max_x_covariance': 0.04,
                'max_y_covariance': 0.04,
                'max_yaw_covariance': 0.03,

                'min_good_amcl_messages': 5,
                'amcl_timeout_seconds': 10.0,

                # While AMCL is not accepted:
                # if scan is safe, rotate in place with full X.
                'amcl_block_joystick_x': 100,
                'amcl_block_joystick_y': 0,

                # LiDAR emergency stop before and after AMCL.
                'use_obstacle_gate': True,
                'scan_topic': '/scan',

                # 0.70 is quite strict. Use 0.50 first if walls stop it.
                'full_scan_stop_distance_m': 0.70,

                'scan_timeout_seconds': 0.5,
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
            LogInfo(msg='STARTING RVIZ FROM NAVIGATION LAUNCH NOW'),
            rviz,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Maps/hall_m_new_map.yaml'
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
