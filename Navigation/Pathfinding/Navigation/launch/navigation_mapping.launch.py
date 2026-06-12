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

    conditional_cmd_vel_limiter = Node(
        package='navigation',
        executable='conditional_cmd_vel_limiter',
        name='conditional_cmd_vel_limiter',
        output='screen',
        parameters=[
            {
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/cmd_vel_limited',

                # If abs(linear.x) <= this, it counts as pure rotation.
                'linear_zero_threshold': 0.02,

                # While driving forward/backward, angular velocity is capped to +/- 0.25.
                'moving_angular_limit': 0.25,

                # During pure rotation, keep the high angular velocity.
                # The velocity_smoother still applies its own final limits.
                'pure_rotation_angular_limit': 10.0,
            }
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
            ('cmd_vel', 'cmd_vel_limited'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    cmd_vel_to_joystick = Node(
        package='navigation',
        executable='cmd_vel_to_joystick_pid',
        name='cmd_vel_to_joystick_pid',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',
                'joystick_topic': '/joystick_cmd',
                'calibration_file': 'joystick_calibration.json',

                'publish_rate_hz': 20.0,
                'timeout_seconds': 0.5,
                'odom_timeout_seconds': 0.5,

                # For mapping, I prefer false.
                # If Nav2 stops publishing /cmd_vel, this sends [0, 0].
                'send_nothing_without_cmd_vel_publisher': False,

                # PID needs odom feedback.
                'require_fresh_odom': True,

                'max_joystick_x': 100,
                'max_joystick_y': 100,
                'invert_x': True,
                'invert_y': False,

                # Larger deadbands stop tiny near-goal corrections
                # from becoming breakaway joystick commands.
                'linear_cmd_deadband': 0.04,
                'angular_cmd_deadband': 0.05,
                'measured_stop_linear_deadband': 0.03,
                'measured_stop_angular_deadband': 0.03,
                'measured_velocity_filter_alpha': 0.35,

                'linear_kp': 25.0,
                'linear_ki': 3.0,
                'linear_kd': 0.0,
                'linear_integral_limit': 1.0,
                'linear_pid_output_limit': 15.0,

                'angular_kp': 25.0,
                'angular_ki': 3.0,
                'angular_kd': 0.0,
                'angular_integral_limit': 1.0,
                'angular_pid_output_limit': 15.0,

                'max_joystick_x_delta_per_s': 80.0,
                'max_joystick_y_delta_per_s': 80.0,

                # Fallback mapping if joystick_calibration.json is missing.
                'fallback_max_linear_speed': 1.39,
                'fallback_max_angular_speed': 0.42,
                'fallback_min_forward_joystick': 25,
                'fallback_min_backward_joystick': 40,
                'fallback_min_turn_joystick': 25,

                # IMPORTANT FOR MAPPING:
                # Do not wait for AMCL. SLAM provides map -> odom.
                'require_accurate_amcl': False,

                # These can stay here because require_accurate_amcl is false.
                # They will not block motion.
                'amcl_pose_topic': '/amcl_pose',
                'max_x_covariance': 0.04,
                'max_y_covariance': 0.04,
                'max_yaw_covariance': 0.03,
                'min_good_amcl_messages': 5,
                'amcl_timeout_seconds': 10.0,
                'amcl_block_joystick_x': 0,
                'amcl_block_joystick_y': 0,

                # Joystick-level emergency stop during mapping.
                'use_obstacle_gate': True,
                'scan_topic': '/scan',
                'full_scan_stop_distance_m': 0.50,
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
            LogInfo(msg='STARTING NAV2 WHILE MAPPING'),
            planner_server,
            controller_server,
            bt_navigator,
            behavior_server,
            conditional_cmd_vel_limiter,
            velocity_smoother,
            cmd_vel_to_joystick,
            lifecycle_manager_navigation,
        ],
    )

    delayed_rviz = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='STARTING RVIZ FROM NAVIGATION MAPPING PID LAUNCH NOW'),
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
            description='Path where the live SLAM map should be saved.',
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