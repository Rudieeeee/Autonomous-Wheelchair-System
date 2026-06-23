from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
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
    uwb_arduino_port = LaunchConfiguration('uwb_arduino_port')

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

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('map_generator'),
            '/launch/sensors.launch.py',
        ]),
        launch_arguments={
            'left_lidar_port': left_lidar_port,
            'right_lidar_port': right_lidar_port,
            'arduino_port': arduino_port,
            'uwb_arduino_port': uwb_arduino_port,
        }.items(),
    )

    uwb_stupid_follower = Node(
        package='navigation',
        executable='uwb_stupid_follower',
        name='uwb_stupid_follower',
        output='screen',
        parameters=[
            {
                'target_polar_topic': '/uwb/target_polar',
                'cmd_vel_topic': '/cmd_vel_raw',

                'target_timeout_s': 1.0,

                'stop_distance_m': 1.20,
                'slow_distance_m': 2.00,

                'max_linear_speed': 0.20,
                'min_linear_speed': 0.04,
                'max_angular_speed': 0.50,

                'angle_deadband_rad': 0.17,
                'linear_gain': 0.35,
                'angular_gain': 1.00,

                'allow_reverse': False,
            }
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
                'linear_zero_threshold': 0.02,
                'moving_angular_limit': 0.25,
                'pure_rotation_angular_limit': 10.0,
            }
        ],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_limited'),
            ('/cmd_vel_smoothed', '/cmd_vel'),
        ],
    )

    tof_safety_limiter = Node(
        package='navigation',
        executable='tof_safety_limiter',
        name='tof_safety_limiter',
        output='screen',
        parameters=[nav2_params],
    )

    cmd_vel_to_joystick = Node(
        package='navigation',
        executable='cmd_vel_to_joystick_pid',
        name='cmd_vel_to_joystick_pid',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/cmd_vel_safe',
                'odom_topic': '/odom',
                'joystick_topic': '/joystick_cmd',
                'calibration_file': 'joystick_calibration.json',

                'publish_rate_hz': 20.0,
                'timeout_seconds': 0.5,
                'odom_timeout_seconds': 0.5,
                'send_nothing_without_cmd_vel_publisher': False,
                'require_fresh_odom': True,

                'max_joystick_x': 100,
                'max_joystick_y': 100,
                'invert_x': True,
                'invert_y': False,

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

                'fallback_max_linear_speed': 1.39,
                'fallback_max_angular_speed': 0.42,
                'fallback_min_forward_joystick': 25,
                'fallback_min_backward_joystick': 40,
                'fallback_min_turn_joystick': 25,

                'require_accurate_amcl': False,

                'use_obstacle_gate': True,
                'scan_topics': ['/scan', '/tof_scan'],
                'full_scan_stop_distance_m': 0.90,
                'scan_timeout_seconds': 0.5,
            }
        ],
    )

    lifecycle_manager_tracking = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_tracking',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
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
        condition=IfCondition(use_rviz),
    )

    delayed_tracking = TimerAction(
        period=5.0,
        actions=[
            uwb_stupid_follower,
            conditional_cmd_vel_limiter,
            velocity_smoother,
            tof_safety_limiter,
            cmd_vel_to_joystick,
            lifecycle_manager_tracking,
        ],
    )

    delayed_rviz = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='STARTING RVIZ FOR UWB STUPID FOLLOWER'),
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
            description='Serial port for the wheelchair sensor Arduino.',
        ),

        DeclareLaunchArgument(
            'uwb_arduino_port',
            default_value='/dev/arduino_uwb',
            description='Serial port for the UWB Arduino.',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz if true.',
        ),

        sensors_launch,
        delayed_tracking,
        delayed_rviz,
    ])