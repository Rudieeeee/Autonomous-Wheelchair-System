from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    merger_params = PathJoinSubstitution([
        FindPackageShare('ros2_laser_scan_merger'),
        'config',
        'params.yaml',
    ])

    sensor_params = PathJoinSubstitution([
        FindPackageShare('map_generator'),
        'config',
        'sensor_params.yaml',
    ])

    # ekf_params = PathJoinSubstitution([
    #     FindPackageShare('map_generator'),
    #     'config',
    #     'ekf.yaml',
    # ])

    left_lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='left_lidar',
        output='screen',
        parameters=[{
            'serial_port': left_lidar_port,
            'serial_baudrate': 460800,
            'frame_id': 'left_laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', '/scan_left')],
    )

    right_lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='right_lidar',
        output='screen',
        parameters=[{
            'serial_port': right_lidar_port,
            'serial_baudrate': 460800,
            'frame_id': 'right_laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', '/scan_right')],
    )

    base_to_left_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_left_laser',
        arguments=[
            '0.82', '0.27', '0.20',
            '0.0', '0.0', '0.0',
            'base_footprint', 'left_laser',
        ],
    )

    base_to_right_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_right_laser',
        arguments=[
            '0.82', '-0.27', '0.20',
            '0.0', '0.0', '0.0',
            'base_footprint', 'right_laser',
        ],
    )

    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=[
            '0.26', '0.0', '0.15',
            '0.0', '0.0', '0.0',
            'base_footprint', 'imu_link',
        ],
    )

    laser_scan_merger = Node(
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        name='ros2_laser_scan_merger',
        output='screen',
        parameters=[merger_params],
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[merger_params],
    )

    arduino_node = Node(
        package='map_generator',
        executable='arduino_sensor_node',
        name='arduino_sensor_node',
        output='screen',
        parameters=[
            sensor_params,
            {'serial_port': arduino_port},
        ],
    )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_params],
    #     remappings=[
    #         ('odometry/filtered', '/odom'),
    #     ],
    # )

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

        base_to_left_laser,
        base_to_right_laser,
        base_to_imu,

        left_lidar,
        right_lidar,

        laser_scan_merger,
        pointcloud_to_laserscan,

        TimerAction(
            period=2.0,
            actions=[arduino_node],
        ),

        # TimerAction(
        #     period=3.0,
        #     actions=[ekf_node],
        # ),
    ])