from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    sensor_params = PathJoinSubstitution([
        FindPackageShare('map_generator'),
        'config',
        'sensor_params.yaml',
    ])

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
        remappings=[
            ('scan', '/scan_left'),
        ],
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
        remappings=[
            ('scan', '/scan_right'),
        ],
    )

    arduino_node = Node(
        package='map_generator',
        executable='arduino_sensor_node',
        name='arduino_sensor_node',
        output='screen',
        parameters=[
            sensor_params,
            {
                'serial_port': arduino_port,
            },
        ],
    )

    lidar_scan_merger = Node(
        package='map_generator',
        executable='lidar_scan_merger',
        name='lidar_scan_merger',
        output='screen',
        parameters=[
            sensor_params,
        ],
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
            '0.0', '0.0', '0.15',
            '0.0', '0.0', '0.0',
            'base_footprint', 'imu_link',
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

        base_to_left_laser,
        base_to_right_laser,
        base_to_imu,

        left_lidar,

        TimerAction(
            period=8.0,
            actions=[right_lidar],
        ),

        TimerAction(
            period=12.0,
            actions=[arduino_node],
        ),

        TimerAction(
            period=14.0,
            actions=[lidar_scan_merger],
        ),
    ])