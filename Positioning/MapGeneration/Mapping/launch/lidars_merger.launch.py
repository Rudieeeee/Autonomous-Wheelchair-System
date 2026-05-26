from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_lidar_port = LaunchConfiguration('left_lidar_port')
    right_lidar_port = LaunchConfiguration('right_lidar_port')

    # Config file from ros2_laser_scan_merger package
    merger_params = PathJoinSubstitution([
        FindPackageShare('ros2_laser_scan_merger'),
        'config',
        'params.yaml',
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_lidar_port',
            default_value='/dev/ttyUSB0',
        ),
        DeclareLaunchArgument(
            'right_lidar_port',
            default_value='/dev/ttyUSB1',
        ),

        base_to_left_laser,
        base_to_right_laser,

        left_lidar,
        right_lidar,

        laser_scan_merger,
        pointcloud_to_laserscan,
    ])