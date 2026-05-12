#Run: source /opt/ros/jazzy/setup.bash
#source ~/ros2_ws/install/setup.bash
#ros2 launch /home/rudrh/Autonomous-Wheelchair-System/Integration/Sensors/merge_two_lidar_ros.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    left_serial_port = LaunchConfiguration("left_serial_port")
    right_serial_port = LaunchConfiguration("right_serial_port")

    left_lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar_c1_left",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": left_serial_port,
            "serial_baudrate": 460800,
            "frame_id": "laser_left",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        remappings=[
            ("scan", "/scan_left"),
        ],
    )

    right_lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar_c1_right",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": right_serial_port,
            "serial_baudrate": 460800,
            "frame_id": "laser_right",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        remappings=[
            ("scan", "/scan_right"),
        ],
    )

    merge_node = ExecuteProcess(
        cmd=[
            "python3",
            "/home/rudrh/Autonomous-Wheelchair-System/Integration/Sensors/merge_two_lidars.py"
        ],
        output="screen",
    )

    scan_frequency = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "topic", "hz", "/scan"],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "left_serial_port",
            default_value="/dev/ttyUSB0",
            description="USB serial port of the left RPLIDAR C1",
        ),
        DeclareLaunchArgument(
            "right_serial_port",
            default_value="/dev/ttyUSB1",
            description="USB serial port of the right RPLIDAR C1",
        ),

        left_lidar_node,
        right_lidar_node,
        merge_node,
        scan_frequency,
    ])