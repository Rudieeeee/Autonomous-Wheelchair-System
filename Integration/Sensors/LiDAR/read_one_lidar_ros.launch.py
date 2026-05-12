#Run: source /opt/ros/jazzy/setup.bash
#source ~/ros2_ws/install/setup.bash
#ros2 launch /home/rudrh/Autonomous-Wheelchair-System/Integration/Sensors/read_one_lidar_ros.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    frame_id = LaunchConfiguration("frame_id")

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar_c1",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": 460800,
            "frame_id": frame_id,
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        remappings=[
            ("scan", "/scan"),
        ],
    )

    echo_scan_once = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "topic", "echo", "/scan", "--once"],
                output="screen",
            )
        ],
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
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="USB serial port of the RPLIDAR C1",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="laser",
            description="Frame name used in the LaserScan header",
        ),

        lidar_node,
        echo_scan_once,
        scan_frequency,
    ])