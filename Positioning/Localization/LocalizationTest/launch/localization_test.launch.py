from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    log_file = LaunchConfiguration('log_file')
    rate_hz = LaunchConfiguration('rate_hz')
    use_rviz = LaunchConfiguration('use_rviz')
    start_index = LaunchConfiguration('start_index')
    max_entries = LaunchConfiguration('max_entries')

    amcl_config = PathJoinSubstitution([
        FindPackageShare('localization_test'),
        'config',
        'amcl.yaml',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('localization_test'),
        'rviz',
        'localization.rviz',
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'yaml_filename': map_file,
            }
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }
        ],
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz],
    )

    replay = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mapping_test', 'replay_carmen_log',
            '--ros-args',
            '-p', ['log_file:=', log_file],
            '-p', ['rate_hz:=', rate_hz],
            '-p', ['start_index:=', start_index],
            '-p', ['max_entries:=', max_entries],
            '-p', 'keep_last_pose_alive:=true',
            '-p', 'angle_min:=-1.5707963268',
            '-p', 'angle_max:=1.5707963268',
            '-p', 'range_min:=0.1',
            '-p', 'range_max:=50.0',
            '-p', 'laser_frame:=laser',
            '-p', 'base_frame:=base_footprint',
            '-p', 'odom_frame:=odom',
        ],
        output='screen',
    )

    delayed_replay = TimerAction(
        period=8.0,
        actions=[replay],
    )

    global_localization = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/reinitialize_global_localization',
            'std_srvs/srv/Empty',
            '{}',
        ],
        output='screen',
    )

    delayed_global_localization = TimerAction(
        period=15.0,
        actions=[global_localization],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/Maps/my_map.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'log_file',
            default_value=(
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/lidar.txt'
            ),
        ),

        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('start_index', default_value='0'),
        DeclareLaunchArgument('max_entries', default_value='-1'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        map_server,
        amcl,
        lifecycle_manager,

        delayed_rviz,
        delayed_replay,
        delayed_global_localization,
    ])