import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def parse_carmen_log(filepath):
    entries = []

    with open(filepath, 'r') as file:
        for line in file:
            line = line.strip()

            if not line or line.startswith('#'):
                continue

            parts = line.split()

            if parts[0] != 'FLASER':
                continue

            num_readings = int(parts[1])
            ranges = [
                float(value)
                for value in parts[2:2 + num_readings]
            ]

            base = 2 + num_readings

            laser_x = float(parts[base + 0])
            laser_y = float(parts[base + 1])
            laser_theta = float(parts[base + 2])

            odom_x = float(parts[base + 3])
            odom_y = float(parts[base + 4])
            odom_theta = float(parts[base + 5])

            entries.append({
                'ranges': ranges,
                'laser_x': laser_x,
                'laser_y': laser_y,
                'laser_theta': laser_theta,
                'odom_x': odom_x,
                'odom_y': odom_y,
                'odom_theta': odom_theta,
            })

    return entries


class CarmenReplay(Node):
    def __init__(self):
        super().__init__('carmen_replay')

        self.declare_parameter(
            'log_file',
            (
                '/home/rudrh/Autonomous-Wheelchair-System/'
                'Other-Files/GeneralData/lidar.txt'
            ),
        )

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('angle_min', -1.5707963268)
        self.declare_parameter('angle_max', 1.5707963268)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 50.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('laser_frame', 'laser')

        # New replay-control parameters
        self.declare_parameter('start_index', 0)
        self.declare_parameter('max_entries', -1)
        self.declare_parameter('keep_last_pose_alive', False)

        log_file = self.get_parameter('log_file').value

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value

        self.start_index = int(self.get_parameter('start_index').value)
        self.max_entries = int(self.get_parameter('max_entries').value)
        self.keep_last_pose_alive = bool(
            self.get_parameter('keep_last_pose_alive').value
        )

        all_entries = parse_carmen_log(log_file)

        if self.start_index < 0:
            self.get_logger().warn(
                f'start_index={self.start_index} is invalid. Using 0 instead.'
            )
            self.start_index = 0

        if self.start_index >= len(all_entries):
            self.get_logger().error(
                f'start_index={self.start_index} is outside the log. '
                f'The log only has {len(all_entries)} FLASER entries.'
            )
            self.entries = []
        else:
            if self.max_entries is None or self.max_entries < 0:
                end_index = len(all_entries)
            else:
                end_index = min(
                    self.start_index + self.max_entries,
                    len(all_entries),
                )

            self.entries = all_entries[self.start_index:end_index]

        self.get_logger().info(
            f'Loaded {len(all_entries)} total scan entries from {log_file}'
        )
        self.get_logger().info(
            f'Replaying {len(self.entries)} entries '
            f'from start_index={self.start_index}, '
            f'max_entries={self.max_entries}'
        )
        self.get_logger().info(
            f'keep_last_pose_alive={self.keep_last_pose_alive}'
        )

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.index = 0
        self.last_entry = None

        self.timer = self.create_timer(
            1.0 / self.rate_hz,
            self.publish_next,
        )

    def publish_next(self):
        if not self.entries:
            self.get_logger().error('No entries available to replay.')
            self.timer.cancel()
            rclpy.shutdown()
            return

        if self.index >= len(self.entries):
            if self.keep_last_pose_alive:
                self.publish_last_pose_alive()
                return

            self.get_logger().info('Replay finished.')
            self.timer.cancel()
            rclpy.shutdown()
            return

        entry = self.entries[self.index]
        stamp = self.get_clock().now().to_msg()

        self.publish_tf(entry, stamp)
        self.publish_odom(entry, stamp)
        self.publish_scan(entry, stamp)

        self.last_entry = entry

        self.get_logger().info(
            f'[{self.index + 1}/{len(self.entries)}] '
            f'odom=({entry["odom_x"]:.2f}, '
            f'{entry["odom_y"]:.2f}, '
            f'{entry["odom_theta"]:.2f})'
        )

        self.index += 1

    def publish_last_pose_alive(self):
        if self.last_entry is None:
            self.get_logger().warn(
                'keep_last_pose_alive is true, but no last entry exists.'
            )
            return

        stamp = self.get_clock().now().to_msg()

        self.publish_tf(self.last_entry, stamp)
        self.publish_odom(self.last_entry, stamp)
        self.publish_scan(self.last_entry, stamp)

    def publish_tf(self, entry, stamp):
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame

        odom_to_base.transform.translation.x = entry['odom_x']
        odom_to_base.transform.translation.y = entry['odom_y']
        odom_to_base.transform.translation.z = 0.0
        odom_to_base.transform.rotation = yaw_to_quaternion(
            entry['odom_theta']
        )

        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = stamp
        base_to_laser.header.frame_id = self.base_frame
        base_to_laser.child_frame_id = self.laser_frame

        base_to_laser.transform.translation.x = 0.0
        base_to_laser.transform.translation.y = 0.0
        base_to_laser.transform.translation.z = 0.0
        base_to_laser.transform.rotation = yaw_to_quaternion(0.0)

        self.tf_broadcaster.sendTransform([
            odom_to_base,
            base_to_laser,
        ])

    def publish_odom(self, entry, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = entry['odom_x']
        odom_msg.pose.pose.position.y = entry['odom_y']
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = yaw_to_quaternion(
            entry['odom_theta']
        )

        self.odom_pub.publish(odom_msg)

    def publish_scan(self, entry, stamp):
        ranges = entry['ranges']

        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp
        scan_msg.header.frame_id = self.laser_frame

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = (
            self.angle_max - self.angle_min
        ) / (len(ranges) - 1)

        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.rate_hz
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges

        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarmenReplay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()