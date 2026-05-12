#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class TwoLidarHalfMerger(Node):
    def __init__(self):
        super().__init__("two_lidar_half_merger")

        self.left_scan = None
        self.right_scan = None

        self.output_frame = "base_scan"

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        self.left_sub = self.create_subscription(
            LaserScan,
            "/scan_left",
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            LaserScan,
            "/scan_right",
            self.right_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.publish_merged_scan)

        self.get_logger().info("Two LiDAR half-range merger started")
        self.get_logger().info("0 to 180 deg from /scan_left")
        self.get_logger().info("180 to 360 deg from /scan_right")
        self.get_logger().info("Publishing merged scan to /scan")

    def left_callback(self, msg):
        self.left_scan = msg

    def right_callback(self, msg):
        self.right_scan = msg

    def valid_range(self, r, range_min, range_max):
        return math.isfinite(r) and range_min <= r <= range_max

    def angle_to_0_360_deg(self, angle_rad):
        angle_deg = math.degrees(angle_rad)

        while angle_deg < 0.0:
            angle_deg += 360.0

        while angle_deg >= 360.0:
            angle_deg -= 360.0

        return angle_deg

    def publish_merged_scan(self):
        if self.left_scan is None or self.right_scan is None:
            return

        left = self.left_scan
        right = self.right_scan

        if len(left.ranges) != len(right.ranges):
            self.get_logger().warn("Left and right scan sizes do not match")
            return

        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self.output_frame

        merged.angle_min = left.angle_min
        merged.angle_max = left.angle_max
        merged.angle_increment = left.angle_increment
        merged.time_increment = left.time_increment
        merged.scan_time = left.scan_time

        merged.range_min = min(left.range_min, right.range_min)
        merged.range_max = max(left.range_max, right.range_max)

        merged_ranges = []
        merged_intensities = []

        for i in range(len(left.ranges)):
            angle_rad = left.angle_min + i * left.angle_increment
            angle_deg = self.angle_to_0_360_deg(angle_rad)

            if 0.0 <= angle_deg < 180.0:
                r = left.ranges[i]
                intensity = left.intensities[i] if i < len(left.intensities) else 0.0
            else:
                r = right.ranges[i]
                intensity = right.intensities[i] if i < len(right.intensities) else 0.0

            if self.valid_range(r, merged.range_min, merged.range_max):
                merged_ranges.append(r)
                merged_intensities.append(float(intensity))
            else:
                merged_ranges.append(float("inf"))
                merged_intensities.append(0.0)

        merged.ranges = merged_ranges
        merged.intensities = merged_intensities

        self.scan_pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = TwoLidarHalfMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()