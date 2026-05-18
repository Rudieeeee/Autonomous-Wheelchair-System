#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


def normalize_angle_rad(angle: float) -> float:
    """
    Normalize angle to [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def angle_rad_to_0_360_deg(angle_rad: float) -> float:
    """
    Convert radians to degrees in range [0, 360).
    """
    angle_deg = math.degrees(angle_rad) % 360.0
    return angle_deg


def angle_in_blocked_ranges(angle_deg: float, blocked_ranges: List[Tuple[float, float]]) -> bool:
    """
    Checks if angle_deg is inside any blocked range.

    Supports normal ranges:
      45 -> 180

    Also supports wrap-around ranges:
      300 -> 40
    """
    angle_deg = angle_deg % 360.0

    for start_deg, end_deg in blocked_ranges:
        start_deg = start_deg % 360.0
        end_deg = end_deg % 360.0

        if start_deg <= end_deg:
            if start_deg <= angle_deg <= end_deg:
                return True
        else:
            # Wrap-around case, for example 300° to 40°
            if angle_deg >= start_deg or angle_deg <= end_deg:
                return True

    return False


class LidarScanMerger(Node):
    def __init__(self):
        super().__init__("lidar_scan_merger")

        # -------------------- Topics --------------------
        self.declare_parameter("left_scan_topic", "/scan_left")
        self.declare_parameter("right_scan_topic", "/scan_right")
        self.declare_parameter("merged_scan_topic", "/scan")

        # -------------------- Output scan --------------------
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("output_angle_min_deg", -180.0)
        self.declare_parameter("output_angle_max_deg", 180.0)
        self.declare_parameter("output_angle_increment_deg", 0.5)
        self.declare_parameter("output_range_min", 0.05)
        self.declare_parameter("output_range_max", 12.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        # -------------------- Left LiDAR transform --------------------
        # Transform from left_laser frame to base_footprint.
        self.declare_parameter("left_x_m", 0.82)
        self.declare_parameter("left_y_m", 0.27)
        self.declare_parameter("left_yaw_deg", 0.0)

        # -------------------- Right LiDAR transform --------------------
        # Transform from right_laser frame to base_footprint.
        self.declare_parameter("right_x_m", 0.82)
        self.declare_parameter("right_y_m", -0.27)
        self.declare_parameter("right_yaw_deg", 0.0)

        # -------------------- Blocked local LiDAR angle ranges --------------------
        # Your calculated blocked ranges:
        #
        # Left:
        #   total blocked: 45.2° to 180.0°
        #
        # Right:
        #   total blocked: 180.0° to 314.8°
        #
        self.declare_parameter("left_blocked_start_deg", 45.2)
        self.declare_parameter("left_blocked_end_deg", 180.0)

        self.declare_parameter("right_blocked_start_deg", 180.0)
        self.declare_parameter("right_blocked_end_deg", 314.8)

        # -------------------- Read parameters --------------------
        self.left_scan_topic = self.get_parameter("left_scan_topic").value
        self.right_scan_topic = self.get_parameter("right_scan_topic").value
        self.merged_scan_topic = self.get_parameter("merged_scan_topic").value

        self.output_frame = self.get_parameter("output_frame").value

        self.output_angle_min = math.radians(
            float(self.get_parameter("output_angle_min_deg").value)
        )
        self.output_angle_max = math.radians(
            float(self.get_parameter("output_angle_max_deg").value)
        )
        self.output_angle_increment = math.radians(
            float(self.get_parameter("output_angle_increment_deg").value)
        )

        self.output_range_min = float(
            self.get_parameter("output_range_min").value
        )
        self.output_range_max = float(
            self.get_parameter("output_range_max").value
        )

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        self.left_x_m = float(self.get_parameter("left_x_m").value)
        self.left_y_m = float(self.get_parameter("left_y_m").value)
        self.left_yaw_rad = math.radians(
            float(self.get_parameter("left_yaw_deg").value)
        )

        self.right_x_m = float(self.get_parameter("right_x_m").value)
        self.right_y_m = float(self.get_parameter("right_y_m").value)
        self.right_yaw_rad = math.radians(
            float(self.get_parameter("right_yaw_deg").value)
        )

        self.left_blocked_ranges = [
            (
                float(self.get_parameter("left_blocked_start_deg").value),
                float(self.get_parameter("left_blocked_end_deg").value),
            )
        ]

        self.right_blocked_ranges = [
            (
                float(self.get_parameter("right_blocked_start_deg").value),
                float(self.get_parameter("right_blocked_end_deg").value),
            )
        ]

        self.left_scan = None
        self.right_scan = None

        # -------------------- ROS interfaces --------------------
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.merged_scan_topic,
            qos_profile_sensor_data,
        )

        self.left_sub = self.create_subscription(
            LaserScan,
            self.left_scan_topic,
            self.left_callback,
            qos_profile_sensor_data,
        )

        self.right_sub = self.create_subscription(
            LaserScan,
            self.right_scan_topic,
            self.right_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_merged_scan,
        )

        self.get_logger().info(
            f"Merging {self.left_scan_topic} and {self.right_scan_topic} into {self.merged_scan_topic}"
        )
        self.get_logger().info(
            f"Left blocked local range: {self.left_blocked_ranges}"
        )
        self.get_logger().info(
            f"Right blocked local range: {self.right_blocked_ranges}"
        )
        self.get_logger().info(
            f"Left transform: x={self.left_x_m}, y={self.left_y_m}, yaw={math.degrees(self.left_yaw_rad)} deg"
        )
        self.get_logger().info(
            f"Right transform: x={self.right_x_m}, y={self.right_y_m}, yaw={math.degrees(self.right_yaw_rad)} deg"
        )

    def left_callback(self, msg: LaserScan):
        self.left_scan = msg

    def right_callback(self, msg: LaserScan):
        self.right_scan = msg

    def publish_merged_scan(self):
        if self.left_scan is None or self.right_scan is None:
            return

        bin_count = int(
            round(
                (self.output_angle_max - self.output_angle_min)
                / self.output_angle_increment
            )
        ) + 1

        merged_ranges = [float("inf")] * bin_count

        self.insert_transformed_scan(
            scan_msg=self.left_scan,
            merged_ranges=merged_ranges,
            lidar_x_m=self.left_x_m,
            lidar_y_m=self.left_y_m,
            lidar_yaw_rad=self.left_yaw_rad,
            blocked_ranges_deg=self.left_blocked_ranges,
        )

        self.insert_transformed_scan(
            scan_msg=self.right_scan,
            merged_ranges=merged_ranges,
            lidar_x_m=self.right_x_m,
            lidar_y_m=self.right_y_m,
            lidar_yaw_rad=self.right_yaw_rad,
            blocked_ranges_deg=self.right_blocked_ranges,
        )

        stamp = self.get_clock().now().to_msg()

        merged_msg = LaserScan()
        merged_msg.header.stamp = stamp
        merged_msg.header.frame_id = self.output_frame

        merged_msg.angle_min = self.output_angle_min
        merged_msg.angle_max = self.output_angle_max
        merged_msg.angle_increment = self.output_angle_increment

        merged_msg.time_increment = 0.0
        merged_msg.scan_time = 1.0 / self.publish_rate_hz

        merged_msg.range_min = self.output_range_min
        merged_msg.range_max = self.output_range_max
        merged_msg.ranges = merged_ranges

        self.scan_pub.publish(merged_msg)

    def insert_transformed_scan(
        self,
        scan_msg: LaserScan,
        merged_ranges: List[float],
        lidar_x_m: float,
        lidar_y_m: float,
        lidar_yaw_rad: float,
        blocked_ranges_deg: List[Tuple[float, float]],
    ):
        """
        Convert each valid LiDAR point:

        local LiDAR polar:
          range, angle

        to local LiDAR XY:
          x_lidar, y_lidar

        to base_footprint XY:
          x_base, y_base

        to merged polar around base_footprint:
          range_base, angle_base

        Then insert into the output /scan.
        """
        local_angle_rad = scan_msg.angle_min

        cos_yaw = math.cos(lidar_yaw_rad)
        sin_yaw = math.sin(lidar_yaw_rad)

        for range_value in scan_msg.ranges:
            if self.is_valid_input_range(range_value, scan_msg):
                local_angle_deg = angle_rad_to_0_360_deg(local_angle_rad)

                # Remove angles blocked by wheelchair/footrest.
                if not angle_in_blocked_ranges(local_angle_deg, blocked_ranges_deg):
                    # Point in LiDAR local frame.
                    x_lidar = range_value * math.cos(local_angle_rad)
                    y_lidar = range_value * math.sin(local_angle_rad)

                    # Transform LiDAR local point to base_footprint.
                    x_base = (
                        lidar_x_m
                        + cos_yaw * x_lidar
                        - sin_yaw * y_lidar
                    )
                    y_base = (
                        lidar_y_m
                        + sin_yaw * x_lidar
                        + cos_yaw * y_lidar
                    )

                    range_base = math.sqrt(x_base * x_base + y_base * y_base)
                    angle_base = math.atan2(y_base, x_base)
                    angle_base = normalize_angle_rad(angle_base)

                    self.insert_base_point(
                        merged_ranges,
                        range_base,
                        angle_base,
                    )

            local_angle_rad += scan_msg.angle_increment

    def insert_base_point(
        self,
        merged_ranges: List[float],
        range_base: float,
        angle_base: float,
    ):
        if range_base < self.output_range_min:
            return

        if range_base > self.output_range_max:
            return

        if angle_base < self.output_angle_min:
            return

        if angle_base > self.output_angle_max:
            return

        index = int(
            round(
                (angle_base - self.output_angle_min)
                / self.output_angle_increment
            )
        )

        if index < 0 or index >= len(merged_ranges):
            return

        current_value = merged_ranges[index]

        # Keep nearest obstacle if both LiDARs fill the same angle bin.
        if math.isinf(current_value) or range_base < current_value:
            merged_ranges[index] = range_base

    def is_valid_input_range(self, range_value: float, scan_msg: LaserScan) -> bool:
        if math.isnan(range_value):
            return False

        if math.isinf(range_value):
            return False

        if range_value < scan_msg.range_min:
            return False

        if range_value > scan_msg.range_max:
            return False

        if range_value < self.output_range_min:
            return False

        if range_value > self.output_range_max:
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    node = LidarScanMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()