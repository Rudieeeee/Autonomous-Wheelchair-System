#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CarmenMappingLogger(Node):
    def __init__(self):
        super().__init__("carmen_mapping_logger")

        self.declare_parameter("log_file", "/tmp/mapping_log.txt")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("num_readings", 180)
        self.declare_parameter("max_range_value", 50.0)

        self.log_file = self.get_parameter("log_file").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.num_readings = int(self.get_parameter("num_readings").value)
        self.max_range_value = float(self.get_parameter("max_range_value").value)

        self.latest_odom = None

        # The example logger_timestamp starts at 0.000000.
        # ipc_timestamp is absolute-ish runtime seconds from the source message.
        self.start_logger_time = time.monotonic()

        log_dir = os.path.dirname(self.log_file)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        self.file = open(self.log_file, "w", buffering=1)

        self.write_exact_header()

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.get_logger().info(
            f"Writing CARMEN-style mapping log to: {self.log_file}"
        )

    def write_exact_header(self):
        self.file.write(
            "# message_name [message contents] ipc_timestamp ipc_hostname logger_timestamp\n"
        )
        self.file.write(
            "# message formats defined: PARAM SYNC ODOM FLASER RLASER TRUEPOS \n"
        )
        self.file.write("# PARAM param_name param_value\n")
        self.file.write("# SYNC tagname\n")
        self.file.write("# ODOM x y theta tv rv accel\n")
        self.file.write(
            "# FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n"
        )
        self.file.write(
            "# RLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n"
        )
        self.file.write(
            "# TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta\n"
        )
        self.file.write(
            "# NMEA-GGA utc latitude lat_orient longitude long_orient gps_quality num_sattelites hdop sea_level alitude geo_sea_level geo_sep data_age\n"
        )

        self.file.write("PARAM robot_use_laser on nohost 0\n")
        self.file.write(
            f"PARAM simulator_num_readings {self.num_readings} nohost 0\n"
        )
        self.file.write("PARAM robot_frontlaser_offset 0.0 nohost 0\n")
        self.file.write("PARAM robot_rearlaser_offset 0.0 nohost 0\n")

    def stamp_to_float(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def logger_timestamp(self):
        return time.monotonic() - self.start_logger_time

    def odom_callback(self, msg):
        self.latest_odom = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quaternion_to_yaw(msg.pose.pose.orientation)

        tv = msg.twist.twist.linear.x
        rv = msg.twist.twist.angular.z
        accel = 0.0

        ipc_timestamp = self.stamp_to_float(msg.header.stamp)
        logger_timestamp = self.logger_timestamp()

        self.file.write(
            f"ODOM "
            f"{x:.6f} {y:.6f} {theta:.6f} "
            f"{tv:.6f} {rv:.6f} {accel:.6f} "
            f"{ipc_timestamp:.6f} nohost {logger_timestamp:.6f}\n"
        )

    def clean_range(self, value, range_min, range_max):
        if not math.isfinite(value):
            return self.max_range_value

        if value < range_min:
            return self.max_range_value

        if value > range_max:
            return self.max_range_value

        return float(value)

    def resample_ranges_exact_count(self, scan_msg):
        cleaned = [
            self.clean_range(value, scan_msg.range_min, scan_msg.range_max)
            for value in scan_msg.ranges
        ]

        if not cleaned:
            return [self.max_range_value] * self.num_readings

        if len(cleaned) == self.num_readings:
            return cleaned

        output = []

        # Pick evenly spaced samples, so output is exactly 180 readings,
        # like your example file.
        for i in range(self.num_readings):
            if self.num_readings == 1:
                source_index = 0
            else:
                source_index = round(
                    i * (len(cleaned) - 1) / (self.num_readings - 1)
                )

            output.append(cleaned[int(source_index)])

        return output

    def scan_callback(self, msg):
        if self.latest_odom is None:
            return

        odom = self.latest_odom

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        theta = quaternion_to_yaw(odom.pose.pose.orientation)

        odom_x = x
        odom_y = y
        odom_theta = theta

        ranges = self.resample_ranges_exact_count(msg)
        ranges_text = " ".join(f"{value:.2f}" for value in ranges)

        ipc_timestamp = self.stamp_to_float(msg.header.stamp)
        logger_timestamp = self.logger_timestamp()

        self.file.write(
            f"FLASER {self.num_readings} {ranges_text} "
            f"{x:.6f} {y:.6f} {theta:.6f} "
            f"{odom_x:.6f} {odom_y:.6f} {odom_theta:.6f} "
            f"{ipc_timestamp:.6f} nohost {logger_timestamp:.6f}\n"
        )

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CarmenMappingLogger()

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