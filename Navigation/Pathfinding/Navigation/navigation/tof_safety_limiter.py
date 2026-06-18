#!/usr/bin/env python3

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ToFSafetyLimiter(Node):
    def __init__(self):
        super().__init__("tof_safety_limiter")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_cmd_vel_topic", "/cmd_vel"),
                ("output_cmd_vel_topic", "/cmd_vel_safe"),
                ("scan_topics", ["/scan", "/tof_scan"]),
                ("scan_timeout", 0.50),

                ("linear_threshold", 0.03),
                ("angular_threshold", 0.05),

                ("max_slowdown_scale", 0.50),
                ("stop_distance", 0.20),

                ("forward_slowdown_distance", 1.00),
                ("turning_slowdown_distance", 0.40),
                ("reverse_slowdown_distance", 0.60),

                ("forward_straight_angle_min_deg", -35.0),
                ("forward_straight_angle_max_deg", 35.0),

                ("forward_turning_left_angle_min_deg", -20.0),
                ("forward_turning_left_angle_max_deg", 70.0),

                ("forward_turning_right_angle_min_deg", -70.0),
                ("forward_turning_right_angle_max_deg", 20.0),

                ("turning_in_place_left_angle_min_deg", 0.0),
                ("turning_in_place_left_angle_max_deg", 120.0),

                ("turning_in_place_right_angle_min_deg", -120.0),
                ("turning_in_place_right_angle_max_deg", 0.0),

                ("reverse_angle_min_deg", 145.0),
                ("reverse_angle_max_deg", -145.0),
            ],
        )

        self.input_cmd_vel_topic = self.get_parameter("input_cmd_vel_topic").value
        self.output_cmd_vel_topic = self.get_parameter("output_cmd_vel_topic").value
        self.scan_topics = list(self.get_parameter("scan_topics").value)

        self.scan_timeout = float(self.get_parameter("scan_timeout").value)

        self.linear_threshold = float(self.get_parameter("linear_threshold").value)
        self.angular_threshold = float(self.get_parameter("angular_threshold").value)

        self.max_slowdown_scale = float(
            self.get_parameter("max_slowdown_scale").value
        )
        self.stop_distance = float(self.get_parameter("stop_distance").value)

        self.forward_slowdown_distance = float(
            self.get_parameter("forward_slowdown_distance").value
        )
        self.turning_slowdown_distance = float(
            self.get_parameter("turning_slowdown_distance").value
        )
        self.reverse_slowdown_distance = float(
            self.get_parameter("reverse_slowdown_distance").value
        )

        self.latest_scans: Dict[str, LaserScan] = {}

        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.scan_subs = []
        for topic in self.scan_topics:
            sub = self.create_subscription(
                LaserScan,
                topic,
                lambda msg, topic_name=topic: self.scan_callback(msg, topic_name),
                qos_profile_sensor_data,
            )
            self.scan_subs.append(sub)

        self.safe_cmd_pub = self.create_publisher(
            Twist,
            self.output_cmd_vel_topic,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            "/tof_safety_state",
            10,
        )

        self.get_logger().info(
            f"ToF safety limiter active. "
            f"Input: {self.input_cmd_vel_topic}, "
            f"output: {self.output_cmd_vel_topic}, "
            f"scans: {self.scan_topics}"
        )

    def scan_callback(self, msg: LaserScan, topic_name: str):
        self.latest_scans[topic_name] = msg

    def cmd_vel_callback(self, cmd: Twist):
        mode = self.determine_motion_mode(cmd.linear.x, cmd.angular.z)
        min_distance = self.get_min_distance_in_active_sector(mode)

        safety_state = self.evaluate_safety_state(mode, min_distance)
        safe_cmd = self.apply_safety_limit(cmd, mode, min_distance, safety_state)

        self.safe_cmd_pub.publish(safe_cmd)

        state_msg = String()
        if min_distance is None:
            state_msg.data = f"{mode}: {safety_state}, no obstacle in active sector"
        else:
            state_msg.data = f"{mode}: {safety_state}, nearest={min_distance:.2f} m"
        self.state_pub.publish(state_msg)

    def determine_motion_mode(self, linear_x: float, angular_z: float) -> str:
        if linear_x > self.linear_threshold:
            if angular_z > self.angular_threshold:
                return "FORWARD_TURNING_LEFT"
            if angular_z < -self.angular_threshold:
                return "FORWARD_TURNING_RIGHT"
            return "FORWARD_STRAIGHT"

        if linear_x < -self.linear_threshold:
            return "REVERSE"

        if angular_z > self.angular_threshold:
            return "TURNING_IN_PLACE_LEFT"

        if angular_z < -self.angular_threshold:
            return "TURNING_IN_PLACE_RIGHT"

        return "IDLE"

    def get_mode_config(self, mode: str):
        if mode == "FORWARD_STRAIGHT":
            return {
                "angle_min": self.get_parameter(
                    "forward_straight_angle_min_deg"
                ).value,
                "angle_max": self.get_parameter(
                    "forward_straight_angle_max_deg"
                ).value,
                "slowdown_distance": self.forward_slowdown_distance,
            }

        if mode == "FORWARD_TURNING_LEFT":
            return {
                "angle_min": self.get_parameter(
                    "forward_turning_left_angle_min_deg"
                ).value,
                "angle_max": self.get_parameter(
                    "forward_turning_left_angle_max_deg"
                ).value,
                "slowdown_distance": self.forward_slowdown_distance,
            }

        if mode == "FORWARD_TURNING_RIGHT":
            return {
                "angle_min": self.get_parameter(
                    "forward_turning_right_angle_min_deg"
                ).value,
                "angle_max": self.get_parameter(
                    "forward_turning_right_angle_max_deg"
                ).value,
                "slowdown_distance": self.forward_slowdown_distance,
            }

        if mode == "TURNING_IN_PLACE_LEFT":
            return {
                "angle_min": self.get_parameter(
                    "turning_in_place_left_angle_min_deg"
                ).value,
                "angle_max": self.get_parameter(
                    "turning_in_place_left_angle_max_deg"
                ).value,
                "slowdown_distance": self.turning_slowdown_distance,
            }

        if mode == "TURNING_IN_PLACE_RIGHT":
            return {
                "angle_min": self.get_parameter(
                    "turning_in_place_right_angle_min_deg"
                ).value,
                "angle_max": self.get_parameter(
                    "turning_in_place_right_angle_max_deg"
                ).value,
                "slowdown_distance": self.turning_slowdown_distance,
            }

        if mode == "REVERSE":
            return {
                "angle_min": self.get_parameter("reverse_angle_min_deg").value,
                "angle_max": self.get_parameter("reverse_angle_max_deg").value,
                "slowdown_distance": self.reverse_slowdown_distance,
            }

        return {
            "angle_min": None,
            "angle_max": None,
            "slowdown_distance": None,
        }

    def get_min_distance_in_active_sector(self, mode: str) -> Optional[float]:
        if mode == "IDLE":
            return None

        config = self.get_mode_config(mode)
        angle_min_deg = config["angle_min"]
        angle_max_deg = config["angle_max"]

        if angle_min_deg is None or angle_max_deg is None:
            return None

        nearest_distance = None

        for topic_name, scan in self.latest_scans.items():
            if not self.scan_is_fresh(scan):
                self.get_logger().warn(
                    f"Ignoring stale scan from {topic_name}",
                    throttle_duration_sec=2.0,
                )
                continue

            scan_min = self.get_min_distance_from_scan(
                scan,
                float(angle_min_deg),
                float(angle_max_deg),
            )

            if scan_min is None:
                continue

            if nearest_distance is None or scan_min < nearest_distance:
                nearest_distance = scan_min

        return nearest_distance

    def scan_is_fresh(self, scan: LaserScan) -> bool:
        stamp = scan.header.stamp

        if stamp.sec == 0 and stamp.nanosec == 0:
            return True

        scan_time = rclpy.time.Time.from_msg(stamp)
        now = self.get_clock().now()
        age = (now - scan_time).nanoseconds / 1e9

        return age <= self.scan_timeout

    def get_min_distance_from_scan(
        self,
        scan: LaserScan,
        angle_min_deg: float,
        angle_max_deg: float,
    ) -> Optional[float]:
        nearest = None

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue

            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle_rad = scan.angle_min + index * scan.angle_increment
            angle_deg = math.degrees(angle_rad)
            angle_deg = self.normalize_angle_deg(angle_deg)

            if not self.angle_is_inside_sector(angle_deg, angle_min_deg, angle_max_deg):
                continue

            if nearest is None or distance < nearest:
                nearest = distance

        return nearest

    @staticmethod
    def normalize_angle_deg(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    def angle_is_inside_sector(
        self,
        angle: float,
        sector_min: float,
        sector_max: float,
    ) -> bool:
        angle = self.normalize_angle_deg(angle)
        sector_min = self.normalize_angle_deg(sector_min)
        sector_max = self.normalize_angle_deg(sector_max)

        if sector_min <= sector_max:
            return sector_min <= angle <= sector_max

        return angle >= sector_min or angle <= sector_max

    def evaluate_safety_state(self, mode: str, min_distance: Optional[float]) -> str:
        if mode == "IDLE":
            return "CLEAR"

        if min_distance is None:
            return "CLEAR"

        config = self.get_mode_config(mode)
        slowdown_distance = config["slowdown_distance"]

        if min_distance <= self.stop_distance:
            return "STOP"

        if slowdown_distance is not None and min_distance <= slowdown_distance:
            return "SLOW_DOWN"

        return "CLEAR"

    def apply_safety_limit(
        self,
        cmd: Twist,
        mode: str,
        min_distance: Optional[float],
        safety_state: str,
    ) -> Twist:
        safe_cmd = Twist()

        safe_cmd.linear.x = cmd.linear.x
        safe_cmd.linear.y = cmd.linear.y
        safe_cmd.linear.z = cmd.linear.z

        safe_cmd.angular.x = cmd.angular.x
        safe_cmd.angular.y = cmd.angular.y
        safe_cmd.angular.z = cmd.angular.z

        if safety_state == "STOP":
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0

            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            safe_cmd.angular.z = 0.0

            return safe_cmd

        if safety_state == "SLOW_DOWN":
            scale = self.compute_distance_scale(mode, min_distance)

            safe_cmd.linear.x *= scale
            safe_cmd.linear.y *= scale
            safe_cmd.angular.z *= scale

            return safe_cmd

        return safe_cmd

    def compute_distance_scale(
        self,
        mode: str,
        min_distance: Optional[float],
    ) -> float:
        if min_distance is None:
            return 1.0

        config = self.get_mode_config(mode)
        slowdown_distance = config["slowdown_distance"]

        if slowdown_distance is None:
            return 1.0

        if min_distance <= self.stop_distance:
            return 0.0

        if min_distance >= slowdown_distance:
            return 1.0

        distance_window = slowdown_distance - self.stop_distance

        if distance_window <= 0.0:
            return self.max_slowdown_scale

        raw_scale = (min_distance - self.stop_distance) / distance_window

        return max(self.max_slowdown_scale, min(1.0, raw_scale))


def main(args=None):
    rclpy.init(args=args)

    node = ToFSafetyLimiter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()