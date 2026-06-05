#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray


class CmdVelToJoystickMapping(Node):
    """
    Converts Nav2 /cmd_vel into Arduino joystick commands on /joystick_cmd.

    This version is for SLAM+Nav2 while mapping:
    - No AMCL covariance gate
    - Optional full-scan obstacle gate
    - Sends [0, 0] when command is stale or scan is stale
    """

    def __init__(self):
        super().__init__("cmd_vel_to_joystick_mapping")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joystick_topic", "/joystick_cmd")
        self.declare_parameter("scan_topic", "/scan")

        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.5)

        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("deadzone_percent", 3)
        self.declare_parameter("timeout_seconds", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.declare_parameter("send_nothing_without_cmd_vel_publisher", True)

        self.declare_parameter("use_obstacle_gate", True)
        self.declare_parameter("full_scan_stop_distance_m", 0.45)
        self.declare_parameter("scan_timeout_seconds", 0.5)

        self.declare_parameter("debug", True)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.joystick_topic = self.get_parameter("joystick_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value

        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )

        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.deadzone_percent = int(
            self.get_parameter("deadzone_percent").value
        )
        self.timeout_seconds = float(
            self.get_parameter("timeout_seconds").value
        )
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        self.send_nothing_without_cmd_vel_publisher = bool(
            self.get_parameter("send_nothing_without_cmd_vel_publisher").value
        )

        self.use_obstacle_gate = bool(
            self.get_parameter("use_obstacle_gate").value
        )
        self.full_scan_stop_distance_m = float(
            self.get_parameter("full_scan_stop_distance_m").value
        )
        self.scan_timeout_seconds = float(
            self.get_parameter("scan_timeout_seconds").value
        )

        self.debug = bool(self.get_parameter("debug").value)

        self.last_twist = None
        self.last_cmd_time = None

        self.last_scan_time = None
        self.scan_is_clear = False
        self.nearest_obstacle_m = math.inf

        self.last_debug_time = 0.0

        self.joystick_pub = self.create_publisher(
            Int16MultiArray,
            self.joystick_topic,
            10,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            "cmd_vel_to_joystick_mapping started: "
            f"cmd_vel_topic={self.cmd_vel_topic}, "
            f"joystick_topic={self.joystick_topic}, "
            f"scan_topic={self.scan_topic}, "
            f"max_linear_speed={self.max_linear_speed}, "
            f"max_angular_speed={self.max_angular_speed}, "
            f"use_obstacle_gate={self.use_obstacle_gate}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = time.time()

    def scan_callback(self, msg: LaserScan):
        now = time.time()
        self.last_scan_time = now

        nearest = math.inf

        for value in msg.ranges:
            if math.isfinite(value):
                if msg.range_min <= value <= msg.range_max:
                    nearest = min(nearest, value)

        self.nearest_obstacle_m = nearest
        self.scan_is_clear = nearest > self.full_scan_stop_distance_m

    def clamp_percent(self, value):
        value = int(round(value))
        return max(-100, min(100, value))

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone_percent:
            return 0
        return value

    def twist_to_joystick(self, twist: Twist):
        linear = twist.linear.x
        angular = twist.angular.z

        if self.max_linear_speed > 0.0:
            y = 100.0 * linear / self.max_linear_speed
        else:
            y = 0.0

        if self.max_angular_speed > 0.0:
            x = 100.0 * angular / self.max_angular_speed
        else:
            x = 0.0

        if self.invert_x:
            x = -x

        if self.invert_y:
            y = -y

        x = self.apply_deadzone(self.clamp_percent(x))
        y = self.apply_deadzone(self.clamp_percent(y))

        return x, y

    def publish_joystick(self, x, y):
        msg = Int16MultiArray()
        msg.data = [int(x), int(y)]
        self.joystick_pub.publish(msg)

    def timer_callback(self):
        now = time.time()

        cmd_is_available = self.last_twist is not None
        cmd_is_fresh = (
            self.last_cmd_time is not None and
            now - self.last_cmd_time <= self.timeout_seconds
        )

        if not cmd_is_available:
            if not self.send_nothing_without_cmd_vel_publisher:
                self.publish_joystick(0, 0)
            self.debug_throttled(
                "No /cmd_vel received yet; publishing nothing."
                if self.send_nothing_without_cmd_vel_publisher
                else "No /cmd_vel received yet; publishing [0, 0]."
            )
            return

        if not cmd_is_fresh:
            self.publish_joystick(0, 0)
            self.debug_throttled("Stale /cmd_vel; publishing [0, 0].")
            return

        if self.use_obstacle_gate:
            scan_is_fresh = (
                self.last_scan_time is not None and
                now - self.last_scan_time <= self.scan_timeout_seconds
            )

            if not scan_is_fresh:
                self.publish_joystick(0, 0)
                self.debug_throttled("Stale/missing /scan; publishing [0, 0].")
                return

            if not self.scan_is_clear:
                self.publish_joystick(0, 0)
                self.debug_throttled(
                    f"Obstacle too close ({self.nearest_obstacle_m:.3f} m); "
                    "publishing [0, 0]."
                )
                return

        x, y = self.twist_to_joystick(self.last_twist)
        self.publish_joystick(x, y)

        self.debug_throttled(
            f"Publishing joystick [x={x}, y={y}], "
            f"nearest_obstacle={self.nearest_obstacle_m:.3f}"
        )

    def debug_throttled(self, message):
        if not self.debug:
            return

        now = time.time()
        if now - self.last_debug_time < 1.0:
            return

        self.last_debug_time = now
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelToJoystickMapping()

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
