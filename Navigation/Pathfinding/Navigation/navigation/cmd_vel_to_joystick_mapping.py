#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy


class CmdVelToJoystickMapping(Node):
    """
    Converts Nav2 /cmd_vel into Arduino joystick commands on /joystick_cmd.

    This version is for SLAM + Nav2 while mapping:
    - No AMCL covariance gate
    - Uses /scan obstacle gate
    - Sends [0, 0] when /cmd_vel is stale
    - Sends [0, 0] when /scan is stale
    - Uses BEST_EFFORT QoS for LaserScan
    """

    def __init__(self):
        super().__init__("cmd_vel_to_joystick_mapping")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joystick_topic", "/joystick_cmd")
        self.declare_parameter("scan_topic", "/scan")

        # IMPORTANT:
        # These are NOT Nav2 maximum speeds.
        # These are the wheelchair speed represented by joystick 100.
        #
        # If your launch overrides these with 0.25 and 0.5, then Nav2 max speed
        # becomes joystick 100 immediately.
        self.declare_parameter("max_linear_speed", 1.67)
        self.declare_parameter("max_angular_speed", 1.1)

        self.declare_parameter("invert_x", True)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("deadzone_percent", 3)

        # Wheelchair often does not move below this joystick value.
        # 0 is still allowed for stopping.
        self.declare_parameter("minimum_nonzero_joystick", 52)

        # Extra safety caps for autonomous mode.
        # X = turning joystick, Y = forward/backward joystick.
        self.declare_parameter("max_joystick_x", 80)
        self.declare_parameter("max_joystick_y", 80)

        # Pure rotation cap.
        # Do not force pure rotations to 100, because that can swing into objects.
        self.declare_parameter("pure_rotation_joystick", 60)

        self.declare_parameter("timeout_seconds", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        # If True:
        # - no /cmd_vel publisher -> publish nothing
        # - /cmd_vel publisher exists but messages stop -> publish [0, 0]
        self.declare_parameter("send_nothing_without_cmd_vel_publisher", True)

        # Full scan obstacle gate.
        self.declare_parameter("use_obstacle_gate", True)
        self.declare_parameter("full_scan_stop_distance_m", 0.45)
        self.declare_parameter("scan_timeout_seconds", 0.5)

        self.declare_parameter("debug", True)
        self.declare_parameter("debug_period_seconds", 1.0)

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
        self.minimum_nonzero_joystick = int(
            self.get_parameter("minimum_nonzero_joystick").value
        )

        self.max_joystick_x = int(
            self.get_parameter("max_joystick_x").value
        )
        self.max_joystick_y = int(
            self.get_parameter("max_joystick_y").value
        )

        self.pure_rotation_joystick = int(
            self.get_parameter("pure_rotation_joystick").value
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
        self.debug_period_seconds = float(
            self.get_parameter("debug_period_seconds").value
        )

        self.current_x = 0
        self.current_y = 0

        self.has_received_cmd_vel = False
        self.last_cmd_time = self.get_clock().now()

        self.has_received_scan = False
        self.last_scan_time = self.get_clock().now()
        self.closest_scan_distance = float("inf")
        self.obstacle_too_close = False

        self.last_debug_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            Int16MultiArray,
            self.joystick_topic,
            10,
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        # Many LaserScan publishers use BEST_EFFORT QoS.
        # Default reliable QoS may not connect to them.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info("cmd_vel_to_joystick_mapping started")
        self.get_logger().info(f"Subscribing to cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"Subscribing to scan: {self.scan_topic}")
        self.get_logger().info(f"Publishing joystick to: {self.joystick_topic}")
        self.get_logger().info(f"max_linear_speed={self.max_linear_speed}")
        self.get_logger().info(f"max_angular_speed={self.max_angular_speed}")
        self.get_logger().info(
            f"minimum_nonzero_joystick={self.minimum_nonzero_joystick}"
        )
        self.get_logger().info(
            f"max_joystick_x={self.max_joystick_x}, "
            f"max_joystick_y={self.max_joystick_y}"
        )
        self.get_logger().info(
            f"pure_rotation_joystick={self.pure_rotation_joystick}"
        )
        self.get_logger().info(
            f"invert_x={self.invert_x}, invert_y={self.invert_y}"
        )
        self.get_logger().info(
            f"Obstacle gate enabled={self.use_obstacle_gate}, "
            f"full_scan_stop_distance_m={self.full_scan_stop_distance_m}, "
            f"scan_timeout_seconds={self.scan_timeout_seconds}, "
            f"scan_qos=BEST_EFFORT"
        )

    def clamp(self, value, min_value=-100, max_value=100):
        return max(min_value, min(max_value, value))

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone_percent:
            return 0
        return value

    def apply_minimum_nonzero_joystick(self, value):
        value = int(value)

        if value == 0:
            return 0

        minimum = abs(self.minimum_nonzero_joystick)
        minimum = self.clamp(minimum, 0, 100)

        if minimum == 0:
            return value

        if abs(value) < minimum:
            return minimum if value > 0 else -minimum

        return value

    def scale_to_joystick(self, value, max_value, max_joystick):
        if max_value <= 0.0:
            return 0

        scaled = int((value / max_value) * 100.0)
        scaled = self.clamp(scaled)
        scaled = self.apply_deadzone(scaled)
        scaled = self.apply_minimum_nonzero_joystick(scaled)

        max_joystick = abs(int(max_joystick))
        max_joystick = self.clamp(max_joystick, 0, 100)

        scaled = self.clamp(scaled, -max_joystick, max_joystick)

        return scaled

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Joystick X = turn left/right.
        joystick_x = self.scale_to_joystick(
            angular_z,
            self.max_angular_speed,
            self.max_joystick_x,
        )

        # Joystick Y = forward/backward.
        joystick_y = self.scale_to_joystick(
            linear_x,
            self.max_linear_speed,
            self.max_joystick_y,
        )

        if self.invert_x:
            joystick_x = -joystick_x

        if self.invert_y:
            joystick_y = -joystick_y

        # If Nav2 commands pure rotation, do not force full 100.
        # Full rotation can make the wheelchair swing into nearby objects.
        if joystick_y == 0 and joystick_x != 0:
            pure = abs(self.pure_rotation_joystick)
            pure = self.clamp(pure, 0, 100)
            pure = min(pure, abs(self.max_joystick_x))

            joystick_x = pure if joystick_x > 0 else -pure

        self.current_x = joystick_x
        self.current_y = joystick_y

        self.has_received_cmd_vel = True
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        closest_distance = float("inf")

        for distance in msg.ranges:
            if math.isinf(distance) or math.isnan(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            closest_distance = min(closest_distance, distance)

        self.closest_scan_distance = closest_distance
        self.obstacle_too_close = (
            closest_distance < self.full_scan_stop_distance_m
        )

        self.has_received_scan = True
        self.last_scan_time = self.get_clock().now()

        if self.obstacle_too_close:
            self.debug_throttled(
                f"Obstacle too close somewhere in full scan: "
                f"{closest_distance:.2f} m. Blocking joystick movement."
            )

    def publish_joystick(self, x, y):
        x = int(self.clamp(x, -100, 100))
        y = int(self.clamp(y, -100, 100))

        msg = Int16MultiArray()
        msg.data = [x, y]
        self.publisher.publish(msg)

    def scan_allows_movement(self):
        if not self.use_obstacle_gate:
            return True

        if not self.has_received_scan:
            self.debug_throttled(
                "No /scan received yet. Publishing [0, 0]."
            )
            return False

        now = self.get_clock().now()
        scan_age = (now - self.last_scan_time).nanoseconds / 1e9

        if scan_age > self.scan_timeout_seconds:
            self.debug_throttled(
                "No recent /scan received. Publishing [0, 0]."
            )
            return False

        if self.obstacle_too_close:
            self.debug_throttled(
                f"Obstacle gate blocked movement. "
                f"Closest scan point: {self.closest_scan_distance:.2f} m"
            )
            return False

        return True

    def timer_callback(self):
        cmd_vel_publishers = self.count_publishers(self.cmd_vel_topic)

        # If Nav2 /cmd_vel does not exist as a publisher, send nothing.
        if (
            self.send_nothing_without_cmd_vel_publisher
            and cmd_vel_publishers == 0
        ):
            self.has_received_cmd_vel = False
            self.current_x = 0
            self.current_y = 0
            self.debug_throttled(
                "No /cmd_vel publisher. Publishing nothing."
            )
            return

        now = self.get_clock().now()
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9

        # If /cmd_vel publisher exists, but commands stop, send neutral.
        if (not self.has_received_cmd_vel) or cmd_age > self.timeout_seconds:
            self.publish_joystick(0, 0)
            self.debug_throttled(
                "No recent /cmd_vel. Publishing [0, 0]."
            )
            return

        # Obstacle gate for mapping mode.
        # This is intentionally full-scan: if anything is too close, stop.
        if not self.scan_allows_movement():
            self.publish_joystick(0, 0)
            return

        self.publish_joystick(self.current_x, self.current_y)

        self.debug_throttled(
            f"Publishing joystick [{self.current_x}, {self.current_y}], "
            f"closest_scan={self.closest_scan_distance:.2f} m"
        )

    def debug_throttled(self, message):
        if not self.debug:
            return

        now = self.get_clock().now()
        age = (now - self.last_debug_time).nanoseconds / 1e9

        if age < self.debug_period_seconds:
            return

        self.last_debug_time = now
        self.get_logger().info(message)

    def destroy_node(self):
        # Do NOT publish [0, 0] during shutdown.
        # During launch shutdown this could still be forwarded by the Arduino node.
        super().destroy_node()


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