#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray


class CmdVelToJoystick(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_joystick")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joystick_topic", "/joystick_cmd")

        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.5)

        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("deadzone_percent", 3)
        self.declare_parameter("timeout_seconds", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        # If True:
        # - no /cmd_vel publisher  -> publish nothing
        # - /cmd_vel publisher exists but messages stop -> publish neutral [0, 0]
        self.declare_parameter("send_nothing_without_cmd_vel_publisher", True)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.joystick_topic = self.get_parameter("joystick_topic").value

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
            self.get_parameter(
                "send_nothing_without_cmd_vel_publisher"
            ).value
        )

        self.current_x = 0
        self.current_y = 0

        self.has_received_cmd_vel = False
        self.last_cmd_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            Int16MultiArray,
            self.joystick_topic,
            10,
        )

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info("cmd_vel_to_joystick started")
        self.get_logger().info(f"Subscribing to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Publishing to:   {self.joystick_topic}")
        self.get_logger().info(f"max_linear_speed={self.max_linear_speed}")
        self.get_logger().info(f"max_angular_speed={self.max_angular_speed}")

    def clamp(self, value, min_value=-100, max_value=100):
        return max(min_value, min(max_value, value))

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone_percent:
            return 0
        return value

    def scale_to_joystick(self, value, max_value):
        if max_value <= 0.0:
            return 0

        scaled = int((value / max_value) * 100.0)
        scaled = self.clamp(scaled)
        scaled = self.apply_deadzone(scaled)

        return scaled

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Joystick X = turn left/right
        joystick_x = self.scale_to_joystick(
            angular_z,
            self.max_angular_speed,
        )

        # Joystick Y = forward/backward
        joystick_y = self.scale_to_joystick(
            linear_x,
            self.max_linear_speed,
        )

        if self.invert_x:
            joystick_x = -joystick_x

        if self.invert_y:
            joystick_y = -joystick_y

        self.current_x = joystick_x
        self.current_y = joystick_y

        self.has_received_cmd_vel = True
        self.last_cmd_time = self.get_clock().now()

    def publish_joystick(self, x, y):
        msg = Int16MultiArray()
        msg.data = [int(x), int(y)]
        self.publisher.publish(msg)

    def timer_callback(self):
        cmd_vel_publishers = self.count_publishers(self.cmd_vel_topic)

        # If Nav2 /cmd_vel does not exist as a publisher, send nothing.
        # This prevents the Arduino from receiving joystick commands when Nav2 is not running.
        if (
            self.send_nothing_without_cmd_vel_publisher
            and cmd_vel_publishers == 0
        ):
            self.has_received_cmd_vel = False
            self.current_x = 0
            self.current_y = 0
            return

        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9

        # If /cmd_vel publisher exists, but commands stop, send neutral for safety.
        if (not self.has_received_cmd_vel) or age > self.timeout_seconds:
            self.publish_joystick(0, 0)
            return

        self.publish_joystick(self.current_x, self.current_y)

    def destroy_node(self):
        # Only publish neutral if a /cmd_vel publisher existed before.
        if self.count_publishers(self.cmd_vel_topic) > 0:
            self.publish_joystick(0, 0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelToJoystick()

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