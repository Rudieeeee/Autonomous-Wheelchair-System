#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ConditionalCmdVelLimiter(Node):
    def __init__(self):
        super().__init__("conditional_cmd_vel_limiter")

        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel_limited")

        # If abs(linear.x) is below this, it counts as pure rotation.
        self.declare_parameter("linear_zero_threshold", 0.02)

        # Angular cap when the robot is also driving forward/backward.
        self.declare_parameter("moving_angular_limit", 0.15)

        # Optional safety cap during pure rotation.
        # Set high because velocity_smoother already limits final speed.
        self.declare_parameter("pure_rotation_angular_limit", 10.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.linear_zero_threshold = float(self.get_parameter("linear_zero_threshold").value)
        self.moving_angular_limit = abs(float(self.get_parameter("moving_angular_limit").value))
        self.pure_rotation_angular_limit = abs(
            float(self.get_parameter("pure_rotation_angular_limit").value)
        )

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.cmd_vel_callback,
            10,
        )

        self.get_logger().info("conditional_cmd_vel_limiter started")
        self.get_logger().info(f"input_topic={self.input_topic}")
        self.get_logger().info(f"output_topic={self.output_topic}")
        self.get_logger().info(f"linear_zero_threshold={self.linear_zero_threshold}")
        self.get_logger().info(f"moving_angular_limit={self.moving_angular_limit}")
        self.get_logger().info(f"pure_rotation_angular_limit={self.pure_rotation_angular_limit}")

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, value))

    def cmd_vel_callback(self, msg: Twist):
        out = Twist()

        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z

        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y

        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        if abs(linear_x) <= self.linear_zero_threshold:
            # Pure rotation: keep high angular velocity.
            out.angular.z = self.clamp(
                angular_z,
                self.pure_rotation_angular_limit,
            )
        else:
            # Driving + turning: reduce angular velocity.
            out.angular.z = self.clamp(
                angular_z,
                self.moving_angular_limit,
            )

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConditionalCmdVelLimiter()

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