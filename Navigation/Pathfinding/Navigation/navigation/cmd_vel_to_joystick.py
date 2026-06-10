#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy


class CmdVelToJoystick(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_joystick")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joystick_topic", "/joystick_cmd")

        # These are wheelchair calibration values, not Nav2 limits.
        # If joystick Y=100 gives about 5 km/h, that is about 1.39 m/s.
        self.declare_parameter("max_linear_speed", 1.39)
        self.declare_parameter("max_angular_speed", 0.42)

        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("deadzone_percent", 3)

        # Your measured breakaway joystick values.
        self.declare_parameter("min_forward_joystick", 25)
        self.declare_parameter("min_backward_joystick", 40)

        self.declare_parameter("max_joystick_x", 100)
        self.declare_parameter("max_joystick_y", 100)

        # Full-power rotation value. Used only for large angular commands.
        self.declare_parameter("pure_rotation_joystick", 100)

        # OLD behaviour made every command above this become +-100.
        # In this new code it is only used as the high/strong turn threshold fallback.
        self.declare_parameter("pure_rotation_angular_threshold", 0.18)

        # In mixed driving, X is capped so forward+turn does not become too violent.
        self.declare_parameter("mixed_turn_max_joystick", 35)

        self.declare_parameter("linear_cmd_deadband", 0.01)
        self.declare_parameter("angular_cmd_deadband", 0.01)

        # Stepped turn behaviour. This prevents overshoot.
        # Small angular command  -> small joystick X.
        # Medium angular command -> medium joystick X.
        # Large angular command  -> full joystick X.
        self.declare_parameter("slow_turn_angular_threshold", 0.08)
        self.declare_parameter("medium_turn_angular_threshold", 0.18)
        self.declare_parameter("slow_turn_joystick", 35)
        self.declare_parameter("medium_turn_joystick", 60)

        self.declare_parameter("timeout_seconds", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.declare_parameter("send_nothing_without_cmd_vel_publisher", True)

        # AMCL startup gate.
        self.declare_parameter("require_accurate_amcl", True)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        self.declare_parameter("max_x_covariance", 0.04)
        self.declare_parameter("max_y_covariance", 0.04)
        self.declare_parameter("max_yaw_covariance", 0.03)

        self.declare_parameter("min_good_amcl_messages", 5)
        self.declare_parameter("amcl_timeout_seconds", 10.0)

        # While AMCL is not accepted, rotate in place to help localization.
        # Obstacle gate can still block this.
        self.declare_parameter("amcl_block_joystick_x", 100)
        self.declare_parameter("amcl_block_joystick_y", 0)

        # LiDAR emergency stop gate.
        self.declare_parameter("use_obstacle_gate", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("full_scan_stop_distance_m", 0.50)
        self.declare_parameter("scan_timeout_seconds", 0.5)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.joystick_topic = self.get_parameter("joystick_topic").value

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.deadzone_percent = int(self.get_parameter("deadzone_percent").value)

        self.min_forward_joystick = int(self.get_parameter("min_forward_joystick").value)
        self.min_backward_joystick = int(self.get_parameter("min_backward_joystick").value)

        self.max_joystick_x = int(self.get_parameter("max_joystick_x").value)
        self.max_joystick_y = int(self.get_parameter("max_joystick_y").value)

        self.pure_rotation_joystick = int(self.get_parameter("pure_rotation_joystick").value)
        self.pure_rotation_angular_threshold = float(
            self.get_parameter("pure_rotation_angular_threshold").value
        )

        self.mixed_turn_max_joystick = int(self.get_parameter("mixed_turn_max_joystick").value)

        self.linear_cmd_deadband = float(self.get_parameter("linear_cmd_deadband").value)
        self.angular_cmd_deadband = float(self.get_parameter("angular_cmd_deadband").value)

        self.slow_turn_angular_threshold = float(
            self.get_parameter("slow_turn_angular_threshold").value
        )
        self.medium_turn_angular_threshold = float(
            self.get_parameter("medium_turn_angular_threshold").value
        )
        self.slow_turn_joystick = int(self.get_parameter("slow_turn_joystick").value)
        self.medium_turn_joystick = int(self.get_parameter("medium_turn_joystick").value)

        self.timeout_seconds = float(self.get_parameter("timeout_seconds").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.send_nothing_without_cmd_vel_publisher = bool(
            self.get_parameter("send_nothing_without_cmd_vel_publisher").value
        )

        self.require_accurate_amcl = bool(self.get_parameter("require_accurate_amcl").value)
        self.amcl_pose_topic = self.get_parameter("amcl_pose_topic").value

        self.max_x_covariance = float(self.get_parameter("max_x_covariance").value)
        self.max_y_covariance = float(self.get_parameter("max_y_covariance").value)
        self.max_yaw_covariance = float(self.get_parameter("max_yaw_covariance").value)

        self.min_good_amcl_messages = int(self.get_parameter("min_good_amcl_messages").value)
        self.amcl_timeout_seconds = float(self.get_parameter("amcl_timeout_seconds").value)

        self.amcl_block_joystick_x = int(self.get_parameter("amcl_block_joystick_x").value)
        self.amcl_block_joystick_y = int(self.get_parameter("amcl_block_joystick_y").value)

        self.use_obstacle_gate = bool(self.get_parameter("use_obstacle_gate").value)
        self.scan_topic = self.get_parameter("scan_topic").value
        self.full_scan_stop_distance_m = float(
            self.get_parameter("full_scan_stop_distance_m").value
        )
        self.scan_timeout_seconds = float(self.get_parameter("scan_timeout_seconds").value)

        self.current_x = 0
        self.current_y = 0

        self.has_received_cmd_vel = False
        self.last_cmd_time = self.get_clock().now()

        self.amcl_is_accurate = False
        self.amcl_pose_estimation_done = False
        self.good_amcl_count = 0
        self.has_received_amcl = False
        self.last_amcl_time = self.get_clock().now()

        self.obstacle_too_close = False
        self.has_received_scan = False
        self.last_scan_time = self.get_clock().now()
        self.closest_scan_distance = float("inf")

        self.publisher = self.create_publisher(Int16MultiArray, self.joystick_topic, 10)

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_pose_callback,
            10,
        )

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

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.get_logger().info("cmd_vel_to_joystick started")
        self.get_logger().info(f"Subscribing to cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"Publishing joystick to: {self.joystick_topic}")
        self.get_logger().info(f"max_linear_speed={self.max_linear_speed}")
        self.get_logger().info(f"max_angular_speed={self.max_angular_speed}")
        self.get_logger().info(
            f"min_forward_joystick={self.min_forward_joystick}, "
            f"min_backward_joystick={self.min_backward_joystick}"
        )
        self.get_logger().info(
            f"stepped turning: deadband={self.angular_cmd_deadband}, "
            f"slow<{self.slow_turn_angular_threshold} -> {self.slow_turn_joystick}, "
            f"medium<{self.medium_turn_angular_threshold} -> {self.medium_turn_joystick}, "
            f"large -> {self.pure_rotation_joystick}"
        )
        self.get_logger().info(
            f"mixed_turn_max_joystick={self.mixed_turn_max_joystick}"
        )
        self.get_logger().info(
            f"invert_x={self.invert_x}, invert_y={self.invert_y}"
        )
        self.get_logger().info(
            f"require_accurate_amcl={self.require_accurate_amcl}"
        )
        self.get_logger().info(
            "AMCL covariance limits: "
            f"x={self.max_x_covariance}, "
            f"y={self.max_y_covariance}, "
            f"yaw={self.max_yaw_covariance}"
        )
        self.get_logger().info(
            "AMCL equivalent standard deviation limits: "
            f"x={math.sqrt(self.max_x_covariance):.3f} m, "
            f"y={math.sqrt(self.max_y_covariance):.3f} m, "
            f"yaw={math.degrees(math.sqrt(self.max_yaw_covariance)):.1f} deg"
        )
        self.get_logger().info(
            "AMCL startup command: "
            f"[{self.amcl_block_joystick_x}, {self.amcl_block_joystick_y}]"
        )
        self.get_logger().info(
            f"Obstacle gate enabled={self.use_obstacle_gate}, "
            f"scan_topic={self.scan_topic}, "
            f"full_scan_stop_distance_m={self.full_scan_stop_distance_m}, "
            f"scan_qos=BEST_EFFORT"
        )

    def clamp(self, value, min_value=-100, max_value=100):
        return max(min_value, min(max_value, value))

    def apply_deadzone_percent(self, value):
        if abs(value) < self.deadzone_percent:
            return 0
        return value

    def scale_linear_to_y(self, linear_x):
        if self.max_linear_speed <= 0.0:
            return 0

        scaled = int((linear_x / self.max_linear_speed) * 100.0)
        scaled = self.clamp(scaled, -self.max_joystick_y, self.max_joystick_y)
        scaled = self.apply_deadzone_percent(scaled)

        if scaled > 0 and abs(scaled) < self.min_forward_joystick:
            scaled = self.min_forward_joystick
        elif scaled < 0 and abs(scaled) < self.min_backward_joystick:
            scaled = -self.min_backward_joystick

        return int(self.clamp(scaled, -self.max_joystick_y, self.max_joystick_y))

    def scale_angular_to_x_continuous(self, angular_z, x_limit):
        if self.max_angular_speed <= 0.0:
            return 0

        scaled = int((angular_z / self.max_angular_speed) * 100.0)
        scaled = self.clamp(scaled, -x_limit, x_limit)
        scaled = self.apply_deadzone_percent(scaled)
        return int(self.clamp(scaled, -x_limit, x_limit))

    def stepped_rotation_x(self, angular_z):
        """
        Converts pure rotation angular.z to stepped joystick X.
        This avoids overshoot caused by instantly forcing every turn to +-100.
        """
        abs_w = abs(angular_z)

        if abs_w < self.angular_cmd_deadband:
            return 0

        if abs_w < self.slow_turn_angular_threshold:
            x = abs(self.slow_turn_joystick)
        elif abs_w < self.medium_turn_angular_threshold:
            x = abs(self.medium_turn_joystick)
        else:
            x = abs(self.pure_rotation_joystick)

        x = int(self.clamp(x, 0, self.max_joystick_x))
        return x if angular_z > 0.0 else -x

    def cmd_vel_callback(self, msg: Twist):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        joystick_x = 0
        joystick_y = 0

        linear_active = abs(linear_x) >= self.linear_cmd_deadband
        angular_active = abs(angular_z) >= self.angular_cmd_deadband

        if not linear_active and not angular_active:
            joystick_x = 0
            joystick_y = 0

        elif not linear_active and angular_active:
            # Pure turn. New stepped logic: small command -> small turn,
            # medium command -> medium turn, large command -> full turn.
            joystick_x = self.stepped_rotation_x(angular_z)
            joystick_y = 0

        elif linear_active and not angular_active:
            # Straight forward/backward.
            joystick_x = 0
            joystick_y = self.scale_linear_to_y(linear_x)

        else:
            # Mixed command. Keep Y active, cap X.
            # If angular is very large, rotate in place instead of sending [100, small Y].
            if abs(angular_z) >= self.pure_rotation_angular_threshold:
                joystick_x = self.stepped_rotation_x(angular_z)
                joystick_y = 0
            else:
                joystick_x = self.scale_angular_to_x_continuous(
                    angular_z,
                    abs(self.mixed_turn_max_joystick),
                )
                joystick_y = self.scale_linear_to_y(linear_x)

        if self.invert_x:
            joystick_x = -joystick_x

        if self.invert_y:
            joystick_y = -joystick_y

        self.current_x = int(self.clamp(joystick_x, -100, 100))
        self.current_y = int(self.clamp(joystick_y, -100, 100))

        self.has_received_cmd_vel = True
        self.last_cmd_time = self.get_clock().now()

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.amcl_pose_estimation_done:
            return

        covariance = msg.pose.covariance

        x_cov = covariance[0]
        y_cov = covariance[7]
        yaw_cov = covariance[35]

        pose_good = (
            x_cov <= self.max_x_covariance
            and y_cov <= self.max_y_covariance
            and yaw_cov <= self.max_yaw_covariance
        )

        if pose_good:
            self.good_amcl_count += 1
        else:
            self.good_amcl_count = 0

        self.amcl_is_accurate = self.good_amcl_count >= self.min_good_amcl_messages
        self.has_received_amcl = True
        self.last_amcl_time = self.get_clock().now()

        if self.amcl_is_accurate:
            self.amcl_pose_estimation_done = True
            self.get_logger().info(
                "AMCL pose estimation accepted once. "
                "Normal Nav2 movement is now unlocked, "
                "but LiDAR emergency stop remains active."
            )
        else:
            self.get_logger().warn(
                f"Waiting for accurate AMCL pose: "
                f"x_cov={x_cov:.4f}, "
                f"y_cov={y_cov:.4f}, "
                f"yaw_cov={yaw_cov:.4f}, "
                f"good_count={self.good_amcl_count}/{self.min_good_amcl_messages}"
            )

    def scan_callback(self, msg: LaserScan):
        closest_distance = float("inf")

        for distance in msg.ranges:
            if math.isinf(distance) or math.isnan(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            closest_distance = min(closest_distance, distance)

        self.closest_scan_distance = closest_distance
        self.obstacle_too_close = closest_distance < self.full_scan_stop_distance_m

        self.has_received_scan = True
        self.last_scan_time = self.get_clock().now()

        if self.obstacle_too_close:
            self.get_logger().warn(
                f"Obstacle too close somewhere in full scan: {closest_distance:.2f} m."
            )

    def publish_joystick(self, x, y):
        x = int(self.clamp(x, -100, 100))
        y = int(self.clamp(y, -100, 100))

        msg = Int16MultiArray()
        msg.data = [x, y]
        self.publisher.publish(msg)

    def amcl_allows_movement(self):
        if not self.require_accurate_amcl:
            return True

        if self.amcl_pose_estimation_done:
            return True

        if not self.has_received_amcl:
            self.get_logger().warn(
                "No /amcl_pose received yet. Blocking normal Nav2 movement."
            )
            return False

        now = self.get_clock().now()
        amcl_age = (now - self.last_amcl_time).nanoseconds / 1e9

        if amcl_age > self.amcl_timeout_seconds:
            self.amcl_is_accurate = False
            self.good_amcl_count = 0
            self.get_logger().warn(
                f"/amcl_pose timeout: age={amcl_age:.2f}s. Blocking normal movement."
            )
            return False

        return self.amcl_is_accurate

    def obstacle_gate_allows_movement(self):
        if not self.use_obstacle_gate:
            return True

        if not self.has_received_scan:
            self.get_logger().warn(
                "No /scan received yet. Emergency stop active."
            )
            return False

        now = self.get_clock().now()
        scan_age = (now - self.last_scan_time).nanoseconds / 1e9

        if scan_age > self.scan_timeout_seconds:
            self.get_logger().warn(
                f"/scan timeout: age={scan_age:.2f}s. Emergency stop active."
            )
            return False

        if self.obstacle_too_close:
            return False

        return True

    def cmd_vel_is_fresh(self):
        if not self.has_received_cmd_vel:
            return False

        now = self.get_clock().now()
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        return cmd_age <= self.timeout_seconds

    def timer_callback(self):
        # LiDAR emergency stop has priority before and after AMCL.
        if not self.obstacle_gate_allows_movement():
            self.publish_joystick(0, 0)
            return

        # Before AMCL is accepted, do localization rotation.
        if not self.amcl_allows_movement():
            self.publish_joystick(
                self.amcl_block_joystick_x,
                self.amcl_block_joystick_y,
            )
            return

        # After AMCL is accepted, only use fresh /cmd_vel.
        if not self.cmd_vel_is_fresh():
            if self.send_nothing_without_cmd_vel_publisher:
                return
            self.publish_joystick(0, 0)
            return

        self.publish_joystick(self.current_x, self.current_y)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoystick()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_joystick(0, 0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
