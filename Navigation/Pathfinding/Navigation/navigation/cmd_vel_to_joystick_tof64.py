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
        super().__init__("cmd_vel_to_joystick_tof64")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joystick_topic", "/joystick_cmd")

        # IMPORTANT:
        # These are NOT the Nav2 maximum speeds.
        # These are the wheelchair speed represented by joystick 100.
        # UPDATED: 6 km/h = 1.67 m/s, and angular updated to 1.1
        self.declare_parameter("max_linear_speed", 1.67)
        self.declare_parameter("max_angular_speed", 1.1)

        # Set these if the wheelchair drives/turns opposite to the command.
        # CHANGED TO TRUE: Fixes the inverted clockwise map rotation issue
        self.declare_parameter("invert_x", True)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("deadzone_percent", 3)

        # Minimum joystick magnitude for any non-zero Nav2 command.
        # 0 is still allowed for stop, but movement commands become at least +/-52
        # because the wheelchair does not move below this value.
        self.declare_parameter("minimum_nonzero_joystick", 52)

        # Extra safety caps for autonomous mode.
        # X = turning joystick, Y = forward/backward joystick.
        # UPDATED: Expanded to 100 to allow the wheelchair full dynamic hardware range
        self.declare_parameter("max_joystick_x", 100)
        self.declare_parameter("max_joystick_y", 100)

        self.declare_parameter("timeout_seconds", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        # If True:
        # - no /cmd_vel publisher -> publish nothing
        # - /cmd_vel publisher exists but messages stop -> publish neutral [0, 0]
        self.declare_parameter("send_nothing_without_cmd_vel_publisher", True)

        # AMCL safety gate.
        # If require_accurate_amcl is True, normal Nav2 movement is blocked
        # until /amcl_pose covariance is low enough once.
        self.declare_parameter("require_accurate_amcl", True)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        # Covariance limits.
        # 0.04 means std dev = sqrt(0.04) = 0.20 m.
        self.declare_parameter("max_x_covariance", 0.04)
        self.declare_parameter("max_y_covariance", 0.04)

        # 0.03 means std dev = sqrt(0.03) = 0.173 rad = about 10 degrees.
        self.declare_parameter("max_yaw_covariance", 0.03)

        # Number of good AMCL messages needed before normal movement is allowed.
        self.declare_parameter("min_good_amcl_messages", 5)

        # If AMCL is older than this before first accepted pose, normal movement is blocked.
        self.declare_parameter("amcl_timeout_seconds", 10.0)

        # Command sent while AMCL is uncertain.
        # Requested behavior: while AMCL is still checking, rotate with [100, 0].
        # This command is only sent when the obstacle gate says the scan is clear.
        # It is NOT limited by max_joystick_x/max_joystick_y because those caps
        # are only for normal Nav2 path-following commands.
        self.declare_parameter("amcl_block_joystick_x", 100)
        self.declare_parameter("amcl_block_joystick_y", 0)

        # Obstacle safety gate.
        # This checks the full LaserScan while AMCL is uncertain.
        self.declare_parameter("use_obstacle_gate", True)
        self.declare_parameter("scan_topic", "/scan_multi")
        self.declare_parameter("full_scan_stop_distance_m", 0.45)
        self.declare_parameter("scan_timeout_seconds", 0.5)

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
        self.minimum_nonzero_joystick = int(
            self.get_parameter("minimum_nonzero_joystick").value
        )
        self.max_joystick_x = int(self.get_parameter("max_joystick_x").value)
        self.max_joystick_y = int(self.get_parameter("max_joystick_y").value)

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

        self.require_accurate_amcl = bool(
            self.get_parameter("require_accurate_amcl").value
        )
        self.amcl_pose_topic = self.get_parameter("amcl_pose_topic").value

        self.max_x_covariance = float(
            self.get_parameter("max_x_covariance").value
        )
        self.max_y_covariance = float(
            self.get_parameter("max_y_covariance").value
        )
        self.max_yaw_covariance = float(
            self.get_parameter("max_yaw_covariance").value
        )

        self.min_good_amcl_messages = int(
            self.get_parameter("min_good_amcl_messages").value
        )
        self.amcl_timeout_seconds = float(
            self.get_parameter("amcl_timeout_seconds").value
        )

        self.amcl_block_joystick_x = int(
            self.get_parameter("amcl_block_joystick_x").value
        )
        self.amcl_block_joystick_y = int(
            self.get_parameter("amcl_block_joystick_y").value
        )

        self.use_obstacle_gate = bool(
            self.get_parameter("use_obstacle_gate").value
        )
        self.scan_topic = self.get_parameter("scan_topic").value
        self.full_scan_stop_distance_m = float(
            self.get_parameter("full_scan_stop_distance_m").value
        )
        self.scan_timeout_seconds = float(
            self.get_parameter("scan_timeout_seconds").value
        )

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

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_pose_callback,
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

        self.get_logger().info("cmd_vel_to_joystick started")
        self.get_logger().info(f"Subscribing to cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"Publishing joystick to: {self.joystick_topic}")
        self.get_logger().info(f"max_linear_speed={self.max_linear_speed}")
        self.get_logger().info(f"max_angular_speed={self.max_angular_speed}")
        self.get_logger().info(
            f"minimum_nonzero_joystick={self.minimum_nonzero_joystick}"
        )
        self.get_logger().info(
            f"max_joystick_x={self.max_joystick_x}, max_joystick_y={self.max_joystick_y}"
        )
        self.get_logger().info(
            f"invert_x={self.invert_x}, invert_y={self.invert_y}"
        )
        self.get_logger().info(
            f"require_accurate_amcl={self.require_accurate_amcl}"
        )
        self.get_logger().info(f"Subscribing to AMCL: {self.amcl_pose_topic}")
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
            "AMCL uncertain command: "
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

        self.current_x = joystick_x
        self.current_y = joystick_y

        self.has_received_cmd_vel = True
        self.last_cmd_time = self.get_clock().now()

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        # Once AMCL has been good enough once, keep normal movement unlocked.
        # Do not keep re-checking AMCL covariance after that.
        if self.amcl_pose_estimation_done:
            return

        covariance = msg.pose.covariance

        # PoseWithCovariance covariance matrix:
        # x variance   = covariance[0]
        # y variance   = covariance[7]
        # yaw variance = covariance[35]
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

        self.amcl_is_accurate = (
            self.good_amcl_count >= self.min_good_amcl_messages
        )

        self.has_received_amcl = True
        self.last_amcl_time = self.get_clock().now()

        if self.amcl_is_accurate:
            self.amcl_pose_estimation_done = True
            self.get_logger().info(
                "AMCL pose estimation accepted once. "
                "Normal movement is now permanently unlocked for this run."
            )
        else:
            self.get_logger().warn(
                f"Waiting for accurate AMCL pose: "
                f"x_cov={x_cov:.4f}, "
                f"y_cov={y_cov:.4f}, "
                f"yaw_cov={yaw_cov:.4f}, "
                f"good_count={self.good_amcl_count}/"
                f"{self.min_good_amcl_messages}"
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
        self.obstacle_too_close = (
            closest_distance < self.full_scan_stop_distance_m
        )

        self.has_received_scan = True
        self.last_scan_time = self.get_clock().now()

        if self.obstacle_too_close:
            self.get_logger().warn(
                f"Obstacle too close somewhere in full scan: "
                f"{closest_distance:.2f} m. Blocking AMCL recovery movement."
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
                "No recent /amcl_pose received. Blocking normal Nav2 movement."
            )
            return False

        if not self.amcl_is_accurate:
            self.get_logger().warn(
                "AMCL pose is not accurate yet. Blocking normal Nav2 movement."
            )
            return False

        return True

    def obstacle_allows_recovery_movement(self):
        if not self.use_obstacle_gate:
            return True

        if not self.has_received_scan:
            self.get_logger().warn(
                "No /scan received yet. Blocking AMCL recovery movement."
            )
            return False

        now = self.get_clock().now()
        scan_age = (now - self.last_scan_time).nanoseconds / 1e9

        if scan_age > self.scan_timeout_seconds:
            self.get_logger().warn(
                "No recent /scan received. Blocking AMCL recovery movement."
            )
            return False

        if self.obstacle_too_close:
            self.get_logger().warn(
                f"Obstacle gate blocked AMCL recovery movement. "
                f"Closest scan point: {self.closest_scan_distance:.2f} m"
            )
            return False

        return True

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

        # AMCL safety gate.
        # If AMCL is accurate once, allow normal Nav2 /cmd_vel -> /joystick_cmd.
        # If AMCL is uncertain, perform only a slow recovery turn if scan is clear.
        if not self.amcl_allows_movement():
            if self.obstacle_allows_recovery_movement():
                # AMCL recovery/checking command is intentionally allowed to use
                # the full joystick range. Normal Nav2 commands are still capped
                # separately in scale_to_joystick().
                recovery_x = int(self.clamp(self.amcl_block_joystick_x, -100, 100))
                recovery_y = int(self.clamp(self.amcl_block_joystick_y, -100, 100))

                self.current_x = recovery_x
                self.current_y = recovery_y
                self.get_logger().warn(
                    f"AMCL CHECKING: publishing joystick [{recovery_x}, {recovery_y}]"
                )
                self.publish_joystick(recovery_x, recovery_y)
            else:
                self.current_x = 0
                self.current_y = 0
                self.publish_joystick(0, 0)

            return

        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9

        # If /cmd_vel publisher exists, but commands stop, send neutral for safety.
        if (not self.has_received_cmd_vel) or age > self.timeout_seconds:
            self.publish_joystick(0, 0)
            return

        self.publish_joystick(self.current_x, self.current_y)

    def destroy_node(self):
        # Do NOT publish [0, 0] during shutdown.
        # During launch shutdown this could still be forwarded by
        # arduino_sensor_node as serial J,0,0.
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