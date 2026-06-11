#!/usr/bin/env python3

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class PidController:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)
        self.integral = 0.0
        self.previous_error: Optional[float] = None

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0 or dt > 1.0:
            return 0.0

        self.integral += error * dt
        self.integral = self.clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return self.clamp(output, -self.output_limit, self.output_limit)


class CmdVelToJoystickPid(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_joystick_pid_guarded")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joystick_topic", "/joystick_cmd")
        self.declare_parameter("calibration_file", "joystick_calibration.json")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("timeout_seconds", 0.5)
        self.declare_parameter("odom_timeout_seconds", 0.5)
        self.declare_parameter("send_nothing_without_cmd_vel_publisher", True)
        self.declare_parameter("require_fresh_odom", True)

        self.declare_parameter("max_joystick_x", 100)
        self.declare_parameter("max_joystick_y", 100)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("linear_cmd_deadband", 0.01)
        self.declare_parameter("angular_cmd_deadband", 0.01)
        self.declare_parameter("measured_stop_linear_deadband", 0.03)
        self.declare_parameter("measured_stop_angular_deadband", 0.03)

        self.declare_parameter("measured_velocity_filter_alpha", 0.35)

        self.declare_parameter("linear_kp", 25.0)
        self.declare_parameter("linear_ki", 3.0)
        self.declare_parameter("linear_kd", 0.0)
        self.declare_parameter("linear_integral_limit", 1.0)
        self.declare_parameter("linear_pid_output_limit", 15.0)

        self.declare_parameter("angular_kp", 25.0)
        self.declare_parameter("angular_ki", 3.0)
        self.declare_parameter("angular_kd", 0.0)
        self.declare_parameter("angular_integral_limit", 1.0)
        self.declare_parameter("angular_pid_output_limit", 15.0)

        self.declare_parameter("max_joystick_x_delta_per_s", 80.0)
        self.declare_parameter("max_joystick_y_delta_per_s", 80.0)

        self.declare_parameter("fallback_max_linear_speed", 1.39)
        self.declare_parameter("fallback_max_angular_speed", 0.42)
        self.declare_parameter("fallback_min_forward_joystick", 25)
        self.declare_parameter("fallback_min_backward_joystick", 40)
        self.declare_parameter("fallback_min_turn_joystick", 25)

        self.declare_parameter("require_accurate_amcl", True)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("max_x_covariance", 0.04)
        self.declare_parameter("max_y_covariance", 0.04)
        self.declare_parameter("max_yaw_covariance", 0.03)
        self.declare_parameter("min_good_amcl_messages", 5)
        self.declare_parameter("amcl_timeout_seconds", 10.0)
        self.declare_parameter("amcl_block_joystick_x", 40)
        self.declare_parameter("amcl_block_joystick_y", 0)

        self.declare_parameter("use_obstacle_gate", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("full_scan_stop_distance_m", 0.50)
        self.declare_parameter("scan_timeout_seconds", 0.5)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.joystick_topic = self.get_parameter("joystick_topic").value
        self.calibration_file = self.get_parameter("calibration_file").value

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timeout_seconds = float(self.get_parameter("timeout_seconds").value)
        self.odom_timeout_seconds = float(self.get_parameter("odom_timeout_seconds").value)
        self.send_nothing_without_cmd_vel_publisher = bool(
            self.get_parameter("send_nothing_without_cmd_vel_publisher").value
        )
        self.require_fresh_odom = bool(self.get_parameter("require_fresh_odom").value)

        self.max_joystick_x = int(self.get_parameter("max_joystick_x").value)
        self.max_joystick_y = int(self.get_parameter("max_joystick_y").value)
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.linear_cmd_deadband = float(self.get_parameter("linear_cmd_deadband").value)
        self.angular_cmd_deadband = float(self.get_parameter("angular_cmd_deadband").value)
        self.measured_stop_linear_deadband = float(
            self.get_parameter("measured_stop_linear_deadband").value
        )
        self.measured_stop_angular_deadband = float(
            self.get_parameter("measured_stop_angular_deadband").value
        )
        self.measured_velocity_filter_alpha = float(
            self.get_parameter("measured_velocity_filter_alpha").value
        )

        self.max_joystick_x_delta_per_s = float(
            self.get_parameter("max_joystick_x_delta_per_s").value
        )
        self.max_joystick_y_delta_per_s = float(
            self.get_parameter("max_joystick_y_delta_per_s").value
        )

        self.fallback_max_linear_speed = float(self.get_parameter("fallback_max_linear_speed").value)
        self.fallback_max_angular_speed = float(self.get_parameter("fallback_max_angular_speed").value)
        self.fallback_min_forward_joystick = int(
            self.get_parameter("fallback_min_forward_joystick").value
        )
        self.fallback_min_backward_joystick = int(
            self.get_parameter("fallback_min_backward_joystick").value
        )
        self.fallback_min_turn_joystick = int(
            self.get_parameter("fallback_min_turn_joystick").value
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
        self.full_scan_stop_distance_m = float(self.get_parameter("full_scan_stop_distance_m").value)
        self.scan_timeout_seconds = float(self.get_parameter("scan_timeout_seconds").value)

        self.linear_pid = PidController(
            float(self.get_parameter("linear_kp").value),
            float(self.get_parameter("linear_ki").value),
            float(self.get_parameter("linear_kd").value),
            float(self.get_parameter("linear_integral_limit").value),
            float(self.get_parameter("linear_pid_output_limit").value),
        )

        self.angular_pid = PidController(
            float(self.get_parameter("angular_kp").value),
            float(self.get_parameter("angular_ki").value),
            float(self.get_parameter("angular_kd").value),
            float(self.get_parameter("angular_integral_limit").value),
            float(self.get_parameter("angular_pid_output_limit").value),
        )

        self.calibration = self.load_calibration(self.calibration_file)

        self.desired_v = 0.0
        self.desired_w = 0.0
        self.has_received_cmd_vel = False
        self.last_cmd_time = self.get_clock().now()

        self.measured_v = 0.0
        self.measured_w = 0.0
        self.filtered_v = 0.0
        self.filtered_w = 0.0
        self.has_received_odom = False
        self.last_odom_time = self.get_clock().now()
        self.last_good_odom_time = self.get_clock().now()

        self.current_x = 0.0
        self.current_y = 0.0
        self.last_control_time = self.get_clock().now()

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

        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.amcl_sub = self.create_subscription(
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
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.get_logger().info("cmd_vel_to_joystick_pid_guarded started")
        self.get_logger().info(f"cmd_vel_topic={self.cmd_vel_topic}")
        self.get_logger().info(f"odom_topic={self.odom_topic}")
        self.get_logger().info(f"joystick_topic={self.joystick_topic}")
        self.get_logger().info(f"calibration_file={self.calibration_file}")

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def sign(value: float) -> float:
        if value > 0.0:
            return 1.0
        if value < 0.0:
            return -1.0
        return 0.0

    def load_calibration(self, path: str) -> dict:
        try:
            with open(path, "r") as handle:
                data = json.load(handle)
            self.get_logger().info(f"Loaded calibration from {path}")
            return data
        except Exception as error:
            self.get_logger().warn(f"Could not load calibration file {path}: {error}")
            self.get_logger().warn("Using fallback linear mapping until calibration is available")
            return {}

    def calibration_axis_to_command(
        self,
        desired_value: float,
        positive_key: str,
        negative_key: str,
        max_command: int,
        fallback_max_speed: float,
        fallback_min_command_positive: int,
        fallback_min_command_negative: int,
    ) -> float:
        if desired_value == 0.0:
            return 0.0

        key = positive_key if desired_value > 0.0 else negative_key
        data = self.calibration.get(key, {})
        desired_abs = abs(desired_value)
        direction = self.sign(desired_value)

        if data.get("valid", False):
            slope = float(data.get("slope", 0.0))
            intercept = float(data.get("intercept", 0.0))
            min_output = float(data.get("min_output_command", 0.0))
            max_output = min(float(data.get("max_output_command", max_command)), float(max_command))

            if slope > 0.0:
                command_abs = (desired_abs - intercept) / slope
                command_abs = self.clamp(command_abs, min_output, max_output)
                return direction * command_abs

        if fallback_max_speed <= 0.0:
            return 0.0

        command_abs = desired_abs / fallback_max_speed * float(max_command)
        min_command = fallback_min_command_positive if desired_value > 0.0 else fallback_min_command_negative
        command_abs = self.clamp(command_abs, float(min_command), float(max_command))
        return direction * command_abs

    def feedforward_y(self, desired_v: float) -> float:
        if abs(desired_v) < self.linear_cmd_deadband:
            return 0.0

        return self.calibration_axis_to_command(
            desired_v,
            "linear_positive",
            "linear_negative",
            self.max_joystick_y,
            self.fallback_max_linear_speed,
            self.fallback_min_forward_joystick,
            self.fallback_min_backward_joystick,
        )

    def feedforward_x(self, desired_w: float) -> float:
        if abs(desired_w) < self.angular_cmd_deadband:
            return 0.0

        return self.calibration_axis_to_command(
            desired_w,
            "angular_positive",
            "angular_negative",
            self.max_joystick_x,
            self.fallback_max_angular_speed,
            self.fallback_min_turn_joystick,
            self.fallback_min_turn_joystick,
        )

    def cmd_vel_callback(self, msg: Twist):
        self.desired_v = float(msg.linear.x)
        self.desired_w = float(msg.angular.z)
        self.has_received_cmd_vel = True
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        measured_v = float(msg.twist.twist.linear.x)
        measured_w = float(msg.twist.twist.angular.z)

        self.measured_v = measured_v
        self.measured_w = measured_w

        alpha = self.clamp(self.measured_velocity_filter_alpha, 0.0, 1.0)
        if not self.has_received_odom:
            self.filtered_v = self.measured_v
            self.filtered_w = self.measured_w
        else:
            self.filtered_v = alpha * self.measured_v + (1.0 - alpha) * self.filtered_v
            self.filtered_w = alpha * self.measured_w + (1.0 - alpha) * self.filtered_w

        self.has_received_odom = True
        self.last_odom_time = now
        self.last_good_odom_time = now

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
            self.get_logger().info("AMCL pose accepted. Normal movement unlocked.")
        else:
            self.get_logger().warn(
                f"Waiting for accurate AMCL pose: "
                f"x_cov={x_cov:.4f}, y_cov={y_cov:.4f}, yaw_cov={yaw_cov:.4f}, "
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
            self.get_logger().warn(f"Obstacle too close in full scan: {closest_distance:.2f} m")

    def publish_joystick(self, x: float, y: float):
        x = int(round(self.clamp(x, -100.0, 100.0)))
        y = int(round(self.clamp(y, -100.0, 100.0)))

        msg = Int16MultiArray()
        msg.data = [x, y]
        self.publisher.publish(msg)

    def cmd_vel_is_fresh(self) -> bool:
        if not self.has_received_cmd_vel:
            return False
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        return age <= self.timeout_seconds

    def odom_is_fresh(self) -> bool:
        if not self.has_received_odom:
            return False
        now = self.get_clock().now()
        age = (now - self.last_good_odom_time).nanoseconds / 1e9
        return age <= self.odom_timeout_seconds

    def amcl_allows_movement(self) -> bool:
        if not self.require_accurate_amcl:
            return True
        if self.amcl_pose_estimation_done:
            return True
        if not self.has_received_amcl:
            self.get_logger().warn("No /amcl_pose received yet. Blocking normal movement.")
            return False

        now = self.get_clock().now()
        age = (now - self.last_amcl_time).nanoseconds / 1e9
        if age > self.amcl_timeout_seconds:
            self.amcl_is_accurate = False
            self.good_amcl_count = 0
            self.get_logger().warn(f"/amcl_pose timeout: age={age:.2f}s. Blocking movement.")
            return False
        return self.amcl_is_accurate

    def obstacle_gate_allows_movement(self) -> bool:
        if not self.use_obstacle_gate:
            return True

        # Only use the simple full-scan joystick stop while AMCL is still being accepted.
        # After AMCL is accepted, obstacle avoidance is handled by Nav2/local_costmap.
        if self.amcl_pose_estimation_done:
            return True

        # No scan-timeout emergency stop here. If no scan has arrived yet, do not block
        # the joystick bridge only because of missing/stale scan data.
        if not self.has_received_scan:
            return True

        return not self.obstacle_too_close

    def rate_limit(self, target: float, current: float, max_delta_per_s: float, dt: float) -> float:
        allowed_delta = abs(max_delta_per_s) * max(0.0, dt)
        delta = self.clamp(target - current, -allowed_delta, allowed_delta)
        return current + delta

    def reset_control(self):
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.current_x = 0.0
        self.current_y = 0.0
        self.publish_joystick(0, 0)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now

        if not self.obstacle_gate_allows_movement():
            self.reset_control()
            return

        if not self.amcl_allows_movement():
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.current_x = self.rate_limit(
                self.amcl_block_joystick_x,
                self.current_x,
                self.max_joystick_x_delta_per_s,
                dt,
            )
            self.current_y = self.rate_limit(
                self.amcl_block_joystick_y,
                self.current_y,
                self.max_joystick_y_delta_per_s,
                dt,
            )
            self.publish_joystick(self.current_x, self.current_y)
            return

        if not self.cmd_vel_is_fresh():
            if self.send_nothing_without_cmd_vel_publisher:
                self.linear_pid.reset()
                self.angular_pid.reset()
                return
            self.reset_control()
            return

        if self.require_fresh_odom and not self.odom_is_fresh():
            self.get_logger().warn("Fresh /odom is required but unavailable. Stopping.")
            self.reset_control()
            return

        desired_v = self.desired_v
        desired_w = self.desired_w

        if abs(desired_v) < self.linear_cmd_deadband:
            desired_v = 0.0
        if abs(desired_w) < self.angular_cmd_deadband:
            desired_w = 0.0

        if desired_v == 0.0 and desired_w == 0.0:
            self.reset_control()
            return

        ff_y = self.feedforward_y(desired_v)
        ff_x = self.feedforward_x(desired_w)

        error_v = desired_v - self.filtered_v
        error_w = desired_w - self.filtered_w

        pid_y = self.linear_pid.update(error_v, dt) if desired_v != 0.0 else 0.0
        pid_x = self.angular_pid.update(error_w, dt) if desired_w != 0.0 else 0.0

        target_y = ff_y + pid_y
        target_x = ff_x + pid_x

        target_y = self.clamp(target_y, -float(self.max_joystick_y), float(self.max_joystick_y))
        target_x = self.clamp(target_x, -float(self.max_joystick_x), float(self.max_joystick_x))

        if self.invert_y:
            target_y = -target_y
        if self.invert_x:
            target_x = -target_x

        self.current_y = self.rate_limit(target_y, self.current_y, self.max_joystick_y_delta_per_s, dt)
        self.current_x = self.rate_limit(target_x, self.current_x, self.max_joystick_x_delta_per_s, dt)

        self.publish_joystick(self.current_x, self.current_y)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoystickPid()

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
