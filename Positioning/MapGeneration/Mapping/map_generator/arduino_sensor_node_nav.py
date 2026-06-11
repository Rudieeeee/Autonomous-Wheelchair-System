#!/usr/bin/env python3

import math
import threading
import time
from collections import deque
from typing import Optional

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray
from tf2_ros import TransformBroadcaster


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Quaternion:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    return euler_to_quaternion(0.0, 0.0, yaw_rad)


class ArduinoSensorNodeNav(Node):
    def __init__(self):
        super().__init__("arduino_sensor_node_nav")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timer_period_s", 0.01)
        self.declare_parameter("clear_serial_buffers_on_start", True)
        self.declare_parameter("serial_startup_delay_s", 2.0)
        self.declare_parameter("startup_skip_lines", 20)

        self.declare_parameter("wheel_diameter_m", 0.35)
        self.declare_parameter("magnets_per_wheel", 12)
        self.declare_parameter("wheel_base_m", 0.55)
        self.declare_parameter("left_tick_sign", 1.0)
        self.declare_parameter("right_tick_sign", 1.0)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("use_imu_yaw", True)

        self.declare_parameter("joystick_cmd_topic", "/joystick_cmd")
        self.declare_parameter("enable_joystick_serial_output", True)

        self.declare_parameter("enable_encoder_sanity_filter", True)
        self.declare_parameter("one_wheel_fallback_enabled", True)
        self.declare_parameter("use_imu_yaw_rate_for_angular_velocity", True)
        self.declare_parameter("max_delta_ticks_per_update", 8)
        self.declare_parameter("max_single_wheel_speed_mps", 2.2)
        self.declare_parameter("max_single_wheel_accel_mps2", 12.0)
        self.declare_parameter("max_wheel_imu_angular_error_radps", 1.2)
        self.declare_parameter("velocity_window_s", 0.30)
        self.declare_parameter("velocity_filter_alpha", 0.35)

        self.declare_parameter("use_imu_accel_fallback", False)
        self.declare_parameter("imu_accel_x_bias_mps2", 0.0)
        self.declare_parameter("imu_accel_x_sign", 1.0)
        self.declare_parameter("max_accel_fallback_speed_mps", 0.8)
        self.declare_parameter("max_imu_accel_fallback_mps2", 1.5)

        self.declare_parameter("debug_enabled", False)
        self.declare_parameter("debug_raw_serial", False)
        self.declare_parameter("debug_rejected_ticks", True)
        self.declare_parameter("debug_summary_period_s", 2.0)

        self.serial_port_name = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.timer_period_s = float(self.get_parameter("timer_period_s").value)
        self.clear_serial_buffers_on_start = bool(self.get_parameter("clear_serial_buffers_on_start").value)
        self.serial_startup_delay_s = float(self.get_parameter("serial_startup_delay_s").value)
        self.startup_skip_lines = int(self.get_parameter("startup_skip_lines").value)

        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.magnets_per_wheel = int(self.get_parameter("magnets_per_wheel").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.left_tick_sign = float(self.get_parameter("left_tick_sign").value)
        self.right_tick_sign = float(self.get_parameter("right_tick_sign").value)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_imu_enabled = bool(self.get_parameter("publish_imu").value)
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)

        self.joystick_cmd_topic = self.get_parameter("joystick_cmd_topic").value
        self.enable_joystick_serial_output = bool(self.get_parameter("enable_joystick_serial_output").value)

        self.enable_encoder_sanity_filter = bool(self.get_parameter("enable_encoder_sanity_filter").value)
        self.one_wheel_fallback_enabled = bool(self.get_parameter("one_wheel_fallback_enabled").value)
        self.use_imu_yaw_rate_for_angular_velocity = bool(
            self.get_parameter("use_imu_yaw_rate_for_angular_velocity").value
        )
        self.max_delta_ticks_per_update = int(self.get_parameter("max_delta_ticks_per_update").value)
        self.max_single_wheel_speed_mps = float(self.get_parameter("max_single_wheel_speed_mps").value)
        self.max_single_wheel_accel_mps2 = float(self.get_parameter("max_single_wheel_accel_mps2").value)
        self.max_wheel_imu_angular_error_radps = float(
            self.get_parameter("max_wheel_imu_angular_error_radps").value
        )
        self.velocity_window_s = float(self.get_parameter("velocity_window_s").value)
        self.velocity_filter_alpha = float(self.get_parameter("velocity_filter_alpha").value)

        self.use_imu_accel_fallback = bool(self.get_parameter("use_imu_accel_fallback").value)
        self.imu_accel_x_bias_mps2 = float(self.get_parameter("imu_accel_x_bias_mps2").value)
        self.imu_accel_x_sign = float(self.get_parameter("imu_accel_x_sign").value)
        self.max_accel_fallback_speed_mps = float(self.get_parameter("max_accel_fallback_speed_mps").value)
        self.max_imu_accel_fallback_mps2 = float(self.get_parameter("max_imu_accel_fallback_mps2").value)

        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_raw_serial = bool(self.get_parameter("debug_raw_serial").value)
        self.debug_rejected_ticks = bool(self.get_parameter("debug_rejected_ticks").value)
        self.debug_summary_period_s = float(self.get_parameter("debug_summary_period_s").value)

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.distance_per_tick_m = self.wheel_circumference_m / float(self.magnets_per_wheel)

        self.serial_lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0
        self.distance_total_m = 0.0
        self.previous_data = None
        self.initial_imu_yaw_rad: Optional[float] = None

        self.previous_left_speed_mps = 0.0
        self.previous_right_speed_mps = 0.0
        self.filtered_linear_velocity_mps = 0.0
        self.filtered_angular_velocity_radps = 0.0
        self.history = deque()

        self.last_joystick_x = 0
        self.last_joystick_y = 0
        self.last_summary_time = time.monotonic()
        self.valid_data_count = 0
        self.rejected_left_count = 0
        self.rejected_right_count = 0
        self.one_wheel_fallback_count = 0
        self.both_wheels_rejected_count = 0
        self.imu_accel_fallback_count = 0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joystick_sub = self.create_subscription(
            Int16MultiArray,
            self.joystick_cmd_topic,
            self.joystick_cmd_callback,
            10,
        )

        self.serial_port = serial.Serial(
            self.serial_port_name,
            self.baud_rate,
            timeout=0.01,
            write_timeout=0.01,
            exclusive=True,
        )

        if self.clear_serial_buffers_on_start:
            if self.serial_startup_delay_s > 0.0:
                time.sleep(self.serial_startup_delay_s)
            with self.serial_lock:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()

        self.send_protocol_line("START")
        self.timer = self.create_timer(self.timer_period_s, self.read_serial)

        self.get_logger().info(
            "arduino_sensor_node_nav started. "
            f"distance_per_tick_m={self.distance_per_tick_m:.4f}, "
            f"velocity_window_s={self.velocity_window_s:.2f}, "
            f"one_wheel_fallback_enabled={self.one_wheel_fallback_enabled}, "
            f"use_imu_accel_fallback={self.use_imu_accel_fallback}"
        )

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def send_protocol_line(self, line: str):
        if not hasattr(self, "serial_port") or not self.serial_port.is_open:
            return
        try:
            with self.serial_lock:
                self.serial_port.write((line + "\n").encode("ascii"))
                self.serial_port.flush()
        except serial.SerialException as error:
            self.get_logger().error(f"Serial protocol write error while sending {line!r}: {error}")

    def joystick_cmd_callback(self, msg: Int16MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(f"Invalid joystick command: expected [x, y], got {msg.data}")
            return

        x = int(self.clamp(int(msg.data[0]), -100, 100))
        y = int(self.clamp(int(msg.data[1]), -100, 100))
        self.last_joystick_x = x
        self.last_joystick_y = y

        if not self.enable_joystick_serial_output:
            return

        line = f"J,{x},{y}\n"
        try:
            with self.serial_lock:
                self.serial_port.write(line.encode("ascii"))
        except serial.SerialException as error:
            self.get_logger().error(f"Serial joystick write error: {error}")

    def parse_data_line(self, line: str):
        parts = line.split(",")
        if len(parts) not in (9, 12):
            self.get_logger().warn(
                "Invalid DATA line. Expected 9 fields or 12 fields when accel is added, "
                f"got {len(parts)} fields: {line!r}"
            )
            return None
        if parts[0] != "DATA":
            return None

        try:
            data = {
                "time_ms": int(parts[1]),
                "left_ticks": int(parts[2]),
                "right_ticks": int(parts[3]),
                "left_state": int(parts[4]),
                "right_state": int(parts[5]),
                "yaw_deg": float(parts[6]),
                "pitch_deg": float(parts[7]),
                "roll_deg": float(parts[8]),
                "has_accel": False,
                "accel_x_mps2": 0.0,
                "accel_y_mps2": 0.0,
                "accel_z_mps2": 0.0,
            }
            if len(parts) == 12:
                data["has_accel"] = True
                data["accel_x_mps2"] = float(parts[9])
                data["accel_y_mps2"] = float(parts[10])
                data["accel_z_mps2"] = float(parts[11])
            return data
        except ValueError as error:
            self.get_logger().warn(f"Could not parse DATA line {line!r}: {error}")
            return None

    def read_serial(self):
        max_lines_per_timer = 50
        lines_read = 0

        while lines_read < max_lines_per_timer:
            try:
                with self.serial_lock:
                    if self.serial_port.in_waiting <= 0:
                        break
                    raw_line = self.serial_port.readline()
            except serial.SerialException as error:
                self.get_logger().error(f"Serial read error: {error}")
                return

            if not raw_line:
                break

            lines_read += 1
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if self.debug_raw_serial:
                self.get_logger().info(f"RAW SERIAL: {line}")

            if self.startup_skip_lines > 0:
                self.startup_skip_lines -= 1
                continue
            if not line.startswith("DATA"):
                continue

            data = self.parse_data_line(line)
            if data is None:
                continue
            self.valid_data_count += 1
            self.update_odometry(data)

        self.debug_summary_if_needed()

    def imu_relative_yaw(self, data) -> float:
        imu_yaw_rad = math.radians(data["yaw_deg"])
        if self.initial_imu_yaw_rad is None:
            self.initial_imu_yaw_rad = imu_yaw_rad
        return normalize_angle(imu_yaw_rad - self.initial_imu_yaw_rad)

    def wheel_step_is_valid(
        self,
        name: str,
        delta_ticks: float,
        speed_mps: float,
        previous_speed_mps: float,
        dt: float,
    ) -> bool:
        if not self.enable_encoder_sanity_filter:
            return True

        reasons = []
        if abs(delta_ticks) > self.max_delta_ticks_per_update:
            reasons.append(f"tick jump {delta_ticks:.1f}")
        if abs(speed_mps) > self.max_single_wheel_speed_mps:
            reasons.append(f"speed {speed_mps:.2f} mps")
        if dt > 0.0:
            accel = (speed_mps - previous_speed_mps) / dt
            if abs(accel) > self.max_single_wheel_accel_mps2:
                reasons.append(f"accel {accel:.2f} mps2")

        if reasons:
            if name == "left":
                self.rejected_left_count += 1
            else:
                self.rejected_right_count += 1
            if self.debug_rejected_ticks:
                self.get_logger().warn(f"Rejected {name} wheel step: {', '.join(reasons)}")
            return False
        return True

    def choose_one_wheel_when_pair_disagrees(
        self,
        left_speed_mps: float,
        right_speed_mps: float,
        imu_angular_velocity_radps: float,
        dt: float,
    ):
        wheel_angular_velocity = (right_speed_mps - left_speed_mps) / self.wheel_base_m
        angular_error = abs(wheel_angular_velocity - imu_angular_velocity_radps)
        if angular_error <= self.max_wheel_imu_angular_error_radps:
            return True, True

        left_accel = abs(left_speed_mps - self.previous_left_speed_mps) / max(dt, 1.0e-6)
        right_accel = abs(right_speed_mps - self.previous_right_speed_mps) / max(dt, 1.0e-6)

        if left_accel <= right_accel:
            self.rejected_right_count += 1
            if self.debug_rejected_ticks:
                self.get_logger().warn(
                    "Wheel pair disagrees with IMU yaw rate. Keeping left wheel, rejecting right wheel."
                )
            return True, False

        self.rejected_left_count += 1
        if self.debug_rejected_ticks:
            self.get_logger().warn(
                "Wheel pair disagrees with IMU yaw rate. Keeping right wheel, rejecting left wheel."
            )
        return False, True

    def estimate_distance_from_accel(self, data, dt: float) -> Optional[float]:
        if not self.use_imu_accel_fallback:
            return None
        if not data.get("has_accel", False):
            return None

        accel_x = self.imu_accel_x_sign * (data["accel_x_mps2"] - self.imu_accel_x_bias_mps2)
        accel_x = self.clamp(
            accel_x,
            -self.max_imu_accel_fallback_mps2,
            self.max_imu_accel_fallback_mps2,
        )
        predicted_speed = self.filtered_linear_velocity_mps + accel_x * dt
        predicted_speed = self.clamp(
            predicted_speed,
            -self.max_accel_fallback_speed_mps,
            self.max_accel_fallback_speed_mps,
        )
        self.imu_accel_fallback_count += 1
        return predicted_speed * dt

    def update_odometry(self, data):
        if self.previous_data is None:
            self.previous_data = data
            self.initial_imu_yaw_rad = math.radians(data["yaw_deg"])
            self.yaw_rad = 0.0
            self.history.append((data["time_ms"] / 1000.0, self.distance_total_m, self.yaw_rad))
            return

        current_time_s = data["time_ms"] / 1000.0
        previous_time_s = self.previous_data["time_ms"] / 1000.0
        dt = current_time_s - previous_time_s
        if dt <= 0.0:
            self.previous_data = data
            return

        previous_yaw = self.yaw_rad
        imu_yaw = self.imu_relative_yaw(data)
        imu_heading_delta = normalize_angle(imu_yaw - previous_yaw)
        imu_angular_velocity = imu_heading_delta / dt

        raw_delta_left_ticks = data["left_ticks"] - self.previous_data["left_ticks"]
        raw_delta_right_ticks = data["right_ticks"] - self.previous_data["right_ticks"]

        delta_left_ticks = raw_delta_left_ticks * self.left_tick_sign
        delta_right_ticks = raw_delta_right_ticks * self.right_tick_sign

        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m
        left_speed_mps = left_distance_m / dt
        right_speed_mps = right_distance_m / dt

        left_valid = self.wheel_step_is_valid(
            "left",
            delta_left_ticks,
            left_speed_mps,
            self.previous_left_speed_mps,
            dt,
        )
        right_valid = self.wheel_step_is_valid(
            "right",
            delta_right_ticks,
            right_speed_mps,
            self.previous_right_speed_mps,
            dt,
        )

        if left_valid and right_valid and self.use_imu_yaw:
            left_valid, right_valid = self.choose_one_wheel_when_pair_disagrees(
                left_speed_mps,
                right_speed_mps,
                imu_angular_velocity,
                dt,
            )

        if self.use_imu_yaw:
            self.yaw_rad = imu_yaw
            angular_velocity_for_pose = imu_angular_velocity
        else:
            if left_valid and right_valid:
                angular_velocity_for_pose = (right_speed_mps - left_speed_mps) / self.wheel_base_m
            else:
                angular_velocity_for_pose = imu_angular_velocity
            self.yaw_rad = normalize_angle(previous_yaw + angular_velocity_for_pose * dt)

        if left_valid and right_valid:
            center_distance_m = (left_distance_m + right_distance_m) / 2.0
        elif left_valid and self.one_wheel_fallback_enabled:
            center_distance_m = left_distance_m + angular_velocity_for_pose * self.wheel_base_m * dt / 2.0
            self.one_wheel_fallback_count += 1
        elif right_valid and self.one_wheel_fallback_enabled:
            center_distance_m = right_distance_m - angular_velocity_for_pose * self.wheel_base_m * dt / 2.0
            self.one_wheel_fallback_count += 1
        else:
            accel_distance = self.estimate_distance_from_accel(data, dt)
            if accel_distance is None:
                center_distance_m = 0.0
            else:
                center_distance_m = accel_distance
            self.both_wheels_rejected_count += 1

        heading_delta = normalize_angle(self.yaw_rad - previous_yaw)
        heading_mid = normalize_angle(previous_yaw + heading_delta / 2.0)
        self.x += center_distance_m * math.cos(heading_mid)
        self.y += center_distance_m * math.sin(heading_mid)
        self.distance_total_m += center_distance_m

        self.previous_left_speed_mps = left_speed_mps if left_valid else self.previous_left_speed_mps
        self.previous_right_speed_mps = right_speed_mps if right_valid else self.previous_right_speed_mps

        self.history.append((current_time_s, self.distance_total_m, self.yaw_rad))
        while len(self.history) > 2 and current_time_s - self.history[0][0] > max(1.0, self.velocity_window_s * 3.0):
            self.history.popleft()

        linear_velocity_mps, angular_velocity_radps = self.compute_window_velocity(current_time_s)
        alpha = self.clamp(self.velocity_filter_alpha, 0.0, 1.0)
        self.filtered_linear_velocity_mps = alpha * linear_velocity_mps + (1.0 - alpha) * self.filtered_linear_velocity_mps
        self.filtered_angular_velocity_radps = alpha * angular_velocity_radps + (1.0 - alpha) * self.filtered_angular_velocity_radps

        stamp = self.get_clock().now().to_msg()
        if self.publish_odom_enabled:
            self.publish_odom(stamp, self.filtered_linear_velocity_mps, self.filtered_angular_velocity_radps)
        if self.publish_tf_enabled:
            self.publish_tf(stamp)
        if self.publish_imu_enabled:
            self.publish_imu(stamp, data, imu_angular_velocity)

        self.previous_data = data

    def compute_window_velocity(self, current_time_s: float):
        if len(self.history) < 2:
            return 0.0, 0.0

        reference = self.history[0]
        for item in self.history:
            if current_time_s - item[0] >= self.velocity_window_s:
                reference = item
            else:
                break

        ref_time, ref_distance, ref_yaw = reference
        dt = current_time_s - ref_time
        if dt <= 0.0:
            return self.filtered_linear_velocity_mps, self.filtered_angular_velocity_radps

        linear_velocity = (self.distance_total_m - ref_distance) / dt
        angular_velocity = normalize_angle(self.yaw_rad - ref_yaw) / dt
        return linear_velocity, angular_velocity

    def publish_odom(self, stamp, linear_velocity_mps: float, angular_velocity_radps: float):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw_rad)
        odom_msg.twist.twist.linear.x = linear_velocity_mps
        odom_msg.twist.twist.angular.z = angular_velocity_radps

        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.10
        odom_msg.twist.covariance[0] = 0.10
        odom_msg.twist.covariance[35] = 0.15
        self.odom_pub.publish(odom_msg)

    def publish_tf(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = yaw_to_quaternion(self.yaw_rad)
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_imu(self, stamp, data, angular_velocity_radps: float):
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame
        roll_rad = math.radians(data["roll_deg"])
        pitch_rad = math.radians(data["pitch_deg"])
        imu_msg.orientation = euler_to_quaternion(roll_rad, pitch_rad, self.yaw_rad)
        imu_msg.angular_velocity.z = angular_velocity_radps

        if data.get("has_accel", False):
            imu_msg.linear_acceleration.x = data["accel_x_mps2"]
            imu_msg.linear_acceleration.y = data["accel_y_mps2"]
            imu_msg.linear_acceleration.z = data["accel_z_mps2"]
            imu_msg.linear_acceleration_covariance[0] = 0.50
            imu_msg.linear_acceleration_covariance[4] = 0.50
            imu_msg.linear_acceleration_covariance[8] = 0.50
        else:
            imu_msg.linear_acceleration_covariance[0] = -1.0

        imu_msg.orientation_covariance[0] = 0.10
        imu_msg.orientation_covariance[4] = 0.10
        imu_msg.orientation_covariance[8] = 0.10
        imu_msg.angular_velocity_covariance[0] = 99999.0
        imu_msg.angular_velocity_covariance[4] = 99999.0
        imu_msg.angular_velocity_covariance[8] = 0.15
        self.imu_pub.publish(imu_msg)

    def debug_summary_if_needed(self):
        if not self.debug_enabled:
            return
        now = time.monotonic()
        if now - self.last_summary_time < self.debug_summary_period_s:
            return
        self.last_summary_time = now
        self.get_logger().info(
            "Navigation odom summary: "
            f"valid_data={self.valid_data_count}, "
            f"rejected_left={self.rejected_left_count}, "
            f"rejected_right={self.rejected_right_count}, "
            f"one_wheel_fallback={self.one_wheel_fallback_count}, "
            f"both_wheels_rejected={self.both_wheels_rejected_count}, "
            f"imu_accel_fallback={self.imu_accel_fallback_count}, "
            f"v={self.filtered_linear_velocity_mps:.3f}, "
            f"w={self.filtered_angular_velocity_radps:.3f}"
        )

    def destroy_node(self):
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            try:
                with self.serial_lock:
                    self.serial_port.write(b"STOP\n")
                    self.serial_port.flush()
                    time.sleep(0.05)
                    self.serial_port.close()
            except Exception as error:
                self.get_logger().warn(f"Error while closing serial port: {error}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSensorNodeNav()
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
