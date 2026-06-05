#!/usr/bin/env python3

import math
import threading
import time
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


def euler_to_quaternion(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> Quaternion:
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


class ArduinoSensorNode(Node):
    def __init__(self):
        super().__init__("arduino_sensor_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timer_period_s", 0.01)

        self.declare_parameter("clear_serial_buffers_on_start", True)
        self.declare_parameter("serial_startup_delay_s", 2.0)
        self.declare_parameter("startup_skip_lines", 20)

        self.declare_parameter("wheel_diameter_m", 0.35)
        self.declare_parameter("magnets_per_wheel", 12)
        self.declare_parameter("wheel_base_m", 0.55)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")

        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_imu", True)

        self.declare_parameter("use_imu_yaw", True)

        self.declare_parameter("left_tick_sign", 1.0)
        self.declare_parameter("right_tick_sign", 1.0)

        self.declare_parameter("joystick_cmd_topic", "/joystick_cmd")
        self.declare_parameter("enable_joystick_serial_output", True)

        # Debug parameters.
        # debug_no_data_period_s: how often to print when in_waiting == 0.
        # debug_summary_period_s: how often to print counters.
        # debug_raw_serial: print every received raw serial line.
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_no_data_period_s", 1.0)
        self.declare_parameter("debug_summary_period_s", 2.0)
        self.declare_parameter("debug_raw_serial", True)
        self.declare_parameter("debug_publish_messages", False)

        self.serial_port_name = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.timer_period_s = float(self.get_parameter("timer_period_s").value)

        self.clear_serial_buffers_on_start = bool(
            self.get_parameter("clear_serial_buffers_on_start").value
        )
        self.serial_startup_delay_s = float(
            self.get_parameter("serial_startup_delay_s").value
        )
        self.startup_skip_lines = int(self.get_parameter("startup_skip_lines").value)

        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.magnets_per_wheel = int(self.get_parameter("magnets_per_wheel").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value

        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_imu_enabled = bool(self.get_parameter("publish_imu").value)

        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)

        self.left_tick_sign = float(self.get_parameter("left_tick_sign").value)
        self.right_tick_sign = float(self.get_parameter("right_tick_sign").value)

        self.joystick_cmd_topic = self.get_parameter("joystick_cmd_topic").value
        self.enable_joystick_serial_output = bool(
            self.get_parameter("enable_joystick_serial_output").value
        )

        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_no_data_period_s = float(
            self.get_parameter("debug_no_data_period_s").value
        )
        self.debug_summary_period_s = float(
            self.get_parameter("debug_summary_period_s").value
        )
        self.debug_raw_serial = bool(self.get_parameter("debug_raw_serial").value)
        self.debug_publish_messages = bool(
            self.get_parameter("debug_publish_messages").value
        )

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.distance_per_tick_m = self.wheel_circumference_m / self.magnets_per_wheel

        # -----------------------------
        # Runtime state
        # -----------------------------
        self.serial_lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0

        self.previous_data = None
        self.initial_imu_yaw_rad = None

        # Debug counters.
        self.timer_calls = 0
        self.no_data_count = 0
        self.raw_line_count = 0
        self.empty_line_count = 0
        self.non_data_line_count = 0
        self.parse_fail_count = 0
        self.valid_data_count = 0
        self.odom_init_count = 0
        self.odom_publish_count = 0
        self.imu_publish_count = 0
        self.tf_publish_count = 0
        self.invalid_dt_count = 0
        self.serial_exception_count = 0
        self.joystick_write_count = 0
        self.joystick_write_fail_count = 0

        self.last_no_data_log_time = time.monotonic()
        self.last_summary_log_time = time.monotonic()
        self.last_valid_data_wall_time = None
        self.last_raw_line_wall_time = None

        # -----------------------------
        # ROS publishers/subscribers
        # -----------------------------
        self.get_logger().info("DEBUG INIT: creating ROS publishers/subscribers")

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joystick_sub = self.create_subscription(
            Int16MultiArray,
            self.joystick_cmd_topic,
            self.joystick_cmd_callback,
            10,
        )

        # -----------------------------
        # Serial open
        # -----------------------------
        self.get_logger().info(
            "DEBUG SERIAL OPEN: "
            f"port={self.serial_port_name}, baud={self.baud_rate}, "
            "timeout=0.01, write_timeout=0.01, exclusive=True"
        )

        try:
            self.serial_port = serial.Serial(
                self.serial_port_name,
                self.baud_rate,
                timeout=0.01,
                write_timeout=0.01,
                exclusive=True,
            )
        except serial.SerialException as error:
            self.get_logger().error(
                f"DEBUG SERIAL OPEN FAILED: could not open {self.serial_port_name}: {error}"
            )
            raise error

        self.get_logger().info(
            "DEBUG SERIAL OPEN OK: "
            f"is_open={self.serial_port.is_open}, "
            f"name={self.serial_port.name}, "
            f"baudrate={self.serial_port.baudrate}, "
            f"timeout={self.serial_port.timeout}, "
            f"write_timeout={self.serial_port.write_timeout}"
        )

        # -----------------------------
        # Startup buffer handling
        # -----------------------------
        if self.clear_serial_buffers_on_start:
            self.get_logger().info(
                "DEBUG STARTUP BUFFER: clear_serial_buffers_on_start=True, "
                f"sleeping {self.serial_startup_delay_s}s before reset_input_buffer()"
            )

            if self.serial_startup_delay_s > 0.0:
                time.sleep(self.serial_startup_delay_s)

            try:
                with self.serial_lock:
                    waiting_before_clear = self.serial_port.in_waiting
                    self.get_logger().info(
                        "DEBUG STARTUP BUFFER: "
                        f"in_waiting before clear={waiting_before_clear}"
                    )
                    self.serial_port.reset_input_buffer()
                    self.serial_port.reset_output_buffer()
                    waiting_after_clear = self.serial_port.in_waiting

                self.get_logger().info(
                    "DEBUG STARTUP BUFFER: serial input/output buffers cleared, "
                    f"in_waiting after clear={waiting_after_clear}"
                )
            except serial.SerialException as error:
                self.get_logger().warn(
                    f"DEBUG STARTUP BUFFER FAILED: could not clear serial buffers: {error}"
                )
        else:
            self.get_logger().info(
                "DEBUG STARTUP BUFFER: clear_serial_buffers_on_start=False, not clearing buffers"
            )

        self.get_logger().info(
            f"Connected to {self.serial_port_name} at {self.baud_rate} baud"
        )

        self.get_logger().info(
            "Expected Arduino format: "
            "DATA,time_ms,left_ticks,right_ticks,left_state,right_state,"
            "yaw_deg,pitch_deg,roll_deg"
        )

        self.get_logger().info(
            f"Joystick command topic: {self.joystick_cmd_topic}, "
            f"enabled={self.enable_joystick_serial_output}"
        )

        self.get_logger().info(
            "DEBUG PARAMS: "
            f"wheel_diameter_m={self.wheel_diameter_m}, "
            f"wheel_circumference_m={self.wheel_circumference_m}, "
            f"magnets_per_wheel={self.magnets_per_wheel}, "
            f"distance_per_tick_m={self.distance_per_tick_m}, "
            f"wheel_base_m={self.wheel_base_m}, "
            f"use_imu_yaw={self.use_imu_yaw}, "
            f"clear_serial_buffers_on_start={self.clear_serial_buffers_on_start}, "
            f"serial_startup_delay_s={self.serial_startup_delay_s}, "
            f"startup_skip_lines={self.startup_skip_lines}, "
            f"publish_odom={self.publish_odom_enabled}, "
            f"publish_tf={self.publish_tf_enabled}, "
            f"publish_imu={self.publish_imu_enabled}, "
            f"debug_enabled={self.debug_enabled}, "
            f"debug_raw_serial={self.debug_raw_serial}"
        )

        self.get_logger().info(
            f"DEBUG TIMER: creating read_serial timer with period={self.timer_period_s}s"
        )

        self.timer = self.create_timer(self.timer_period_s, self.read_serial)

    def debug_log_summary_if_needed(self):
        if not self.debug_enabled:
            return

        now = time.monotonic()
        if now - self.last_summary_log_time < self.debug_summary_period_s:
            return

        self.last_summary_log_time = now

        seconds_since_raw = None
        if self.last_raw_line_wall_time is not None:
            seconds_since_raw = round(now - self.last_raw_line_wall_time, 3)

        seconds_since_valid = None
        if self.last_valid_data_wall_time is not None:
            seconds_since_valid = round(now - self.last_valid_data_wall_time, 3)

        try:
            with self.serial_lock:
                waiting = self.serial_port.in_waiting
                is_open = self.serial_port.is_open
        except Exception as error:
            waiting = "error"
            is_open = "error"
            self.get_logger().warn(
                f"DEBUG SUMMARY: could not read serial state: {error}"
            )

        self.get_logger().warn(
            "DEBUG SUMMARY: "
            f"timer_calls={self.timer_calls}, "
            f"in_waiting={waiting}, is_open={is_open}, "
            f"raw_lines={self.raw_line_count}, valid_data={self.valid_data_count}, "
            f"non_data={self.non_data_line_count}, parse_fail={self.parse_fail_count}, "
            f"empty={self.empty_line_count}, no_data={self.no_data_count}, "
            f"invalid_dt={self.invalid_dt_count}, odom_pub={self.odom_publish_count}, "
            f"tf_pub={self.tf_publish_count}, imu_pub={self.imu_publish_count}, "
            f"seconds_since_raw={seconds_since_raw}, "
            f"seconds_since_valid={seconds_since_valid}, "
            f"previous_data_exists={self.previous_data is not None}"
        )

    def joystick_cmd_callback(self, msg: Int16MultiArray):
        self.get_logger().info(
            f"DEBUG JOYSTICK CALLBACK: received msg.data={list(msg.data)}"
        )

        if not self.enable_joystick_serial_output:
            self.get_logger().warn(
                "DEBUG JOYSTICK CALLBACK: serial output disabled, not writing to Arduino"
            )
            return

        if len(msg.data) < 2:
            self.get_logger().warn(
                f"Invalid joystick_cmd message: expected [x, y], got {msg.data}"
            )
            return

        x = int(msg.data[0])
        y = int(msg.data[1])

        x = max(-100, min(100, x))
        y = max(-100, min(100, y))

        line = f"J,{x},{y}\n"

        try:
            with self.serial_lock:
                before_waiting = self.serial_port.in_waiting
                self.serial_port.write(line.encode("ascii"))
                after_waiting = self.serial_port.in_waiting

            self.joystick_write_count += 1
            self.get_logger().info(
                "DEBUG JOYSTICK WRITE OK: "
                f"line={line.strip()}, writes={self.joystick_write_count}, "
                f"in_waiting_before={before_waiting}, in_waiting_after={after_waiting}"
            )
        except serial.SerialException as error:
            self.joystick_write_fail_count += 1
            self.get_logger().error(
                f"Serial joystick write error: {error}; failures={self.joystick_write_fail_count}"
            )

    def parse_data_line(self, line: str):
        parts = line.split(",")

        if len(parts) != 9:
            self.get_logger().warn(
                "DEBUG PARSE FAIL: wrong number of comma parts, "
                f"n_parts={len(parts)}, parts={parts}, line={line!r}"
            )
            return None

        if parts[0] != "DATA":
            self.get_logger().warn(
                f"DEBUG PARSE FAIL: first field is not DATA, first={parts[0]!r}, line={line!r}"
            )
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
            }
        except ValueError as error:
            self.get_logger().warn(
                f"DEBUG PARSE FAIL: ValueError={error}, parts={parts}, line={line!r}"
            )
            return None

        self.get_logger().info(
            "DEBUG PARSE OK: "
            f"time_ms={data['time_ms']}, left_ticks={data['left_ticks']}, "
            f"right_ticks={data['right_ticks']}, yaw_deg={data['yaw_deg']}"
        )
        return data

    def read_serial(self):
        self.timer_calls += 1
        self.debug_log_summary_if_needed()

        lines_read = 0
        max_lines_per_timer = 50

        while lines_read < max_lines_per_timer:
            try:
                with self.serial_lock:
                    waiting = self.serial_port.in_waiting

                    if waiting <= 0:
                        self.no_data_count += 1

                        now = time.monotonic()
                        if (
                            self.debug_enabled
                            and now - self.last_no_data_log_time >= self.debug_no_data_period_s
                        ):
                            self.last_no_data_log_time = now
                            self.get_logger().warn(
                                "DEBUG SERIAL NO DATA: "
                                f"in_waiting=0, is_open={self.serial_port.is_open}, "
                                f"timer_calls={self.timer_calls}, "
                                f"no_data_count={self.no_data_count}, "
                                f"lines_read_this_timer={lines_read}"
                            )
                        break

                    if self.debug_enabled:
                        self.get_logger().info(
                            "DEBUG SERIAL BEFORE READ: "
                            f"in_waiting={waiting}, lines_read_this_timer={lines_read}"
                        )

                    raw_line = self.serial_port.readline()

                    if self.debug_enabled:
                        waiting_after = self.serial_port.in_waiting
                        self.get_logger().info(
                            "DEBUG SERIAL AFTER READ: "
                            f"raw_len={len(raw_line)}, in_waiting_after={waiting_after}"
                        )

            except serial.SerialException as error:
                self.serial_exception_count += 1
                self.get_logger().error(
                    f"Serial read error: {error}; serial_exception_count={self.serial_exception_count}"
                )
                return

            if not raw_line:
                self.empty_line_count += 1
                self.get_logger().warn(
                    "DEBUG SERIAL EMPTY READ: readline() returned empty bytes even though in_waiting was positive"
                )
                break

            lines_read += 1
            self.raw_line_count += 1
            self.last_raw_line_wall_time = time.monotonic()

            try:
                line = raw_line.decode("utf-8", errors="ignore").strip()
            except Exception as error:
                self.parse_fail_count += 1
                self.get_logger().warn(
                    f"DEBUG DECODE FAIL: error={error}, raw_line={raw_line!r}"
                )
                continue

            if not line:
                self.empty_line_count += 1
                self.get_logger().warn(
                    f"DEBUG SERIAL BLANK LINE: raw_line={raw_line!r}"
                )
                continue

            if self.debug_raw_serial:
                self.get_logger().info(f"RAW SERIAL: {line}")

            if self.startup_skip_lines > 0:
                self.startup_skip_lines -= 1
                self.get_logger().info(
                    "DEBUG STARTUP SKIP: "
                    f"skipped line={line!r}, remaining={self.startup_skip_lines}"
                )
                continue

            if not line.startswith("DATA"):
                self.non_data_line_count += 1
                self.get_logger().warn(
                    f"Non-DATA line received: {line}; non_data_count={self.non_data_line_count}"
                )
                continue

            data = self.parse_data_line(line)

            if data is None:
                self.parse_fail_count += 1
                parts = line.split(",")
                self.get_logger().warn(
                    "Could not parse DATA line. "
                    f"n_parts={len(parts)}, parts={parts}, parse_fail_count={self.parse_fail_count}"
                )
                continue

            self.valid_data_count += 1
            self.last_valid_data_wall_time = time.monotonic()
            self.get_logger().info(
                f"VALID DATA #{self.valid_data_count}: {data}"
            )
            self.update_odometry(data)

        if self.debug_enabled and lines_read >= max_lines_per_timer:
            self.get_logger().warn(
                "DEBUG SERIAL MAX LINES: reached max_lines_per_timer=50 in one timer callback; "
                "serial stream may be faster than processing/logging"
            )

    def update_odometry(self, data):
        self.get_logger().info(
            "DEBUG ODOM UPDATE ENTER: "
            f"previous_data_exists={self.previous_data is not None}, data_time_ms={data['time_ms']}"
        )

        if self.previous_data is None:
            self.previous_data = data
            self.initial_imu_yaw_rad = math.radians(data["yaw_deg"])
            self.odom_init_count += 1

            self.get_logger().info(
                "Received first DATA line. Odometry initialized. "
                f"odom_init_count={self.odom_init_count}, "
                f"initial_imu_yaw_rad={self.initial_imu_yaw_rad}"
            )
            return

        current_time_ms = data["time_ms"]
        previous_time_ms = self.previous_data["time_ms"]

        dt = (current_time_ms - previous_time_ms) / 1000.0

        self.get_logger().info(
            "DEBUG ODOM TIMING: "
            f"current_time_ms={current_time_ms}, previous_time_ms={previous_time_ms}, dt={dt}"
        )

        if dt <= 0.0:
            self.invalid_dt_count += 1
            self.get_logger().warn(
                f"Invalid dt={dt}. current_time_ms={current_time_ms}, "
                f"previous_time_ms={previous_time_ms}. Skipping odometry update. "
                f"invalid_dt_count={self.invalid_dt_count}"
            )
            self.previous_data = data
            return

        delta_left_ticks = data["left_ticks"] - self.previous_data["left_ticks"]
        delta_right_ticks = data["right_ticks"] - self.previous_data["right_ticks"]

        raw_delta_left_ticks = delta_left_ticks
        raw_delta_right_ticks = delta_right_ticks

        delta_left_ticks *= self.left_tick_sign
        delta_right_ticks *= self.right_tick_sign

        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m

        left_speed_mps = left_distance_m / dt
        right_speed_mps = right_distance_m / dt

        linear_velocity_mps = (left_speed_mps + right_speed_mps) / 2.0

        angular_velocity_radps_from_wheels = (
            right_speed_mps - left_speed_mps
        ) / self.wheel_base_m

        previous_yaw_rad = self.yaw_rad
        imu_yaw_rad = math.radians(data["yaw_deg"])

        if self.use_imu_yaw:
            self.yaw_rad = normalize_angle(imu_yaw_rad - self.initial_imu_yaw_rad)
            angular_velocity_radps = normalize_angle(
                self.yaw_rad - previous_yaw_rad
            ) / dt
            yaw_source = "imu"
        else:
            self.yaw_rad = normalize_angle(
                self.yaw_rad + angular_velocity_radps_from_wheels * dt
            )
            angular_velocity_radps = angular_velocity_radps_from_wheels
            yaw_source = "wheel_ticks"

        distance_m = (left_distance_m + right_distance_m) / 2.0

        heading_delta = normalize_angle(self.yaw_rad - previous_yaw_rad)
        heading_mid = normalize_angle(previous_yaw_rad + heading_delta / 2.0)

        old_x = self.x
        old_y = self.y

        self.x += distance_m * math.cos(heading_mid)
        self.y += distance_m * math.sin(heading_mid)

        self.get_logger().info(
            "DEBUG ODOM CALC: "
            f"raw_delta_left_ticks={raw_delta_left_ticks}, "
            f"raw_delta_right_ticks={raw_delta_right_ticks}, "
            f"signed_delta_left_ticks={delta_left_ticks}, "
            f"signed_delta_right_ticks={delta_right_ticks}, "
            f"left_distance_m={left_distance_m:.6f}, "
            f"right_distance_m={right_distance_m:.6f}, "
            f"linear_velocity_mps={linear_velocity_mps:.6f}, "
            f"angular_velocity_radps={angular_velocity_radps:.6f}, "
            f"yaw_source={yaw_source}, previous_yaw_rad={previous_yaw_rad:.6f}, "
            f"yaw_rad={self.yaw_rad:.6f}, old_x={old_x:.6f}, old_y={old_y:.6f}, "
            f"new_x={self.x:.6f}, new_y={self.y:.6f}"
        )

        stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            "DEBUG ODOM PUBLISH DECISION: "
            f"publish_odom={self.publish_odom_enabled}, "
            f"publish_tf={self.publish_tf_enabled}, "
            f"publish_imu={self.publish_imu_enabled}, "
            f"stamp={stamp.sec}.{stamp.nanosec:09d}"
        )

        if self.publish_odom_enabled:
            self.publish_odom(stamp, linear_velocity_mps, angular_velocity_radps)
        else:
            self.get_logger().warn("DEBUG ODOM SKIP: publish_odom is disabled")

        if self.publish_tf_enabled:
            self.publish_tf(stamp)
        else:
            self.get_logger().warn("DEBUG TF SKIP: publish_tf is disabled")

        if self.publish_imu_enabled:
            self.publish_imu(stamp, data, angular_velocity_radps)
        else:
            self.get_logger().warn("DEBUG IMU SKIP: publish_imu is disabled")

        self.previous_data = data
        self.get_logger().info(
            "DEBUG ODOM UPDATE EXIT: previous_data updated to current DATA line"
        )

    def publish_odom(
        self,
        stamp,
        linear_velocity_mps: float,
        angular_velocity_radps: float,
    ):
        quat = yaw_to_quaternion(self.yaw_rad)

        odom_msg = Odometry()

        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quat

        odom_msg.twist.twist.linear.x = linear_velocity_mps
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity_radps

        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.10

        odom_msg.twist.covariance[0] = 0.10
        odom_msg.twist.covariance[35] = 0.20

        self.odom_pub.publish(odom_msg)
        self.odom_publish_count += 1

        if self.debug_publish_messages:
            self.get_logger().info(
                "DEBUG ODOM PUBLISHED: "
                f"count={self.odom_publish_count}, frame={self.odom_frame}, "
                f"child={self.base_frame}, x={self.x:.6f}, y={self.y:.6f}, "
                f"yaw_rad={self.yaw_rad:.6f}, v={linear_velocity_mps:.6f}, "
                f"w={angular_velocity_radps:.6f}"
            )
        elif self.odom_publish_count % 20 == 0:
            self.get_logger().info(
                f"DEBUG ODOM PUBLISHED SUMMARY: count={self.odom_publish_count}"
            )

    def publish_tf(self, stamp):
        quat = yaw_to_quaternion(self.yaw_rad)

        tf_msg = TransformStamped()

        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = quat

        self.tf_broadcaster.sendTransform(tf_msg)
        self.tf_publish_count += 1

        if self.debug_publish_messages:
            self.get_logger().info(
                "DEBUG TF PUBLISHED: "
                f"count={self.tf_publish_count}, {self.odom_frame}->{self.base_frame}, "
                f"x={self.x:.6f}, y={self.y:.6f}, yaw_rad={self.yaw_rad:.6f}"
            )

    def publish_imu(self, stamp, data, angular_velocity_radps: float):
        imu_msg = Imu()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame

        roll_rad = math.radians(data["roll_deg"])
        pitch_rad = math.radians(data["pitch_deg"])
        yaw_rad = self.yaw_rad

        imu_msg.orientation = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = angular_velocity_radps

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        imu_msg.orientation_covariance[0] = 0.10
        imu_msg.orientation_covariance[4] = 0.10
        imu_msg.orientation_covariance[8] = 0.10

        imu_msg.angular_velocity_covariance[0] = 99999.0
        imu_msg.angular_velocity_covariance[4] = 99999.0
        imu_msg.angular_velocity_covariance[8] = 0.20

        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)
        self.imu_publish_count += 1

        if self.debug_publish_messages:
            self.get_logger().info(
                "DEBUG IMU PUBLISHED: "
                f"count={self.imu_publish_count}, frame={self.imu_frame}, "
                f"roll_deg={data['roll_deg']}, pitch_deg={data['pitch_deg']}, "
                f"yaw_rad={yaw_rad:.6f}, angular_z={angular_velocity_radps:.6f}"
            )

    def destroy_node(self):
        self.get_logger().warn(
            "DEBUG DESTROY ENTER: "
            f"raw_lines={self.raw_line_count}, valid_data={self.valid_data_count}, "
            f"odom_pub={self.odom_publish_count}, tf_pub={self.tf_publish_count}, "
            f"imu_pub={self.imu_publish_count}, no_data={self.no_data_count}, "
            f"serial_exceptions={self.serial_exception_count}"
        )

        if hasattr(self, "serial_port"):
            self.get_logger().warn(
                "DEBUG DESTROY SERIAL STATE: "
                f"is_open={self.serial_port.is_open}, name={self.serial_port.name}"
            )

        if hasattr(self, "serial_port") and self.serial_port.is_open:
            try:
                with self.serial_lock:
                    self.serial_port.close()
                self.get_logger().info(
                    "Serial port closed without sending shutdown command."
                )
            except Exception as error:
                self.get_logger().warn(
                    f"Error while closing serial port: {error}"
                )

        self.get_logger().warn("DEBUG DESTROY EXIT: calling super().destroy_node()")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("DEBUG MAIN: KeyboardInterrupt received")
    except Exception as error:
        node.get_logger().error(f"DEBUG MAIN: unexpected exception: {error}")
        raise
    finally:
        node.get_logger().warn("DEBUG MAIN: shutting down node")
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
