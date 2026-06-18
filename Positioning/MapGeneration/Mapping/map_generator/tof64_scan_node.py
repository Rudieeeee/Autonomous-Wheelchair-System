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
from sensor_msgs.msg import Imu, LaserScan
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


class CombinedArduinoSensorNode(Node):
    def __init__(self):
        super().__init__('combined_arduino_sensor_node')

        # Serial
        self.declare_parameter('serial_port', '/dev/arduino_wheelchair')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('timer_period_s', 0.01)
        self.declare_parameter('clear_serial_buffers_on_start', True)
        self.declare_parameter('serial_startup_delay_s', 2.0)
        self.declare_parameter('startup_skip_lines', 20)

        # Odometry / IMU
        self.declare_parameter('wheel_diameter_m', 0.35)
        self.declare_parameter('magnets_per_wheel', 12)
        self.declare_parameter('wheel_base_m', 0.55)
        self.declare_parameter('speed_window_s', 1.0)
        self.declare_parameter('min_speed_dt_s', 0.05)
        self.declare_parameter('speed_timeout_s', 1.25)
        self.declare_parameter('speed_lowpass_alpha', 0.35)
        self.declare_parameter('max_reasonable_speed_mps', 3.0)
        self.declare_parameter('max_reasonable_angular_radps', 8.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('left_tick_sign', 1.0)
        self.declare_parameter('right_tick_sign', 1.0)

        # Joystick serial output
        self.declare_parameter('joystick_cmd_topic', '/joystick_cmd')
        self.declare_parameter('enable_joystick_serial_output', True)

        # ToF LaserScan
        self.declare_parameter('scan_topic', '/tof_scan')
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 4.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(1.0))
        self.declare_parameter('use_inf_for_empty_bins', True)
        self.declare_parameter('send_tof_start_sequence', True)
        self.declare_parameter('calibration_delay_sec', 5.0)
        self.declare_parameter('compared_delay_sec', 5.0)
        self.declare_parameter(
            'sensor_poses',
            [
                0.0,    0.0, 0.0,
               -0.042,  0.0, 0.0,
               -0.084,  0.0, 0.0,
               -0.126,  0.0, 0.0,
               -0.168,  0.0, 0.0,
               -0.210,  0.0, 0.0,
               -0.252,  0.0, 0.0,
               -0.294,  0.0, 0.0,
            ],
        )
        self.declare_parameter(
            'column_angles_deg',
            [-22.5, -16.1, -9.6, -3.2, 3.2, 9.6, 16.1, 22.5],
        )

        # Debug
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('debug_non_data', False)
        self.declare_parameter('debug_publish_messages', False)

        # Load params
        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.timer_period_s = float(self.get_parameter('timer_period_s').value)
        self.clear_serial_buffers_on_start = bool(self.get_parameter('clear_serial_buffers_on_start').value)
        self.serial_startup_delay_s = float(self.get_parameter('serial_startup_delay_s').value)
        self.startup_skip_lines = int(self.get_parameter('startup_skip_lines').value)

        self.wheel_diameter_m = float(self.get_parameter('wheel_diameter_m').value)
        self.magnets_per_wheel = int(self.get_parameter('magnets_per_wheel').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.speed_window_s = float(self.get_parameter('speed_window_s').value)
        self.min_speed_dt_s = float(self.get_parameter('min_speed_dt_s').value)
        self.speed_timeout_s = float(self.get_parameter('speed_timeout_s').value)
        self.speed_lowpass_alpha = float(self.get_parameter('speed_lowpass_alpha').value)
        self.max_reasonable_speed_mps = float(self.get_parameter('max_reasonable_speed_mps').value)
        self.max_reasonable_angular_radps = float(self.get_parameter('max_reasonable_angular_radps').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.publish_odom_enabled = bool(self.get_parameter('publish_odom').value)
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)
        self.publish_imu_enabled = bool(self.get_parameter('publish_imu').value)
        self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        self.left_tick_sign = float(self.get_parameter('left_tick_sign').value)
        self.right_tick_sign = float(self.get_parameter('right_tick_sign').value)

        self.joystick_cmd_topic = self.get_parameter('joystick_cmd_topic').value
        self.enable_joystick_serial_output = bool(self.get_parameter('enable_joystick_serial_output').value)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.angle_increment = float(self.get_parameter('angle_increment').value)
        self.use_inf_for_empty_bins = bool(self.get_parameter('use_inf_for_empty_bins').value)
        self.send_tof_start_sequence = bool(self.get_parameter('send_tof_start_sequence').value)
        self.calibration_delay_sec = float(self.get_parameter('calibration_delay_sec').value)
        self.compared_delay_sec = float(self.get_parameter('compared_delay_sec').value)

        self.debug_enabled = bool(self.get_parameter('debug_enabled').value)
        self.debug_non_data = bool(self.get_parameter('debug_non_data').value)
        self.debug_publish_messages = bool(self.get_parameter('debug_publish_messages').value)

        sensor_poses_flat = self.get_parameter('sensor_poses').value
        column_angles_deg = self.get_parameter('column_angles_deg').value
        if len(sensor_poses_flat) != 24:
            raise RuntimeError('sensor_poses must contain exactly 24 numbers')
        self.sensor_poses = []
        for i in range(0, len(sensor_poses_flat), 3):
            self.sensor_poses.append((float(sensor_poses_flat[i]), float(sensor_poses_flat[i + 1]), float(sensor_poses_flat[i + 2])))
        if len(column_angles_deg) != 8:
            raise RuntimeError('column_angles_deg must contain exactly 8 angles')
        self.column_angles_rad = [math.radians(float(a)) for a in column_angles_deg]
        self.scan_bins = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))

        # Odometry state
        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.distance_per_tick_m = self.wheel_circumference_m / self.magnets_per_wheel
        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0
        self.previous_data = None
        self.initial_imu_yaw_rad = None
        self.velocity_history = deque()
        self.filtered_linear_velocity_mps = 0.0
        self.filtered_angular_velocity_radps = 0.0
        self.last_motion_time_s = None

        # Counters
        self.raw_line_count = 0
        self.data_count = 0
        self.tof_count = 0
        self.bad_tof_count = 0
        self.non_data_count = 0
        self.parse_fail_count = 0
        self.invalid_dt_count = 0
        self.odom_init_count = 0
        self.odom_publish_count = 0
        self.imu_publish_count = 0
        self.tf_publish_count = 0
        self.joystick_write_count = 0
        self.joystick_write_fail_count = 0

        # ROS publishers/subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joystick_sub = self.create_subscription(Int16MultiArray, self.joystick_cmd_topic, self.joystick_cmd_callback, 10)

        # Serial: opened only once here
        self.serial_lock = threading.Lock()
        self.running = True
        self.serial_port = serial.Serial(
            self.serial_port_name,
            self.baud_rate,
            timeout=0.01,
            write_timeout=0.2,
            exclusive=True,
        )

        if self.clear_serial_buffers_on_start:
            if self.serial_startup_delay_s > 0.0:
                time.sleep(self.serial_startup_delay_s)
            with self.serial_lock:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()

        self.get_logger().info(f'Connected to {self.serial_port_name} at {self.baud_rate} baud')
        self.get_logger().info('One serial reader handles DATA -> /odom,/imu and TOF64 -> /tof_scan')

        self.send_protocol_line('START')

        if self.send_tof_start_sequence:
            self.sequence_thread = threading.Thread(target=self.tof_start_sequence, daemon=True)
            self.sequence_thread.start()

        self.timer = self.create_timer(self.timer_period_s, self.read_serial)

    def send_protocol_line(self, line: str):
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            return
        try:
            with self.serial_lock:
                self.serial_port.write((line + '\n').encode('ascii'))
                self.serial_port.flush()
        except serial.SerialException as error:
            self.get_logger().error(f'Serial write error while sending {line!r}: {error}')

    def tof_start_sequence(self):
        # 1 = ToF live stream, 2 = ESP calibrate, 3 = compared/calibrated stream.
        self.send_protocol_line('1')
        self.get_logger().info('Sent ToF command 1: live stream')

        end_time = time.monotonic() + self.calibration_delay_sec
        while self.running and time.monotonic() < end_time:
            time.sleep(0.05)
        if not self.running:
            return

        self.send_protocol_line('2')
        self.get_logger().info('Sent ToF command 2: calibration')

        end_time = time.monotonic() + self.compared_delay_sec
        while self.running and time.monotonic() < end_time:
            time.sleep(0.05)
        if not self.running:
            return

        self.send_protocol_line('3')
        self.get_logger().info('Sent ToF command 3: calibrated compared stream')

    def joystick_cmd_callback(self, msg: Int16MultiArray):
        if not self.enable_joystick_serial_output:
            return
        if len(msg.data) < 2:
            self.get_logger().warn(f'Invalid joystick_cmd message: expected [x, y], got {msg.data}')
            return
        x = max(-100, min(100, int(msg.data[0])))
        y = max(-100, min(100, int(msg.data[1])))
        try:
            with self.serial_lock:
                self.serial_port.write(f'J,{x},{y}\n'.encode('ascii'))
                self.serial_port.flush()
            self.joystick_write_count += 1
        except serial.SerialException as error:
            self.joystick_write_fail_count += 1
            self.get_logger().error(f'Serial joystick write error: {error}; failures={self.joystick_write_fail_count}')

    def read_serial(self):
        lines_read = 0
        max_lines_per_timer = 100
        while lines_read < max_lines_per_timer:
            try:
                with self.serial_lock:
                    if self.serial_port.in_waiting <= 0:
                        break
                    raw_line = self.serial_port.readline()
            except serial.SerialException as error:
                self.get_logger().error(f'Serial read error: {error}')
                return

            if not raw_line:
                break

            lines_read += 1
            self.raw_line_count += 1
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if self.startup_skip_lines > 0:
                self.startup_skip_lines -= 1
                continue

            if line.startswith('DATA'):
                data = self.parse_data_line(line)
                if data is None:
                    self.parse_fail_count += 1
                    self.get_logger().warn(f'Could not parse DATA line: {line[:120]}')
                    continue
                self.data_count += 1
                self.update_odometry(data)
                continue

            if line.startswith('TOF64'):
                decoded = self.parse_tof64_line(line)
                if decoded is None:
                    self.bad_tof_count += 1
                    self.get_logger().warn(f'Bad TOF64 line: {line[:120]}')
                    continue
                self.tof_count += 1
                self.publish_scan(decoded)
                continue

            if line.startswith('DBG_'):
                self.get_logger().info(line)
                continue

            self.non_data_count += 1
            if self.debug_non_data:
                self.get_logger().warn(f'Ignored serial line: {line[:120]}')

    # =========================
    # DATA -> odometry / IMU
    # =========================
    def parse_data_line(self, line: str):
        parts = line.split(',')
        if len(parts) != 9 or parts[0] != 'DATA':
            return None
        try:
            return {
                'time_ms': int(parts[1]),
                'left_ticks': int(parts[2]),
                'right_ticks': int(parts[3]),
                'left_state': int(parts[4]),
                'right_state': int(parts[5]),
                'yaw_deg': float(parts[6]),
                'pitch_deg': float(parts[7]),
                'roll_deg': float(parts[8]),
            }
        except ValueError:
            return None

    def signed_total_ticks(self, data):
        return (float(data['left_ticks']) * self.left_tick_sign, float(data['right_ticks']) * self.right_tick_sign)

    def push_velocity_sample(self, time_s: float, data, yaw_rad: float):
        left_ticks, right_ticks = self.signed_total_ticks(data)
        self.velocity_history.append({'time_s': time_s, 'left_ticks': left_ticks, 'right_ticks': right_ticks, 'yaw_rad': yaw_rad})
        while len(self.velocity_history) > 2 and time_s - self.velocity_history[0]['time_s'] > self.speed_window_s:
            self.velocity_history.popleft()

    def estimate_window_velocity(self, current_time_s: float):
        if len(self.velocity_history) < 2:
            return (self.filtered_linear_velocity_mps, self.filtered_angular_velocity_radps)

        newest = self.velocity_history[-1]
        oldest = self.velocity_history[0]
        window_dt = newest['time_s'] - oldest['time_s']
        if window_dt < self.min_speed_dt_s:
            return (self.filtered_linear_velocity_mps, self.filtered_angular_velocity_radps)

        delta_left_ticks = newest['left_ticks'] - oldest['left_ticks']
        delta_right_ticks = newest['right_ticks'] - oldest['right_ticks']
        ticks_changed = abs(delta_left_ticks) > 0.0 or abs(delta_right_ticks) > 0.0

        if ticks_changed:
            self.last_motion_time_s = newest['time_s']
            left_speed_mps = delta_left_ticks * self.distance_per_tick_m / window_dt
            right_speed_mps = delta_right_ticks * self.distance_per_tick_m / window_dt
            raw_linear_velocity_mps = (left_speed_mps + right_speed_mps) / 2.0
            raw_angular_from_wheels = (right_speed_mps - left_speed_mps) / self.wheel_base_m
            if self.use_imu_yaw:
                yaw_delta = normalize_angle(newest['yaw_rad'] - oldest['yaw_rad'])
                raw_angular_velocity_radps = yaw_delta / window_dt
            else:
                raw_angular_velocity_radps = raw_angular_from_wheels
        elif self.last_motion_time_s is not None and current_time_s - self.last_motion_time_s < self.speed_timeout_s:
            raw_linear_velocity_mps = self.filtered_linear_velocity_mps
            raw_angular_velocity_radps = self.filtered_angular_velocity_radps
        else:
            raw_linear_velocity_mps = 0.0
            raw_angular_velocity_radps = 0.0

        if abs(raw_linear_velocity_mps) > self.max_reasonable_speed_mps:
            raw_linear_velocity_mps = self.filtered_linear_velocity_mps
        if abs(raw_angular_velocity_radps) > self.max_reasonable_angular_radps:
            raw_angular_velocity_radps = self.filtered_angular_velocity_radps

        alpha = max(0.0, min(1.0, self.speed_lowpass_alpha))
        self.filtered_linear_velocity_mps = alpha * raw_linear_velocity_mps + (1.0 - alpha) * self.filtered_linear_velocity_mps
        self.filtered_angular_velocity_radps = alpha * raw_angular_velocity_radps + (1.0 - alpha) * self.filtered_angular_velocity_radps
        return (self.filtered_linear_velocity_mps, self.filtered_angular_velocity_radps)

    def update_odometry(self, data):
        current_time_s = data['time_ms'] / 1000.0

        if self.previous_data is None:
            self.previous_data = data
            self.initial_imu_yaw_rad = math.radians(data['yaw_deg'])
            self.yaw_rad = 0.0
            self.push_velocity_sample(current_time_s, data, self.yaw_rad)
            self.odom_init_count += 1
            self.get_logger().info('Received first DATA line. Odometry initialized.')
            return

        dt = (data['time_ms'] - self.previous_data['time_ms']) / 1000.0
        if dt <= 0.0:
            self.invalid_dt_count += 1
            self.previous_data = data
            return

        delta_left_ticks = (data['left_ticks'] - self.previous_data['left_ticks']) * self.left_tick_sign
        delta_right_ticks = (data['right_ticks'] - self.previous_data['right_ticks']) * self.right_tick_sign

        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m
        distance_m = (left_distance_m + right_distance_m) / 2.0

        left_increment_speed_mps = left_distance_m / dt
        right_increment_speed_mps = right_distance_m / dt
        angular_velocity_radps_from_wheels = (right_increment_speed_mps - left_increment_speed_mps) / self.wheel_base_m

        previous_yaw_rad = self.yaw_rad
        imu_yaw_rad = math.radians(data['yaw_deg'])
        if self.use_imu_yaw:
            self.yaw_rad = normalize_angle(imu_yaw_rad - self.initial_imu_yaw_rad)
        else:
            self.yaw_rad = normalize_angle(self.yaw_rad + angular_velocity_radps_from_wheels * dt)

        heading_delta = normalize_angle(self.yaw_rad - previous_yaw_rad)
        heading_mid = normalize_angle(previous_yaw_rad + heading_delta / 2.0)
        self.x += distance_m * math.cos(heading_mid)
        self.y += distance_m * math.sin(heading_mid)

        self.push_velocity_sample(current_time_s, data, self.yaw_rad)
        linear_velocity_mps, angular_velocity_radps = self.estimate_window_velocity(current_time_s)
        stamp = self.get_clock().now().to_msg()

        if self.publish_odom_enabled:
            self.publish_odom(stamp, linear_velocity_mps, angular_velocity_radps)
        if self.publish_tf_enabled:
            self.publish_tf(stamp)
        if self.publish_imu_enabled:
            self.publish_imu(stamp, data, angular_velocity_radps)

        self.previous_data = data

    def publish_odom(self, stamp, linear_velocity_mps: float, angular_velocity_radps: float):
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
        odom_msg.pose.covariance[35] = 0.1
        odom_msg.twist.covariance[0] = 0.1
        odom_msg.twist.covariance[35] = 0.2
        self.odom_pub.publish(odom_msg)
        self.odom_publish_count += 1

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

    def publish_imu(self, stamp, data, angular_velocity_radps: float):
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame
        imu_msg.orientation = euler_to_quaternion(math.radians(data['roll_deg']), math.radians(data['pitch_deg']), self.yaw_rad)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = angular_velocity_radps
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.orientation_covariance[0] = 0.1
        imu_msg.orientation_covariance[4] = 0.1
        imu_msg.orientation_covariance[8] = 0.1
        imu_msg.angular_velocity_covariance[0] = 99999.0
        imu_msg.angular_velocity_covariance[4] = 99999.0
        imu_msg.angular_velocity_covariance[8] = 0.2
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)
        self.imu_publish_count += 1

    # =========================
    # TOF64 -> LaserScan
    # =========================
    def parse_tof64_line(self, line: str) -> Optional[dict]:
        parts = line.split(',')
        if len(parts) != 4 or parts[0] != 'TOF64':
            return None
        try:
            time_ms = int(parts[1])
            seq = int(parts[2])
            hex_data = parts[3].strip()
        except ValueError:
            return None
        if len(hex_data) != 64 * 4:
            return None

        points = []
        for i in range(64):
            word_hex = hex_data[i * 4:(i + 1) * 4]
            try:
                packed = int(word_hex, 16)
            except ValueError:
                return None
            distance_mm = packed >> 3
            column = packed & 0x07
            tof_id = i // 8
            points.append({'tof_id': tof_id, 'column': column, 'distance_m': distance_mm / 1000.0})

        return {'time_ms': time_ms, 'seq': seq, 'points': points}

    def publish_scan(self, decoded: dict):
        stamp = self.get_clock().now().to_msg()
        empty_value = math.inf if self.use_inf_for_empty_bins else float('nan')
        ranges = [empty_value for _ in range(self.scan_bins)]

        for p in decoded['points']:
            tof_id = p['tof_id']
            col = p['column']
            distance_m = p['distance_m']

            if tof_id < 0 or tof_id >= len(self.sensor_poses):
                continue
            if col < 0 or col >= len(self.column_angles_rad):
                continue
            if distance_m < self.range_min or distance_m > self.range_max:
                continue

            sensor_x, sensor_y, sensor_yaw = self.sensor_poses[tof_id]
            global_angle = float(sensor_yaw) + self.column_angles_rad[col]
            point_x = float(sensor_x) + distance_m * math.cos(global_angle)
            point_y = float(sensor_y) + distance_m * math.sin(global_angle)
            scan_angle = math.atan2(point_y, point_x)
            scan_range = math.hypot(point_x, point_y)

            if scan_range < self.range_min or scan_range > self.range_max:
                continue
            if scan_angle < self.angle_min or scan_angle >= self.angle_max:
                continue

            bin_index = int((scan_angle - self.angle_min) / self.angle_increment)
            if bin_index < 0 or bin_index >= self.scan_bins:
                continue

            current = ranges[bin_index]
            if math.isinf(current) or math.isnan(current) or scan_range < current:
                ranges[bin_index] = scan_range

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.scan_bins * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0666667
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            try:
                with self.serial_lock:
                    self.serial_port.write(b'0\n')
                    self.serial_port.write(b'STOP\n')
                    self.serial_port.flush()
                    time.sleep(0.05)
                    self.serial_port.close()
                self.get_logger().info('Serial port closed after sending 0 and STOP.')
            except Exception as error:
                self.get_logger().warn(f'Error while closing serial port: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CombinedArduinoSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
