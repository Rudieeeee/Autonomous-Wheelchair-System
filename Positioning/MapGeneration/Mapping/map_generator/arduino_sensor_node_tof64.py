#!/usr/bin/env python3

import math
import struct
import threading
import time
from typing import Dict, List, Optional

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
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


def make_xyz_pointcloud2(frame_id: str, stamp, points: List[tuple]) -> PointCloud2:
    """Create a PointCloud2 with x,y,z float32 fields."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", float(x), float(y), float(z)) for x, y, z in points)
    return msg


class ArduinoSensorNode(Node):
    def __init__(self):
        super().__init__("arduino_sensor_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 460800)
        self.declare_parameter("timer_period_s", 0.01)

        self.declare_parameter("clear_serial_buffers_on_start", True)
        self.declare_parameter("serial_startup_delay_s", 2.0)
        self.declare_parameter("startup_skip_lines", 0)

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

        # ToF64 serial format: TOF64,time_ms,seq,<64*4 hex chars>
        self.declare_parameter("publish_tof_cloud", True)
        self.declare_parameter("tof_cloud_topic", "/tof_cloud")
        self.declare_parameter("tof_frame", "base_footprint")
        self.declare_parameter("tof_count", 8)
        self.declare_parameter("tof_columns", 8)
        self.declare_parameter("tof_min_distance_m", 0.03)
        self.declare_parameter("tof_max_distance_m", 4.0)
        self.declare_parameter("tof_invalid_distance_mm", 0)

        # Geometry of the 8 ToF sensors in base_footprint. Change these in sensor_params.yaml.
        self.declare_parameter("tof_sensor_x", [0.0] * 8)
        self.declare_parameter("tof_sensor_y", [0.0] * 8)
        self.declare_parameter("tof_sensor_z", [0.20] * 8)
        self.declare_parameter("tof_sensor_yaw_deg", [0.0] * 8)

        # 8 angular columns per ToF sensor. Tune these to your ToF module FoV.
        self.declare_parameter(
            "tof_column_angles_deg",
            [-22.5, -16.1, -9.6, -3.2, 3.2, 9.6, 16.1, 22.5],
        )

        self.serial_port_name = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.timer_period_s = float(self.get_parameter("timer_period_s").value)

        self.clear_serial_buffers_on_start = bool(self.get_parameter("clear_serial_buffers_on_start").value)
        self.serial_startup_delay_s = float(self.get_parameter("serial_startup_delay_s").value)
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
        self.enable_joystick_serial_output = bool(self.get_parameter("enable_joystick_serial_output").value)

        self.publish_tof_cloud_enabled = bool(self.get_parameter("publish_tof_cloud").value)
        self.tof_cloud_topic = self.get_parameter("tof_cloud_topic").value
        self.tof_frame = self.get_parameter("tof_frame").value
        self.tof_count = int(self.get_parameter("tof_count").value)
        self.tof_columns = int(self.get_parameter("tof_columns").value)
        self.tof_min_distance_m = float(self.get_parameter("tof_min_distance_m").value)
        self.tof_max_distance_m = float(self.get_parameter("tof_max_distance_m").value)
        self.tof_invalid_distance_mm = int(self.get_parameter("tof_invalid_distance_mm").value)

        self.tof_sensor_x = list(self.get_parameter("tof_sensor_x").value)
        self.tof_sensor_y = list(self.get_parameter("tof_sensor_y").value)
        self.tof_sensor_z = list(self.get_parameter("tof_sensor_z").value)
        self.tof_sensor_yaw_deg = list(self.get_parameter("tof_sensor_yaw_deg").value)
        self.tof_column_angles_deg = list(self.get_parameter("tof_column_angles_deg").value)
        self._validate_tof_parameters()

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.distance_per_tick_m = self.wheel_circumference_m / self.magnets_per_wheel

        self.serial_lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0
        self.previous_data: Optional[Dict] = None
        self.initial_imu_yaw_rad: Optional[float] = None

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.tof_cloud_pub = self.create_publisher(PointCloud2, self.tof_cloud_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joystick_sub = self.create_subscription(
            Int16MultiArray,
            self.joystick_cmd_topic,
            self.joystick_cmd_callback,
            10,
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
            self.get_logger().error(f"Could not open {self.serial_port_name}: {error}")
            raise

        if self.clear_serial_buffers_on_start:
            if self.serial_startup_delay_s > 0.0:
                time.sleep(self.serial_startup_delay_s)
            try:
                with self.serial_lock:
                    self.serial_port.reset_input_buffer()
                    self.serial_port.reset_output_buffer()
            except serial.SerialException as error:
                self.get_logger().warn(f"Could not clear serial buffers: {error}")

        self.get_logger().info(f"Connected to {self.serial_port_name} at {self.baud_rate} baud")
        self.get_logger().info("Serial formats: DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg and TOF64,time_ms,seq,<hex_data>")
        self.send_protocol_line("START")

        self.timer = self.create_timer(self.timer_period_s, self.read_serial)

    def _validate_tof_parameters(self):
        expected_points = self.tof_count * self.tof_columns
        if self.tof_count != 8 or self.tof_columns != 8:
            self.get_logger().warn(
                f"TOF64 expects 8 sensors x 8 columns. Current tof_count={self.tof_count}, tof_columns={self.tof_columns}."
            )
        if len(self.tof_sensor_x) != self.tof_count:
            raise ValueError("tof_sensor_x length must equal tof_count")
        if len(self.tof_sensor_y) != self.tof_count:
            raise ValueError("tof_sensor_y length must equal tof_count")
        if len(self.tof_sensor_z) != self.tof_count:
            raise ValueError("tof_sensor_z length must equal tof_count")
        if len(self.tof_sensor_yaw_deg) != self.tof_count:
            raise ValueError("tof_sensor_yaw_deg length must equal tof_count")
        if len(self.tof_column_angles_deg) != self.tof_columns:
            raise ValueError("tof_column_angles_deg length must equal tof_columns")
        if expected_points != 64:
            raise ValueError("TOF64 packet contains exactly 64 values")

    def send_protocol_line(self, line: str):
        if not hasattr(self, "serial_port") or not self.serial_port.is_open:
            return
        try:
            with self.serial_lock:
                self.serial_port.write((line + "\n").encode("ascii"))
                self.serial_port.flush()
        except serial.SerialException as error:
            self.get_logger().error(f"Serial write error while sending {line!r}: {error}")

    def joystick_cmd_callback(self, msg: Int16MultiArray):
        if not self.enable_joystick_serial_output:
            return
        if len(msg.data) < 2:
            self.get_logger().warn(f"Invalid joystick_cmd message: expected [x, y], got {list(msg.data)}")
            return

        x = max(-100, min(100, int(msg.data[0])))
        y = max(-100, min(100, int(msg.data[1])))
        line = f"J,{x},{y}\n"

        try:
            with self.serial_lock:
                self.serial_port.write(line.encode("ascii"))
        except serial.SerialException as error:
            self.get_logger().error(f"Serial joystick write error: {error}")

    def parse_data_line(self, line: str) -> Optional[Dict]:
        parts = line.split(",")
        if len(parts) != 9 or parts[0] != "DATA":
            return None
        try:
            return {
                "time_ms": int(parts[1]),
                "left_ticks": int(parts[2]),
                "right_ticks": int(parts[3]),
                "left_state": int(parts[4]),
                "right_state": int(parts[5]),
                "yaw_deg": float(parts[6]),
                "pitch_deg": float(parts[7]),
                "roll_deg": float(parts[8]),
            }
        except ValueError:
            return None

    def parse_tof64_line(self, line: str) -> Optional[Dict]:
        parts = line.split(",")
        if len(parts) != 4 or parts[0] != "TOF64":
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
        try:
            for i in range(64):
                word_hex = hex_data[i * 4:(i + 1) * 4]
                packed = int(word_hex, 16)
                distance_mm = packed >> 3
                column = packed & 0x07
                tof_id = i // 8

                points.append({
                    "tof_id": tof_id,
                    "column": column,
                    "distance_mm": distance_mm,
                })
        except ValueError:
            return None

        return {"time_ms": time_ms, "seq": seq, "points": points}

    def read_serial(self):
        max_lines_per_timer = 100
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

            if self.startup_skip_lines > 0:
                self.startup_skip_lines -= 1
                continue

            if line.startswith("DATA"):
                data = self.parse_data_line(line)
                if data is None:
                    self.get_logger().warn(f"Could not parse DATA line: {line}")
                    continue
                self.update_odometry(data)
                continue

            if line.startswith("TOF64"):
                tof_data = self.parse_tof64_line(line)
                if tof_data is None:
                    self.get_logger().warn(f"Could not parse TOF64 line")
                    continue
                self.publish_tof_cloud(tof_data)
                continue

            # Ignore STATUS/heartbeat or other non-data lines without debug spam.

    def update_odometry(self, data: Dict):
        if self.previous_data is None:
            self.previous_data = data
            self.initial_imu_yaw_rad = math.radians(data["yaw_deg"])
            return

        current_time_ms = data["time_ms"]
        previous_time_ms = self.previous_data["time_ms"]
        dt = (current_time_ms - previous_time_ms) / 1000.0
        if dt <= 0.0:
            self.previous_data = data
            return

        delta_left_ticks = (data["left_ticks"] - self.previous_data["left_ticks"]) * self.left_tick_sign
        delta_right_ticks = (data["right_ticks"] - self.previous_data["right_ticks"]) * self.right_tick_sign

        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m

        left_speed_mps = left_distance_m / dt
        right_speed_mps = right_distance_m / dt
        linear_velocity_mps = (left_speed_mps + right_speed_mps) / 2.0

        angular_velocity_radps_from_wheels = (right_speed_mps - left_speed_mps) / self.wheel_base_m

        previous_yaw_rad = self.yaw_rad
        imu_yaw_rad = math.radians(data["yaw_deg"])

        if self.use_imu_yaw:
            self.yaw_rad = normalize_angle(imu_yaw_rad - self.initial_imu_yaw_rad)
            angular_velocity_radps = normalize_angle(self.yaw_rad - previous_yaw_rad) / dt
        else:
            self.yaw_rad = normalize_angle(self.yaw_rad + angular_velocity_radps_from_wheels * dt)
            angular_velocity_radps = angular_velocity_radps_from_wheels

        distance_m = (left_distance_m + right_distance_m) / 2.0
        heading_delta = normalize_angle(self.yaw_rad - previous_yaw_rad)
        heading_mid = normalize_angle(previous_yaw_rad + heading_delta / 2.0)

        self.x += distance_m * math.cos(heading_mid)
        self.y += distance_m * math.sin(heading_mid)

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
        odom_msg.twist.twist.angular.z = angular_velocity_radps

        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.10
        odom_msg.twist.covariance[0] = 0.10
        odom_msg.twist.covariance[35] = 0.20

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

    def publish_imu(self, stamp, data: Dict, angular_velocity_radps: float):
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame

        roll_rad = math.radians(data["roll_deg"])
        pitch_rad = math.radians(data["pitch_deg"])
        imu_msg.orientation = euler_to_quaternion(roll_rad, pitch_rad, self.yaw_rad)
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

    def publish_tof_cloud(self, tof_data: Dict):
        if not self.publish_tof_cloud_enabled:
            return

        cloud_points = []
        for point in tof_data["points"]:
            tof_id = int(point["tof_id"])
            column = int(point["column"])
            distance_mm = int(point["distance_mm"])

            if distance_mm == self.tof_invalid_distance_mm:
                continue

            distance_m = distance_mm / 1000.0
            if distance_m < self.tof_min_distance_m or distance_m > self.tof_max_distance_m:
                continue

            sensor_x = float(self.tof_sensor_x[tof_id])
            sensor_y = float(self.tof_sensor_y[tof_id])
            sensor_z = float(self.tof_sensor_z[tof_id])
            sensor_yaw_rad = math.radians(float(self.tof_sensor_yaw_deg[tof_id]))
            column_angle_rad = math.radians(float(self.tof_column_angles_deg[column]))
            angle_rad = sensor_yaw_rad + column_angle_rad

            x = sensor_x + distance_m * math.cos(angle_rad)
            y = sensor_y + distance_m * math.sin(angle_rad)
            z = sensor_z
            cloud_points.append((x, y, z))

        stamp = self.get_clock().now().to_msg()
        cloud_msg = make_xyz_pointcloud2(self.tof_frame, stamp, cloud_points)
        self.tof_cloud_pub.publish(cloud_msg)

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
    node = ArduinoSensorNode()
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
