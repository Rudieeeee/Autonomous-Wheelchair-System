#!/usr/bin/env python3

import math
import threading
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

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timer_period_s", 0.01)

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

        # New joystick command input.
        self.declare_parameter("joystick_cmd_topic", "/joystick_cmd")
        self.declare_parameter("enable_joystick_serial_output", True)

        self.serial_port_name = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.timer_period_s = float(
            self.get_parameter("timer_period_s").value
        )

        self.wheel_diameter_m = float(
            self.get_parameter("wheel_diameter_m").value
        )
        self.magnets_per_wheel = int(
            self.get_parameter("magnets_per_wheel").value
        )
        self.wheel_base_m = float(
            self.get_parameter("wheel_base_m").value
        )

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value

        self.publish_odom_enabled = bool(
            self.get_parameter("publish_odom").value
        )
        self.publish_tf_enabled = bool(
            self.get_parameter("publish_tf").value
        )
        self.publish_imu_enabled = bool(
            self.get_parameter("publish_imu").value
        )

        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)

        self.left_tick_sign = float(
            self.get_parameter("left_tick_sign").value
        )
        self.right_tick_sign = float(
            self.get_parameter("right_tick_sign").value
        )

        self.joystick_cmd_topic = self.get_parameter(
            "joystick_cmd_topic"
        ).value
        self.enable_joystick_serial_output = bool(
            self.get_parameter("enable_joystick_serial_output").value
        )

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.distance_per_tick_m = (
            self.wheel_circumference_m / self.magnets_per_wheel
        )

        self.serial_lock = threading.Lock()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joystick_sub = self.create_subscription(
            Int16MultiArray,
            self.joystick_cmd_topic,
            self.joystick_cmd_callback,
            10,
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0

        self.previous_data = None
        self.initial_imu_yaw_rad = None

        try:
            self.serial_port = serial.Serial(
                self.serial_port_name,
                self.baud_rate,
                timeout=0.01,
                write_timeout=0.01,
            )
        except serial.SerialException as error:
            self.get_logger().error(
                f"Could not open serial port {self.serial_port_name}: {error}"
            )
            raise error

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
            f"wheel_diameter_m={self.wheel_diameter_m}, "
            f"wheel_circumference_m={self.wheel_circumference_m}, "
            f"magnets_per_wheel={self.magnets_per_wheel}, "
            f"distance_per_tick_m={self.distance_per_tick_m}, "
            f"wheel_base_m={self.wheel_base_m}, "
            f"use_imu_yaw={self.use_imu_yaw}"
        )

        self.timer = self.create_timer(
            self.timer_period_s,
            self.read_serial,
        )

    def joystick_cmd_callback(self, msg: Int16MultiArray):
        if not self.enable_joystick_serial_output:
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
                self.serial_port.write(line.encode("ascii"))
        except serial.SerialException as error:
            self.get_logger().error(
                f"Serial joystick write error: {error}"
            )

    def parse_data_line(self, line: str):
        parts = line.split(",")

        if len(parts) != 9:
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
            }
        except ValueError:
            return None

        return data

    def read_serial(self):
        try:
            with self.serial_lock:
                raw_line = self.serial_port.readline()
        except serial.SerialException as error:
            self.get_logger().error(f"Serial read error: {error}")
            return

        if not raw_line:
            return

        line = raw_line.decode("utf-8", errors="ignore").strip()

        if not line:
            return

        if not line.startswith("DATA"):
            self.get_logger().info(line)
            return

        data = self.parse_data_line(line)

        if data is None:
            self.get_logger().warn(f"Could not parse line: {line}")
            return

        self.update_odometry(data)

    def update_odometry(self, data):
        if self.previous_data is None:
            self.previous_data = data
            self.initial_imu_yaw_rad = math.radians(data["yaw_deg"])

            self.get_logger().info(
                "Received first DATA line. Odometry initialized."
            )
            return

        current_time_ms = data["time_ms"]
        previous_time_ms = self.previous_data["time_ms"]

        dt = (current_time_ms - previous_time_ms) / 1000.0

        if dt <= 0.0:
            self.get_logger().warn(
                f"Invalid dt={dt}. Skipping odometry update."
            )
            return

        delta_left_ticks = (
            data["left_ticks"] - self.previous_data["left_ticks"]
        )
        delta_right_ticks = (
            data["right_ticks"] - self.previous_data["right_ticks"]
        )

        delta_left_ticks *= self.left_tick_sign
        delta_right_ticks *= self.right_tick_sign

        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m

        left_speed_mps = left_distance_m / dt
        right_speed_mps = right_distance_m / dt

        linear_velocity_mps = (
            left_speed_mps + right_speed_mps
        ) / 2.0

        angular_velocity_radps_from_wheels = (
            right_speed_mps - left_speed_mps
        ) / self.wheel_base_m

        previous_yaw_rad = self.yaw_rad
        imu_yaw_rad = math.radians(data["yaw_deg"])

        if self.use_imu_yaw:
            self.yaw_rad = normalize_angle(
                imu_yaw_rad - self.initial_imu_yaw_rad
            )

            angular_velocity_radps = normalize_angle(
                self.yaw_rad - previous_yaw_rad
            ) / dt

        else:
            self.yaw_rad = normalize_angle(
                self.yaw_rad + angular_velocity_radps_from_wheels * dt
            )

            angular_velocity_radps = angular_velocity_radps_from_wheels

        distance_m = (left_distance_m + right_distance_m) / 2.0

        heading_delta = normalize_angle(self.yaw_rad - previous_yaw_rad)
        heading_mid = normalize_angle(previous_yaw_rad + heading_delta / 2.0)

        self.x += distance_m * math.cos(heading_mid)
        self.y += distance_m * math.sin(heading_mid)

        stamp = self.get_clock().now().to_msg()

        if self.publish_odom_enabled:
            self.publish_odom(
                stamp,
                linear_velocity_mps,
                angular_velocity_radps,
            )

        if self.publish_tf_enabled:
            self.publish_tf(stamp)

        if self.publish_imu_enabled:
            self.publish_imu(
                stamp,
                data,
                angular_velocity_radps,
            )

        self.previous_data = data

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

    def publish_imu(self, stamp, data, angular_velocity_radps: float):
        imu_msg = Imu()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame

        roll_rad = math.radians(data["roll_deg"])
        pitch_rad = math.radians(data["pitch_deg"])
        yaw_rad = self.yaw_rad

        imu_msg.orientation = euler_to_quaternion(
            roll_rad,
            pitch_rad,
            yaw_rad,
        )

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

    def destroy_node(self):
        # Important:
        # Do NOT send J,0,0 during shutdown.
        # Sending serial data while the launch file is closing can make the
        # Arduino think that a real joystick command was received.
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