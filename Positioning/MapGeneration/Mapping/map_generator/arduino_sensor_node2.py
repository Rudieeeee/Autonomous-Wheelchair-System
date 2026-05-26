#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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


class ArduinoSensorNode(Node):
    def __init__(self):
        super().__init__("arduino_sensor_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 460800)
        self.declare_parameter("timer_period_s", 0.005)

        self.declare_parameter("wheel_diameter_m", 0.35)
        self.declare_parameter("magnets_per_wheel", 12)
        self.declare_parameter("wheel_base_m", 0.55)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")

        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("imu_topic", "/imu/data")

        self.declare_parameter("publish_wheel_odom", True)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_imu", True)

        self.declare_parameter("left_tick_sign", 1.0)
        self.declare_parameter("right_tick_sign", 1.0)

        self.declare_parameter("invert_imu_yaw", False)
        self.declare_parameter("invert_gyro_z", False)

        self.declare_parameter("log_every_n", 50)

        self.serial_port_name = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.timer_period_s = float(self.get_parameter("timer_period_s").value)

        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.magnets_per_wheel = int(self.get_parameter("magnets_per_wheel").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value

        self.wheel_odom_topic = self.get_parameter("wheel_odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value

        self.publish_wheel_odom_enabled = bool(
            self.get_parameter("publish_wheel_odom").value
        )
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_imu_enabled = bool(self.get_parameter("publish_imu").value)

        self.left_tick_sign = float(self.get_parameter("left_tick_sign").value)
        self.right_tick_sign = float(self.get_parameter("right_tick_sign").value)

        self.invert_imu_yaw = bool(self.get_parameter("invert_imu_yaw").value)
        self.invert_gyro_z = bool(self.get_parameter("invert_gyro_z").value)

        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self.update_count = 0

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.distance_per_tick_m = (
            self.wheel_circumference_m / self.magnets_per_wheel
        )

        self.wheel_odom_pub = self.create_publisher(
            Odometry,
            self.wheel_odom_topic,
            10,
        )

        self.imu_pub = self.create_publisher(
            Imu,
            self.imu_topic,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0

        self.previous_data = None
        self.initial_imu_yaw_rad = None

        try:
            self.serial_port = serial.Serial(
                self.serial_port_name,
                self.baud_rate,
                timeout=0.005,
            )
            self.serial_port.reset_input_buffer()
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
            "DATA,time_ms,left_ticks,right_ticks,"
            "gyro_x_radps,gyro_y_radps,gyro_z_radps,"
            "accel_x,accel_y,accel_z,"
            "yaw_deg,pitch_deg,roll_deg,"
            "cal_sys,cal_gyro,cal_accel,cal_mag"
        )

        self.get_logger().info(
            f"wheel_diameter_m={self.wheel_diameter_m}, "
            f"wheel_circumference_m={self.wheel_circumference_m:.4f}, "
            f"magnets_per_wheel={self.magnets_per_wheel}, "
            f"distance_per_tick_m={self.distance_per_tick_m:.4f}, "
            f"wheel_base_m={self.wheel_base_m}"
        )

        self.timer = self.create_timer(
            self.timer_period_s,
            self.read_serial,
        )

    def parse_data_line(self, line: str):
        parts = line.split(",")

        if len(parts) != 17:
            return None

        if parts[0] != "DATA":
            return None

        try:
            data = {
                "time_ms": int(parts[1]),
                "left_ticks": int(parts[2]),
                "right_ticks": int(parts[3]),

                "gyro_x_radps": float(parts[4]),
                "gyro_y_radps": float(parts[5]),
                "gyro_z_radps": float(parts[6]),

                "accel_x": float(parts[7]),
                "accel_y": float(parts[8]),
                "accel_z": float(parts[9]),

                "yaw_deg": float(parts[10]),
                "pitch_deg": float(parts[11]),
                "roll_deg": float(parts[12]),

                "cal_sys": int(parts[13]),
                "cal_gyro": int(parts[14]),
                "cal_accel": int(parts[15]),
                "cal_mag": int(parts[16]),
            }
        except ValueError:
            return None

        if self.invert_gyro_z:
            data["gyro_z_radps"] *= -1.0

        if self.invert_imu_yaw:
            data["yaw_deg"] *= -1.0

        return data

    def read_serial(self):
        try:
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

        self.update_odometry_and_imu(data)

    def update_odometry_and_imu(self, data):
        if self.previous_data is None:
            self.previous_data = data
            self.initial_imu_yaw_rad = math.radians(data["yaw_deg"])

            self.get_logger().info("Received first DATA line. Initialized.")
            return

        current_time_ms = data["time_ms"]
        previous_time_ms = self.previous_data["time_ms"]

        dt = (current_time_ms - previous_time_ms) / 1000.0

        if dt <= 0.0:
            self.get_logger().warn(f"Invalid dt={dt}. Skipping update.")
            return

        delta_left_ticks = data["left_ticks"] - self.previous_data["left_ticks"]
        delta_right_ticks = data["right_ticks"] - self.previous_data["right_ticks"]

        delta_left_ticks *= self.left_tick_sign
        delta_right_ticks *= self.right_tick_sign

        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m

        left_speed_mps = left_distance_m / dt
        right_speed_mps = right_distance_m / dt

        linear_velocity_mps = (left_speed_mps + right_speed_mps) / 2.0

        wheel_angular_velocity_radps = (
            right_speed_mps - left_speed_mps
        ) / self.wheel_base_m

        previous_yaw_rad = self.yaw_rad

        self.yaw_rad = normalize_angle(
            self.yaw_rad + wheel_angular_velocity_radps * dt
        )

        distance_m = (left_distance_m + right_distance_m) / 2.0

        heading_delta = normalize_angle(self.yaw_rad - previous_yaw_rad)
        heading_mid = normalize_angle(previous_yaw_rad + heading_delta / 2.0)

        self.x += distance_m * math.cos(heading_mid)
        self.y += distance_m * math.sin(heading_mid)

        stamp = self.get_clock().now().to_msg()

        if self.publish_wheel_odom_enabled:
            self.publish_wheel_odom(
                stamp,
                linear_velocity_mps,
                wheel_angular_velocity_radps,
            )

        if self.publish_tf_enabled:
            self.publish_tf(stamp)

        if self.publish_imu_enabled:
            self.publish_imu(stamp, data)

        self.previous_data = data
        self.update_count += 1

        if self.log_every_n > 0 and self.update_count % self.log_every_n == 0:
            imu_yaw_relative = normalize_angle(
                math.radians(data["yaw_deg"]) - self.initial_imu_yaw_rad
            )

            self.get_logger().info(
                f"wheel_odom: x={self.x:.3f}, y={self.y:.3f}, "
                f"wheel_yaw={math.degrees(self.yaw_rad):.2f} deg, "
                f"imu_yaw_rel={math.degrees(imu_yaw_relative):.2f} deg, "
                f"v={linear_velocity_mps:.3f} m/s, "
                f"wheel_w={wheel_angular_velocity_radps:.3f} rad/s, "
                f"gyro_z={data['gyro_z_radps']:.3f} rad/s, "
                f"dL={delta_left_ticks}, dR={delta_right_ticks}, "
                f"cal={data['cal_sys']}/{data['cal_gyro']}/"
                f"{data['cal_accel']}/{data['cal_mag']}"
            )

    def publish_wheel_odom(
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

        # Covariance order:
        # x, y, z, roll, pitch, yaw
        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[14] = 99999.0
        odom_msg.pose.covariance[21] = 99999.0
        odom_msg.pose.covariance[28] = 99999.0
        odom_msg.pose.covariance[35] = 0.20

        odom_msg.twist.covariance[0] = 0.10
        odom_msg.twist.covariance[7] = 99999.0
        odom_msg.twist.covariance[14] = 99999.0
        odom_msg.twist.covariance[21] = 99999.0
        odom_msg.twist.covariance[28] = 99999.0
        odom_msg.twist.covariance[35] = 0.20

        self.wheel_odom_pub.publish(odom_msg)

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

    def publish_imu(self, stamp, data):
        imu_msg = Imu()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame

        imu_yaw_rad = normalize_angle(
            math.radians(data["yaw_deg"]) - self.initial_imu_yaw_rad
        )
        roll_rad = math.radians(data["roll_deg"])
        pitch_rad = math.radians(data["pitch_deg"])

        imu_msg.orientation = euler_to_quaternion(
            roll_rad,
            pitch_rad,
            imu_yaw_rad,
        )

        imu_msg.angular_velocity.x = data["gyro_x_radps"]
        imu_msg.angular_velocity.y = data["gyro_y_radps"]
        imu_msg.angular_velocity.z = data["gyro_z_radps"]

        imu_msg.linear_acceleration.x = data["accel_x"]
        imu_msg.linear_acceleration.y = data["accel_y"]
        imu_msg.linear_acceleration.z = data["accel_z"]

        # We publish orientation, but do not fuse orientation in EKF at first.
        imu_msg.orientation_covariance[0] = 0.50
        imu_msg.orientation_covariance[4] = 0.50
        imu_msg.orientation_covariance[8] = 1.00

        imu_msg.angular_velocity_covariance[0] = 0.10
        imu_msg.angular_velocity_covariance[4] = 0.10
        imu_msg.angular_velocity_covariance[8] = 0.05

        imu_msg.linear_acceleration_covariance[0] = 0.50
        imu_msg.linear_acceleration_covariance[4] = 0.50
        imu_msg.linear_acceleration_covariance[8] = 0.50

        self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")

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