#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw_rad):
    """
    Convert yaw angle in radians to a ROS quaternion.
    Roll and pitch are assumed to be zero for 2D odometry.
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def normalize_angle(angle):
    """
    Keep angle between -pi and +pi.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class SerialOdomNode(Node):
    def __init__(self):
        super().__init__("serial_odom_node")

        # -------------------- Parameters --------------------
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)

        # Wheel parameters
        self.declare_parameter("wheel_diameter_m", 0.35)
        self.declare_parameter("magnets_per_wheel", 4)
        self.declare_parameter("wheel_base_m", 0.55)

        # Frame names
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        # Whether to use IMU yaw for heading
        # True  = heading comes from BNO055 yaw
        # False = heading comes from wheel encoder integration
        self.declare_parameter("use_imu_yaw", True)

        # If your left/right wheel direction is reversed, change these.
        self.declare_parameter("left_tick_sign", 1.0)
        self.declare_parameter("right_tick_sign", 1.0)

        # Main loop rate for reading serial
        self.declare_parameter("timer_period_s", 0.01)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)

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

        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)

        self.left_tick_sign = float(
            self.get_parameter("left_tick_sign").value
        )
        self.right_tick_sign = float(
            self.get_parameter("right_tick_sign").value
        )

        self.timer_period_s = float(
            self.get_parameter("timer_period_s").value
        )

        # -------------------- Derived values --------------------
        self.wheel_circumference_m = math.pi * self.wheel_diameter_m

        self.distance_per_tick_m = (
            self.wheel_circumference_m / self.magnets_per_wheel
        )

        self.get_logger().info(
            f"wheel_diameter_m={self.wheel_diameter_m}"
        )
        self.get_logger().info(
            f"wheel_circumference_m={self.wheel_circumference_m}"
        )
        self.get_logger().info(
            f"magnets_per_wheel={self.magnets_per_wheel}"
        )
        self.get_logger().info(
            f"distance_per_tick_m={self.distance_per_tick_m}"
        )
        self.get_logger().info(
            f"wheel_base_m={self.wheel_base_m}"
        )
        self.get_logger().info(
            f"use_imu_yaw={self.use_imu_yaw}"
        )

        # -------------------- ROS publishers --------------------
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -------------------- Robot state --------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0

        self.previous_data = None
        self.initial_imu_yaw_rad = None

        # -------------------- Serial connection --------------------
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.01,
            )
        except serial.SerialException as error:
            self.get_logger().error(
                f"Could not open serial port {self.serial_port}: {error}"
            )
            raise error

        self.get_logger().info(
            f"Connected to {self.serial_port} at {self.baud_rate} baud"
        )

        # -------------------- Timer --------------------
        self.timer = self.create_timer(
            self.timer_period_s,
            self.read_serial,
        )

    def parse_data_line(self, line):
        """
        Expected Arduino line:

        DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg

        Example:

        DATA,1200,15,14,1,1,184.812,-1.125,0.875
        """

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
        """
        Reads one serial line and processes it.
        """

        try:
            raw_line = self.ser.readline()
        except serial.SerialException as error:
            self.get_logger().error(f"Serial read error: {error}")
            return

        if not raw_line:
            return

        line = raw_line.decode("utf-8", errors="ignore").strip()

        if not line:
            return

        # Print status lines from Arduino, but do not use them for odom.
        if not line.startswith("DATA"):
            self.get_logger().info(line)
            return

        data = self.parse_data_line(line)

        if data is None:
            self.get_logger().warn(f"Could not parse line: {line}")
            return

        self.update_odometry(data)

    def update_odometry(self, data):
        """
        Converts encoder ticks + IMU yaw into odometry.
        """

        if self.previous_data is None:
            self.previous_data = data

            self.initial_imu_yaw_rad = math.radians(data["yaw_deg"])

            self.get_logger().info(
                "Received first data line. Odometry initialized."
            )
            return

        # -------------------- Time difference --------------------
        current_time_ms = data["time_ms"]
        previous_time_ms = self.previous_data["time_ms"]

        dt = (current_time_ms - previous_time_ms) / 1000.0

        if dt <= 0.0:
            self.get_logger().warn(
                f"Invalid dt={dt}. Skipping odometry update."
            )
            return

        # -------------------- Tick difference --------------------
        delta_left_ticks = (
            data["left_ticks"] - self.previous_data["left_ticks"]
        )

        delta_right_ticks = (
            data["right_ticks"] - self.previous_data["right_ticks"]
        )

        # Apply direction signs
        delta_left_ticks *= self.left_tick_sign
        delta_right_ticks *= self.right_tick_sign

        # -------------------- Convert ticks to distance --------------------
        left_distance_m = delta_left_ticks * self.distance_per_tick_m
        right_distance_m = delta_right_ticks * self.distance_per_tick_m

        # -------------------- Wheel speeds --------------------
        left_speed_mps = left_distance_m / dt
        right_speed_mps = right_distance_m / dt

        # -------------------- Robot linear and angular velocity --------------------
        linear_velocity_mps = (
            left_speed_mps + right_speed_mps
        ) / 2.0

        angular_velocity_radps_from_wheels = (
            right_speed_mps - left_speed_mps
        ) / self.wheel_base_m

        # -------------------- Heading --------------------
        previous_yaw_rad = self.yaw_rad

        if self.use_imu_yaw:
            imu_yaw_rad = math.radians(data["yaw_deg"])

            # Make yaw start at zero when the node starts
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

        # -------------------- Position integration --------------------
        distance_m = (left_distance_m + right_distance_m) / 2.0

        # Use midpoint heading for smoother integration
        heading_mid = normalize_angle(
            previous_yaw_rad + normalize_angle(self.yaw_rad - previous_yaw_rad) / 2.0
        )

        self.x += distance_m * math.cos(heading_mid)
        self.y += distance_m * math.sin(heading_mid)

        # -------------------- Publish ROS messages --------------------
        self.publish_odom(
            linear_velocity_mps,
            angular_velocity_radps,
        )

        self.previous_data = data

        self.get_logger().info(
            f"x={self.x:.3f}, y={self.y:.3f}, "
            f"yaw={math.degrees(self.yaw_rad):.2f} deg, "
            f"v={linear_velocity_mps:.3f} m/s, "
            f"w={angular_velocity_radps:.3f} rad/s, "
            f"dL={delta_left_ticks}, dR={delta_right_ticks}"
        )

    def publish_odom(self, linear_velocity_mps, angular_velocity_radps):
        """
        Publishes nav_msgs/Odometry and TF odom -> base_footprint.
        """

        stamp = self.get_clock().now().to_msg()
        quat = yaw_to_quaternion(self.yaw_rad)

        # -------------------- Odometry message --------------------
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

        # Simple covariance values.
        # You can tune these later.
        odom_msg.pose.covariance[0] = 0.05   # x
        odom_msg.pose.covariance[7] = 0.05   # y
        odom_msg.pose.covariance[35] = 0.10  # yaw

        odom_msg.twist.covariance[0] = 0.10   # linear x velocity
        odom_msg.twist.covariance[35] = 0.20  # angular z velocity

        self.odom_pub.publish(odom_msg)

        # -------------------- TF message --------------------
        tf_msg = TransformStamped()

        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = quat

        self.tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        """
        Close serial port cleanly when node stops.
        """

        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SerialOdomNode()

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