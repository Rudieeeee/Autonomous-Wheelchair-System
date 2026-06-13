#!/usr/bin/env python3

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:
    serial = None


class Tof64ScanNode(Node):
    def __init__(self):
        super().__init__("tof64_scan_node")

        self.declare_parameter("serial_port", "/dev/arduino_wheelchair")
        self.declare_parameter("baudrate", 115200)

        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("scan_topic", "/tof_scan")

        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 4.0)

        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))

        self.declare_parameter("use_inf_for_empty_bins", True)

        # Flat format:
        # [x0, y0, yaw0, x1, y1, yaw1, ... x7, y7, yaw7]
        self.declare_parameter(
            "sensor_poses",
            [
                0.20,  0.15,  0.0,
                0.20,  0.05,  0.0,
                0.20, -0.05,  0.0,
                0.10, -0.18, -1.5708,
                0.00, -0.18, -1.5708,
               -0.10, -0.18, -1.5708,
               -0.20, -0.08,  3.1416,
               -0.20,  0.08,  3.1416,
            ],
        )

        self.declare_parameter(
            "column_angles_deg",
            [-22.5, -16.1, -9.6, -3.2, 3.2, 9.6, 16.1, 22.5],
        )

        self.serial_port = self.get_parameter("serial_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)

        self.base_frame = self.get_parameter("base_frame").value
        self.scan_topic = self.get_parameter("scan_topic").value

        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)

        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)

        self.use_inf_for_empty_bins = bool(
            self.get_parameter("use_inf_for_empty_bins").value
        )

        sensor_poses_flat = self.get_parameter("sensor_poses").value
        self.column_angles_deg = self.get_parameter("column_angles_deg").value
        self.column_angles_rad = [
            math.radians(float(a)) for a in self.column_angles_deg
        ]

        if len(sensor_poses_flat) != 24:
            raise RuntimeError(
                "sensor_poses must contain exactly 24 numbers: "
                "[x0, y0, yaw0, x1, y1, yaw1, ... x7, y7, yaw7]"
            )

        self.sensor_poses = []
        for i in range(0, len(sensor_poses_flat), 3):
            self.sensor_poses.append(
                (
                    float(sensor_poses_flat[i]),
                    float(sensor_poses_flat[i + 1]),
                    float(sensor_poses_flat[i + 2]),
                )
            )

        if len(self.sensor_poses) != 8:
            raise RuntimeError("sensor_poses must contain exactly 8 sensor poses")

        if len(self.column_angles_rad) != 8:
            raise RuntimeError("column_angles_deg must contain exactly 8 angles")

        self.scan_bins = int(
            math.ceil((self.angle_max - self.angle_min) / self.angle_increment)
        )

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.running = True
        self.serial_handle = None
        self.serial_thread = None

        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. Install with: sudo apt install python3-serial"
            )

        self.open_serial()

        self.get_logger().info(
            f"Reading TOF64 from {self.serial_port} at {self.baudrate} baud"
        )
        self.get_logger().info(
            f"Publishing combined LaserScan on {self.scan_topic} "
            f"in frame {self.base_frame}"
        )

    def open_serial(self):
        self.serial_handle = serial.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=0.1,
        )

        self.serial_thread = threading.Thread(
            target=self.serial_loop,
            daemon=True,
        )
        self.serial_thread.start()

    def serial_loop(self):
        while self.running and rclpy.ok():
            try:
                raw = self.serial_handle.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                # Existing localization line. Ignore here.
                if line.startswith("DATA"):
                    continue

                if not line.startswith("TOF64"):
                    continue

                decoded = self.parse_tof64_line(line)
                if decoded is None:
                    self.get_logger().warn(f"Bad TOF64 line: {line[:80]}")
                    continue

                self.publish_scan(decoded)

            except Exception as exc:
                self.get_logger().error(f"Serial read error: {exc}")

    def parse_tof64_line(self, line: str) -> Optional[dict]:
        parts = line.split(",")

        if len(parts) != 4:
            return None

        if parts[0] != "TOF64":
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

            points.append(
                {
                    "tof_id": tof_id,
                    "column": column,
                    "distance_m": distance_mm / 1000.0,
                }
            )

        return {
            "time_ms": time_ms,
            "seq": seq,
            "points": points,
        }

    def publish_scan(self, decoded: dict):
        stamp = self.get_clock().now().to_msg()

        empty_value = math.inf if self.use_inf_for_empty_bins else float("nan")
        ranges = [empty_value for _ in range(self.scan_bins)]

        for p in decoded["points"]:
            tof_id = p["tof_id"]
            col = p["column"]
            distance_m = p["distance_m"]

            if tof_id < 0 or tof_id >= len(self.sensor_poses):
                continue

            if col < 0 or col >= len(self.column_angles_rad):
                continue

            if distance_m < self.range_min or distance_m > self.range_max:
                continue

            sensor_x, sensor_y, sensor_yaw = self.sensor_poses[tof_id]

            local_angle = self.column_angles_rad[col]
            global_angle = float(sensor_yaw) + local_angle

            # Point in base_footprint.
            point_x = float(sensor_x) + distance_m * math.cos(global_angle)
            point_y = float(sensor_y) + distance_m * math.sin(global_angle)

            # Convert point to LaserScan polar coordinates around base_footprint.
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

            # Keep nearest obstacle if multiple ToF points fall into the same bin.
            if math.isinf(current) or math.isnan(current) or scan_range < current:
                ranges[bin_index] = scan_range

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.scan_bins * self.angle_increment
        scan.angle_increment = self.angle_increment

        scan.time_increment = 0.0
        scan.scan_time = 0.1

        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)

    def destroy_node(self):
        self.running = False

        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Tof64ScanNode()

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