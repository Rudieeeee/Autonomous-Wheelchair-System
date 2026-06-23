import math
import statistics
import time
from collections import deque
from operator import sub

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray, String

try:
    import serial
except ImportError:
    serial = None


class UwbSerialNode(Node):
    def __init__(self):
        super().__init__('uwb_serial_node')

        self.declare_parameter('serial_port', '/dev/arduino_uwb')
        self.declare_parameter('baud_rate', 115200)

        self.declare_parameter('anchor_1_id', '1111')
        self.declare_parameter('anchor_2_id', '2222')
        self.declare_parameter('anchor_3_id', '3333')

        self.declare_parameter('anchor_1_x', 0.0)
        self.declare_parameter('anchor_1_y', 0.0)
        self.declare_parameter('anchor_2_x', 1.0)
        self.declare_parameter('anchor_2_y', 0.0)
        self.declare_parameter('anchor_3_x', 0.0)
        self.declare_parameter('anchor_3_y', 0.8)

        self.declare_parameter('min_distance_m', 0.05)
        self.declare_parameter('max_distance_m', 8.0)
        self.declare_parameter('max_jump_m', 1.0)
        self.declare_parameter('filter_window', 5)
        self.declare_parameter('range_timeout_s', 1.0)

        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('debug_print', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)

        self.anchor_ids = [
            str(self.get_parameter('anchor_1_id').value),
            str(self.get_parameter('anchor_2_id').value),
            str(self.get_parameter('anchor_3_id').value),
        ]

        self.anchor_positions = {
            self.anchor_ids[0]: (
                float(self.get_parameter('anchor_1_x').value),
                float(self.get_parameter('anchor_1_y').value),
            ),
            self.anchor_ids[1]: (
                float(self.get_parameter('anchor_2_x').value),
                float(self.get_parameter('anchor_2_y').value),
            ),
            self.anchor_ids[2]: (
                float(self.get_parameter('anchor_3_x').value),
                float(self.get_parameter('anchor_3_y').value),
            ),
        }

        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.max_distance_m = float(self.get_parameter('max_distance_m').value)
        self.max_jump_m = float(self.get_parameter('max_jump_m').value)
        self.filter_window = int(self.get_parameter('filter_window').value)
        self.range_timeout_s = float(self.get_parameter('range_timeout_s').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.debug_print = bool(self.get_parameter('debug_print').value)

        self.buffers = {
            anchor_id: deque(maxlen=self.filter_window)
            for anchor_id in self.anchor_ids
        }

        self.filtered_ranges = {
            anchor_id: None
            for anchor_id in self.anchor_ids
        }

        self.last_update_time = {
            anchor_id: 0.0
            for anchor_id in self.anchor_ids
        }

        self.serial_connection = None
        self.rx_buffer = ''

        self.ranges_pub = self.create_publisher(
            Float32MultiArray,
            '/uwb/ranges',
            10,
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            '/uwb/target_base',
            10,
        )

        self.polar_pub = self.create_publisher(
            Vector3Stamped,
            '/uwb/target_polar',
            10,
        )

        self.debug_pub = self.create_publisher(
            String,
            '/uwb/debug',
            10,
        )

        self.open_serial()

        self.read_timer = self.create_timer(0.02, self.read_serial)
        self.publish_timer = self.create_timer(0.10, self.publish_target)

    def open_serial(self):
        if serial is None:
            self.get_logger().error('pyserial is not installed')
            return

        try:
            self.serial_connection = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.0,
            )
            self.get_logger().info(
                f'Opened UWB serial port {self.serial_port} at {self.baud_rate}'
            )
        except serial.SerialException as error:
            self.serial_connection = None
            self.get_logger().error(
                f'Could not open UWB serial port {self.serial_port}: {error}'
            )

    def read_serial(self):
        if self.serial_connection is None:
            return

        try:
            waiting = self.serial_connection.in_waiting
            if waiting == 0:
                return

            data = self.serial_connection.read(waiting).decode(
                errors='ignore'
            )

        except serial.SerialException as error:
            self.get_logger().error(f'UWB serial read failed: {error}')
            self.serial_connection = None
            return

        for char in data:
            if char == '\n':
                line = self.rx_buffer.strip()
                self.rx_buffer = ''

                if line:
                    self.handle_line(line)

            elif char != '\r':
                self.rx_buffer += char

    def handle_line(self, line):
        parsed = self.parse_line(line)

        if parsed is None:
            return

        anchor_id, distance_m = parsed

        if anchor_id not in self.anchor_ids:
            return

        if distance_m < self.min_distance_m:
            return

        if distance_m > self.max_distance_m:
            return

        previous = self.filtered_ranges[anchor_id]

        if previous is not None:
            jump = abs(sub(distance_m, previous))

            if jump > self.max_jump_m:
                self.publish_debug(
                    f'rejected jump anchor {anchor_id}: {distance_m:.2f} m'
                )
                return

        self.buffers[anchor_id].append(distance_m)

        filtered = statistics.median(self.buffers[anchor_id])
        self.filtered_ranges[anchor_id] = filtered
        self.last_update_time[anchor_id] = time.time()

        if self.debug_print:
            self.publish_debug(
                f'anchor {anchor_id}: raw {distance_m:.2f} m filtered {filtered:.2f} m'
            )

    def parse_line(self, line):
        parts = line.split(',')

        if len(parts) < 3:
            return None

        if parts[0] != 'UWB':
            return None

        anchor_id = parts[1].strip()
        distance_cm_text = parts[2].strip()

        try:
            distance_cm = float(distance_cm_text)
        except ValueError:
            return None

        distance_m = distance_cm / 100.0

        return anchor_id, distance_m

    def publish_target(self):
        now = time.time()

        for anchor_id in self.anchor_ids:
            last_update = self.last_update_time[anchor_id]

            if sub(now, last_update) > self.range_timeout_s:
                return

            if self.filtered_ranges[anchor_id] is None:
                return

        r1 = self.filtered_ranges[self.anchor_ids[0]]
        r2 = self.filtered_ranges[self.anchor_ids[1]]
        r3 = self.filtered_ranges[self.anchor_ids[2]]

        point = self.trilaterate(r1, r2, r3)

        if point is None:
            self.publish_debug('trilateration failed')
            return

        x, y = point

        self.publish_ranges(r1, r2, r3)
        self.publish_point(x, y)
        self.publish_polar(x, y)

    def trilaterate(self, r1, r2, r3):
        id1 = self.anchor_ids[0]
        id2 = self.anchor_ids[1]
        id3 = self.anchor_ids[2]

        x1, y1 = self.anchor_positions[id1]
        x2, y2 = self.anchor_positions[id2]
        x3, y3 = self.anchor_positions[id3]

        a = 2.0 * sub(x2, x1)
        b = 2.0 * sub(y2, y1)

        c = (
            r1 * r1
            + x2 * x2
            + y2 * y2
            + sub(0.0, r2 * r2)
            + sub(0.0, x1 * x1)
            + sub(0.0, y1 * y1)
        )

        d = 2.0 * sub(x3, x1)
        e = 2.0 * sub(y3, y1)

        f = (
            r1 * r1
            + x3 * x3
            + y3 * y3
            + sub(0.0, r3 * r3)
            + sub(0.0, x1 * x1)
            + sub(0.0, y1 * y1)
        )

        determinant = a * e + sub(0.0, b * d)

        if abs(determinant) < 0.001:
            return None

        x = (c * e + sub(0.0, b * f)) / determinant
        y = (a * f + sub(0.0, c * d)) / determinant

        return x, y

    def publish_ranges(self, r1, r2, r3):
        msg = Float32MultiArray()
        msg.data = [float(r1), float(r2), float(r3)]
        self.ranges_pub.publish(msg)

    def publish_point(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.target_pub.publish(msg)

    def publish_polar(self, x, y):
        distance = math.sqrt(x * x + y * y)
        angle = math.atan2(y, x)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.vector.x = float(distance)
        msg.vector.y = float(angle)
        msg.vector.z = 0.0
        self.polar_pub.publish(msg)

    def publish_debug(self, text):
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.serial_connection is not None:
        node.serial_connection.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()