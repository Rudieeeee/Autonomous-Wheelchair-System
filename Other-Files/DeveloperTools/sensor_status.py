#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


class SensorStatusNode(Node):
    def __init__(self):
        super().__init__("sensor_status_node")

        # Topics based on your launch file + normal localization/navigation topics.
        self.required_topics = [
            "/scan_left",     # left_lidar remapping
            "/scan_right",    # right_lidar remapping
            "/tof_scan",      # tof64_scan_node output
            "/tf",            # static + dynamic transforms
            "/tf_static",
            "/odom",          # odometry from localization/wheel encoder node
            "/imu/data",      # IMU data
            "/map",           # map from SLAM/map server
            "/amcl_pose",     # AMCL localization output
            "/cmd_vel",       # navigation velocity command
        ]

        # These are useful possible outputs from merger/conversion.
        # The checker will show them only if they exist.
        self.optional_topics = [
            "/scan",
            "/merged_scan",
            "/cloud",
            "/merged_cloud",
        ]

        self.timeout_s = 2.0
        self.last_seen = {}
        self.counts = {}
        self.topic_types = {}
        self.subscriptions_by_topic = {}

        self.create_timer(1.0, self.discover_topics)
        self.create_timer(1.0, self.print_status)

        self.get_logger().info("Sensor Status started.")

    def qos_for_topic(self, topic):
        if topic == "/tf_static":
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

        # Best effort works well for sensor streams like LaserScan.
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    def discover_topics(self):
        available = dict(self.get_topic_names_and_types())
        all_topics_to_check = self.required_topics + self.optional_topics

        for topic in all_topics_to_check:
            if topic in self.subscriptions_by_topic:
                continue

            if topic not in available:
                continue

            msg_type_name = available[topic][0]

            try:
                msg_type = get_message(msg_type_name)
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not load message type for {topic}: {msg_type_name} ({exc})"
                )
                continue

            self.subscriptions_by_topic[topic] = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.sensor_callback(t),
                self.qos_for_topic(topic),
            )

            self.topic_types[topic] = msg_type_name
            self.last_seen[topic] = None
            self.counts[topic] = 0

            self.get_logger().info(f"Subscribed to {topic} [{msg_type_name}]")

    def sensor_callback(self, topic):
        self.last_seen[topic] = time.time()
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def status_line(self, topic, required=True):
        now = time.time()

        if topic not in self.subscriptions_by_topic:
            label = "NOT FOUND" if required else "optional not found"
            return f"{topic:18s} {label}"

        last = self.last_seen.get(topic)
        msg_type = self.topic_types.get(topic, "?")

        if last is None:
            return f"{topic:18s} NO DATA YET   type={msg_type}"

        age = now - last

        if age <= self.timeout_s:
            return f"{topic:18s} OK            last={age:4.1f}s  count={self.counts.get(topic, 0):6d}  type={msg_type}"

        return f"{topic:18s} STALE         last={age:4.1f}s  count={self.counts.get(topic, 0):6d}  type={msg_type}"

    def print_status(self):
        lines = []
        lines.append("")
        lines.append("========== SENSOR STATUS ==========")
        lines.append("Required topics:")

        for topic in self.required_topics:
            lines.append(self.status_line(topic, required=True))

        existing_optional = [
            topic for topic in self.optional_topics
            if topic in self.subscriptions_by_topic
        ]

        if existing_optional:
            lines.append("")
            lines.append("Optional detected topics:")
            for topic in existing_optional:
                lines.append(self.status_line(topic, required=False))

        lines.append("===================================")

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = SensorStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
