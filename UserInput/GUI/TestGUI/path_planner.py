#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

import matplotlib.pyplot as plt


class PathPlotter(Node):
    def __init__(self):
        super().__init__("path_plotter")

        self.subscription = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )

        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.get_logger().info("Listening to /plan and plotting path...")

    def path_callback(self, msg):
        x_points = []
        y_points = []

        for pose_stamped in msg.poses:
            x_points.append(pose_stamped.pose.position.x)
            y_points.append(pose_stamped.pose.position.y)

        if len(x_points) == 0:
            self.get_logger().warn("Received empty path")
            return

        self.ax.clear()
        self.ax.plot(x_points, y_points, marker="o")
        self.ax.set_title("Nav2 Global Path /plan")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.axis("equal")
        self.ax.grid(True)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.get_logger().info(f"Plotted path with {len(x_points)} points")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()