import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class GoalSubscriber(Node):
    def __init__(self):
        super().__init__("goal_subscriber")

        self.subscription = self.create_subscription(
            Point,
            "/goal_point",
            self.goal_callback,
            10
        )

        self.get_logger().info("Goal subscriber started")

    def goal_callback(self, msg):
        self.get_logger().info(
            f"Received goal: x={msg.x}, y={msg.y}, z={msg.z}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GoalSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()