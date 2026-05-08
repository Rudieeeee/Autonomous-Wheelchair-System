import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")

        self.publisher = self.create_publisher(Point, "/goal_point", 10)

        self.timer = self.create_timer(0.001, self.publish_goal)

        self.get_logger().info("Goal publisher started")

    def publish_goal(self):
        msg = Point()
        msg.x = 2.0
        msg.y = 1.5
        msg.z = 0.0

        self.publisher.publish(msg)

        self.get_logger().info(f"Published goal: x={msg.x}, y={msg.y}")


def main(args=None):
    rclpy.init(args=args)

    node = GoalPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()