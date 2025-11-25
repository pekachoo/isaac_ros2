#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CartEffortPublisher(Node):
    def __init__(self):
        super().__init__('cart_effort_pub')

        self.force_value = 15.0   # change this to whatever value you want!
        self.pub = self.create_publisher(Float32, 'cart_effort', 10)

        self.timer = self.create_timer(0.02, self.publish_force)

        self.get_logger().info(f"Publishing constant effort: {self.force_value} N on /cart_effort")

    def publish_force(self):
        msg = Float32()
        msg.data = self.force_value
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartEffortPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
