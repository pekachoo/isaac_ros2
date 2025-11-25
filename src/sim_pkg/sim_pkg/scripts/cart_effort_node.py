#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CartEffortPublisher(Node):
    def __init__(self):
        super().__init__('cart_effort_pub')

        self.force_value = -1000.0
        
        self.joint_name = "slider_to_cart"

        self.pub = self.create_publisher(JointState, 'cart_effort', 10)

        self.timer = self.create_timer(0.02, self.publish_effort)

    def publish_effort(self):
        msg = JointState()
        msg.name = [self.joint_name]
        msg.position = []  
        msg.velocity = []  
        msg.effort = [self.force_value]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartEffortPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
