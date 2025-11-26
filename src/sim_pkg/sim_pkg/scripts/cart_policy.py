#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String


class CartPolicy(Node):
    def __init__(self):
        super().__init__("double_pendulum_debug")

        self.cart_joint = "slider_to_cart"
        self.pole1_joint = "cart_to_pole"
        self.pole2_joint = "pole_to_pendulum"

        self.sub = self.create_subscription(
            JointState,
            "/joint_states_read",
            self.joint_state_cb,
            10,
        )

        self.debug_pub = self.create_publisher(String, "debug", 10)

    def joint_state_cb(self, msg: JointState):
        names = list(msg.name)
        pos = dict(zip(names, msg.position))
        vel = dict(zip(names, msg.velocity))

        try:
            cart_pos = pos[self.cart_joint]
            cart_vel = vel[self.cart_joint]

            pole1_pos = pos[self.pole1_joint]
            pole1_vel = vel[self.pole1_joint]

            pole2_pos = pos[self.pole2_joint]
            pole2_vel = vel[self.pole2_joint]
        except KeyError as e:
            self.get_logger().warn(f"Missing joint in JointState: {e}")
            return

        debug_text = (
            f"theta1={pole1_pos:.3f}, theta1_vel={pole1_vel:.3f}, "
            f"theta2={pole2_pos:.3f}, theta2_vel={pole2_vel:.3f}, "
            f"x={cart_pos:.3f}, x_dot={cart_vel:.3f}"
        )

        msg_out = String()
        msg_out.data = debug_text
        self.debug_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = CartPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
