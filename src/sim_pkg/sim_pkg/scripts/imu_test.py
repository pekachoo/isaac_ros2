#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuPrinter(Node):
    def __init__(self):
        super().__init__('imu_printer')

        self.sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_cb,
            10
        )

    def imu_cb(self, msg: Imu):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        q = msg.orientation

        self.get_logger().info(
            f"ACC  [x y z] = [{a.x:+.3f}, {a.y:+.3f}, {a.z:+.3f}]  "
            f"GYRO [x y z] = [{g.x:+.3f}, {g.y:+.3f}, {g.z:+.3f}]  "
            f"QUAT [x y z w] = [{q.x:+.3f}, {q.y:+.3f}, {q.z:+.3f}, {q.w:+.3f}]"
        )

def main():
    rclpy.init()
    node = ImuPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
