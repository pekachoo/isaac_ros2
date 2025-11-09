#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BoxCmd(Node):
    def __init__(self):
        super().__init__('box_cmd')
        self.declare_parameter('v', 0.5)   # m/s
        self.declare_parameter('w', 0.0)   # rad/s
        self.pub = self.create_publisher(Twist, '/box/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz

    def _tick(self):
        v = float(self.get_parameter('v').value)
        w = float(self.get_parameter('w').value)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(BoxCmd())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
