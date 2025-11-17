#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, JointState

class StbPID(Node):
    def __init__(self):
        super().__init__('stb_pid')
        self.declare_parameter('kP', 0.5)   # m/s
        self.declare_parameter('kD', 0.0)   # rad/s
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscriber(JointState, 'joint_states', 10)
        self.debug = self.create_publisher()
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        kp = float(self.get_parameter('kP').value)
        kd = float(self.get_parameter('kD').value)
        msg = JointState()
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(StbPID())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
