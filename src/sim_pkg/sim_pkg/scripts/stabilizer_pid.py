#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


class StbPID(Node):
    def __init__(self):
        super().__init__('stb_pid')

        # PID params
        self.declare_parameter('kP', 0.5)
        self.declare_parameter('kD', 0.0)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # just a test publisher to see my node
        self.debug = self.create_publisher(String, '/stb_debug', 10)

        # buffer + listener fot TF
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):

        kp = float(self.get_parameter('kP').value)
        kd = float(self.get_parameter('kD').value)

        # Try to lookup transform
        try:
            t = self.buffer.lookup_transform(
                'World',        
                'base_extrusion', 
                rclpy.time.Time()
            )

            #get angles via quaternion
            q = t.transform.rotation
            quat = [q.x, q.y, q.z, q.w]

            #conver tt rpy
            roll, pitch, yaw = euler_from_quaternion(quat)

            lean_angle = roll

            debug_msg = String()
            debug_msg.data = f"Lean angle: {lean_angle:.4f} rad  ({lean_angle*180/3.14159:.2f}°)"
            self.debug.publish(debug_msg)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        cmd = Twist()
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = StbPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
