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
        self.declare_parameter('kP', -3)
        self.declare_parameter('kD', -8)

        self.prev_error = 0.0
        self.prev_time = None

        self.prev_d = 0.0
        self.alpha = 0.3

        self.latest_teleop_cmd = Twist()

        self.pub = self.create_publisher(Twist, '/cmd_vel_bb', 10)
        self.teleop_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.teleop_callback,
            10
        )

        # just a test publisher to see my node
        self.debug = self.create_publisher(String, '/stb_debug', 10)

        # buffer + listener fot TF
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.timer = self.create_timer(0.1, self._tick)
    
    def teleop_callback(self, msg: Twist):
        self.latest_teleop_cmd = msg

    def _tick(self):

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

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        forward_vel = self.latest_teleop_cmd.linear.x
        angular_vel = self.latest_teleop_cmd.angular.z
        adjustment_vel, p_term, d_term = self.computePID(lean_angle, 0.05)
        cmd = Twist()
        cmd.linear.x = adjustment_vel
        cmd.angular.z = angular_vel
        self.pub.publish(cmd)

        debug_msg = String()
        debug_msg.data = f"P: {p_term:.3f}, D: {d_term:.3f}, OUT: {adjustment_vel:.3f}"

        self.debug.publish(debug_msg)

    
    def computePID(self, current, target):
        kp = float(self.get_parameter('kP').value)
        kd = float(self.get_parameter('kD').value)

        error = target - current
        
        now = self.get_clock().now().nanoseconds / 1e9
        if self.prev_time is None:
            dt = 0.1 
        else:
            dt = now - self.prev_time
            if dt <= 0:
                dt = 1e-3


        p_term = kp * error

        d_raw = (error - self.prev_error) / dt

        # low-pass filter
        d_term = kd * (self.alpha * d_raw + (1 - self.alpha) * self.prev_d)

        self.prev_d = (self.alpha * d_raw + (1 - self.alpha) * self.prev_d)

        self.prev_error = error
        self.prev_time = now

        output = p_term + d_term
        return output, p_term, d_term 


def main():
    rclpy.init()
    node = StbPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
