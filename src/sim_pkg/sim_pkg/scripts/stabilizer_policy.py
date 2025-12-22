#!/usr/bin/env python3
from __future__ import annotations

import math
import torch
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


# matching isaaclab scaling
ACTION_SCALE = 60.0

MAX_CMD = 80.0


class BracketBotPolicy(Node):
    def __init__(self):
        super().__init__("bracketbot_policy_node")

        self.left_joint = "drive_left"
        self.right_joint = "drive_right"

        policy_path = "/home/jliu/isaac_ws/src/sim_pkg/sim_pkg/assets/policies/policy.pt"
        self.policy = torch.jit.load(policy_path, map_location="cpu")
        self.policy.eval()
        self.get_logger().info(f"Loaded TorchScript policy: {policy_path}")

        self.latest_js: JointState | None = None
        self.latest_roll: float | None = None

        # joint state input
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10
        )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_parent = "World"
        self.tf_child = "base_extrusion"

        # debug + command outputs
        self.debug_pub = self.create_publisher(String, "/bb_policy_debug", 10)

        self.cmd_pub = self.create_publisher(JointState, "/wheel_vel_cmd", 10)

        # control loop
        self.timer = self.create_timer(1.0 / 60.0, self.control_step)

    def joint_state_cb(self, msg: JointState):
        self.latest_js = msg

    def read_roll_from_tf(self) -> float | None:
        try:
            t = self.buffer.lookup_transform(self.tf_parent, self.tf_child, rclpy.time.Time())
            q = t.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            return float(roll)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def control_step(self):
        if self.latest_js is None:
            return
        roll = self.read_roll_from_tf()
        if roll is None:
            return

        msg = self.latest_js
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        if self.left_joint not in name_to_idx or self.right_joint not in name_to_idx:
            self.get_logger().warn("Missing drive_left/drive_right in JointState")
            return

        li = name_to_idx[self.left_joint]
        ri = name_to_idx[self.right_joint]

        # wheel velocities
        left_vel = float(msg.velocity[li]) if li < len(msg.velocity) else 0.0
        right_vel = float(msg.velocity[ri]) if ri < len(msg.velocity) else 0.0

        #manually build tensor for one bb, dont need to worry abt more
        obs = torch.tensor(
            [[
                0.0, 0.0,        # joint_pos forced to 0
                left_vel, right_vel,  # joint_vel
                roll,            # roll
            ]],
            dtype=torch.float32,
        )

        with torch.no_grad():
            act = self.policy(obs)

        if isinstance(act, torch.Tensor):
            act = act.view(-1)
        else:
            act = act[0].view(-1)

        if act.numel() < 2:
            self.get_logger().error(f"Policy returned {act.numel()} actions, expected 2")
            return

        a_left = float(act[0].item())
        a_right = float(act[1].item())

        # match IsaacLab scaling
        cmd_left = a_left * ACTION_SCALE
        cmd_right = a_right * ACTION_SCALE

        # clamp
        cmd_left = max(-MAX_CMD, min(MAX_CMD, cmd_left))
        cmd_right = max(-MAX_CMD, min(MAX_CMD, cmd_right))

        # publish
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.left_joint, self.right_joint]

        out.velocity = [1, -1]
        out.position = [0.0, 0.0]
        out.effort = [0.0, 0.0]
        # print("publishing")
        self.cmd_pub.publish(out)

        dbg = String()
        dbg.data = f"roll={roll:+.3f} left_vel={left_vel:+.3f} right_vel={right_vel:+.3f} a=[{a_left:+.3f},{a_right:+.3f}] cmd=[{cmd_left:+.1f},{cmd_right:+.1f}]"
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = BracketBotPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
