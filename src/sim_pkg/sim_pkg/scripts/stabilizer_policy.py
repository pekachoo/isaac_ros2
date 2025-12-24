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

ACTION_SCALE = 60.0
MAX_CMD = 999.0
MAX_DV_PER_S = 10000000.0
CTRL_HZ = 120.0


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

        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10
        )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_parent = "World"
        self.tf_child = "base_extrusion"

        self.debug_pub = self.create_publisher(String, "/bb_policy_debug", 10)
        self.cmd_pub = self.create_publisher(JointState, "/wheel_vel_cmd", 10)

        self.ctrl_dt = 1.0 / CTRL_HZ
        self.prev_cmd_left = 0.0
        self.prev_cmd_right = 0.0
        self.prev_t = self.get_clock().now()

        self.timer = self.create_timer(1.0 / CTRL_HZ, self.control_step)

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

        left_vel = float(msg.velocity[li]) if li < len(msg.velocity) else 0.0
        right_vel = float(msg.velocity[ri]) if ri < len(msg.velocity) else 0.0
        # print(left_vel)

        obs = torch.tensor(
            [[
                left_vel, right_vel,
                roll,
            ]],
            dtype=torch.float32,
        )
        # print(obs)
        # obs = obs * 0.0
        with torch.no_grad():
            act = self.policy(obs)
        print(act)
        if isinstance(act, torch.Tensor):
            act = act.view(-1)
        else:
            act = act[0].view(-1)

        if act.numel() < 2:
            self.get_logger().error(f"Policy returned {act.numel()} actions, expected 2")
            return

        # act_normalized = torch.tanh(act)
        act_normalized = act

        a_left = float(act_normalized[0].item())
        a_right = float(act_normalized[1].item())

        cmd_left = a_left * ACTION_SCALE
        cmd_right = a_right * ACTION_SCALE

        cmd_left = max(-MAX_CMD, min(MAX_CMD, cmd_left))
        cmd_right = max(-MAX_CMD, min(MAX_CMD, cmd_right))

        now = self.get_clock().now()
        dt = (now - self.prev_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = self.ctrl_dt

        max_dv = MAX_DV_PER_S * dt

        cmd_left = max(self.prev_cmd_left - max_dv, min(self.prev_cmd_left + max_dv, cmd_left))
        cmd_right = max(self.prev_cmd_right - max_dv, min(self.prev_cmd_right + max_dv, cmd_right))

        self.prev_cmd_left = cmd_left
        self.prev_cmd_right = cmd_right
        self.prev_t = now

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.left_joint, self.right_joint]

        out.velocity = [cmd_left, -cmd_right]
        print(out.velocity)
        out.position = [0.0, 0.0]
        out.effort = [0.0, 0.0]
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
