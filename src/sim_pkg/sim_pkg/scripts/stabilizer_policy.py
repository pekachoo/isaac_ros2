#!/usr/bin/env python3
from __future__ import annotations

import math
import torch
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String

ACTION_SCALE = 60.0
MAX_CMD = 999.0
MAX_DV_PER_S = 10000000.0
CTRL_HZ = 60.0

ACTIVATE_ANGLE_DEG = 0.0
DEACTIVATE_ANGLE_DEG = 1.0
DEACTIVATE_HOLD_SEC = 99999.0
ZERO_CMD_WHEN_INACTIVE = True  # temp


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
        self.latest_imu: Imu | None = None

        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10
        )
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)

        self.debug_pub = self.create_publisher(String, "/bb_policy_debug", 10)
        self.cmd_pub = self.create_publisher(JointState, "/wheel_vel_cmd", 10)

        self.ctrl_dt = 1.0 / CTRL_HZ
        self.prev_cmd_left = 0.0
        self.prev_cmd_right = 0.0

        t0 = self.get_clock().now()
        self.prev_step_t = t0
        self.prev_lim_t = t0

        # thresholds in radians
        self.on_rad = math.radians(ACTIVATE_ANGLE_DEG)
        self.off_rad = math.radians(DEACTIVATE_ANGLE_DEG)

        self.policy_active = False
        self.below_off_since = None

        self.timer = self.create_timer(self.ctrl_dt, self.control_step)

    def joint_state_cb(self, msg: JointState):
        self.latest_js = msg

    def imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def read_obs_from_imu(self) -> float | None:
        if self.latest_imu is None:
            return None

        imu = self.latest_imu
        q = imu.orientation
        wz = imu.angular_velocity.z 

        orientation_nonzero = not (q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0)

        if not orientation_nonzero:
            return None
        
        # For +X=DOWN, +Y=FORWARD, +Z=LEFT
        sinp = 2.0 * (q.w * q.x + q.y * q.z)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        
        return float(pitch), float(wz)
    
    def publish_zero_cmd(self):
        self.prev_cmd_left = 0.0
        self.prev_cmd_right = 0.0

        t = self.get_clock().now()
        self.prev_step_t = t
        self.prev_lim_t = t

        out = JointState()
        out.header.stamp = t.to_msg()
        out.name = [self.left_joint, self.right_joint]
        out.velocity = [0.0, 0.0]
        out.position = [0.0, 0.0]
        out.effort = [0.0, 0.0]
        self.cmd_pub.publish(out)

    def control_step(self):
        now = self.get_clock().now()
        step_dt = (now - self.prev_step_t).nanoseconds * 1e-9
        self.prev_step_t = now

        if self.latest_js is None or self.latest_imu is None:
            return

        pitch, wz = self.read_obs_from_imu()
        if pitch is None:
            return
        print(wz) 

        abs_pitch = abs(pitch)

        # activation gating
        if not self.policy_active and abs_pitch >= self.on_rad:
            self.policy_active = True
            self.below_off_since = None

        if self.policy_active:
            if abs_pitch <= self.off_rad:
                if self.below_off_since is None:
                    self.below_off_since = now
                else:
                    dt_below = (now - self.below_off_since).nanoseconds * 1e-9
                    if dt_below >= DEACTIVATE_HOLD_SEC:
                        self.policy_active = False
                        self.below_off_since = None
                        if ZERO_CMD_WHEN_INACTIVE:
                            self.publish_zero_cmd()
                        dbg = String()
                        dbg.data = (
                            f"DEACTIVATED | pitch={pitch:+.3f} rad ({math.degrees(pitch):+.2f} deg)"
                        )
                        self.debug_pub.publish(dbg)
                        return
            else:
                self.below_off_since = None

        if not self.policy_active:
            if ZERO_CMD_WHEN_INACTIVE:
                self.publish_zero_cmd()
            dbg = String()
            dbg.data = (
                f"INACTIVE | pitch={pitch:+.3f} rad ({math.degrees(pitch):+.2f} deg) "
                f"on={ACTIVATE_ANGLE_DEG:.1f} off={DEACTIVATE_ANGLE_DEG:.1f} hold={DEACTIVATE_HOLD_SEC:.1f}s"
            )
            self.debug_pub.publish(dbg)
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

        # Policy input
        obs = torch.tensor([[left_vel, right_vel, pitch, wz]], dtype=torch.float32)

        with torch.no_grad():
            act = self.policy(obs)

        if isinstance(act, torch.Tensor):
            act = act.view(-1)
        else:
            act = act[0].view(-1)

        if act.numel() < 2:
            self.get_logger().error(f"Policy returned {act.numel()} actions, expected 2")
            return

        # action -> command
        act_normalized = torch.tanh(act)
        a_left = float(act_normalized[0].item())
        a_right = float(act_normalized[1].item())

        cmd_left = max(-MAX_CMD, min(MAX_CMD, a_left * ACTION_SCALE))
        cmd_right = max(-MAX_CMD, min(MAX_CMD, a_right * ACTION_SCALE))

        # limiter dt
        dt = (now - self.prev_lim_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = self.ctrl_dt
        self.prev_lim_t = now

        max_dv = MAX_DV_PER_S * dt
        cmd_left = max(self.prev_cmd_left - max_dv, min(self.prev_cmd_left + max_dv, cmd_left))
        cmd_right = max(self.prev_cmd_right - max_dv, min(self.prev_cmd_right + max_dv, cmd_right))

        self.prev_cmd_left = cmd_left
        self.prev_cmd_right = cmd_right

        out = JointState()
        out.header.stamp = now.to_msg()
        out.name = [self.left_joint, self.right_joint]
        # keep your original sign convention
        out.velocity = [cmd_left, -cmd_right]
        out.position = [0.0, 0.0]
        out.effort = [0.0, 0.0]
        self.cmd_pub.publish(out)
        # print(pitch)
        dbg = String()
        dbg.data = (
            f"ACTIVE | pitch={pitch:+.3f} rad ({math.degrees(pitch):+.2f} deg) "
            f"left_vel={left_vel:+.3f} right_vel={right_vel:+.3f} "
            f"a=[{a_left:+.3f},{a_right:+.3f}] cmd=[{cmd_left:+.1f},{cmd_right:+.1f}]"
        )
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = BracketBotPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
