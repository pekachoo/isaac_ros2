import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
import torch

# MUST match TwoJointEnvAdaptableCfg.action_scale
ACTION_SCALE = 9


class CartPolicy(Node):
    def __init__(self):
        super().__init__("cart_policy_node")

        # Joint names must match your robot
        self.cart_joint = "slider_to_cart"
        self.pole1_joint = "cart_to_pole"
        self.pole2_joint = "pole_to_pendulum"

        POLICY_PATH = "/home/jliu/isaac_ws/src/sim_pkg/sim_pkg/assets/policies/policy.pt"

        # cache latest joint state
        self.latest_js: JointState | None = None

        # load policy
        try:
            self.policy = torch.jit.load(POLICY_PATH, map_location="cpu")
            self.get_logger().info(f"Loaded TorchScript policy from: {POLICY_PATH}")
        except Exception as e:
            self.get_logger().warn(
                f"torch.jit.load failed ({e}), trying torch.load instead..."
            )
            self.policy = torch.load(POLICY_PATH, map_location="cpu")
            self.get_logger().info(f"Loaded policy with torch.load from: {POLICY_PATH}")

        self.policy.eval()

        # subscribe to joint states
        self.sub = self.create_subscription(
            JointState,
            "/joint_states_read",
            self.joint_state_cb,
            10,
        )

        # 60 Hz control loop
        self.timer = self.create_timer(1.0 / 60.0, self.control_step)

        self.debug_pub = self.create_publisher(String, "debug", 10)
        self.effort_pub = self.create_publisher(JointState, "cart_effort", 10)

        self.get_logger().info("CartPolicy node started, waiting for /joint_states_read")

    # --- callbacks ---

    def joint_state_cb(self, msg: JointState):
        """Just cache the most recent joint state."""
        self.latest_js = msg

    def control_step(self):
        """Run at ~60 Hz: build obs, run policy, publish effort."""
        if self.latest_js is None:
            return

        msg = self.latest_js

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

        obs = torch.tensor(
            [[
                pole1_pos,
                pole1_vel,
                pole2_pos,
                pole2_vel,
                cart_pos,
                cart_vel,
            ]],
            dtype=torch.float32,
        )

        # run policy
        with torch.no_grad():
            action = self.policy(obs)

        if isinstance(action, torch.Tensor):
            a_norm = float(action.view(-1)[0].item())
        else:
            a_norm = float(action[0].view(-1)[0].item())

        # match IsaacLab: effort = action * action_scale
        effort = a_norm * ACTION_SCALE

        # publish effort as JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.cart_joint]
        js.position = [cart_pos] 
        js.velocity = [cart_vel]
        js.effort = [effort]
        print(effort)

        self.effort_pub.publish(js)

        debug_text = (
            f"theta1={pole1_pos:.3f}, theta1_vel={pole1_vel:.3f}, "
            f"theta2={pole2_pos:.3f}, theta2_vel={pole2_vel:.3f}, "
            f"x={cart_pos:.3f}, x_dot={cart_vel:.3f}, "
            f"a_norm={a_norm:.3f}, effort={effort:.3f}"
        )
        msg_out = String()
        msg_out.data = debug_text
        self.debug_pub.publish(msg_out)
        self.get_logger().info(debug_text)


def main(args=None):
    rclpy.init(args=args)
    node = CartPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
