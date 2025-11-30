import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
import torch


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi] so it matches what the RL policy was trained on."""
    return math.atan2(math.sin(a), math.cos(a))


class CartPolicy(Node):
    def __init__(self):
        super().__init__("cart_policy_node")

        # ---- joint names must match your USD ----
        self.cart_joint = "slider_to_cart"
        self.pole1_joint = "cart_to_pole"
        self.pole2_joint = "pole_to_pendulum"

        # ---- load policy.pt ----
        # TODO: change this to the real path of your exported policy.pt
        POLICY_PATH = "/home/jliu/isaac_ws/src/sim_pkg/sim_pkg/assets/policies/policy.pt"

        try:
            # most Isaac Lab exports are TorchScript
            self.policy = torch.jit.load(POLICY_PATH, map_location="cpu")
            self.get_logger().info(f"Loaded TorchScript policy from: {POLICY_PATH}")
        except Exception as e:
            self.get_logger().warn(
                f"torch.jit.load failed ({e}), trying torch.load instead..."
            )
            self.policy = torch.load(POLICY_PATH, map_location="cpu")
            self.get_logger().info(f"Loaded policy with torch.load from: {POLICY_PATH}")

        self.policy.eval()

        # ---- ROS I/O ----
        self.sub = self.create_subscription(
            JointState,
            "/joint_states_read",
            self.joint_state_cb,
            10,
        )

        self.debug_pub = self.create_publisher(String, "debug", 10)
        self.effort_pub = self.create_publisher(Float32, "cart_effort", 10)

        self.get_logger().info("CartPolicy node started, waiting for /joint_states_read")

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

        # Optionally wrap angles into [-pi, pi] before passing to policy
        pole1_pos_wrapped = wrap_angle(pole1_pos)
        pole2_pos_wrapped = wrap_angle(pole2_pos)

        # ---- build observation in SAME order as env _get_observations ----
        # [θ1, θ̇1, θ2, θ̇2, x, ẋ]
        obs = torch.tensor(
            [[
                pole1_pos_wrapped,
                pole1_vel,
                pole2_pos_wrapped,
                pole2_vel,
                cart_pos,
                cart_vel,
            ]],
            dtype=torch.float32,
        )

        # ---- run policy ----
        with torch.no_grad():
            action = self.policy(obs)

        # assume shape (1, 1) → scalar effort
        if isinstance(action, torch.Tensor):
            effort = float(action.view(-1)[0].item())
        else:
            # in case policy returns something weird like (tensor, extra_info)
            effort = float(action[0].view(-1)[0].item())

        # ---- publish effort ----
        eff_msg = Float32()
        eff_msg.data = effort
        self.effort_pub.publish(eff_msg)

        # ---- debug string ----
        debug_text = (
            f"theta1={pole1_pos_wrapped:.3f}, theta1_vel={pole1_vel:.3f}, "
            f"theta2={pole2_pos_wrapped:.3f}, theta2_vel={pole2_vel:.3f}, "
            f"x={cart_pos:.3f}, x_dot={cart_vel:.3f}, "
            f"effort={effort:.3f}"
        )

        msg_out = String()
        msg_out.data = debug_text
        self.debug_pub.publish(msg_out)

        # Also log occasionally if you want (comment out if too spammy)
        self.get_logger().info(debug_text)


def main(args=None):
    rclpy.init(args=args)
    node = CartPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
