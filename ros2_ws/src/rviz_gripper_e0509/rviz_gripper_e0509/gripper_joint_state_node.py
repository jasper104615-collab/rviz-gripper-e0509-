"""Real /gripper/state (pulse) -> /joint_states (rh_r1 rad) for gripper-only RViz."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

from rviz_gripper_e0509.gripper_conversion import pulse_to_rad

try:
    from rh_p12_rna_controller_interfaces.srv import GetState
except ImportError:
    GetState = None

GRIPPER_JOINTS = ["rh_r1", "rh_l1", "rh_r2", "rh_l2"]
FEEDBACK_NAME = "gripper_joint"


class GripperJointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_joint_state")

        self.declare_parameter("gripper_state_topic", "/gripper/state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("open_rad", 0.0)
        self.declare_parameter("closed_rad", 1.101)
        self.declare_parameter("pulse_open", 0)
        self.declare_parameter("pulse_closed", 700)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("poll_gripper_service", True)

        self._open_rad = self.get_parameter("open_rad").value
        self._closed_rad = self.get_parameter("closed_rad").value
        self._pulse_open = int(self.get_parameter("pulse_open").value)
        self._pulse_closed = int(self.get_parameter("pulse_closed").value)
        self._pulse = 0.0
        self._have_feedback = False

        gripper_topic = self.get_parameter("gripper_state_topic").value
        out_topic = self.get_parameter("joint_states_topic").value

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, gripper_topic, self._on_gripper, qos)
        self._pub = self.create_publisher(JointState, out_topic, 10)

        if self.get_parameter("poll_gripper_service").value and GetState is not None:
            self._get_state_cli = self.create_client(GetState, "/gripper/get_state")
            self.create_timer(0.5, self._poll_service)

        hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(f"{gripper_topic} (pulse) -> {out_topic} (rad)")

    def _on_gripper(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == FEEDBACK_NAME:
                self._pulse = pos
                self._have_feedback = True
                return

    def _poll_service(self) -> None:
        if not self._get_state_cli.service_is_ready():
            return
        future = self._get_state_cli.call_async(GetState.Request())
        future.add_done_callback(self._on_service)

    def _on_service(self, future) -> None:
        try:
            res = future.result()
            if res and res.state and res.state.status_text == "ok":
                self._pulse = float(res.state.position)
                self._have_feedback = True
        except Exception:
            pass

    def _master_rad(self) -> float:
        return pulse_to_rad(
            int(round(self._pulse)),
            open_rad=self._open_rad,
            closed_rad=self._closed_rad,
            pulse_open=self._pulse_open,
            pulse_closed=self._pulse_closed,
        )

    def _publish(self) -> None:
        if not self._have_feedback:
            return
        rad = self._master_rad()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(GRIPPER_JOINTS)
        msg.position = [rad, rad, rad, rad]
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
