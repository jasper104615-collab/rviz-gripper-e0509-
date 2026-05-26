#!/usr/bin/env python3
"""Bridge Isaac gripper joint (rad) to real RH-P12 / e0509 gripper commands."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

from isaac_gripper_sim2real.gripper_conversion import rad_to_pulse, rad_to_stroke

try:
    from rh_p12_rna_controller_interfaces.srv import SetPosition
except ImportError:  # pragma: no cover
    SetPosition = None  # type: ignore[misc, assignment]

OUTPUT_E0509_TOPIC = 'e0509_topic'
OUTPUT_RH_P12_DIRECT = 'rh_p12_direct'
OUTPUT_RH_P12_SERVICE = 'rh_p12_service'

RH_P12_MODES = (OUTPUT_RH_P12_DIRECT, OUTPUT_RH_P12_SERVICE)
ALL_OUTPUT_MODES = (OUTPUT_E0509_TOPIC, *RH_P12_MODES)


class IsaacGripperBridge(Node):
    """Isaac /isaac/joint_states (rh_r1 rad) -> real gripper command."""

    def __init__(self) -> None:
        super().__init__('isaac_gripper_bridge')

        self.declare_parameter('input_topic', '/isaac/joint_states')
        self.declare_parameter('master_joint_name', 'rh_r1')
        self.declare_parameter('open_rad', 0.0)
        self.declare_parameter('closed_rad', 1.101)
        self.declare_parameter('stroke_max', 700)
        # rh_p12_direct: TCP Modbus via gripper_service_node (no drl_start per cmd)
        self.declare_parameter('output_mode', OUTPUT_RH_P12_DIRECT)
        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('position_cmd_topic', '')
        self.declare_parameter('direct_cmd_topic', '/gripper/cmd_direct')
        self.declare_parameter('set_position_service', '/gripper/set_position')
        self.declare_parameter('pulse_open', 0)
        self.declare_parameter('pulse_closed', 700)
        self.declare_parameter('goal_current', 300)
        self.declare_parameter('set_position_timeout_sec', 5.0)
        self.declare_parameter('enable_command', False)
        self.declare_parameter('max_publish_hz', 15.0)
        self.declare_parameter('command_stream_hz', 12.0)
        self.declare_parameter('command_burst_delta', 50)
        self.declare_parameter('command_settle_pulse', 2)
        self.declare_parameter('min_delta_stroke', 2)
        self.declare_parameter('min_delta_pulse', 2)
        self.declare_parameter('input_timeout_sec', 3.0)
        self.declare_parameter('debug_log_hz', 1.0)

        self.input_topic = self._str('input_topic')
        self.master_joint = self._str('master_joint_name')
        self.open_rad = self._float('open_rad')
        self.closed_rad = self._float('closed_rad')
        self.stroke_max = self._int('stroke_max')
        self.output_mode = self._str('output_mode').strip().lower()
        self.robot_ns = self._str('robot_ns').strip('/')
        self.position_cmd_topic = self._str('position_cmd_topic').strip()
        self.direct_cmd_topic = self._str('direct_cmd_topic').strip()
        self.set_position_service = self._str('set_position_service')
        self.pulse_open = self._int('pulse_open')
        self.pulse_closed = self._int('pulse_closed')
        self.goal_current = self._int('goal_current')
        self.set_position_timeout = self._float('set_position_timeout_sec')
        self.enable_command = self._bool('enable_command')
        self.max_publish_hz = max(0.1, self._float('max_publish_hz'))
        self.command_stream_hz = max(0.0, self._float('command_stream_hz'))
        self.command_burst_delta = max(1, self._int('command_burst_delta'))
        self.command_settle_pulse = max(0, self._int('command_settle_pulse'))
        self.min_delta_stroke = self._int('min_delta_stroke')
        self.min_delta_pulse = self._int('min_delta_pulse')
        self.input_timeout_sec = self._float('input_timeout_sec')
        self.debug_log_hz = self._float('debug_log_hz')

        if self.output_mode not in ALL_OUTPUT_MODES:
            raise ValueError(
                f"output_mode must be one of: {', '.join(ALL_OUTPUT_MODES)}"
            )
        if self.output_mode in RH_P12_MODES and SetPosition is None:
            raise RuntimeError(
                'rh_p12_rna_controller_interfaces not available; build workspace first '
                "or use output_mode:=e0509_topic"
            )

        if not self.position_cmd_topic:
            self.position_cmd_topic = f'/{self.robot_ns}/gripper/position_cmd'

        self._target_rad: Optional[float] = None
        self._last_stroke: Optional[int] = None
        self._last_pulse: Optional[int] = None
        self._last_cmd_stroke: Optional[int] = None
        self._last_cmd_pulse: Optional[int] = None
        self._last_cmd_publish_ns = 0
        self._last_timer_pulse: Optional[int] = None
        self._input_count = 0
        self._command_count = 0
        self._logged_names = False
        self._warned_no_input = False
        self._service_inflight = False
        self._start_ns = self.get_clock().now().nanoseconds

        self.create_subscription(
            JointState,
            self.input_topic,
            self._on_isaac_joint_state,
            qos_profile_sensor_data,
        )

        period = 1.0 / self.max_publish_hz
        self.create_timer(period, self._on_timer)
        if self.input_timeout_sec > 0.0:
            self.create_timer(1.0, self._watchdog_input)
        if self.debug_log_hz > 0.0:
            self.create_timer(1.0 / self.debug_log_hz, self._debug_log)

        self._stroke_pub = None
        self._direct_pub = None
        self._set_pos_cli = None
        if self.output_mode == OUTPUT_E0509_TOPIC:
            self._stroke_pub = self.create_publisher(
                Int32, self.position_cmd_topic, 10
            )
        elif self.output_mode == OUTPUT_RH_P12_DIRECT:
            self._direct_pub = self.create_publisher(String, self.direct_cmd_topic, 10)
        else:
            self._set_pos_cli = self.create_client(SetPosition, self.set_position_service)

        self._log_startup()

    def _log_startup(self) -> None:
        self.get_logger().info(
            f'Isaac gripper bridge | input={self.input_topic} '
            f'master={self.master_joint} mode={self.output_mode} '
            f'rad=[{self.open_rad:.3f},{self.closed_rad:.3f}]'
        )
        if self.output_mode == OUTPUT_E0509_TOPIC:
            self.get_logger().info(
                f'publish: {self.position_cmd_topic} (Int32 stroke, no DRL)'
            )
        elif self.output_mode == OUTPUT_RH_P12_DIRECT:
            self.get_logger().info(
                f'publish: {self.direct_cmd_topic} (String custom PULSE CUR, TCP path)'
            )
            self.get_logger().info(
                'DRL note: gripper_service_node MUST use command_transport:=tcp '
                'and direct_cmd_topic_enabled:=true (see launch/sim2real_gripper.launch.py)'
            )
        else:
            self.get_logger().info(f'service: {self.set_position_service} (sync)')
            self.get_logger().warn(
                'rh_p12_service uses SetPosition srv; prefer rh_p12_direct for sim2real streaming. '
                'gripper_service_node MUST use command_transport:=tcp (never drl).'
            )

        self.get_logger().info(
            'Arm sim2real (sub_joint_state) uses move_joint/servoj_rt (DRFL) — '
            'does NOT share drl_start with gripper when gripper uses TCP mode.'
        )
        if not self.enable_command:
            self.get_logger().warn('enable_command:=false — commands computed only, not sent')

    def _on_isaac_joint_state(self, msg: JointState) -> None:
        if not self._logged_names and msg.name:
            self.get_logger().info(f'Isaac joint names: {list(msg.name)}')
            self._logged_names = True

        rad = self._extract_master_rad(msg)
        if rad is None:
            return
        self._target_rad = rad
        self._input_count += 1

    def _on_timer(self) -> None:
        if self._target_rad is None:
            return

        stroke = rad_to_stroke(
            self._target_rad,
            open_rad=self.open_rad,
            closed_rad=self.closed_rad,
            stroke_max=self.stroke_max,
        )
        pulse = rad_to_pulse(
            self._target_rad,
            open_rad=self.open_rad,
            closed_rad=self.closed_rad,
            pulse_open=self.pulse_open,
            pulse_closed=self.pulse_closed,
        )
        self._last_stroke = stroke
        self._last_pulse = pulse

        if not self.enable_command:
            return

        if self.output_mode == OUTPUT_E0509_TOPIC:
            self._publish_stroke_if_changed(stroke)
        elif self.output_mode == OUTPUT_RH_P12_DIRECT:
            self._publish_direct_if_changed(pulse)
        else:
            self._call_set_position_if_changed(pulse)

    def _publish_stroke_if_changed(self, stroke: int) -> None:
        if self._stroke_pub is None:
            return
        if (
            self._last_cmd_stroke is not None
            and abs(stroke - self._last_cmd_stroke) < self.min_delta_stroke
        ):
            return
        msg = Int32()
        msg.data = int(stroke)
        self._stroke_pub.publish(msg)
        self._last_cmd_stroke = stroke
        self._command_count += 1

    def _emit_direct(self, pulse: int, now_ns: int) -> None:
        if self._direct_pub is None:
            return
        msg = String()
        msg.data = f'custom {int(pulse)} {int(self.goal_current)}'
        self._direct_pub.publish(msg)
        self._last_cmd_pulse = int(pulse)
        self._last_cmd_publish_ns = now_ns
        self._command_count += 1

    def _publish_direct_if_changed(self, pulse: int) -> None:
        if self._direct_pub is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        prev_pulse = self._last_timer_pulse
        self._last_timer_pulse = int(pulse)
        moving = prev_pulse is not None and int(pulse) != int(prev_pulse)

        if self._last_cmd_pulse is None:
            self._emit_direct(pulse, now_ns)
            return

        delta = abs(int(pulse) - int(self._last_cmd_pulse))

        if delta >= self.command_burst_delta:
            self._emit_direct(pulse, now_ns)
            return

        if moving and self.command_stream_hz > 0.0:
            interval_ns = int(1e9 / self.command_stream_hz)
            if now_ns - self._last_cmd_publish_ns >= interval_ns:
                self._emit_direct(pulse, now_ns)
            return

        if not moving and delta >= self.command_settle_pulse:
            self._emit_direct(pulse, now_ns)

    def _call_set_position_if_changed(self, pulse: int) -> None:
        if self._set_pos_cli is None or SetPosition is None:
            return
        if self._service_inflight:
            return
        if (
            self._last_cmd_pulse is not None
            and abs(pulse - self._last_cmd_pulse) < self.min_delta_pulse
        ):
            return
        if not self._set_pos_cli.service_is_ready():
            return

        req = SetPosition.Request()
        req.position = int(pulse)
        req.current = int(self.goal_current)
        req.timeout_sec = float(self.set_position_timeout)

        self._service_inflight = True
        future = self._set_pos_cli.call_async(req)
        future.add_done_callback(
            lambda f, p=pulse: self._on_set_position_done(f, p)
        )

    def _on_set_position_done(self, future, pulse: int) -> None:
        self._service_inflight = False
        try:
            resp = future.result()
            if resp.success:
                self._last_cmd_pulse = pulse
                self._command_count += 1
            else:
                self.get_logger().warning(f'set_position failed: {resp.message}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'set_position call error: {exc}')

    def _extract_master_rad(self, msg: JointState) -> Optional[float]:
        if not msg.name or not msg.position:
            return None
        by_name = dict(zip(msg.name, msg.position))
        if self.master_joint not in by_name:
            return None
        return float(by_name[self.master_joint])

    def _watchdog_input(self) -> None:
        if self._input_count > 0 or self._warned_no_input:
            return
        elapsed = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
        if elapsed < self.input_timeout_sec:
            return
        self._warned_no_input = True
        self.get_logger().error(
            f'No input on {self.input_topic} for {elapsed:.1f}s '
            f'(expect master joint "{self.master_joint}")'
        )

    def _debug_log(self) -> None:
        if self._target_rad is None:
            return
        deg = math.degrees(self._target_rad)
        self.get_logger().info(
            f'rad={self._target_rad:.4f} ({deg:.1f}°) '
            f'stroke={self._last_stroke} pulse={self._last_pulse} '
            f'cmds={self._command_count} inputs={self._input_count}'
        )

    def _str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacGripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
