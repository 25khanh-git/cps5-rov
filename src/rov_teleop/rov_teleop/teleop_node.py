"""teleop_node: tay cầm (/joy) -> /mavros/rc/override cho ArduSub.

- Gửi RC override LIÊN TỤC ở rate_hz (kể cả khi đứng yên = 1500), vì ArduSub
  coi việc ngừng nhận lệnh quá FS_PILOT_TIMEOUT là mất điều khiển (failsafe).
- Không có /joy mới trong joy_timeout giây -> gửi trung tính.
- Nút: ARM / DISARM / đổi chế độ MANUAL, STABILIZE, ALT_HOLD / tăng giảm gain.
- Phát /rov/topside_heartbeat để safety_node (chạy trên Pi) biết máy điều khiển còn sống.
"""

from __future__ import annotations

import time

import rclpy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

from rov_teleop.mapping import (AxisMap, ButtonEdges, build_channels, clamp, is_near_neutral,
                                joy_to_command, neutral_command)

STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')
        p = self.declare_parameter
        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.joy_timeout = float(p('joy_timeout', 0.5).value)
        self.pwm_range = int(p('pwm_range', 400).value)
        self.gain = float(p('gain_default', 0.5).value)
        self.gain_step = float(p('gain_step', 0.1).value)
        self.deadzone = float(p('deadzone', 0.05).value)
        self.arm_requires_neutral = bool(p('arm_requires_neutral', True).value)
        self.maps = {
            name: AxisMap(int(p(f'axis_{name}', d_idx).value), float(p(f'sign_{name}', d_sign).value))
            for name, d_idx, d_sign in (('forward', 1, 1.0), ('lateral', 0, -1.0),
                                        ('yaw', 2, -1.0), ('throttle', 3, 1.0))
        }
        self.buttons = {
            name: int(p(f'button_{name}', default).value)
            for name, default in (('arm', 6), ('disarm', 4), ('manual', 0), ('stabilize', 1),
                                  ('alt_hold', 3), ('gain_down', 9), ('gain_up', 10))
        }

        self.last_joy: Joy | None = None
        self.last_joy_time = None
        self.state = State()
        self.edges = ButtonEdges()

        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/rov/topside_heartbeat', 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_subscription(State, '/mavros/state', self._on_state, STATE_QOS)
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'teleop_node: {self.rate_hz:.0f} Hz, gain {self.gain:.1f}. '
            'Chưa ARM — nhấn nút arm (Options / phím 1 trên bàn phím).')

    # ------------------------------------------------------------------ callbacks
    def _on_joy(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_joy_time = time.monotonic()   # đồng hồ đơn điệu, không nhảy
        for idx in self.edges.pressed(msg.buttons):
            self._handle_button(idx, msg)

    def _on_state(self, msg: State) -> None:
        if msg.armed != self.state.armed or msg.mode != self.state.mode:
            self.get_logger().info(f"ArduSub: {'ARMED' if msg.armed else 'disarmed'}, mode {msg.mode}")
        self.state = msg

    def _joy_fresh(self) -> bool:
        if self.last_joy is None or self.last_joy_time is None:
            return False
        age = time.monotonic() - self.last_joy_time
        return age <= self.joy_timeout

    def _tick(self) -> None:
        if self._joy_fresh():
            cmd = joy_to_command(self.last_joy.axes, self.maps, self.gain, self.pwm_range,
                                 self.deadzone)
        else:
            cmd = neutral_command()
        msg = OverrideRCIn()
        msg.channels = build_channels(cmd)
        self.override_pub.publish(msg)
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'topside'
        self.heartbeat_pub.publish(hb)

    # ------------------------------------------------------------------ buttons
    def _handle_button(self, idx: int, joy: Joy) -> None:
        b = self.buttons
        if idx == b['arm']:
            if self.arm_requires_neutral and not is_near_neutral(joy.axes, self.maps, self.deadzone):
                self.get_logger().warn('Không ARM: hãy đưa cần ga / tiến về giữa trước.')
                return
            self._arm(True)
        elif idx == b['disarm']:
            self._arm(False)
        elif idx == b['manual']:
            self._set_mode('MANUAL')
        elif idx == b['stabilize']:
            self._set_mode('STABILIZE')
        elif idx == b['alt_hold']:
            self._set_mode('ALT_HOLD')
        elif idx == b['gain_up']:
            self.gain = clamp(round(self.gain + self.gain_step, 2), 0.1, 1.0)
            self.get_logger().info(f'gain = {self.gain:.1f}')
        elif idx == b['gain_down']:
            self.gain = clamp(round(self.gain - self.gain_step, 2), 0.1, 1.0)
            self.get_logger().info(f'gain = {self.gain:.1f}')

    def _arm(self, value: bool) -> None:
        if not self.arming_cli.service_is_ready():
            self.get_logger().error('Dịch vụ /mavros/cmd/arming chưa sẵn sàng (MAVROS đã chạy chưa?)')
            return
        fut = self.arming_cli.call_async(CommandBool.Request(value=value))
        word = 'ARM' if value else 'DISARM'

        def done(f):
            res = f.result()
            if res is not None and res.success:
                self.get_logger().info(f'{word}: OK')
            else:
                self.get_logger().error(f'{word} bị từ chối (xem /mavros/statustext/recv để biết lý do)')
        fut.add_done_callback(done)

    def _set_mode(self, mode: str) -> None:
        if not self.mode_cli.service_is_ready():
            self.get_logger().error('Dịch vụ /mavros/set_mode chưa sẵn sàng')
            return
        fut = self.mode_cli.call_async(SetMode.Request(base_mode=0, custom_mode=mode))

        def done(f):
            res = f.result()
            ok = res is not None and res.mode_sent
            (self.get_logger().info if ok else self.get_logger().error)(
                f'Đổi mode {mode}: {"đã gửi" if ok else "thất bại"}')
        fut.add_done_callback(done)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Gửi trung tính lần cuối trước khi thoát (nếu context còn sống)
        try:
            msg = OverrideRCIn()
            msg.channels = build_channels(neutral_command())
            node.override_pub.publish(msg)
        except Exception:  # noqa: BLE001 — đang tắt, bỏ qua
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
