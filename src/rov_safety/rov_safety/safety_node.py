"""safety_node: lớp an toàn thứ hai, chạy cùng nơi với MAVROS (laptop hoặc Pi khi chạy thật).

Theo dõi:
  * /rov/topside_heartbeat (từ teleop_node trên laptop) — mất quá heartbeat_timeout khi ARM -> DISARM
  * /mavros/battery — dưới battery_critical liên tục critical_hold giây -> SURFACE
  * /mavros/global_position/rel_alt (âm = dưới nước) — sâu hơn max_depth -> SURFACE
Phát /rov/safety/status (chuỗi tóm tắt, 1 Hz).
"""

from __future__ import annotations

import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, Header, String

from rov_safety.logic import DISARM, SURFACE, SafetyConfig, SafetyMonitor

STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_node')
        p = self.declare_parameter
        cfg = SafetyConfig(
            heartbeat_timeout=float(p('heartbeat_timeout', 3.0).value),
            battery_warn=float(p('battery_warn', 10.5).value),
            battery_critical=float(p('battery_critical', 9.9).value),
            critical_hold=float(p('critical_hold', 3.0).value),
            max_depth=float(p('max_depth', 5.0).value),
        )
        rate = float(p('check_rate', 5.0).value)
        self.mon = SafetyMonitor(cfg)

        self.create_subscription(State, '/mavros/state', self._on_state, STATE_QOS)
        self.create_subscription(Header, '/rov/topside_heartbeat', self._on_hb, 10)
        self.create_subscription(BatteryState, '/mavros/battery', self._on_batt,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self._on_alt,
                                 qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/rov/safety/status', 10)
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(1.0 / rate, self._check)
        self.create_timer(1.0, self._publish_status)
        self._last_warn = ''
        self.get_logger().info(
            f'safety_node: heartbeat {cfg.heartbeat_timeout}s, pin {cfg.battery_warn}/'
            f'{cfg.battery_critical} V, sâu tối đa {cfg.max_depth} m')

    @staticmethod
    def _now() -> float:
        # Dùng đồng hồ đơn điệu (monotonic): không bao giờ nhảy. Đồng hồ hệ thống (ROS clock)
        # có thể nhảy khi WSL2 / NTP đồng bộ giờ, khiến node tưởng mất heartbeat và DISARM nhầm.
        return time.monotonic()

    def _on_state(self, m: State) -> None:
        self.mon.on_state(self._now(), m.connected, m.armed, m.mode)

    def _on_hb(self, _m: Header) -> None:
        self.mon.on_heartbeat(self._now())

    def _on_batt(self, m: BatteryState) -> None:
        self.mon.on_battery(self._now(), float(m.voltage))

    def _on_alt(self, m: Float64) -> None:
        self.mon.on_depth(-float(m.data))   # rel_alt âm dưới nước -> depth dương

    def _check(self) -> None:
        for action in self.mon.evaluate(self._now()):
            self.get_logger().error(f'AN TOÀN: {action.reason} -> {action.kind.upper()}')
            if action.kind == DISARM and self.arming_cli.service_is_ready():
                self.arming_cli.call_async(CommandBool.Request(value=False))
            elif action.kind == SURFACE and self.mode_cli.service_is_ready():
                self.mode_cli.call_async(SetMode.Request(base_mode=0, custom_mode='SURFACE'))
        warn = '; '.join(self.mon.warnings)
        if warn and warn != self._last_warn:
            self.get_logger().warn(warn)
        self._last_warn = warn

    def _publish_status(self) -> None:
        m = self.mon
        age = m.heartbeat_age(self._now())
        parts = [
            'kết nối' if m.connected else 'MẤT KẾT NỐI',
            'ARMED' if m.armed else 'disarmed',
            m.mode or '?',
            f'heartbeat {age:.1f}s' if age is not None else 'chưa có heartbeat',
            f'pin {m.voltage:.2f}V' if m.voltage else 'pin ?',
            f'sâu {m.depth:.2f}m' if m.depth is not None else 'sâu ?',
        ]
        self.status_pub.publish(String(data=' | '.join(parts)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
