"""keyboard_teleop: dùng bàn phím thay tay cầm (tiện trong WSL, nơi tay cầm USB khó dùng).

Node này không gửi lệnh trực tiếp cho ArduSub. Nó phát /joy giả lập đúng bố cục tay cầm
mặc định trong teleop.yaml, nên teleop_node xử lý y hệt như tay cầm thật.

Giữ phím thì robot chạy, nhả phím thì dừng (sau ~0.6 s). Biên độ chỉnh bằng [ / ] (gain).
Dòng trạng thái hiển thị ARM / mode / độ sâu / hướng la bàn, cập nhật liên tục.
Phải chạy trong terminal riêng:  ros2 run rov_teleop keyboard_teleop
"""

from __future__ import annotations

import math
import select
import shutil
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import RCOut, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64

from rov_teleop.keymap import HoldKeys, OrbitState, compass_deg

HELP = """
CPS 5 — điều khiển bằng bàn phím: GIỮ phím để chạy, NHẢ ra để dừng
  w / s : tiến / lùi            a / d : sang trái / phải
  q / e : quay trái / phải      r / f : nổi lên / lặn xuống
  Space : DỪNG ngay
  1 : ARM    0 : DISARM
  m : MANUAL   t : STABILIZE (giữ thăng bằng)   h : ALT_HOLD (giữ độ sâu — dễ lái nhất)
  [ / ] : giảm / tăng lực (gain)     Ctrl-C : thoát
CAMERA: j / l : xoay quanh robot   i / k : nhìn cao / thấp   c : về sau đuôi
Mẹo: ARM rồi nhấn h (ALT_HOLD) — robot tự giữ độ sâu, chỉ cần lái tiến/quay.
"""

STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        rate = float(self.declare_parameter('rate_hz', 20.0).value)
        release = float(self.declare_parameter('release_timeout', 0.6).value)
        self.keys = HoldKeys(release_timeout=release)
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.state = State()
        self.depth = None
        self.heading = None
        self.create_subscription(State, '/mavros/state', self._on_state, STATE_QOS)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self._on_alt,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(RCOut, '/mavros/rc/out', self._on_rcout, qos_profile_sensor_data)
        self.servo = None
        self.warn_until = 0.0
        self.create_timer(1.0 / rate, self._publish)
        self.orbit = OrbitState()
        self.orbit_pub = self.create_publisher(Vector3, '/chase_camera/orbit', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_orbit()

    def publish_orbit(self) -> None:
        self.orbit_pub.publish(Vector3(x=self.orbit.azimuth, y=self.orbit.elevation, z=0.0))

    def _on_state(self, m: State) -> None:
        self.state = m

    def _on_alt(self, m: Float64) -> None:
        self.depth = -m.data

    def _on_rcout(self, m: RCOut) -> None:
        self.servo = list(m.channels[:8])   # Vectored-6DOF: 8 thruster

    def on_move_key(self) -> None:
        """Gọi khi nhấn phím di chuyển: nhắc nếu robot chưa sẵn sàng."""
        if not self.state.connected or not self.state.armed:
            self.warn_until = time.monotonic() + 2.0

    def _on_imu(self, m: Imu) -> None:
        q = m.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.heading = compass_deg(yaw)

    def _publish(self, stop: bool = False) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        axes = [0.0] * 6 if stop else self.keys.axes(time.monotonic())
        msg.axes = [float(a) for a in axes]
        msg.buttons = self.keys.take_buttons()
        self.pub.publish(msg)

    def status(self) -> str:
        a = self.keys.axes(time.monotonic())
        moves = []
        if a[1]:
            moves.append('TIẾN' if a[1] > 0 else 'LÙI')
        if a[0]:
            moves.append('TRÁI' if a[0] > 0 else 'PHẢI')
        if a[2]:
            moves.append('QUAY TRÁI' if a[2] > 0 else 'QUAY PHẢI')
        if a[3]:
            moves.append('NỔI' if a[3] > 0 else 'LẶN')
        s = self.state
        arm = 'ARMED' if s.armed else 'disarmed'
        conn = '' if s.connected else ' (chưa nối ArduSub)'
        depth = f'{self.depth:5.2f} m' if self.depth is not None else '  ?  '
        head = f'{self.heading:5.1f}°' if self.heading is not None else '  ?  '
        lenh = ' + '.join(moves) if moves else 'đứng yên'
        # PWM thực tế ArduSub gửi tới 8 thruster (1500 = không đẩy)
        if self.servo:
            pwm = 'PWM ' + ' '.join(str(v) for v in self.servo)
        else:
            pwm = 'thruster: chưa có dữ liệu'
        line = (f'[{arm} {s.mode or "?"}]{conn} sâu {depth} | hướng {head} | '
                f'{lenh} | {pwm}')
        if time.monotonic() < self.warn_until:
            line = ('CHƯA ARM — nhấn 1 trước! | ' if s.connected else 'CHƯA NỐI ArduSub! | ') + line
        # vừa đúng 1 dòng terminal, để \r ghi đè sạch (dòng dài bị xuống dòng sẽ in lặp)
        width = shutil.get_terminal_size((100, 20)).columns - 1
        return '\r' + line[:width].ljust(width)


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print('keyboard_teleop cần một terminal thật. Chạy: ros2 run rov_teleop keyboard_teleop')
        return
    rclpy.init(args=args)
    node = KeyboardTeleop()
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    last_draw = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if ready:
                key = sys.stdin.read(1)
                if key == '\x03':
                    break
                if node.orbit.press(key):
                    node.publish_orbit()
                elif node.keys.press(key, time.monotonic()) and key.lower() in 'wsadqerf':
                    node.on_move_key()
            now = time.monotonic()
            if now - last_draw > 0.1:
                sys.stdout.write(node.status())
                sys.stdout.flush()
                last_draw = now
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            node._publish(stop=True)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
