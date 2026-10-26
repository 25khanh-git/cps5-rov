"""check_sim: kiểm tra tự động sim theo 9 mục (bảng ở README mục 6).

Cần sim đang chạy với teleop tắt (script này tự gửi lệnh lái):
    ros2 launch rov_bringup sim.launch.py teleop:=false
    ros2 run rov_bringup check_sim
Hoặc gọn hơn: bash scripts/check_sim.sh (tự bật sim không cửa sổ, chạy kiểm tra, tắt sim).

So sánh lệnh gửi cho ArduSub với chuyển động "thật" của robot trong Gazebo
(/cps5/ground_truth/odom), nên phát hiện được sai thứ tự motor, sai chiều thruster,
sai đổi hệ trục ENU/NED, sai lực nổi, sai cấu hình failsafe.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import rclpy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header

from rov_bringup.geometry import body_displacement, quat_to_rpy, wrap_pi

STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
NEUTRAL = 1500
NOCHANGE = 65535


@dataclass
class Result:
    name: str
    status: str            # PASS | FAIL | SKIP
    detail: str = ''


@dataclass
class Pose:
    t: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class Checker:
    results: List[Result] = field(default_factory=list)

    def add(self, name: str, ok: Optional[bool], detail: str) -> bool:
        status = 'SKIP' if ok is None else ('PASS' if ok else 'FAIL')
        self.results.append(Result(name, status, detail))
        print(f'  [{status}] {name}: {detail}', flush=True)
        return bool(ok)


class SimCheck(Node):
    def __init__(self) -> None:
        super().__init__('check_sim')
        p = self.declare_parameter
        self.depth_hold_tol = float(p('depth_hold_tolerance', 0.15).value)
        self.pilot_timeout = float(p('fs_pilot_timeout', 1.5).value)
        self.hb_timeout = float(p('heartbeat_timeout', 3.0).value)

        self.state = State()
        self.pose: Optional[Pose] = None
        self.yaw_unwrapped = 0.0
        self._last_yaw: Optional[float] = None
        self.imu_rpy = None
        self.image: Optional[Image] = None
        self.lock = threading.Lock()

        self.cmd = {'throttle': NEUTRAL, 'yaw': NEUTRAL, 'forward': NEUTRAL, 'lateral': NEUTRAL}
        self.send_override = True
        self.send_heartbeat = True

        self.create_subscription(State, '/mavros/state', self._on_state, STATE_QOS)
        self.create_subscription(Odometry, '/cps5/ground_truth/odom', self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.hb_pub = self.create_publisher(Header, '/rov/topside_heartbeat', 10)
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(0.05, self._tick)   # 20 Hz giống teleop_node

    # ------------------------------------------------------------------ callbacks
    def _on_state(self, m: State) -> None:
        self.state = m

    def _on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        r, pt, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        with self.lock:
            if self._last_yaw is not None:
                self.yaw_unwrapped += wrap_pi(y - self._last_yaw)
            else:
                self.yaw_unwrapped = y
            self._last_yaw = y
            pos = m.pose.pose.position
            self.pose = Pose(time.monotonic(), pos.x, pos.y, pos.z, r, pt, y)

    def _on_imu(self, m: Imu) -> None:
        q = m.orientation
        self.imu_rpy = quat_to_rpy(q.x, q.y, q.z, q.w)

    def _on_image(self, m: Image) -> None:
        self.image = m

    def _tick(self) -> None:
        if self.send_override:
            msg = OverrideRCIn()
            ch = [NOCHANGE] * 18
            ch[0] = ch[1] = NEUTRAL
            ch[2], ch[3] = self.cmd['throttle'], self.cmd['yaw']
            ch[4], ch[5] = self.cmd['forward'], self.cmd['lateral']
            msg.channels = ch
            self.override_pub.publish(msg)
        if self.send_heartbeat:
            hb = Header()
            hb.stamp = self.get_clock().now().to_msg()
            hb.frame_id = 'check_sim'
            self.hb_pub.publish(hb)

    # ------------------------------------------------------------------ helpers
    def snapshot(self) -> Optional[Pose]:
        with self.lock:
            return self.pose

    def yaw_total(self) -> float:
        with self.lock:
            return self.yaw_unwrapped

    def set_cmd(self, **kw) -> None:
        self.cmd = {**{k: NEUTRAL for k in self.cmd}, **kw}

    def call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        if not wait_for(fut.done, timeout):
            return None
        return fut.result()

    def set_mode(self, mode: str, timeout: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            res = self.call(self.mode_cli, SetMode.Request(base_mode=0, custom_mode=mode))
            if res is not None and res.mode_sent and wait_for(lambda: self.state.mode == mode, 2.0):
                return True
            time.sleep(0.5)
        return False

    def arm(self, value: bool, timeout: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            res = self.call(self.arming_cli, CommandBool.Request(value=value))
            if res is not None and res.success and wait_for(lambda: self.state.armed == value, 3.0):
                return True
            time.sleep(2.0)   # thường do EKF chưa sẵn sàng ngay sau khi khởi động
        return False


def wait_for(pred: Callable[[], bool], timeout: float, period: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(period)
    return bool(pred())


def hold(node: SimCheck, seconds: float, **cmd) -> None:
    node.set_cmd(**cmd)
    time.sleep(seconds)
    node.set_cmd()


# ============================================================================ tests

def run(node: SimCheck) -> Checker:
    c = Checker()
    deg = math.degrees

    print('\n=== CPS 5 — KIỂM TRA SIMULATION ===', flush=True)
    names = [n for n, _ in node.get_node_names_and_namespaces()]
    if 'teleop_node' in names:
        c.add('Chuẩn bị', False, 'teleop_node đang chạy — hãy chạy sim với teleop:=false')
        return c

    # 1. Kết nối ---------------------------------------------------------------------
    ok = wait_for(lambda: node.state.connected, 90.0)
    c.add('1. MAVROS kết nối ArduSub SITL', ok,
          '/mavros/state connected = true' if ok else 'quá 90 s chưa kết nối (SITL chạy chưa? cổng 14550?)')
    if not ok:
        return c
    ok = wait_for(lambda: node.snapshot() is not None, 30.0)
    c.add('   Ground truth từ Gazebo', ok, '/cps5/ground_truth/odom' if ok else 'không nhận được odom (bridge?)')
    if not ok:
        return c

    # 2. Hệ trục: hướng EKF của ArduSub (qua MAVROS, ENU) phải khớp hướng thật trong Gazebo
    def frame_error():
        p = node.snapshot()
        if node.imu_rpy is None or p is None:
            return None
        r, pt, y = node.imu_rpy
        return (abs(wrap_pi(y - p.yaw)), abs(wrap_pi(r - p.roll)), abs(wrap_pi(pt - p.pitch)))

    def frame_ok() -> bool:
        err = frame_error()
        return err is not None and max(err) < math.radians(10)

    ok = wait_for(frame_ok, 60.0, 0.5)   # EKF cần vài giây để hội tụ
    e = frame_error()
    c.add('2. Hệ trục ENU/NED + IMU', ok,
          'không có /mavros/imu/data' if e is None else
          f'lệch yaw {deg(e[0]):.1f}°, roll {deg(e[1]):.1f}°, pitch {deg(e[2]):.1f}° (cần < 10°)')

    # 3. ARM ở MANUAL -------------------------------------------------------------------
    node.set_cmd()
    if not node.set_mode('MANUAL'):
        c.add('3. Chế độ MANUAL + ARM', False, 'không đổi được sang MANUAL')
        return c
    ok = node.arm(True)
    c.add('3. Chế độ MANUAL + ARM', ok, 'ARMED' if ok else
          'ArduSub từ chối ARM trong 60 s (xem: ros2 topic echo /mavros/statustext/recv)')
    if not ok:
        return c
    time.sleep(1.0)

    # 4a. Lặn: throttle 1300 -> 4 thruster đứng (M5-M8) đẩy xuống ------------------------
    # (bể sâu 3 m: các bước sau dùng throttle 1500 để robot nổi chậm, không chạm đáy)
    p0 = node.snapshot()
    hold(node, 4.0, throttle=1300)
    p1 = node.snapshot()
    dz = p1.z - p0.z
    c.add('4a. Lặn (throttle 1300, 4 s)', dz < -0.5,
          f'độ sâu thay đổi {dz:+.2f} m (cần < -0.50), roll {deg(p1.roll):+.1f}°')

    # 4b. Tiến: 4 thruster ngang xéo (M1-M4) cộng lực về trước --------------------------
    p0 = node.snapshot()
    hold(node, 3.0, forward=1650)
    p1 = node.snapshot()
    fwd, right = body_displacement(p0.x, p0.y, p0.yaw, p1.x, p1.y)
    c.add('4b. Tiến (forward 1650, 3 s)', fwd > 0.3 and abs(right) < fwd,
          f'tiến {fwd:+.2f} m, lệch ngang {right:+.2f} m (cần tiến > 0.30)')

    # 4c. Quay phải: M1, M4 đẩy dương, M2, M3 đẩy âm -> yaw ENU giảm -------------------
    time.sleep(1.5)   # chờ robot dừng trôi
    y0 = node.yaw_total()
    hold(node, 2.0, yaw=1600)
    time.sleep(0.5)
    dyaw = node.yaw_total() - y0
    c.add('4c. Quay phải (yaw 1600, 2 s)', dyaw < -math.radians(20),
          f'yaw thay đổi {deg(dyaw):+.0f}° (quay phải = âm trong ENU, cần < -20°)')

    # 4d. Sang phải: 4 thruster ngang xéo (M1-M4) cộng lực sang phải -------------------
    time.sleep(1.5)
    p0 = node.snapshot()
    hold(node, 3.0, lateral=1700)
    p1 = node.snapshot()
    fwd, right = body_displacement(p0.x, p0.y, p0.yaw, p1.x, p1.y)
    c.add('4d. Sang phải (lateral 1700, 3 s)', right > 0.2,
          f'sang phải {right:+.2f} m (cần > 0.20), tiến/lùi {fwd:+.2f} m')

    # 5. Giữ độ sâu (ALT_HOLD) ----------------------------------------------------------
    hold(node, 3.0, throttle=1300)
    if node.set_mode('ALT_HOLD'):
        node.set_cmd()
        time.sleep(4.0)
        z0 = node.snapshot().z
        zs = []
        t_end = time.monotonic() + 10.0
        while time.monotonic() < t_end:
            zs.append(node.snapshot().z)
            time.sleep(0.1)
        err = max(abs(z - z0) for z in zs)
        c.add('5. ALT_HOLD giữ độ sâu (10 s)', err < node.depth_hold_tol,
              f'độ sâu ~{-z0:.2f} m, sai lệch lớn nhất {err * 100:.1f} cm '
              f'(cần < {node.depth_hold_tol * 100:.0f} cm; chỉnh PSC_* nếu dao động)')
    else:
        c.add('5. ALT_HOLD giữ độ sâu (10 s)', False, 'không đổi được sang ALT_HOLD (cảm biến độ sâu?)')

    # 6. Nổi dương: disarm dưới nước -> robot tự nổi và tự cân bằng ---------------------
    node.set_mode('MANUAL')
    node.set_cmd()
    node.arm(False, timeout=10.0)
    start_z = node.snapshot().z
    ok = wait_for(lambda: node.snapshot().z > -0.10, 60.0, 0.2)
    time.sleep(3.0)
    p = node.snapshot()
    level = abs(p.roll) < math.radians(5) and abs(p.pitch) < math.radians(5)
    c.add('6. Thả tay -> tự nổi + tự cân bằng', ok and level,
          f'từ z={start_z:.2f} lên z={p.z:.3f} m, roll {deg(p.roll):+.1f}°, pitch {deg(p.pitch):+.1f}°')

    # 7. Camera -------------------------------------------------------------------------
    node.image = None
    ok = wait_for(lambda: node.image is not None, 10.0)
    c.add('7. Camera /camera/image_raw', ok if ok else None,
          f'{node.image.width}x{node.image.height}' if ok else
          'không có ảnh (camera:=false, hoặc render lỗi trên WSL — thử render_engine:=ogre)')

    # 8. Failsafe ArduSub: ngừng gửi RC override -> ArduSub tự DISARM --------------------
    node.set_mode('MANUAL')
    if node.arm(True, timeout=20.0):
        time.sleep(1.0)
        node.send_override = False
        t0 = time.monotonic()
        limit = node.pilot_timeout + 3.0
        ok = wait_for(lambda: not node.state.armed, limit)
        c.add('8. Mất lệnh lái -> ArduSub DISARM', ok,
              f'disarm sau {time.monotonic() - t0:.1f} s (FS_PILOT_TIMEOUT {node.pilot_timeout} s)' if ok
              else f'vẫn ARMED sau {limit:.1f} s — kiểm tra FS_PILOT_INPUT (ArduSub < 4.7 tắt failsafe này trong SITL)')
        node.send_override = True
        node.arm(False, timeout=5.0)
    else:
        c.add('8. Mất lệnh lái -> ArduSub DISARM', False, 'không ARM lại được')

    # 9. safety_node: mất heartbeat máy điều khiển -> DISARM --------------------------------
    names = [n for n, _ in node.get_node_names_and_namespaces()]
    if 'safety_node' not in names:
        c.add('9. Mất heartbeat -> safety_node DISARM', None, 'safety_node không chạy (safety:=false)')
    elif node.arm(True, timeout=20.0):
        time.sleep(1.0)
        node.send_heartbeat = False
        t0 = time.monotonic()
        limit = node.hb_timeout + 3.0
        ok = wait_for(lambda: not node.state.armed, limit)
        c.add('9. Mất heartbeat -> safety_node DISARM', ok,
              f'disarm sau {time.monotonic() - t0:.1f} s' if ok else f'vẫn ARMED sau {limit:.1f} s')
        node.send_heartbeat = True
        node.arm(False, timeout=5.0)
    else:
        c.add('9. Mất heartbeat -> safety_node DISARM', False, 'không ARM lại được')
    return c


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCheck()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    code = 1
    try:
        checker = run(node)
        fails = [r for r in checker.results if r.status == 'FAIL']
        passes = [r for r in checker.results if r.status == 'PASS']
        print(f'\n=== KẾT QUẢ: {len(passes)} đạt, {len(fails)} không đạt, '
              f'{len(checker.results) - len(passes) - len(fails)} bỏ qua ===', flush=True)
        code = 0 if not fails else 1
    except KeyboardInterrupt:
        print('\nĐã dừng.')
    finally:
        node.set_cmd()
        time.sleep(0.2)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
