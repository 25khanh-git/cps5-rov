"""chase_camera: camera của cửa sổ Gazebo bám đuôi robot, chuyển động mượt và không lắc.

Vì sao không dùng "Follow" của Gazebo: Follow xoay camera theo cả ba góc nghiêng của
robot (roll, pitch, yaw), nên robot lắc nhẹ trong nước là hình cũng rung theo.

Node này đọc vị trí thật của robot (/cps5/ground_truth/odom), chỉ lấy vị trí + hướng đi
(yaw, có làm mượt), rồi đặt camera của cửa sổ Gazebo ở phía sau và hơi cao hơn robot,
nhìn chéo xuống robot (giống camera trong game đua xe). Camera bỏ qua roll/pitch.
Xoay vòng quanh robot bằng phím j / l / i / k / c trong keyboard_teleop (topic /chase_camera/orbit).

Gửi lệnh qua dịch vụ /gui/move_to/pose của Gazebo (cần gói apt
python3-gz-transport13).

Tham số (đổi khi chạy: ros2 run rov_bringup chase_camera --ros-args -p distance:=2.0):
  distance      khoảng cách phía sau robot (m)
  height        độ cao trên robot (m)
  yaw_smoothing hằng số thời gian làm mượt hướng (s) — lớn hơn = camera xoay chậm, êm hơn
  follow_heading true: camera luôn ở sau đuôi robot; false: camera giữ hướng cố định (nhìn về +x)
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
import math

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from rov_bringup.geometry import YawFilter, orbit_pose, quat_to_rpy

ORBIT_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ChaseCamera(Node):
    def __init__(self) -> None:
        super().__init__('chase_camera')
        p = self.declare_parameter
        self.distance = float(p('distance', 1.3).value)
        self.height = float(p('height', 0.5).value)
        self.rate = float(p('rate_hz', 10.0).value)
        self.follow_heading = bool(p('follow_heading', True).value)
        self.filter = YawFilter(float(p('yaw_smoothing', 0.8).value))
        self.odom: Optional[Odometry] = None
        self.lock = threading.Lock()
        self.create_subscription(Odometry, '/cps5/ground_truth/odom', self._on_odom,
                                 qos_profile_sensor_data)
        # Xoay vòng quanh robot: keyboard_teleop gửi (x = góc phương vị °, y = góc ngẩng °).
        # Bán kính giữ nguyên = khoảng cách mặc định.
        self.radius = math.hypot(self.distance, self.height)
        self.azimuth = 0.0
        self.elevation = math.atan2(self.height, self.distance)
        self.create_subscription(Vector3, '/chase_camera/orbit', self._on_orbit, ORBIT_QOS)

        try:
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.gui_camera_pb2 import GUICamera
            from gz.transport13 import Node as GzNode
        except ImportError:
            self.get_logger().error(
                'Thiếu thư viện Python của Gazebo. Cài bằng:\n'
                '    sudo apt install python3-gz-transport13')
            raise
        self._GUICamera, self._Boolean = GUICamera, Boolean
        self.gz = GzNode()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'chase_camera: sau {self.distance} m, cao {self.height} m, '
            f'{"bám hướng robot" if self.follow_heading else "hướng cố định"}')

    def _on_orbit(self, m: Vector3) -> None:
        with self.lock:
            self.azimuth = math.radians(m.x)
            self.elevation = math.radians(m.y)

    def _on_odom(self, m: Odometry) -> None:
        with self.lock:
            self.odom = m

    def _loop(self) -> None:
        period = 1.0 / self.rate
        warned = False
        while self.running and rclpy.ok():
            t0 = time.monotonic()
            with self.lock:
                m = self.odom
                azimuth, elevation = self.azimuth, self.elevation
            if m is not None:
                pos, q = m.pose.pose.position, m.pose.pose.orientation
                _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
                yaw = self.filter.update(yaw, t0) if self.follow_heading else 0.0
                (cx, cy, cz), (qx, qy, qz, qw) = orbit_pose(
                    pos.x, pos.y, pos.z, yaw + azimuth, elevation, self.radius)
                req = self._GUICamera()
                req.pose.position.x, req.pose.position.y, req.pose.position.z = cx, cy, cz
                req.pose.orientation.x, req.pose.orientation.y = qx, qy
                req.pose.orientation.z, req.pose.orientation.w = qz, qw
                ok, _ = self.gz.request('/gui/move_to/pose', req, self._GUICamera,
                                        self._Boolean, 500)
                if not ok and not warned:
                    self.get_logger().warn(
                        'Chưa gọi được /gui/move_to/pose — cửa sổ Gazebo đã mở chưa? (gui:=true)')
                    warned = True
                elif ok:
                    warned = False
            time.sleep(max(0.0, period - (time.monotonic() - t0)))


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ChaseCamera()
    except ImportError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
