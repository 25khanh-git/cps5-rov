"""Hàm hình học nhỏ (không phụ thuộc ROS) dùng cho check_sim."""

from __future__ import annotations

import math
from typing import Optional, Tuple


def wrap_pi(a: float) -> float:
    """Đưa góc về [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Quaternion -> (roll, pitch, yaw), quy ước ZYX như ROS/tf."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def body_displacement(x0: float, y0: float, yaw0: float, x1: float, y1: float) -> Tuple[float, float]:
    """Độ dời (tiến, sang PHẢI) trong hệ thân lúc bắt đầu. Thế giới ENU, yaw ngược chiều kim đồng hồ."""
    dx, dy = x1 - x0, y1 - y0
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    left = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    return forward, -left


def orbit_pose(x: float, y: float, z: float, view_yaw: float, elevation: float,
               radius: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Camera trên mặt cầu bán kính `radius` quanh robot, nhìn vào robot.

    view_yaw: hướng camera NHÌN (ENU); camera đứng phía sau theo hướng đó.
    elevation: góc nhìn chúi xuống (rad, dương = camera cao hơn robot).
    """
    return chase_pose(x, y, z, view_yaw, radius * math.cos(elevation), radius * math.sin(elevation))


def chase_pose(x: float, y: float, z: float, yaw: float, distance: float,
               height: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Vị trí + quaternion (x, y, z, w) của camera đặt sau robot, nhìn vào robot.

    Camera Gazebo nhìn theo +x của nó; pitch dương = chúi xuống.
    """
    cx = x - distance * math.cos(yaw)
    cy = y - distance * math.sin(yaw)
    cz = z + height
    pitch = math.atan2(height, distance)
    # quaternion từ roll=0, pitch, yaw (ZYX)
    cyw, syw = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    q = (-syw * sp, cyw * sp, syw * cp, cyw * cp)
    return (cx, cy, cz), q


class YawFilter:
    """Lọc thông thấp cho góc, xử lý đúng chỗ nhảy ±180°."""

    def __init__(self, tau: float) -> None:
        self.tau = max(tau, 1e-3)
        self.value: Optional[float] = None
        self._t: Optional[float] = None

    def update(self, yaw: float, t: float) -> float:
        if self.value is None:
            self.value, self._t = yaw, t
            return yaw
        dt = max(0.0, t - self._t)
        self._t = t
        alpha = 1.0 - math.exp(-dt / self.tau)
        self.value = wrap_pi(self.value + alpha * wrap_pi(yaw - self.value))
        return self.value
