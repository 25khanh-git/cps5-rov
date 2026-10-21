"""Bố cục phím cho keyboard_teleop (logic thuần, không phụ thuộc ROS).

Kiểu "GIỮ PHÍM": giữ phím thì robot chạy, nhả ra thì dừng.
Terminal không báo sự kiện nhả phím. Nhưng khi GIỮ phím, hệ điều hành tự lặp lại phím
(~30 lần/giây, sau độ trễ đầu ~0.5 s). Vì vậy một trục coi là "đang giữ" nếu phím của nó
được nhận trong vòng `release_timeout` giây gần nhất; quá thời gian đó thì về 0.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

NUM_AXES = 6
NUM_BUTTONS = 15
# Bố cục mặc định (teleop.yaml): axes 0 lateral(trái=+), 1 forward, 2 yaw(trái=+), 3 throttle
AXIS_KEYS = {
    'w': (1, +1), 's': (1, -1),
    'a': (0, +1), 'd': (0, -1),
    'q': (2, +1), 'e': (2, -1),
    'r': (3, +1), 'f': (3, -1),
}
BUTTON_KEYS = {'1': 6, '0': 4, 'm': 0, 't': 1, 'h': 3, '[': 9, ']': 10}
PULSE_TICKS = 3   # giữ nút "nhấn" trong 3 chu kỳ để teleop_node chắc chắn thấy


class HoldKeys:
    """Trạng thái các trục theo kiểu giữ-phím."""

    def __init__(self, release_timeout: float = 0.6, level: float = 1.0) -> None:
        self.release_timeout = release_timeout
        self.level = level
        self._held: Dict[int, Tuple[int, float]] = {}   # trục -> (hướng, thời điểm nhận phím)
        self.pulses: Dict[int, int] = {}

    def press(self, key: str, now: float) -> bool:
        """Xử lý một phím. Trả về True nếu phím có ý nghĩa."""
        k = key.lower()
        if k in AXIS_KEYS:
            idx, direction = AXIS_KEYS[k]
            self._held[idx] = (direction, now)     # phím ngược chiều ghi đè ngay
            return True
        if key == ' ':
            self._held.clear()
            return True
        if k in BUTTON_KEYS:
            self.pulses[BUTTON_KEYS[k]] = PULSE_TICKS
            return True
        return False

    def axes(self, now: float) -> List[float]:
        out = [0.0] * NUM_AXES
        for idx, (direction, t) in list(self._held.items()):
            if now - t <= self.release_timeout:
                out[idx] = direction * self.level
            else:
                del self._held[idx]
        return out

    def take_buttons(self) -> List[int]:
        """Mảng nút cho một lần phát /joy (mỗi lần gọi giảm bộ đếm xung)."""
        buttons = [0] * NUM_BUTTONS
        for idx in list(self.pulses):
            buttons[idx] = 1
            self.pulses[idx] -= 1
            if self.pulses[idx] <= 0:
                del self.pulses[idx]
        return buttons


def compass_deg(enu_yaw_rad: float) -> float:
    """Yaw ENU (0 = Đông, ngược chiều kim đồng hồ) -> hướng la bàn (0 = Bắc, thuận chiều)."""
    import math
    return (90.0 - math.degrees(enu_yaw_rad)) % 360.0


# ----------------------------------------------------------------------------- camera
ORBIT_KEYS = {
    'j': ('azimuth', +20.0), 'l': ('azimuth', -20.0),     # xoay quanh robot sang trái / phải
    'i': ('elevation', +10.0), 'k': ('elevation', -10.0),  # nhìn từ cao hơn / thấp hơn
}


class OrbitState:
    """Góc nhìn camera quanh robot (độ). azimuth 0 = sau đuôi; 180 = trước mặt robot."""

    ELEV_MIN, ELEV_MAX = -30.0, 85.0

    def __init__(self, elevation: float = 21.0) -> None:
        self.default_elevation = elevation
        self.azimuth = 0.0
        self.elevation = elevation

    def press(self, key: str) -> bool:
        k = key.lower()
        if k == 'c':
            self.azimuth, self.elevation = 0.0, self.default_elevation
            return True
        if k not in ORBIT_KEYS:
            return False
        name, step = ORBIT_KEYS[k]
        if name == 'azimuth':
            self.azimuth = (self.azimuth + step + 180.0) % 360.0 - 180.0
        else:
            self.elevation = max(self.ELEV_MIN, min(self.ELEV_MAX, self.elevation + step))
        return True
