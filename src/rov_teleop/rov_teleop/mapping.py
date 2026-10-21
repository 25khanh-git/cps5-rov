"""Logic thuần (không phụ thuộc ROS): joystick -> giá trị PWM cho các kênh RC của ArduSub.

Kênh RC mặc định của ArduSub (RCMAP):
    1 pitch, 2 roll, 3 throttle (lên/xuống), 4 yaw, 5 forward, 6 lateral
1500 = trung tính. Ở MANUAL, throttle 1500 = không có lực đứng;
ở STABILIZE/ALT_HOLD, throttle 1500 = giữ nguyên.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900
NUM_CHANNELS = 18          # mavros_msgs/OverrideRCIn có 18 kênh
CHAN_NOCHANGE = 65535      # "không đụng tới kênh này"

# kênh (1-based) -> tên
CHANNELS = {1: 'pitch', 2: 'roll', 3: 'throttle', 4: 'yaw', 5: 'forward', 6: 'lateral'}
CONTROL_CHANNELS = ('throttle', 'yaw', 'forward', 'lateral')


@dataclass
class AxisMap:
    index: int
    sign: float = 1.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def read_axis(axes: Sequence[float], amap: AxisMap, deadzone: float) -> float:
    """Giá trị trục trong [-1, 1] đã áp dấu và vùng chết. Trục không tồn tại -> 0."""
    if amap.index < 0 or amap.index >= len(axes):
        return 0.0
    v = clamp(float(axes[amap.index]), -1.0, 1.0) * amap.sign
    if abs(v) < deadzone:
        return 0.0
    # nối liền sau vùng chết để không bị "giật" ở mép
    return (abs(v) - deadzone) / (1.0 - deadzone) * (1.0 if v > 0 else -1.0)


def to_pwm(value: float, gain: float, pwm_range: int) -> int:
    pwm = NEUTRAL + clamp(value, -1.0, 1.0) * clamp(gain, 0.0, 1.0) * pwm_range
    return int(round(clamp(pwm, PWM_MIN, PWM_MAX)))


def neutral_command() -> Dict[str, int]:
    return {name: NEUTRAL for name in CONTROL_CHANNELS}


def joy_to_command(axes: Sequence[float], maps: Dict[str, AxisMap], gain: float,
                   pwm_range: int, deadzone: float) -> Dict[str, int]:
    return {name: to_pwm(read_axis(axes, maps[name], deadzone), gain, pwm_range)
            for name in CONTROL_CHANNELS}


def build_channels(cmd: Dict[str, int]) -> List[int]:
    """Mảng 18 kênh cho OverrideRCIn. Kênh 1-2 (pitch/roll) giữ 1500; kênh >6 không đổi."""
    ch = [CHAN_NOCHANGE] * NUM_CHANNELS
    ch[0] = NEUTRAL                       # 1 pitch
    ch[1] = NEUTRAL                       # 2 roll
    ch[2] = int(cmd.get('throttle', NEUTRAL))
    ch[3] = int(cmd.get('yaw', NEUTRAL))
    ch[4] = int(cmd.get('forward', NEUTRAL))
    ch[5] = int(cmd.get('lateral', NEUTRAL))
    return ch


def is_near_neutral(axes: Sequence[float], maps: Dict[str, AxisMap], deadzone: float,
                    tol: float = 0.1) -> bool:
    return all(abs(read_axis(axes, maps[n], deadzone)) <= tol for n in ('throttle', 'forward'))


class ButtonEdges:
    """Phát hiện cạnh lên của nút (nhấn = 1 lần, không lặp khi giữ)."""

    def __init__(self) -> None:
        self._prev: List[int] = []

    def pressed(self, buttons: Sequence[int]) -> List[int]:
        prev = self._prev + [0] * max(0, len(buttons) - len(self._prev))
        edges = [i for i, b in enumerate(buttons) if b and not prev[i]]
        self._prev = list(buttons)
        return edges
