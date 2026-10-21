"""Kiểm tra logic ánh xạ joystick -> PWM (chạy được không cần ROS: pytest src/rov_teleop/test)."""

from rov_teleop.keymap import HoldKeys, compass_deg
from rov_teleop.mapping import (CHAN_NOCHANGE, NEUTRAL, AxisMap, ButtonEdges, build_channels,
                                is_near_neutral, joy_to_command, read_axis, to_pwm)

MAPS = {
    'forward': AxisMap(1, 1.0),
    'lateral': AxisMap(0, -1.0),
    'yaw': AxisMap(2, -1.0),
    'throttle': AxisMap(3, 1.0),
}


def test_neutral_sticks_give_1500():
    cmd = joy_to_command([0, 0, 0, 0, 0, 0], MAPS, gain=1.0, pwm_range=400, deadzone=0.05)
    assert cmd == {'throttle': 1500, 'yaw': 1500, 'forward': 1500, 'lateral': 1500}


def test_directions_match_ardusub_convention():
    # cần trái lên (+1) -> tiến (>1500)
    assert joy_to_command([0, 1, 0, 0], MAPS, 1.0, 400, 0.05)['forward'] == 1900
    # cần trái sang PHẢI (joy = -1) -> lateral > 1500 (sang phải)
    assert joy_to_command([-1, 0, 0, 0], MAPS, 1.0, 400, 0.05)['lateral'] == 1900
    # cần phải sang PHẢI (joy = -1) -> yaw > 1500 (quay phải)
    assert joy_to_command([0, 0, -1, 0], MAPS, 1.0, 400, 0.05)['yaw'] == 1900
    # cần phải xuống (-1) -> throttle < 1500 (lặn)
    assert joy_to_command([0, 0, 0, -1], MAPS, 1.0, 400, 0.05)['throttle'] == 1100


def test_gain_limits_range_and_clamps():
    assert to_pwm(1.0, 0.5, 400) == 1700
    assert to_pwm(-1.0, 0.5, 400) == 1300
    assert to_pwm(5.0, 1.0, 400) == 1900
    assert to_pwm(1.0, 3.0, 400) == 1900


def test_deadzone_and_missing_axis():
    assert read_axis([0.04], AxisMap(0), 0.05) == 0.0
    assert read_axis([], AxisMap(3), 0.05) == 0.0
    v = read_axis([0.06], AxisMap(0), 0.05)
    assert 0.0 < v < 0.02  # liền mạch sau vùng chết


def test_channel_layout():
    ch = build_channels({'throttle': 1400, 'yaw': 1550, 'forward': 1600, 'lateral': 1450})
    assert len(ch) == 18
    assert ch[:6] == [NEUTRAL, NEUTRAL, 1400, 1550, 1600, 1450]
    assert all(c == CHAN_NOCHANGE for c in ch[6:])


def test_button_edges():
    e = ButtonEdges()
    assert e.pressed([0, 1, 0]) == [1]
    assert e.pressed([0, 1, 0]) == []        # giữ nút: không lặp
    assert e.pressed([1, 0, 0, 1]) == [0, 3]


def test_arm_neutral_check():
    assert is_near_neutral([0, 0, 0.9, 0], MAPS, 0.05)        # yaw không tính
    assert not is_near_neutral([0, 0.5, 0, 0], MAPS, 0.05)    # đang đẩy tiến


def test_keyboard_layout_matches_teleop_defaults():
    k = HoldKeys(release_timeout=0.6)
    for key in 'wder':
        k.press(key, 0.0)
    cmd = joy_to_command(k.axes(0.1), MAPS, 1.0, 400, 0.05)
    assert cmd['forward'] > 1500      # w
    assert cmd['lateral'] > 1500      # d = sang phải
    assert cmd['throttle'] > 1500     # r = nổi lên
    assert cmd['yaw'] > 1500          # e = quay phải


def test_hold_then_release_stops():
    k = HoldKeys(release_timeout=0.6)
    for t in (0.0, 0.5, 0.53, 0.56):  # giữ phím: OS lặp lại phím
        k.press('w', t)
    assert k.axes(1.0)[1] == 1.0       # vẫn đang giữ
    assert k.axes(1.2)[1] == 0.0       # nhả > 0.6 s -> dừng
    k.press('w', 2.0)
    k.press('s', 2.1)                  # phím ngược chiều ghi đè
    assert k.axes(2.2)[1] == -1.0
    k.press(' ', 2.3)
    assert k.axes(2.3) == [0.0] * 6


def test_buttons_pulse_then_clear():
    k = HoldKeys()
    k.press('1', 0.0)
    assert k.take_buttons()[6] == 1
    k.take_buttons()
    k.take_buttons()
    assert k.take_buttons()[6] == 0


def test_compass():
    import math
    assert compass_deg(0.0) == 90.0                 # ENU Đông -> la bàn 90
    assert abs(compass_deg(math.pi / 2)) < 1e-9     # Bắc -> 0
    assert compass_deg(-math.pi / 2) == 180.0       # Nam


def test_orbit_keys():
    from rov_teleop.keymap import OrbitState
    o = OrbitState(elevation=21.0)
    for _ in range(9):
        o.press('j')
    assert o.azimuth == -180.0          # 180° = nhìn từ phía trước robot (quấn góc)
    o.press('l')
    assert o.azimuth == 160.0
    for _ in range(20):
        o.press('i')
    assert o.elevation == OrbitState.ELEV_MAX
    o.press('c')
    assert (o.azimuth, o.elevation) == (0.0, 21.0)
    assert not o.press('w')
