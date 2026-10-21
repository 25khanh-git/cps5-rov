import math

from rov_bringup.geometry import body_displacement, quat_to_rpy, wrap_pi


def test_wrap():
    assert abs(wrap_pi(3 * math.pi / 2) + math.pi / 2) < 1e-9
    assert abs(wrap_pi(-3 * math.pi / 2) - math.pi / 2) < 1e-9


def test_quat_yaw_90():
    s = math.sqrt(0.5)
    r, p, y = quat_to_rpy(0, 0, s, s)
    assert abs(y - math.pi / 2) < 1e-9 and abs(r) < 1e-9 and abs(p) < 1e-9


def test_body_displacement_heading_east_and_north():
    # mũi hướng Đông (+x): đi tới +x là tiến, đi -y (Nam) là sang phải
    assert body_displacement(0, 0, 0.0, 1, 0) == (1.0, -0.0)
    f, r = body_displacement(0, 0, 0.0, 0, -1)
    assert abs(f) < 1e-9 and abs(r - 1) < 1e-9
    # mũi hướng Bắc (+y): đi +x (Đông) là sang phải
    f, r = body_displacement(0, 0, math.pi / 2, 1, 0)
    assert abs(f) < 1e-9 and abs(r - 1) < 1e-9


def test_chase_camera_behind_and_looking_at_robot():
    from rov_bringup.geometry import chase_pose
    for yaw in (0.0, 1.0, -2.5, math.pi):
        (cx, cy, cz), (qx, qy, qz, qw) = chase_pose(2.0, -1.0, -1.0, yaw, 1.3, 0.5)
        # hướng nhìn = trục x của camera xoay theo quaternion
        fx = 1 - 2 * (qy * qy + qz * qz)
        fy = 2 * (qx * qy + qw * qz)
        fz = 2 * (qx * qz - qw * qy)
        to_robot = (2.0 - cx, -1.0 - cy, -1.0 - cz)
        n = math.sqrt(sum(c * c for c in to_robot))
        dot = (fx * to_robot[0] + fy * to_robot[1] + fz * to_robot[2]) / n
        assert dot > 0.999                     # nhìn thẳng vào robot
        assert cz > -1.0                       # cao hơn robot
        # nằm phía sau đuôi robot
        assert (cx - 2.0) * math.cos(yaw) + (cy + 1.0) * math.sin(yaw) < 0


def test_yaw_filter_wraps_and_smooths():
    from rov_bringup.geometry import YawFilter
    f = YawFilter(tau=1.0)
    assert f.update(3.1, 0.0) == 3.1
    v = f.update(-3.1, 0.1)                    # qua mốc ±180°: chỉ dịch một chút
    assert abs(math.cos(v) - math.cos(3.1)) < 0.01 and abs(abs(v) - 3.1) < 0.1
    for i in range(1, 100):
        v = f.update(-3.1, 0.1 + i * 0.1)
    assert abs(v - (-3.1)) < 1e-3


def test_orbit_keeps_radius_and_looks_at_robot():
    from rov_bringup.geometry import orbit_pose
    for az in (0.0, 1.5, -2.0):
        for el in (0.1, 0.6, 1.3):
            (cx, cy, cz), (qx, qy, qz, qw) = orbit_pose(1.0, 2.0, -1.5, az, el, 1.4)
            d = (1.0 - cx, 2.0 - cy, -1.5 - cz)
            r = math.sqrt(sum(c * c for c in d))
            assert abs(r - 1.4) < 1e-9
            fx = 1 - 2 * (qy * qy + qz * qz)
            fy = 2 * (qx * qy + qw * qz)
            fz = 2 * (qx * qz - qw * qy)
            assert (fx * d[0] + fy * d[1] + fz * d[2]) / r > 0.999
