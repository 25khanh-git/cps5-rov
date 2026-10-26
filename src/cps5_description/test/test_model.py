"""Kiểm tra bộ sinh SDF: số liệu nhất quán, XML hợp lệ, thứ tự motor đúng Vectored-6DOF."""

import copy
import math
import xml.etree.ElementTree as ET

import pytest

from cps5_description import model
from cps5_gazebo import world

P = model.load_params()


def test_buoyancy_matches_margin():
    d = model.derive(P)
    margin = P['vehicle']['buoyancy_margin']
    assert d.buoyancy_n == pytest.approx(d.weight_n * (1 + margin), rel=1e-9)
    # thể tích hộp collision == thể tích chiếm chỗ
    h = P['hull']
    assert h['length'] * h['width'] * d.hull_height == pytest.approx(d.displaced_volume)


def test_total_mass_in_sdf_equals_yaml():
    root = ET.fromstring(model.standalone_model_sdf(P))
    masses = [float(m.text) for m in root.iter('mass')]
    assert sum(masses) == pytest.approx(P['vehicle']['mass'])


def test_thruster_layout_matches_ardusub_vectored_6dof():
    """Kiểm tra vị trí/hướng thruster cho ra đúng dấu lực/mô-men mà ArduSub mong đợi.

    Hệ thân FRD của ArduSub: x tới, y phải, z xuống. Model: FLU (y trái, z lên).
    Hệ số trong AP_Motors6DOF.cpp, Sub-4.7.2, SUB_FRAME_VECTORED_6DOF
    (roll, pitch, yaw, throttle, forward, lateral).
    Thành phần = 0 chỉ được phép lệch nhỏ (thruster ngang nằm hơi thấp hơn trọng tâm
    nên sinh chút mô-men roll/pitch — ArduSub bỏ qua, robot thật cũng vậy).
    """
    expected = [
        (0, 0, 1, 0, -1, 1),
        (0, 0, -1, 0, -1, -1),
        (0, 0, -1, 0, 1, 1),
        (0, 0, 1, 0, 1, -1),
        (1, -1, 0, -1, 0, 0),
        (-1, -1, 0, -1, 0, 0),
        (1, 1, 0, -1, 0, 0),
        (-1, 1, 0, -1, 0, 0),
    ]
    assert len(P['thrusters']['list']) == len(expected)
    com_z = model.derive(P).com_z
    for t, exp in zip(P['thrusters']['list'], expected):
        # chuyển FLU -> FRD
        r = (t['xyz'][0], -t['xyz'][1], -(t['xyz'][2] - com_z))
        f = (t['axis'][0], -t['axis'][1], -t['axis'][2])
        torque = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
        climb = -f[2]           # throttle dương = đi LÊN = -z (FRD)
        actual = (*torque, climb, f[0], f[1])
        for name, a, e in zip(('roll', 'pitch', 'yaw', 'throttle', 'forward', 'lateral'), actual, exp):
            if e == 0:
                assert abs(a) < 0.03, f"{t['name']} {name} phải ~0, đang {a:+.3f}"
            else:
                assert math.copysign(1, a) == e, f"{t['name']} {name} sai dấu ({a:+.3f})"


def test_ardupilot_plugin_channels_in_order():
    root = ET.fromstring(model.standalone_model_sdf(P))
    plugin = [p for p in root.iter('plugin') if p.get('name') == 'ArduPilotPlugin'][0]
    controls = plugin.findall('control')
    assert [c.get('channel') for c in controls] == [str(i) for i in range(model.NUM_THRUSTERS)]
    assert [c.findtext('jointName') for c in controls] == [
        f'thruster{i}_joint' for i in range(1, model.NUM_THRUSTERS + 1)]
    for c in controls:
        assert float(c.findtext('offset')) == -0.5
        assert float(c.findtext('multiplier')) == pytest.approx(2 * P['thrusters']['max_thrust'])


def test_world_is_valid_xml_and_contains_robot():
    vehicle = copy.deepcopy(P)
    w = world.load_world_params()
    root = ET.fromstring(world.world_sdf(vehicle, w).split('\n', 1)[1])
    names = [m.get('name') for m in root.find('world').findall('model')]
    assert 'cps5' in names
    buoy = [p for p in root.iter('plugin') if p.get('name') == 'gz::sim::systems::Buoyancy'][0]
    assert float(buoy.findtext('graded_buoyancy/default_density')) == P['water']['density']


def test_camera_can_be_disabled():
    vehicle = copy.deepcopy(P)
    vehicle['camera']['enabled'] = False
    text = world.world_sdf(vehicle, world.load_world_params())
    assert 'type="camera"' not in text and 'Sensors' not in text


def test_validation_rejects_bad_values():
    bad = copy.deepcopy(P)
    bad['thrusters']['list'] = bad['thrusters']['list'][:4]
    with pytest.raises(ValueError):
        model.validate(bad)
