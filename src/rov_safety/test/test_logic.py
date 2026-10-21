"""Kiểm tra logic an toàn (không cần ROS): pytest src/rov_safety/test"""

from rov_safety.logic import DISARM, SURFACE, SafetyConfig, SafetyMonitor


def make():
    m = SafetyMonitor(SafetyConfig(heartbeat_timeout=2.0, battery_warn=10.5,
                                   battery_critical=9.9, critical_hold=3.0, max_depth=5.0))
    m.on_state(0.0, connected=True, armed=False, mode='MANUAL')
    return m


def test_no_action_when_disarmed_or_disconnected():
    m = make()
    assert m.evaluate(100.0) == []
    m.on_state(1.0, connected=False, armed=True, mode='MANUAL')
    assert m.evaluate(100.0) == []


def test_heartbeat_loss_disarms_once_then_resends():
    m = make()
    m.on_heartbeat(10.0)
    m.on_state(10.0, True, True, 'MANUAL')
    assert m.evaluate(11.5) == []                       # còn trong hạn
    m.on_heartbeat(11.5)
    assert m.evaluate(13.0) == []
    acts = m.evaluate(13.6)                             # 2.1 s không có heartbeat
    assert [a.kind for a in acts] == [DISARM]
    assert m.evaluate(14.0) == []                       # không spam
    assert [a.kind for a in m.evaluate(15.7)] == [DISARM]   # gửi lại sau 2 s nếu vẫn ARM


def test_grace_period_after_arming_without_heartbeat():
    m = make()
    m.on_state(50.0, True, True, 'MANUAL')              # ARM mà chưa có heartbeat
    assert m.evaluate(51.0) == []
    assert [a.kind for a in m.evaluate(52.5)] == [DISARM]


def test_old_heartbeat_does_not_skip_grace():
    m = make()
    m.on_heartbeat(1.0)
    m.on_state(100.0, True, True, 'MANUAL')
    assert m.evaluate(101.0) == []


def test_critical_battery_needs_hold_time_then_surface():
    m = make()
    m.on_state(0.0, True, True, 'MANUAL')
    for t in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
        m.on_heartbeat(t)
    m.on_battery(1.0, 9.5)
    assert m.evaluate(2.0) == []
    m.on_heartbeat(4.0)
    assert [a.kind for a in m.evaluate(4.1)] == [SURFACE]
    m.on_state(4.2, True, True, 'SURFACE')
    m.on_heartbeat(4.5)
    assert m.evaluate(4.6) == []                        # đã SURFACE rồi


def test_battery_recovers_resets_timer():
    m = make()
    m.on_state(0.0, True, True, 'MANUAL')
    m.on_battery(0.0, 9.5)
    m.on_battery(1.0, 10.0)
    m.on_heartbeat(5.0)
    assert m.evaluate(5.0) == []


def test_low_battery_only_warns():
    m = make()
    m.on_state(0.0, True, True, 'MANUAL')
    m.on_heartbeat(0.5)
    m.on_battery(0.5, 10.2)
    assert m.evaluate(1.0) == []
    assert any('pin yếu' in w for w in m.warnings)


def test_too_deep_surfaces():
    m = make()
    m.on_state(0.0, True, True, 'ALT_HOLD')
    m.on_heartbeat(0.5)
    m.on_depth(5.5)
    assert [a.kind for a in m.evaluate(1.0)] == [SURFACE]
