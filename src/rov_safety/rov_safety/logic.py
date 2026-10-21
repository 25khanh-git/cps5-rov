"""Logic an toàn thuần (không phụ thuộc ROS) — dễ kiểm thử.

SafetyMonitor nhận các quan sát (thời gian, trạng thái ArduSub, pin, độ sâu,
heartbeat của máy điều khiển) và trả về danh sách hành động cần làm.
Lớp này là lớp bảo vệ THỨ HAI: failsafe chính nằm trong ArduSub
(FS_PILOT_INPUT, FS_GCS_ENABLE, BATT_FS_*), chạy ngay trên Pixhawk.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

DISARM = 'disarm'
SURFACE = 'surface'


@dataclass
class SafetyConfig:
    heartbeat_timeout: float = 2.0
    battery_warn: float = 10.5
    battery_critical: float = 9.9
    critical_hold: float = 3.0
    max_depth: float = 5.0


@dataclass
class Action:
    kind: str            # DISARM | SURFACE
    reason: str


@dataclass
class SafetyMonitor:
    cfg: SafetyConfig = field(default_factory=SafetyConfig)
    armed: bool = False
    mode: str = ''
    connected: bool = False
    last_heartbeat: Optional[float] = None
    armed_since: Optional[float] = None
    voltage: Optional[float] = None
    critical_since: Optional[float] = None
    depth: Optional[float] = None
    warnings: List[str] = field(default_factory=list)
    _disarm_sent_at: Optional[float] = None
    _surface_sent_at: Optional[float] = None

    # ------------------------------------------------------------- inputs
    def on_state(self, now: float, connected: bool, armed: bool, mode: str) -> None:
        if armed and not self.armed:
            self.armed_since = now
        if not armed:
            self.armed_since = None
            self._disarm_sent_at = None
        if mode != 'SURFACE':
            self._surface_sent_at = None
        self.connected, self.armed, self.mode = connected, armed, mode

    def on_heartbeat(self, now: float) -> None:
        self.last_heartbeat = now

    def on_battery(self, now: float, voltage: float) -> None:
        self.voltage = voltage
        if voltage > 0 and voltage < self.cfg.battery_critical:
            if self.critical_since is None:
                self.critical_since = now
        else:
            self.critical_since = None

    def on_depth(self, depth_m: float) -> None:
        """depth_m dương = dưới mặt nước."""
        self.depth = depth_m

    # ------------------------------------------------------------- decision
    def heartbeat_age(self, now: float) -> Optional[float]:
        return None if self.last_heartbeat is None else now - self.last_heartbeat

    def evaluate(self, now: float, resend_after: float = 2.0) -> List[Action]:
        """Gọi định kỳ. Mỗi hành động chỉ được gửi lại sau resend_after giây."""
        self.warnings = []
        actions: List[Action] = []
        if not self.connected:
            self.warnings.append('MAVROS chưa kết nối ArduSub')
            return actions
        if not self.armed:
            return actions

        # 1) Máy điều khiển im lặng quá lâu (hoặc ARM mà chưa từng có heartbeat)
        #    Mốc = heartbeat gần nhất, nhưng luôn có heartbeat_timeout giây "ân hạn" sau khi ARM.
        marks = [t for t in (self.last_heartbeat, self.armed_since) if t is not None]
        ref = max(marks) if marks else now
        if now - ref > self.cfg.heartbeat_timeout:
            if self._due(self._disarm_sent_at, now, resend_after):
                self._disarm_sent_at = now
                actions.append(Action(DISARM, f'mất heartbeat máy điều khiển {now - ref:.1f} s'))
            return actions  # đã disarm thì không cần xét tiếp

        # 2) Pin
        if self.voltage is not None and 0 < self.voltage < self.cfg.battery_warn:
            self.warnings.append(f'pin yếu {self.voltage:.2f} V')
        if self.critical_since is not None and now - self.critical_since >= self.cfg.critical_hold:
            if self.mode != 'SURFACE' and self._due(self._surface_sent_at, now, resend_after):
                self._surface_sent_at = now
                actions.append(Action(SURFACE, f'pin nguy hiểm {self.voltage:.2f} V'))

        # 3) Quá sâu
        if self.depth is not None and self.depth > self.cfg.max_depth:
            if self.mode != 'SURFACE' and self._due(self._surface_sent_at, now, resend_after):
                self._surface_sent_at = now
                actions.append(Action(SURFACE, f'quá sâu {self.depth:.2f} m > {self.cfg.max_depth} m'))
        return actions

    @staticmethod
    def _due(last: Optional[float], now: float, period: float) -> bool:
        return last is None or now - last >= period
