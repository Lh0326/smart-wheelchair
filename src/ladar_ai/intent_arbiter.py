"""意图仲裁器：从多个意图源中选出当前活跃意图。

优先级（高到低）：
    emergency_stop > teleop > bci > voice > cruise
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from ladar_ai.vfh_plus import Intent, Twist2D


@dataclass
class ArbiterConfig:
    intent_timeout_teleop_sec: float = 2.0
    intent_timeout_bci_sec: float = 0.5
    intent_timeout_voice_sec: float = 1.0
    cruise_speed_m_s: float = 0.0
    bci_max_speed_m_s: float = 0.4


class IntentArbiter:
    """按优先级和时效性仲裁多源意图。"""

    def __init__(self, config: ArbiterConfig):
        self.cfg = config
        self._intents: Dict[str, Intent] = {}

    def update(self, source: str, twist: Twist2D, now: float) -> None:
        """更新某个意图源的最新输入。"""
        if source == "bci":
            cap = self.cfg.bci_max_speed_m_s
            twist = Twist2D(
                linear_x=max(-cap, min(cap, twist.linear_x)),
                angular_z=twist.angular_z,
            )
        self._intents[source] = Intent(twist=twist, source=source, timestamp=now)

    def _is_active(self, source: str, now: float) -> bool:
        intent = self._intents.get(source)
        if intent is None:
            return False
        if source == "teleop":
            timeout = self.cfg.intent_timeout_teleop_sec
        elif source == "bci":
            timeout = self.cfg.intent_timeout_bci_sec
        elif source == "voice":
            timeout = self.cfg.intent_timeout_voice_sec
        else:
            timeout = 1.0
        return (now - intent.timestamp) <= timeout

    def get_active_intent(self, now: float) -> Intent:
        """返回当前应执行的最高优先级意图。"""
        # emergency_stop 始终优先（5 秒窗口）
        es = self._intents.get("emergency_stop")
        if es is not None and (now - es.timestamp) <= 5.0:
            return es

        for source in ("teleop", "bci", "voice"):
            if self._is_active(source, now):
                return self._intents[source]

        # 兜底：巡航
        return Intent(
            twist=Twist2D(linear_x=self.cfg.cruise_speed_m_s, angular_z=0.0),
            source="cruise",
            timestamp=now,
        )
