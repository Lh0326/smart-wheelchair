"""挤压抬头纹检测：能量阈值 + 时间窗 + 冷却。

输入：emg_level（FocusResult.emg_level，60-95Hz / 1-40Hz 功率比）
输出：FrownResult
"""
from dataclasses import dataclass


@dataclass
class FrownResult:
    is_frowning: bool    # 当前是否处于已确认的抬头纹状态
    event: bool          # rising edge：本轮刚从未确认进入确认，触发一次事件


class FrownDetector:
    """挤压抬头纹检测：能量阈值 + 时间窗 + 冷却。

    输入：emg_level（FocusResult.emg_level，60-95Hz / 1-40Hz 功率比）
    输出：FrownResult
    """

    def __init__(self, threshold: float = 1.5,
                 min_ms: int = 400,
                 max_ms: int = 800,
                 cooldown_ms: int = 1500):
        self.threshold = threshold
        self.min_ms = min_ms
        self.max_ms = max_ms
        self.cooldown_ms = cooldown_ms
        self._above_ms = 0
        self._triggered = False
        self._cooldown_left = 0

    def update(self, emg_level: float, dt_ms: int) -> FrownResult:
        event = False
        above = emg_level >= self.threshold

        if self._cooldown_left > 0:
            self._cooldown_left -= dt_ms

        if above:
            # 冷却期内不累积时间窗，等冷却结束再开始新的尝试
            if self._cooldown_left <= 0:
                self._above_ms += dt_ms
                # 超过 max_ms：放弃本次（持续皱眉不算切换），优先于触发判定
                if self._above_ms > self.max_ms:
                    self._triggered = False
                    self._above_ms = 0
                # 首次确认：持续 >= min_ms 且仍在 [min, max] 区间内
                elif (self._above_ms >= self.min_ms and not self._triggered):
                    self._triggered = True
                    event = True
        else:
            # 信号回落：若刚刚触发过，启动冷却期
            if self._triggered:
                self._cooldown_left = self.cooldown_ms
            self._triggered = False
            self._above_ms = 0

        return FrownResult(is_frowning=self._triggered, event=event)
