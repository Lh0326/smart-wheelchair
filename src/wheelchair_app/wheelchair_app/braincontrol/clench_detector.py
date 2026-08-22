"""咬牙检测：EMG 特征 SVM + 时间窗确认 + 冷却。

与 FrownDetector 的差异：
  - FrownDetector 只接 emg_level（标量），不存历史信号
  - ClenchDetector 接收 2s 的 8 通道 EEG 窗口，自己提取 11 维 EMG 特征
    → 喂 SVM → 取 P(clench) → 阈值判断 → 状态机确认

接口（与 frown_detector.FrownResult 对齐，多一个 proba 字段方便调试）：

    detector = ClenchDetector(model_path='models/clench_svm.joblib', fs=500)
    result = detector.update(eeg_window_2s, dt_ms=100)
    if result.event:
        # rising edge：从未咬牙 → 咬牙，触发一次"紧急锁定"信号

main_eeg.py 集成：每收到一帧 ADS1299 数据，把最近 2s 缓冲拼成 (1000, 8)
传给 update()；内部维护 SVM 调用节流（默认每 0.5s 推断一次，节省 CPU）。

阈值默认 0.5（二分类标准）；min_ms 600ms（咬牙需要持续半秒才算意图）；
cooldown 1500ms（避免连续咬牙齿重复触发）。
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np
from joblib import load as _joblib_load

from .clench_features import (
    CLENCH_STEP_SEC,
    CLENCH_WINDOW_SEC,
    extract_clench_features,
)


@dataclass
class ClenchResult:
    """咬牙检测单帧结果。"""
    is_clenching: bool    # 当前是否处于已确认的咬牙状态
    event: bool           # rising edge：本轮刚从未确认 → 确认，触发一次
    proba: float = 0.0    # SVM 输出 P(clench)，0-1；调试用


class ClenchDetector:
    """咬牙检测：SVM 特征 + 时间窗确认 + 冷却（参考 FrownDetector 状态机）。"""

    def __init__(self,
                 model_path: str = 'models/clench_svm.joblib',
                 fs: int = 500,
                 threshold: float = 0.5,
                 min_ms: int = 600,
                 max_ms: int = 3000,
                 cooldown_ms: int = 1500,
                 infer_every_ms: Optional[int] = None):
        """
        Args:
            model_path: train_clench_svm.save_clench_model 输出的 .joblib
            fs: 采样率 Hz
            threshold: P(clench) 超过此值视为"窗口级咬牙"
            min_ms: 持续 >= min_ms 才确认（防误触）
            max_ms: 持续超过 max_ms 放弃（防止一直咬牙齿算多次）
            cooldown_ms: 触发后回落，冷却期内不再触发
            infer_every_ms: SVM 推断节流（默认 = CLENCH_STEP_SEC*1000 = 500ms）；
                None 表示每次 update 都推断（不推荐，CPU 高）
        """
        import os
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"咬牙检测模型不存在：{model_path}；"
                "请先运行 train_clench_svm.py 训练"
            )
        # 内联 load_clench_model（原 muscles-braincontrol train_clench_svm.py:8-14）
        # 避免运行时依赖训练工具
        _data = _joblib_load(model_path)
        if isinstance(_data, dict) and 'model' in _data:
            self._model = _data['model']
        else:
            self._model = _data  # 兼容裸 Pipeline
        self._classes = list(self._model.classes_)  # sklearn classes 顺序
        self.fs = fs
        self.threshold = threshold
        self.min_ms = min_ms
        self.max_ms = max_ms
        self.cooldown_ms = cooldown_ms
        self.infer_every_ms = (
            int(infer_every_ms) if infer_every_ms is not None
            else int(CLENCH_STEP_SEC * 1000)
        )
        # 状态机字段（与 FrownDetector 完全对称）
        self._above_ms = 0
        self._triggered = False
        self._cooldown_left = 0
        # 推断节流：累计 dt_ms，达到 infer_every_ms 才重新算 P(clench)
        self._since_infer_ms = 0
        self._last_proba = 0.0
        self._first_update = True  # 第一次 update 立即推断，避免冷启动延迟

    def _predict_proba(self, eeg_window: np.ndarray) -> float:
        """提取特征 → SVM → P(clench)。"""
        if eeg_window.ndim != 2 or eeg_window.shape[1] < 2:
            raise ValueError(
                f"eeg_window 必须 (n_samples, n_channels)，n_channels>=2；"
                f"got shape {eeg_window.shape}"
            )
        feat = extract_clench_features(eeg_window, fs=self.fs).reshape(1, -1)
        proba = self._model.predict_proba(feat)[0]
        # 找到 label=1 (clench) 的列
        if 1 in self._classes:
            idx = self._classes.index(1)
        else:
            # 退化：fallback 到第二列（多数情况 [0,1] 顺序）
            idx = 1 if len(proba) >= 2 else 0
        return float(proba[idx])

    def update(self, eeg_window: np.ndarray, dt_ms: int = 100) -> ClenchResult:
        """喂入最近 2s EEG 窗口，更新状态机。

        Args:
            eeg_window: (n_samples, n_channels) 最近 2s 数据；n_samples 应
                ~= 2*fs，n_channels >= 2（CH0=F7/CH7=F8 必需）。
            dt_ms: 距上次 update 的时间间隔（毫秒），用于状态机时间累计

        Returns:
            ClenchResult：is_clenching / event / proba
        """
        event = False

        # 推断节流：第一次立即推断（避免冷启动延迟），之后按 infer_every_ms
        self._since_infer_ms += dt_ms
        if self._first_update or self._since_infer_ms >= self.infer_every_ms:
            self._last_proba = self._predict_proba(eeg_window)
            self._since_infer_ms = 0
            self._first_update = False

        above = self._last_proba >= self.threshold

        # 冷却期倒计时（防止一次咬牙的多个窗口连续触发 toggle）
        if self._cooldown_left > 0:
            self._cooldown_left -= dt_ms

        if above:
            # 达到阈值立即触发（无 min_ms 持续时间要求）
            # 仅在冷却期外 + 当前未触发时，输出 rising edge event
            if self._cooldown_left <= 0 and not self._triggered:
                self._triggered = True
                event = True
                # 触发后启动冷却，等信号回落才能再次触发
                self._cooldown_left = self.cooldown_ms
        else:
            # 信号回落：重置 _triggered，允许下次再次触发
            self._triggered = False

        return ClenchResult(
            is_clenching=self._triggered,
            event=event,
            proba=self._last_proba,
        )
