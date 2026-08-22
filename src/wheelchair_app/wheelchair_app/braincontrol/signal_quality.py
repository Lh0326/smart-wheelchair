# signal_quality.py
"""前额 EEG 信号质量评估（v2 稳健版）：逐通道指标 + 综合评分。

v2 修复：v1 用 max-min 单点 PSD 对伪迹过敏，前 5s 启动漂移期就让所有通道
报 4/100。改用 trimmed range + 频段平均 PSD + median_abs。

阈值基于真实前额 EEG 数据（5.26.csv）调试：
  - 干净 EEG median|.| = 5-50μV，trimmed pp = 100-500μV
  - 启动期（前 5s）median|.| = 1000+μV，trimmed pp = 5000+μV
  - 50Hz 真实污染时 ratio_50hz > 0.1（启动期尖峰会让单点 PSD 失真，但 ratio 稳定）
"""
from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np
from scipy.signal import welch
from scipy.integrate import trapezoid


@dataclass
class ChannelQuality:
    channel: int
    uv_pp: float                  # 峰峰值（μV）— raw max-min，保留诊断价值
    uv_pp_trim: float             # trimmed pp（去除上下 1%）— 用于评分
    uv_median_abs: float          # 中位数绝对值 — 真实信号水平
    uv_rms: float
    variance: float
    line_noise_50hz: float        # @50Hz 单点 PSD（保留诊断显示）
    line_noise_ratio: float       # @50Hz / 总功率 比（评分用，稳健）
    low_high_ratio: float         # 1-4Hz / 30-45Hz PSD 比
    score: int                    # 0-100 综合分
    issues: List[str] = field(default_factory=list)


def _linear_score(value: float, good_max: float, bad_max: float) -> float:
    """value ≤ good_max 满分；value ≥ bad_max 0 分；中间线性。"""
    if value <= good_max:
        return 100.0
    if value >= bad_max:
        return 0.0
    return 100.0 * (bad_max - value) / (bad_max - good_max)


class SignalQualityChecker:
    """逐通道信号质量评估 + 全局综合分（v2 稳健版）。"""

    # trimmed pp：去除上下 1% 后的峰峰值
    GOOD_UV_PP_TRIM = 500.0       # ≤ 500μV 满分
    BAD_UV_PP_TRIM = 5000.0       # ≥ 5000μV 0 分

    # median_abs：中位数绝对值
    GOOD_MEDIAN_LOW = 3.0         # < 3μV 视为电极脱落
    GOOD_MEDIAN_HIGH = 80.0       # ≤ 80μV 满分
    BAD_MEDIAN_HIGH = 500.0       # ≥ 500μV 视为持续饱和/基线漂移

    # 50Hz 占总功率比
    GOOD_RATIO_50HZ = 0.02        # ≤ 2% 满分
    BAD_RATIO_50HZ = 0.20         # ≥ 20% 0 分

    # 1/f 比（前额 EEG 1/f 强是正常，阈值大幅放宽）
    GOOD_LOW_HIGH_RATIO = 50.0
    BAD_LOW_HIGH_RATIO = 500.0

    # 各维度权重（sum=100）
    W_RANGE = 30                  # trimmed pp
    W_MEDIAN = 30                 # median_abs（区分脱落/饱和）
    W_50HZ = 25                   # 50Hz 占比
    W_1F = 15                     # 1/f 比

    def __init__(self, fs: int = 500):
        self.fs = fs

    def check_channel(self, signal: np.ndarray, channel: int) -> ChannelQuality:
        sig = np.asarray(signal, dtype=np.float64).flatten()

        # 稳健幅值统计
        uv_pp = float(sig.max() - sig.min())                    # raw，仅诊断
        p1 = float(np.percentile(sig, 1))
        p99 = float(np.percentile(sig, 99))
        uv_pp_trim = float(p99 - p1)                            # trimmed，评分用
        uv_median_abs = float(np.median(np.abs(sig)))           # 中位数绝对值
        uv_rms = float(np.sqrt(np.mean(sig ** 2)))
        variance = float(sig.var())

        # 频谱
        freqs, psd = welch(sig, fs=self.fs, nperseg=min(512, len(sig)))

        def band_avg_psd(lo: float, hi: float) -> float:
            """频段平均 PSD（不是单点）。"""
            mask = (freqs >= lo) & (freqs <= hi)
            if not mask.any():
                return 0.0
            return float(trapezoid(psd[mask], freqs[mask]) / (hi - lo))

        def psd_at(target_hz: float) -> float:
            idx = int(round(target_hz / (freqs[1] - freqs[0]))) if len(freqs) > 1 else 0
            return float(psd[idx]) if 0 <= idx < len(psd) else 0.0

        # 50Hz 单点（诊断显示）+ 50Hz 占比（评分用）
        line_50 = psd_at(50.0)
        eeg_total = band_avg_psd(1.0, 40.0)
        line_50_band = band_avg_psd(49.0, 51.0)
        line_noise_ratio = float(line_50_band / (eeg_total + 1e-12))

        # 1/f 比
        low_p = band_avg_psd(1.0, 4.0)
        high_p = band_avg_psd(30.0, 45.0)
        low_high_ratio = float(low_p / (high_p + 1e-12))

        # 问题标记
        issues: List[str] = []
        if uv_pp_trim > self.GOOD_UV_PP_TRIM * 2:
            issues.append('range_too_large')
        if uv_median_abs < self.GOOD_MEDIAN_LOW:
            issues.append('low_signal')
        elif uv_median_abs > self.GOOD_MEDIAN_HIGH * 2:
            issues.append('signal_too_strong')
        if line_noise_ratio > self.GOOD_RATIO_50HZ * 2:
            issues.append('high_line_noise')
        if low_high_ratio > self.GOOD_LOW_HIGH_RATIO * 2:
            issues.append('high_1f_noise')

        # 综合评分
        range_score = _linear_score(uv_pp_trim, self.GOOD_UV_PP_TRIM, self.BAD_UV_PP_TRIM)
        if uv_median_abs < self.GOOD_MEDIAN_LOW:
            median_score = 20.0   # 电极脱落
        else:
            median_score = _linear_score(uv_median_abs,
                                         self.GOOD_MEDIAN_HIGH,
                                         self.BAD_MEDIAN_HIGH)
        hz50_score = _linear_score(line_noise_ratio,
                                    self.GOOD_RATIO_50HZ,
                                    self.BAD_RATIO_50HZ)
        ratio_score = _linear_score(low_high_ratio,
                                     self.GOOD_LOW_HIGH_RATIO,
                                     self.BAD_LOW_HIGH_RATIO)

        score = (self.W_RANGE * range_score / 100
                 + self.W_MEDIAN * median_score / 100
                 + self.W_50HZ * hz50_score / 100
                 + self.W_1F * ratio_score / 100)
        score = int(round(max(0, min(100, score))))

        return ChannelQuality(
            channel=channel,
            uv_pp=uv_pp,
            uv_pp_trim=uv_pp_trim,
            uv_median_abs=uv_median_abs,
            uv_rms=uv_rms,
            variance=variance,
            line_noise_50hz=line_50,
            line_noise_ratio=line_noise_ratio,
            low_high_ratio=low_high_ratio,
            score=score,
            issues=issues,
        )

    def check_window(self, window: np.ndarray) -> List[ChannelQuality]:
        """window: (n_samples, n_channels)。返回每通道质量。"""
        window = np.asarray(window)
        n_channels = window.shape[1]
        return [self.check_channel(window[:, ch], ch) for ch in range(n_channels)]

    def overall_score(self, channels: List[ChannelQuality]) -> Tuple[int, List[str]]:
        """综合所有通道：返回 (0-100, 全局 issues)。

        用 25 百分位（下四分位）作为综合分：质量看下限，少数坏通道应拉低整体，
        符合录制前门控"任一通道差就警告"的直觉；任一通道有 issue 则全局标记。
        """
        if not channels:
            return 0, ['no_channels']
        scores = [c.score for c in channels]
        overall = int(round(float(np.percentile(scores, 25))))
        # 全局 issue 聚合（保留通道号方便定位）
        issue_map: dict = {}
        for c in channels:
            for issue in c.issues:
                issue_map.setdefault(issue, []).append(c.channel)
        global_issues = [f'{issue}@ch{chs[0]}' if len(chs) == 1
                         else f'{issue}@ch{chs}'
                         for issue, chs in issue_map.items()]
        return overall, global_issues
