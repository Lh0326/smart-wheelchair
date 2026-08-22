"""咬牙检测特征提取（数据采集/训练/检测共用）。

特征维度（共 11 维，固定顺序，所有调用方依赖此顺序）：
  1-8: 8 通道逐通道 EMG ratio = 60-95Hz / 1-40Hz 功率比
  9  : F7/F8（CH0/CH7）能量对数和
  10 : 8 通道总能量对数
  11 : 8 通道 EMG ratio 均值

为什么独立于 feature_extractor.FeatureExtractor：focus 训练的特征含
alpha 抑制 / theta 比 / Hjorth 参数等专注度相关维度，对"咬牙"信号没有
判别力；咬牙检测只需要 EMG 频段分布。设计上与 FrownDetector 走同一路
思路：用单一物理量（EMG ratio）+ 时间窗。
"""
from typing import List

import numpy as np
from scipy.signal import welch
from scipy.integrate import trapezoid


# 协议默认参数（与 frown_detector / prepare_data 对齐）
CLENCH_WINDOW_SEC: float = 2.0
CLENCH_STEP_SEC: float = 0.5

# EMG 频段（与 emg_handler._hf_ratio_per_ch 完全一致，避免一致性陷阱）
_EEG_BAND = (1, 40)
_EMG_BAND = (60, 95)

# F7=CH0, F8=CH7（颞肌最强）
_F7_CH = 0
_F8_CH = 7


def _ratio_per_channel(sig: np.ndarray, fs: int) -> float:
    """单通道 EMG ratio（60-95Hz / 1-40Hz 功率比）。"""
    nperseg = min(256, len(sig))
    freqs, psd = welch(sig, fs=fs, nperseg=nperseg)
    eeg_mask = (freqs >= _EEG_BAND[0]) & (freqs <= _EEG_BAND[1])
    emg_mask = (freqs >= _EMG_BAND[0]) & (freqs <= _EMG_BAND[1])
    eeg_p = trapezoid(psd[eeg_mask], freqs[eeg_mask])
    emg_p = trapezoid(psd[emg_mask], freqs[emg_mask])
    return float(emg_p / (eeg_p + 1e-12))


def extract_clench_features(window: np.ndarray, fs: int) -> np.ndarray:
    """从 (n_samples, 8) 窗口提取 11 维咬肌 EMG 特征。

    Args:
        window: (n_samples, n_channels) 已滤波 EEG；n_channels 必须 >= 2
                （至少 CH0/CH7 可用）。完整协议下 n_channels=8。
        fs: 采样率 Hz

    Returns:
        (n_features,) float32 特征向量。顺序见 FEATURE_NAMES。
    """
    window = np.asarray(window, dtype=np.float64)
    if window.ndim != 2:
        raise ValueError(
            f"window must be 2D (n_samples, n_channels), got shape {window.shape}"
        )
    n_samples, n_ch = window.shape
    if n_ch < 2:
        raise ValueError(
            f"需要至少 2 个通道（CH0=F7 / CH7=F8），实际 {n_ch}"
        )
    if n_samples < 64:
        raise ValueError(
            f"窗口过短（{n_samples} < 64），welch 无法估计频谱"
        )

    # 1-8: 逐通道 EMG ratio
    ratios = np.array(
        [_ratio_per_channel(window[:, ch], fs) for ch in range(n_ch)],
        dtype=np.float64,
    )
    # 不足 8 通道时补 0（兼容单通道调试场景，但训练/检测默认 8 通道）
    if len(ratios) < 8:
        ratios = np.concatenate([ratios, np.zeros(8 - len(ratios))])

    # 9: F7/F8 能量对数和
    f7_f8_energy = float(
        np.sum(window[:, _F7_CH] ** 2) + np.sum(window[:, _F8_CH] ** 2)
    )
    f7_f8_energy_log = float(np.log10(f7_f8_energy + 1e-12))

    # 10: 8 通道总能量对数
    total_energy = float(np.sum(window ** 2))
    total_energy_log = float(np.log10(total_energy + 1e-12))

    # 11: 8 通道 EMG ratio 均值
    ratio_mean = float(np.mean(ratios[:n_ch]))

    feat = np.array(
        list(ratios[:8]) + [f7_f8_energy_log, total_energy_log, ratio_mean],
        dtype=np.float32,
    )
    return feat


def feature_names() -> List[str]:
    """返回 11 维特征名（顺序与 extract_clench_features 一致）。"""
    return [
        'emg_ratio_ch0', 'emg_ratio_ch1', 'emg_ratio_ch2', 'emg_ratio_ch3',
        'emg_ratio_ch4', 'emg_ratio_ch5', 'emg_ratio_ch6', 'emg_ratio_ch7',
        'f7_f8_energy', 'total_energy_log', 'emg_ratio_mean',
    ]


# 模块加载时即冻结顺序，确保训练/检测/特征名完全一致
FEATURE_NAMES: List[str] = feature_names()
