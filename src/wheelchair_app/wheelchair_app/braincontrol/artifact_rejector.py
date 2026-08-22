import numpy as np
from scipy.signal import welch
from scipy.integrate import trapezoid


class ArtifactRejector:
    """两级 EEG 伪迹检测器：时域幅值/Z-Score + 频域能量比。"""

    def __init__(
        self,
        amplitude_threshold: float = 100.0,
        zscore_threshold: float = 3.0,
        emg_freq_ratio: float = 0.6,
        emg_band: tuple = (40, 100),
        sample_rate: int = 500,
    ):
        self.amplitude_threshold = amplitude_threshold
        self.zscore_threshold = zscore_threshold
        self.emg_freq_ratio = emg_freq_ratio
        self.emg_band = emg_band
        self.sample_rate = sample_rate

    def check(self, window: np.ndarray) -> tuple:
        """检测窗口是否包含伪迹。

        Args:
            window: shape (n_samples, n_channels)

        Returns:
            (is_clean, reason): 干净返回 (True, "")，否则 (False, 原因字符串)
        """
        if window.ndim == 1:
            window = window.reshape(-1, 1)

        # 一级：时域幅值检测
        ptp = np.ptp(window, axis=0)
        if np.any(ptp > self.amplitude_threshold):
            return False, "amplitude"

        # 一级：Z-Score 检测
        # 使用 Donoho-Johnstone 通用阈值缩放，避免对正态噪声过度检测
        n_samples = window.shape[0]
        universal_scale = np.sqrt(2 * np.log(max(n_samples, 10)))
        mean = np.mean(window, axis=0)
        std = np.std(window, axis=0)
        std = np.where(std < 1e-10, 1e-10, std)
        zscores = np.abs((window - mean) / std)
        # 仅当 z-score 同时超过用户阈值和通用阈值时才标记
        effective_threshold = self.zscore_threshold * universal_scale
        if np.any(zscores > effective_threshold):
            return False, "zscore"

        # 二级：频域 EMG 检测
        n_ch = window.shape[1]
        for ch in range(n_ch):
            ratio = self._emg_ratio(window[:, ch])
            if ratio > self.emg_freq_ratio:
                return False, "emg"

        return True, ""

    def _emg_ratio(self, signal_1d: np.ndarray) -> float:
        """计算 EMG 频段能量 / 全频段能量比。"""
        x = signal_1d - np.mean(signal_1d)
        nperseg = min(256, len(x))
        freqs, psd = welch(x, fs=self.sample_rate, nperseg=nperseg, noverlap=nperseg // 2)

        total_power = trapezoid(psd, freqs)
        if total_power < 1e-20:
            return 0.0

        emg_mask = (freqs >= self.emg_band[0]) & (freqs <= self.emg_band[1])
        if not np.any(emg_mask):
            return 0.0

        emg_power = trapezoid(psd[emg_mask], freqs[emg_mask])
        return emg_power / total_power
