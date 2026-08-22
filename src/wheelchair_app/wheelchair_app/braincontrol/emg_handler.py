# emg_handler.py
"""EMG 软去除：双频段检测 + 频谱减法（20-95Hz 全范围）+ 重度丢弃。

关键修复（评估文档 F1/F2/F4 P1）：
  - F1 双频段检测：增加 beta 边带（20-30Hz）污染检测。
    真实 EMG 在 beta 段可超真实 EEG 5-200×（Goncharova 2003），
    只看 60-95Hz 会漏检中度污染。联合 ratio 任一超阈即触发处理。
  - F2 扩展处理范围：频谱减法从只动 60-95Hz 改为 20-95Hz 全 EMG 范围
    （仍避开 50Hz ±5Hz 市电残留区），覆盖 beta 段污染。
  - F4 噪声模板：训练时从 relax 段学习的稳定噪声模板替代窗口尾部 10%
    估计；推理时通过 set_noise_template 注入，方差显著降低。
"""
import numpy as np
from typing import Optional
from scipy.signal import welch, stft, istft
from scipy.integrate import trapezoid


class EMGHandler:
    """4 级 EMG 处理：保留 / 软去除（作特征） / 频谱减法 / 丢弃。

    双频段检测（F1）：
      - hf_ratio = 60-95Hz / 1-40Hz（原高频检测，避开 50Hz 谐波残留）
      - beta_ratio = 20-30Hz / 8-13Hz alpha（beta 段污染检测）
      - 任一超 light_thresh 即触发；任一超 severe_thresh 即丢弃
    """

    def __init__(self, fs: int = 500,
                 light_thresh: float = 1.0,
                 severe_thresh: float = 3.0,
                 beta_light_thresh: float = 1.5,
                 beta_severe_thresh: float = 4.0):
        """阈值针对前额干电极调优。

        高频 ratio 阈值沿用 1.0/3.0（交接文档 §6.1）。
        beta 段 ratio 阈值 1.5/4.0：beta 边带本身含真实 EEG，阈值放宽。
        """
        self.fs = fs
        self.light_thresh = light_thresh
        self.severe_thresh = severe_thresh
        self.beta_light_thresh = beta_light_thresh
        self.beta_severe_thresh = beta_severe_thresh
        self.last_rejected = False
        # F4: 训练时学习的噪声模板（每频率索引的振幅均值，shape=(n_freq,)）
        # 通过 set_noise_template 注入；None 时回退到窗口尾部估计
        self._noise_template: Optional[np.ndarray] = None
        self._noise_freqs: Optional[np.ndarray] = None

    def set_noise_template(self, freqs: np.ndarray, noise_mag: np.ndarray) -> None:
        """注入训练时从 relax 段学习的噪声模板（F4）。

        Args:
            freqs: 频率轴 (n_freq,)，Hz
            noise_mag: 每频率的平均振幅 (n_freq,)，跨通道平均
        """
        self._noise_freqs = np.asarray(freqs, dtype=np.float64)
        self._noise_template = np.asarray(noise_mag, dtype=np.float64)

    def _emg_ratio(self, window: np.ndarray) -> float:
        """高频 EMG ratio（60-95Hz / 1-40Hz），向后兼容接口。"""
        return float(np.mean([self._hf_ratio_per_ch(window[:, ch])
                              for ch in range(window.shape[1])]))

    def _hf_ratio_per_ch(self, sig: np.ndarray) -> float:
        freqs, psd = welch(sig, fs=self.fs, nperseg=min(256, len(sig)))
        eeg_mask = (freqs >= 1) & (freqs <= 40)
        emg_mask = (freqs >= 60) & (freqs <= 95)
        eeg_p = trapezoid(psd[eeg_mask], freqs[eeg_mask])
        emg_p = trapezoid(psd[emg_mask], freqs[emg_mask])
        return float(emg_p / (eeg_p + 1e-12))

    def _beta_ratio_per_ch(self, sig: np.ndarray) -> float:
        """beta 边带污染 ratio：20-30Hz / 8-13Hz alpha 段功率比。

        beta(13-30Hz) 同时含真实 EEG 和 EMG 污染；alpha(8-13Hz) 主要含
        真实 EEG，作为对照。比值高说明 beta 段被 EMG 抬高。
        """
        freqs, psd = welch(sig, fs=self.fs, nperseg=min(256, len(sig)))
        alpha_mask = (freqs >= 8) & (freqs <= 13)
        beta_mask = (freqs >= 20) & (freqs <= 30)  # beta 高段（污染最重）
        alpha_p = trapezoid(psd[alpha_mask], freqs[alpha_mask])
        beta_p = trapezoid(psd[beta_mask], freqs[beta_mask])
        return float(beta_p / (alpha_p + 1e-12))

    def _combined_ratio(self, window: np.ndarray) -> tuple:
        """返回 (hf_ratio, beta_ratio) 的通道均值。"""
        hf_ratios = [self._hf_ratio_per_ch(window[:, ch])
                     for ch in range(window.shape[1])]
        beta_ratios = [self._beta_ratio_per_ch(window[:, ch])
                       for ch in range(window.shape[1])]
        return float(np.mean(hf_ratios)), float(np.mean(beta_ratios))

    def _spectral_subtraction(self, window: np.ndarray) -> np.ndarray:
        """STFT → 减去噪声 → ISTFT，覆盖 20-95Hz 全 EMG 频段（F2）。

        频率索引选择（避开 50Hz 市电残留区 ±5Hz）：
          - 20-45Hz：beta + low gamma（含 EMG beta 段污染）
          - 55-95Hz：高频 EMG（避开 45-55Hz）
        """
        n_samples, n_ch = window.shape
        out = np.zeros_like(window)
        nperseg = 128
        f_sig, _, Z_sig = stft(window.T, fs=self.fs, nperseg=nperseg)
        mag = np.abs(Z_sig).copy()
        phase = np.angle(Z_sig)

        # 噪声估计：优先用注入的模板（F4），否则用窗口尾部 10%
        if self._noise_template is not None and self._noise_freqs is not None:
            noise_mag = np.zeros((n_ch, len(f_sig)))
            for ch in range(n_ch):
                # 噪声模板是跨通道平均的，每个通道用同一份
                noise_mag[ch] = np.interp(f_sig, self._noise_freqs,
                                          self._noise_template)
        else:
            noise_seg = window[int(n_samples * 0.9):, :]
            noise_nperseg = min(128, len(noise_seg))
            f_noise, _, Z_noise = stft(noise_seg.T, fs=self.fs,
                                        nperseg=noise_nperseg)
            noise_mag_raw = np.abs(Z_noise).mean(axis=2)
            noise_mag = np.zeros((n_ch, len(f_sig)))
            for ch in range(n_ch):
                noise_mag[ch] = np.interp(f_sig, f_noise, noise_mag_raw[ch])

        # F2: 处理 20-45Hz + 55-95Hz（避开 45-55Hz 市电残留区）
        emg_mask = ((f_sig >= 20) & (f_sig <= 45)) | \
                   ((f_sig >= 55) & (f_sig <= 95))
        emg_freq_idx = np.where(emg_mask)[0]
        for ch in range(n_ch):
            for fi in emg_freq_idx:
                original = mag[ch, fi, :]
                reduced = original - noise_mag[ch, fi]
                # 半波整流：保留至少 20% 原始振幅，避免过度减法破坏信号
                mag[ch, fi, :] = np.maximum(reduced, 0.2 * original)
            Z_clean = mag[ch] * np.exp(1j * phase[ch])
            _, reconstructed = istft(Z_clean, fs=self.fs,
                                     input_onesided=True, nperseg=nperseg)
            out[:, ch] = reconstructed[:n_samples]
        return out

    def process(self, window: np.ndarray) -> tuple:
        """返回 (cleaned_window, emg_ratio)。设置 self.last_rejected。

        emg_ratio 是高频 ratio（向后兼容）。内部决策基于双频段联合判定。
        """
        self.last_rejected = False
        hf_ratio, beta_ratio = self._combined_ratio(window)

        # 双频段联合判定：任一频段超 severe 即丢弃
        if hf_ratio >= self.severe_thresh or beta_ratio >= self.beta_severe_thresh:
            self.last_rejected = True
            return window, hf_ratio

        # 任一频段超 light 即触发频谱减法（F2 扩展范围会同时清理两频段）
        if hf_ratio >= self.light_thresh or beta_ratio >= self.beta_light_thresh:
            return self._spectral_subtraction(window), hf_ratio

        return window, hf_ratio
