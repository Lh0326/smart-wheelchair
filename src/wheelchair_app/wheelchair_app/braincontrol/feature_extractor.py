# feature_extractor.py
"""专注度特征提取：TBR/Alpha抑制/Fmθ/Hjorth/Beta相对功率/不对称性。

两种模式：
  - mode='reduced'（默认，30 维）：核心特征，适合 900 窗口 RBF SVM 稳定泛化
  - mode='full'（124 维）：完整特征空间，向后兼容旧 pipeline

关键修复（评估文档 F7）：移除 emg_index_mean 特征。原 125 维中的这一维
是 EMG ratio，会让 SVM 学到"有 EMG = focus"的反模式（反事实测试 100%
准确率）。EMG 信息现在由 FocusDetector 的反向门控规则显式处理，不进
特征空间。

参考：内部评估文档(2026-06-16) §3 P0/P2
"""
import numpy as np
from typing import List
from scipy.signal import welch
from scipy.integrate import trapezoid
from .eeg_bands import BANDS, band_power, relative_band_power

CH_F3, CH_FZ_LEFT, CH_FZ_RIGHT, CH_F4 = 2, 3, 4, 5
# Reduced 模式聚焦的核心通道（DLPFC + ACC，参考规格附录 B 方案 B）
CORE_CHANNELS = (CH_F3, CH_FZ_LEFT, CH_FZ_RIGHT, CH_F4)


class FeatureExtractor:
    """从 (n_samples, n_channels) 窗口提取专注度特征向量。

    训练和推理共用同一份代码，保证特征对齐。
    """

    def __init__(self, fs: int = 500,
                 baseline_alpha: np.ndarray = None,
                 mode: str = 'reduced'):
        """mode: 'reduced' (30 维) 或 'full' (124 维，无 EMG 特征)。"""
        if mode not in ('reduced', 'full'):
            raise ValueError(f"未知 mode: {mode}，应为 'reduced' 或 'full'")
        self.fs = fs
        self.mode = mode
        self.baseline_alpha = baseline_alpha if baseline_alpha is not None else np.ones(8)

    def feature_names(self) -> List[str]:
        if self.mode == 'reduced':
            return self._feature_names_reduced()
        return self._feature_names_full()

    def _feature_names_reduced(self) -> List[str]:
        """30 维核心特征（无 EMG 特征）。

        组成：
          - TBR × 8 通道（theta/beta ratio log）
          - Alpha 抑制 × 4 核心通道（F3/FzL/FzR/F4）
          - Fm-theta × 2（Fz 左/右）
          - Alpha 不对称性 × 2（F3-F4 / Fp1-Fp2）
          - Hjorth Activity/Mobility/Complexity × 4 核心通道 = 12
          - 全局相对功率 × 2（beta / alpha 8 通道均值）
        """
        names = []
        for ch in range(8):
            names.append(f'tbr_ch{ch}')
        for ch in CORE_CHANNELS:
            names.append(f'alpha_suppression_ch{ch}')
        names.append('fm_theta_fz_left')
        names.append('fm_theta_fz_right')
        names.append('alpha_asymmetry_f3_f4')
        names.append('alpha_asymmetry_fp1_fp2')
        for ch in CORE_CHANNELS:
            names.append(f'hjorth_activity_ch{ch}')
            names.append(f'hjorth_mobility_ch{ch}')
            names.append(f'hjorth_complexity_ch{ch}')
        names.append('rel_beta_mean')
        names.append('rel_alpha_mean')
        return names

    def _feature_names_full(self) -> List[str]:
        """124 维完整特征空间（移除 emg_index_mean，原 125 维）。

        保留全部 power_* / rel_* / tbr / alpha_suppression / 跨通道
        比率 / Hjorth；删除 emg_index_mean（EMG 反作弊修复）。
        """
        names = []
        for ch in range(8):
            for band in BANDS:
                names.append(f'power_{band}_ch{ch}')
        for ch in range(8):
            for band in BANDS:
                names.append(f'rel_{band}_ch{ch}')
        for ch in range(8):
            names.append(f'tbr_ch{ch}')
        for ch in range(8):
            names.append(f'alpha_suppression_ch{ch}')
        names.append('fm_theta_fz_left')
        names.append('fm_theta_fz_right')
        names.append('alpha_asymmetry_f3_f4')
        names.append('alpha_asymmetry_fp1_fp2')
        for ch in range(8):
            names.append(f'hjorth_activity_ch{ch}')
            names.append(f'hjorth_mobility_ch{ch}')
            names.append(f'hjorth_complexity_ch{ch}')
        # 注意：不再 append 'emg_index_mean'
        return names

    def extract(self, window: np.ndarray) -> np.ndarray:
        if self.mode == 'reduced':
            return self._extract_reduced(window)
        return self._extract_full(window)

    def _compute_powers(self, window: np.ndarray):
        """提取 abs_powers (n_ch, n_bands) 和 rel_powers (n_ch, n_bands)。"""
        n_channels = window.shape[1]
        abs_powers = np.zeros((n_channels, len(BANDS)))
        rel_powers = np.zeros((n_channels, len(BANDS)))
        for ch in range(n_channels):
            p = band_power(window[:, ch], self.fs)
            r = relative_band_power(window[:, ch], self.fs)
            for i, band in enumerate(BANDS):
                abs_powers[ch, i] = p[band]
                rel_powers[ch, i] = r[band]
        return abs_powers, rel_powers

    def _extract_reduced(self, window: np.ndarray) -> np.ndarray:
        abs_powers, rel_powers = self._compute_powers(window)
        n_channels = window.shape[1]
        band_list = list(BANDS.keys())
        theta_idx = band_list.index('theta')
        beta_idx = band_list.index('beta')
        alpha_idx = band_list.index('alpha')

        features = []
        # 1. TBR × 8
        tbr = np.log(
            (abs_powers[:, theta_idx] + 1e-12)
            / (abs_powers[:, beta_idx] + 1e-12)
        )
        features.extend(tbr.tolist())

        # 2. Alpha 抑制 × 4 核心通道
        current_alpha = abs_powers[:, alpha_idx]
        suppression = (
            (self.baseline_alpha[:n_channels] - current_alpha)
            / (self.baseline_alpha[:n_channels] + 1e-12)
        )
        features.extend(suppression[ch] for ch in CORE_CHANNELS)

        # 3. Fm-theta × 2
        features.append(float(np.log(abs_powers[CH_FZ_LEFT, theta_idx] + 1e-12)))
        features.append(float(np.log(abs_powers[CH_FZ_RIGHT, theta_idx] + 1e-12)))

        # 4. Alpha 不对称性 × 2
        alpha_f3 = abs_powers[CH_F3, alpha_idx]
        alpha_f4 = abs_powers[CH_F4, alpha_idx]
        alpha_fp1 = abs_powers[1, alpha_idx]
        alpha_fp2 = abs_powers[6, alpha_idx]
        features.append(float(np.log((alpha_f3 + 1e-12) / (alpha_f4 + 1e-12))))
        features.append(float(np.log((alpha_fp1 + 1e-12) / (alpha_fp2 + 1e-12))))

        # 5. Hjorth × 4 核心通道
        def hjorth(sig):
            d1 = np.diff(sig)
            d2 = np.diff(d1)
            activity = sig.var()
            mobility = np.sqrt(d1.var() / (activity + 1e-12))
            complexity = (np.sqrt(d2.var() / (d1.var() + 1e-12))
                          / (mobility + 1e-12))
            return activity, mobility, complexity

        for ch in CORE_CHANNELS:
            a, m, c = hjorth(window[:, ch])
            features.extend([float(a), float(m), float(c)])

        # 6. 全局相对功率 × 2
        features.append(float(rel_powers[:, beta_idx].mean()))
        features.append(float(rel_powers[:, alpha_idx].mean()))

        return np.array(features, dtype=np.float32)

    def _extract_full(self, window: np.ndarray) -> np.ndarray:
        """124 维完整特征（无 EMG，原 125 - 1）。"""
        n_channels = window.shape[1]
        abs_powers, rel_powers = self._compute_powers(window)

        features = []
        features.extend(abs_powers.flatten())
        features.extend(rel_powers.flatten())

        band_list = list(BANDS.keys())
        theta_idx = band_list.index('theta')
        beta_idx = band_list.index('beta')
        alpha_idx = band_list.index('alpha')
        tbr = np.log(
            (abs_powers[:, theta_idx] + 1e-12)
            / (abs_powers[:, beta_idx] + 1e-12)
        )
        features.extend(tbr)

        current_alpha = abs_powers[:, alpha_idx]
        suppression = (
            (self.baseline_alpha[:n_channels] - current_alpha)
            / (self.baseline_alpha[:n_channels] + 1e-12)
        )
        features.extend(suppression)

        features.extend([
            float(np.log(abs_powers[CH_FZ_LEFT, theta_idx] + 1e-12)),
            float(np.log(abs_powers[CH_FZ_RIGHT, theta_idx] + 1e-12)),
        ])
        alpha_f3 = abs_powers[CH_F3, alpha_idx]
        alpha_f4 = abs_powers[CH_F4, alpha_idx]
        alpha_fp1 = abs_powers[1, alpha_idx]
        alpha_fp2 = abs_powers[6, alpha_idx]
        features.extend([
            float(np.log((alpha_f3 + 1e-12) / (alpha_f4 + 1e-12))),
            float(np.log((alpha_fp1 + 1e-12) / (alpha_fp2 + 1e-12))),
        ])

        def hjorth(sig):
            d1 = np.diff(sig)
            d2 = np.diff(d1)
            activity = sig.var()
            mobility = np.sqrt(d1.var() / (activity + 1e-12))
            complexity = (np.sqrt(d2.var() / (d1.var() + 1e-12))
                          / (mobility + 1e-12))
            return activity, mobility, complexity

        for ch in range(n_channels):
            a, m, c = hjorth(window[:, ch])
            features.extend([float(a), float(m), float(c)])

        return np.array(features, dtype=np.float32)
