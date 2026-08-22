# eeg_bands.py
"""EEG 频段定义与功率提取工具。"""
from typing import Dict, Tuple
import numpy as np
from scipy.signal import welch
from scipy.integrate import trapezoid

BANDS: Dict[str, Tuple[float, float]] = {
    'delta': (1.0, 4.0),
    'theta': (4.0, 8.0),
    'alpha': (8.0, 13.0),
    'beta':  (13.0, 30.0),
    'gamma': (30.0, 45.0),
}

def band_power(signal: np.ndarray, fs: int, nperseg: int = 512) -> Dict[str, float]:
    """Welch PSD + 梯形积分得到各频段绝对功率。"""
    freqs, psd = welch(signal, fs=fs, nperseg=min(nperseg, len(signal)))
    out = {}
    for name, (lo, hi) in BANDS.items():
        mask = (freqs >= lo) & (freqs <= hi)
        out[name] = float(trapezoid(psd[mask], freqs[mask]))
    return out

def relative_band_power(signal: np.ndarray, fs: int) -> Dict[str, float]:
    """各频段相对总功率的比例（和为 1）。"""
    abs_p = band_power(signal, fs)
    total = sum(abs_p.values()) + 1e-12
    return {k: v / total for k, v in abs_p.items()}
