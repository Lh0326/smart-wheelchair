# spatial_filter.py
"""空间滤波器：CAR（共平均参考）+ Laplacian（最近邻差分）。"""
import numpy as np

class CARFilter:
    """Common Average Reference — 减去通道均值，去除共模噪声。"""
    def apply(self, window: np.ndarray) -> np.ndarray:
        # window shape: (n_samples, n_channels)
        return window - window.mean(axis=1, keepdims=True)


# 通道布局：CH0=F7 CH1=Fp1 CH2=F3 CH3=Fz左 CH4=Fz右 CH5=F4 CH6=Fp2 CH7=F8
# 每个通道的最近邻索引（按头环物理位置）
LAPLACIAN_NEIGHBORS = {
    0: [1, 2],         # F7 ← Fp1, F3
    1: [0, 2],         # Fp1 ← F7, F3
    2: [1, 3],         # F3 ← Fp1, Fz左
    3: [2, 4],         # Fz左 ← F3, Fz右
    4: [3, 5],         # Fz右 ← Fz左, F4
    5: [4, 6],         # F4 ← Fz右, Fp2
    6: [5, 7],         # Fp2 ← F4, F8
    7: [5, 6],         # F8 ← F4, Fp2
}


class LaplacianFilter:
    """Hjorth-style Laplacian — 每通道减去邻居均值，突出局部源。"""
    def apply(self, window: np.ndarray) -> np.ndarray:
        out = np.zeros_like(window)
        for ch, neighbors in LAPLACIAN_NEIGHBORS.items():
            out[:, ch] = window[:, ch] - window[:, neighbors].mean(axis=1)
        return out
