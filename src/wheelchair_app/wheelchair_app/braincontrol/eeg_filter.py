import numpy as np
from scipy import signal

from .noth_filter import NotchFilter


class EegFilter:
    """单通道 EEG 连续流滤波器：带通 + 50Hz 陷波。

    带通默认 1-40Hz，保留 theta(4-8Hz) 到 high_beta(18-30Hz) 的专注度相关频段。
    每次调用 process_buffer 会更新内部滤波器状态，适合逐样本或小批量连续处理。
    """

    def __init__(
        self,
        fs: int,
        band_low_hz: float = 1.0,
        band_high_hz: float = 40.0,
        band_order: int = 4,
        notch_hz: float = 50.0,
        notch_q: float = 30.0,
        use_comb_notch: bool = True,
        comb_harmonics: int = 5,
    ):
        self.fs = int(fs)
        self.band_low_hz = float(band_low_hz)
        self.band_high_hz = float(band_high_hz)
        self.band_order = int(band_order)
        self.notch_hz = float(notch_hz)
        self.notch_q = float(notch_q)
        self.use_comb_notch = bool(use_comb_notch)
        self.comb_harmonics = int(comb_harmonics)

        self._sos = self._design_bandpass()
        self._zi = signal.sosfilt_zi(self._sos) * 0.0

        # 梳状陷波器（杀 50Hz 基频 + 100/150/200/250Hz 谐波）— 解决摘下设备
        # 仍残留市电谐波导致 EMG 比值偏高的问题。
        if self.use_comb_notch:
            self._notch = NotchFilter(
                fs=self.fs,
                filter_type='comb',
                notch_freq=self.notch_hz,
                quality_factor=self.notch_q,
                harmonics=self.comb_harmonics,
            )
        else:
            self._notch = NotchFilter(
                fs=self.fs,
                filter_type='butterworth',
                notch_freq=self.notch_hz,
                quality_factor=self.notch_q,
            )

    def _design_bandpass(self):
        nyq = 0.5 * self.fs
        lo = self.band_low_hz / nyq
        hi = self.band_high_hz / nyq
        lo = max(1e-6, min(lo, 0.999))
        hi = max(1e-6, min(hi, 0.999))
        if lo >= hi:
            return signal.butter(self.band_order, lo, btype='highpass', output='sos')
        return signal.butter(self.band_order, [lo, hi], btype='bandpass', output='sos')

    def reset(self):
        self._zi = signal.sosfilt_zi(self._sos) * 0.0
        self._notch.reset()

    def process_buffer(self, data_1d):
        """处理一段数据（list 或 np 数组），返回 np 数组。更新内部状态。"""
        if len(data_1d) == 0:
            return np.array([], dtype=np.float64)

        x = np.asarray(data_1d, dtype=np.float64)
        y, self._zi = signal.sosfilt(self._sos, x, zi=self._zi)
        y = self._notch.process_buffer(y)
        return y
