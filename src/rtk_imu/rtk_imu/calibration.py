"""HWT906P 启动静态 bias 校准。

启动时收集 duration_sec 秒静态数据，计算陀螺 bias。
校准完成后 apply_to_gyro() 减去 bias。

严格指标：bias 残差 < 1.5e-4 rad/s（≈ 0.5°/min 静态漂移）。

完成判定（任一满足即可，且总样本数 ≥ MIN_SAMPLES）：
  1. 时间窗口：wall-clock 经时 ≥ duration_sec（主路径，真实硬件 100Hz 采集）
  2. 样本计数兜底：累计样本数 ≥ sample_rate × duration_sec
     （等价数据量，避免高负载主机时间窗口判定失真）
"""
import time
from typing import Optional

from rtk_imu.packet_parser import PacketResult
from rtk_imu.jy901_protocol import REG_GYRO, REG_ACC, REG_ANGLE, REG_MAG

MIN_SAMPLES = 10  # 最少样本数（避免单包完成）
DEFAULT_SAMPLE_RATE_HZ = 100  # HWT906P 默认输出速率


class BiasCalibrator:
    """静态 bias 校准器。

    用法：
        cal = BiasCalibrator(duration_sec=3.0)
        for result in parser.feed(bytes_from_serial):
            cal.feed(result)
            if not cal.is_calibrating():
                corrected_gyro = cal.apply_to_gyro(result.gyro)
    """

    def __init__(self, duration_sec: float = 3.0,
                 sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ):
        self._duration = duration_sec
        self._sample_rate = sample_rate_hz
        self._start_time: Optional[float] = None
        self._gyro_samples: list = []  # [[wx, wy, wz], ...]
        self._acc_samples: list = []
        self._total_count: int = 0  # 所有包计数（含 angle/mag，HWT906P 适配）
        self._bias = [0.0, 0.0, 0.0]
        self._gravity = [0.0, 0.0, 0.0]
        self._calibrating = True

    def feed(self, result: PacketResult) -> None:
        """喂入一个 PacketResult，累积样本。

        校准完成后此函数变为 no-op。

        注意：HWT906P 等只输出 angle/mag 的型号，bias 计算会退化为 [0,0,0]
        （因为 IMU 内部已做 AHRS 融合，输出的 angle 已是融合后结果，无需 bias 校准）。
        """
        if not self._calibrating:
            return

        now = time.monotonic()
        if self._start_time is None:
            self._start_time = now

        if result.reg == REG_GYRO and result.gyro is not None:
            self._gyro_samples.append(result.gyro)
            self._total_count += 1
        elif result.reg == REG_ACC and result.acc is not None:
            self._acc_samples.append(result.acc)
            self._total_count += 1
        elif result.reg in (REG_ANGLE, REG_MAG):
            # HWT906P 适配：angle/mag 包也计入总样本数（让校准能完成）
            # 但不用于 bias 计算（angle 是 AHRS 融合后结果，bias 不适用）
            self._total_count += 1

        # 检查是否完成
        elapsed = now - self._start_time
        total = self._total_count
        time_ok = elapsed >= self._duration
        count_ok = total >= self._sample_rate * self._duration
        if total >= MIN_SAMPLES and (time_ok or count_ok):
            self._finish()

    def _finish(self) -> None:
        """计算 bias 平均值，结束校准。"""
        n = len(self._gyro_samples)
        if n == 0:
            self._bias = [0.0, 0.0, 0.0]
        else:
            sx = sum(s[0] for s in self._gyro_samples) / n
            sy = sum(s[1] for s in self._gyro_samples) / n
            sz = sum(s[2] for s in self._gyro_samples) / n
            self._bias = [sx, sy, sz]

        # 重力向量（加速度平均）
        n_acc = len(self._acc_samples)
        if n_acc > 0:
            ax = sum(s[0] for s in self._acc_samples) / n_acc
            ay = sum(s[1] for s in self._acc_samples) / n_acc
            az = sum(s[2] for s in self._acc_samples) / n_acc
            self._gravity = [ax, ay, az]

        self._calibrating = False

    def is_calibrating(self) -> bool:
        return self._calibrating

    def get_bias(self) -> list:
        """返回 [bx, by, bz] rad/s。"""
        return self._bias.copy()

    def get_gravity(self) -> list:
        """返回 [gx, gy, gz] m/s²（启动时静态重力向量）。"""
        return self._gravity.copy()

    def apply_to_gyro(self, gyro: list) -> list:
        """从 gyro 减去 bias。

        校准期返回原值（bias 还没算出来，不能减）。
        """
        if self._calibrating:
            return list(gyro)
        return [gyro[i] - self._bias[i] for i in range(3)]
