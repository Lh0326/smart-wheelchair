"""独立的声音播放模块，用于数据采集 GUI 的 beep 提示。

用 NumPy 生成 sine wave + sounddevice 播放。淡入淡出避免 click 杂音。
跨平台（Linux/macOS/Windows）。
"""
import numpy as np
import sounddevice as sd

SAMPLE_RATE = 44100


def play_beep(freq: float, duration: float) -> None:
    """播放指定频率和时长的 sine wave beep（非阻塞）。

    Args:
        freq: 频率（Hz），常用 440（短嘀）和 880（长嘀）
        duration: 时长（秒），常用 0.2（短）和 0.5（长）
    """
    n_samples = int(duration * SAMPLE_RATE)
    t = np.linspace(0, duration, n_samples, False)
    wave = np.sin(2 * np.pi * freq * t)

    # 10% 淡入淡出，避免 click 杂音
    fade_n = max(1, n_samples // 10)
    wave[:fade_n] *= np.linspace(0, 1, fade_n)
    wave[-fade_n:] *= np.linspace(1, 0, fade_n)

    sd.play(wave, SAMPLE_RATE)
