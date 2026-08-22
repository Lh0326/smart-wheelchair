# tests/test_signal_quality.py
"""信号质量门测试。"""
import numpy as np
import pytest


def _clean_signal(fs=500, duration=2.0, seed=0):
    """合成干净 EEG：50μV α节律 + 小噪声。"""
    rng = np.random.default_rng(seed)
    t = np.arange(int(fs * duration)) / fs
    return 50 * np.sin(2 * np.pi * 10 * t) + 5 * rng.standard_normal(len(t))


def _noisy_50hz_signal(fs=500, duration=2.0, seed=0):
    """50Hz 严重污染的信号。"""
    rng = np.random.default_rng(seed)
    t = np.arange(int(fs * duration)) / fs
    return 30 * np.sin(2 * np.pi * 10 * t) + 100 * np.sin(2 * np.pi * 50 * t)


def _saturated_signal(fs=500, duration=2.0):
    """饱和信号：±3000μV。"""
    t = np.arange(int(fs * duration)) / fs
    return 3000 * np.sin(2 * np.pi * 5 * t)


def test_check_channel_returns_dataclass():
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker, ChannelQuality
    sig = _clean_signal()
    checker = SignalQualityChecker(fs=500)
    cq = checker.check_channel(sig, channel=0)
    assert isinstance(cq, ChannelQuality)
    assert cq.channel == 0
    assert 0 <= cq.score <= 100


def test_clean_signal_scores_high():
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker
    sig = _clean_signal()
    checker = SignalQualityChecker(fs=500)
    cq = checker.check_channel(sig, channel=0)
    assert cq.score >= 70
    assert 'range_too_large' not in cq.issues
    assert 'high_line_noise' not in cq.issues


def test_saturated_signal_scores_low():
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker
    sig = _saturated_signal()
    checker = SignalQualityChecker(fs=500)
    cq = checker.check_channel(sig, channel=0)
    assert cq.score <= 30
    assert 'range_too_large' in cq.issues


def test_50hz_pollution_detected():
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker
    sig = _noisy_50hz_signal()
    checker = SignalQualityChecker(fs=500)
    cq = checker.check_channel(sig, channel=0)
    assert cq.line_noise_50hz > 1.0
    assert 'high_line_noise' in cq.issues


def test_check_window_returns_per_channel():
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker
    fs = 500
    rng = np.random.default_rng(0)
    t = np.arange(int(fs * 2)) / fs
    window = np.zeros((len(t), 8))
    for ch in range(8):
        window[:, ch] = 50 * np.sin(2*np.pi*10*t) + 5 * rng.standard_normal(len(t))
    checker = SignalQualityChecker(fs=fs)
    results = checker.check_window(window)
    assert len(results) == 8
    assert all(0 <= r.score <= 100 for r in results)


def test_overall_score_returns_int_and_issues():
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker
    fs = 500
    rng = np.random.default_rng(0)
    t = np.arange(int(fs * 2)) / fs
    window = np.zeros((len(t), 8))
    for ch in range(8):
        window[:, ch] = 50 * np.sin(2*np.pi*10*t) + 5 * rng.standard_normal(len(t))
    checker = SignalQualityChecker(fs=fs)
    channels = checker.check_window(window)
    score, issues = checker.overall_score(channels)
    assert isinstance(score, int)
    assert 0 <= score <= 100
    assert isinstance(issues, list)


def test_overall_score_low_when_half_channels_bad():
    """一半通道严重污染，整体应该不及格。"""
    from wheelchair_app.braincontrol.signal_quality import SignalQualityChecker
    fs = 500
    rng = np.random.default_rng(0)
    t = np.arange(int(fs * 2)) / fs
    window = np.zeros((len(t), 8))
    for ch in range(8):
        if ch < 4:
            window[:, ch] = 50 * np.sin(2*np.pi*10*t) + 5 * rng.standard_normal(len(t))
        else:
            window[:, ch] = 3000 * np.sin(2*np.pi*5*t)
    checker = SignalQualityChecker(fs=fs)
    channels = checker.check_window(window)
    score, issues = checker.overall_score(channels)
    assert score < 60
