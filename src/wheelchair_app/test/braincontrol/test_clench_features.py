"""clench_features 单元测试：合成数据验证特征提取维度与可分性。"""
import numpy as np

from wheelchair_app.braincontrol.clench_features import (
    extract_clench_features,
    FEATURE_NAMES,
    CLENCH_WINDOW_SEC,
    CLENCH_STEP_SEC,
)


def _fs():
    return 500


def _n_win():
    return int(CLENCH_WINDOW_SEC * _fs())


def test_feature_names_count_matches_output():
    """提取出的特征数 == FEATURE_NAMES 长度。"""
    window = np.random.randn(_n_win(), 8).astype(np.float32) * 1.0
    feat = extract_clench_features(window, fs=_fs())
    assert feat.shape == (len(FEATURE_NAMES),)
    assert feat.dtype == np.float32


def test_feature_names_listed():
    """FEATURE_NAMES 至少包含：8 通道 EMG ratio + 总能量 + F7/F8 能量。"""
    # 8 通道 ratio + 1 总能量 + 2 (F7/F8) = 11 维（最小）
    assert len(FEATURE_NAMES) >= 11
    # 必含 ch0 / ch7（F7/F8）的 EMG ratio
    assert any('ch0' in n and 'ratio' in n for n in FEATURE_NAMES)
    assert any('ch7' in n and 'ratio' in n for n in FEATURE_NAMES)


def test_clench_higher_emg_than_relax():
    """咬牙窗口 EMG ratio（F7/F8 均值）显著高于放松窗口。"""
    fs = _fs()
    n = _n_win()
    t = np.arange(n) / fs
    rng = np.random.default_rng(42)

    # 放松：8 通道低幅 EEG 噪声（1-40Hz 占优）
    relax = np.zeros((n, 8))
    for ch in range(8):
        relax[:, ch] = (
            5.0 * np.sin(2 * np.pi * 10 * t)        # alpha
            + 2.0 * np.sin(2 * np.pi * 25 * t)      # beta
            + 0.5 * rng.standard_normal(n)
        )

    # 咬牙：在 F7(ch0) F8(ch7) 上叠加 60-95Hz 强 EMG
    clench = relax.copy()
    for ch in [0, 7]:
        emg = 30.0 * np.sin(2 * np.pi * 75 * t) + 25.0 * rng.standard_normal(n)
        clench[:, ch] = emg + 3.0 * np.sin(2 * np.pi * 10 * t)

    f_relax = extract_clench_features(relax, fs=fs)
    f_clench = extract_clench_features(clench, fs=fs)

    # 取 ch0/ch7 ratio 的均值，clench 应明显高于 relax
    ch0_idx = FEATURE_NAMES.index('emg_ratio_ch0')
    ch7_idx = FEATURE_NAMES.index('emg_ratio_ch7')
    relax_ratio = 0.5 * (f_relax[ch0_idx] + f_relax[ch7_idx])
    clench_ratio = 0.5 * (f_clench[ch0_idx] + f_clench[ch7_idx])
    assert clench_ratio > relax_ratio * 2.0


def test_f7_f8_energy_marker():
    """F7/F8 能量特征在咬牙时显著大于放松。

    relax 段用小幅真实 EEG（σ=0.3），clench 在 ch0/ch7 叠加大幅 EMG
    噪声 → 能量应有数量级差异。
    """
    fs = _fs()
    n = _n_win()
    t = np.arange(n) / fs
    rng = np.random.default_rng(0)
    relax = rng.standard_normal((n, 8)) * 0.3  # 真实 EEG 量级
    clench = relax.copy()
    clench[:, 0] += 20.0 * np.sin(2 * np.pi * 80 * t)
    clench[:, 7] += 20.0 * np.sin(2 * np.pi * 80 * t)
    f_relax = extract_clench_features(relax, fs=fs)
    f_clench = extract_clench_features(clench, fs=fs)
    idx = FEATURE_NAMES.index('f7_f8_energy')
    # log10 尺度：σ=0.3 noise × 2 通道 × 1000 样本 ≈ 180 → log10≈2.26
    # 加 20 单位 sine → ~200000 → log10≈5.3。差 ~3 个数量级。
    assert f_clench[idx] > 4.0
    assert f_relax[idx] < 2.5


def test_total_energy_increases():
    """总能量（log）在咬牙时升高。"""
    fs = _fs()
    n = _n_win()
    rng = np.random.default_rng(1)
    relax = rng.standard_normal((n, 8)) * 0.5
    clench = relax.copy() * 1.0
    clench[:, 0] += 15.0 * rng.standard_normal(n)
    f_relax = extract_clench_features(relax, fs=fs)
    f_clench = extract_clench_features(clench, fs=fs)
    idx = FEATURE_NAMES.index('total_energy_log')
    assert f_clench[idx] > f_relax[idx]


def test_step_and_window_constants():
    """默认窗口/步长与协议一致：2s 窗 / 0.5s 步。"""
    assert CLENCH_WINDOW_SEC == 2.0
    assert CLENCH_STEP_SEC == 0.5
