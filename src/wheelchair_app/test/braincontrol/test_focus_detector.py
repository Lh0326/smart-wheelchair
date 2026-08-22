# tests/test_focus_detector.py
import numpy as np
from unittest.mock import MagicMock
from wheelchair_app.braincontrol.focus_detector import FocusDetector, FocusResult


def _make_focus_detector_without_model():
    """不依赖真模型，用 mock classifier 测编排逻辑。

    用合成 EEG 信号（alpha + beta + theta）作为输入；白噪声高频段功率天然
    较高，会触发新版双频段 EMG 检测，所以用更接近真实 EEG 的合成信号。
    """
    det = FocusDetector(fs=500, model_type='svm', model_path=None)
    mock_clf = MagicMock()
    mock_clf.predict_proba.return_value = (0.85, 0.15)
    det.classifier = mock_clf
    return det


def _make_clean_eeg_window(seed=0):
    """合成干净 EEG 窗口：alpha + beta + theta + 微噪声。"""
    rng = np.random.default_rng(seed)
    t = np.arange(1000) / 500
    win = np.zeros((1000, 8))
    for ch in range(8):
        win[:, ch] = (
            40 * np.sin(2*np.pi*10*t)
            + 25 * np.sin(2*np.pi*20*t)
            + 20 * np.sin(2*np.pi*6*t)
            + 2 * rng.standard_normal(1000)
        )
    return win


def test_pipeline_returns_focus_result():
    det = _make_focus_detector_without_model()
    window = _make_clean_eeg_window()
    result = det.update(window)
    assert isinstance(result, FocusResult)
    assert 0 <= result.score <= 100
    assert result.state in ('focused', 'neutral', 'relaxed')
    assert 'tbr_ch3' in result.features or len(result.features) > 0


def test_pipeline_with_repeated_focus_input_converges_to_focused():
    det = _make_focus_detector_without_model()
    window = _make_clean_eeg_window()
    for _ in range(10):
        result = det.update(window)
    assert result.state == 'focused'


def test_emg_rejection_propagates_to_result():
    """EMG 重度污染时 FocusDetector 不崩溃。

    注：用户要求"取消 EMG 阈值"，severe_thresh 设为 inf，
    所以 artifact_rejected 永远是 False。此测试保留验证不崩溃。
    """
    det = _make_focus_detector_without_model()
    t = np.arange(1000) / 500
    severe = np.stack([20*np.sin(2*np.pi*80*t)] * 8, axis=1)
    result = det.update(severe)
    # EMG 阈值取消后不再触发丢弃，但应该正常返回 FocusResult（不崩溃）
    assert result.artifact_rejected is False
    assert result.emg_level > 0  # EMG 数值仍被记录，只是不用于门控


def test_focus_detector_accepts_explicit_baseline_alpha():
    """无模型时显式传 baseline_alpha，extractor 应使用它（非默认 ones）。"""
    baseline = np.array([0.5] * 8)
    det = FocusDetector(
        fs=500, model_type='svm', model_path=None, baseline_alpha=baseline,
    )
    np.testing.assert_allclose(det.extractor.baseline_alpha, baseline)


def test_focus_detector_loads_baseline_from_svm_model(tmp_path):
    """加载 SVM 模型时，extractor 自动用模型持久化的 baseline_alpha。"""
    import os
    from wheelchair_app.braincontrol.classifiers import SVMClassifier
    rng = np.random.default_rng(0)
    X = rng.normal(0, 1, (40, 5))
    y = (X[:, 0] > 0).astype(int)
    baseline = np.array([0.15, 0.18, 0.12, 0.20, 0.19, 0.13, 0.16, 0.14])
    clf = SVMClassifier(baseline_alpha=baseline)
    clf.fit(X, y)
    model_path = tmp_path / "svm.joblib"
    clf.save(str(model_path))

    det = FocusDetector(fs=500, model_type='svm', model_path=str(model_path))
    np.testing.assert_allclose(
        det.extractor.baseline_alpha, baseline, rtol=1e-6,
    )


def test_focus_detector_legacy_model_falls_back_to_explicit_baseline(tmp_path):
    """加载旧版 SVM（无 baseline_alpha），显式传入的 baseline 应被采用。"""
    import os
    from joblib import dump
    from sklearn.svm import SVC
    from sklearn.preprocessing import StandardScaler
    from sklearn.pipeline import Pipeline
    rng = np.random.default_rng(0)
    X = rng.normal(0, 1, (40, 5))
    y = (X[:, 0] > 0).astype(int)
    legacy = Pipeline([
        ('scaler', StandardScaler()),
        ('svm', SVC(kernel='rbf', probability=True, random_state=42)),
    ])
    legacy.fit(X, y)
    legacy_path = tmp_path / "legacy.joblib"
    dump(legacy, str(legacy_path))

    fallback = np.array([0.42] * 8)
    det = FocusDetector(
        fs=500, model_type='svm', model_path=str(legacy_path),
        baseline_alpha=fallback,
    )
    np.testing.assert_allclose(det.extractor.baseline_alpha, fallback)
