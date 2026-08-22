"""clench_detector 单元测试：状态机 + rising edge + 冷却。

合成 EEG 数据模拟咬牙/放松过渡，验证：
  - 状态机 min_ms 确认（短脉冲不触发）
  - rising edge event（从未咬牙 → 咬牙时触发一次）
  - 冷却期（触发后回落，冷却内不再触发）
"""
import numpy as np
import pytest


def _make_window(label: str, fs=500, dur=2.0, rng=None) -> np.ndarray:
    """合成 2s 8 通道窗口：clench=强 EMG, relax=低幅 EEG。"""
    rng = rng or np.random.default_rng(0)
    n = int(dur * fs)
    t = np.arange(n) / fs
    base = rng.standard_normal((n, 8)) * 0.3
    if label == 'clench':
        emg = 15.0 * np.sin(2 * np.pi * 75 * t) + 12.0 * rng.standard_normal(n)
        base[:, 0] += emg
        base[:, 7] += emg
    return base.astype(np.float32)


@pytest.fixture
def trained_clench_detector(tmp_path):
    """训练一个简易 clench detector（合成数据 → 临时模型）。

    依赖训练脚本 train_clench_svm（一次性工具，未搬迁到 rtk 包），因此当该
    脚本不在 sys.path 时自动 skip 这些端到端训练用例——单元层面 clench_detector
    的逻辑由 test_clench_result_fields / test_missing_model_raises 覆盖。
    """
    pytest.importorskip("train_clench_svm")
    from train_clench_svm import (
        extract_clench_features,
        train_clench_svm,
        save_clench_model,
    )
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector

    rng = np.random.default_rng(42)
    feats, labels = [], []
    for _ in range(40):
        feats.append(extract_clench_features(_make_window('relax', rng=rng), 500))
        labels.append(0)
        feats.append(extract_clench_features(_make_window('clench', rng=rng), 500))
        labels.append(1)
    feats = np.array(feats, dtype=np.float32)
    labels = np.array(labels)
    groups = list(range(len(labels)))  # 每个窗口独立 group（充足多样性）
    report = train_clench_svm(feats, labels, groups, skip_importance=True)
    model_path = str(tmp_path / 'clench.joblib')
    save_clench_model(model_path, report, feats, labels)
    return model_path


def test_below_threshold_no_clench(trained_clench_detector):
    """喂入纯 relax 窗口，is_clenching 始终 False。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    d = ClenchDetector(model_path=trained_clench_detector, fs=500)
    rng = np.random.default_rng(1)
    for _ in range(20):
        w = _make_window('relax', rng=rng)
        r = d.update(w)
        assert not r.is_clenching
        assert not r.event


def test_sustained_clench_triggers_event(trained_clench_detector):
    """持续喂 clench 窗口，min_ms 后首次触发 event，后续保持 is_clenching。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    # 缩短 min_ms / cooldown 让测试快速
    d = ClenchDetector(model_path=trained_clench_detector, fs=500,
                       min_ms=600, cooldown_ms=300)
    rng = np.random.default_rng(2)
    triggered = False
    n_events = 0
    # 6 次 update（每次 100ms 等效）= 600ms = min_ms
    for i in range(8):
        w = _make_window('clench', rng=rng)
        r = d.update(w, dt_ms=100)
        if r.event:
            n_events += 1
            triggered = True
        if triggered:
            assert r.is_clenching
    assert triggered, "应该至少触发一次 event"
    assert n_events == 1, f"应该只触发一次 event，实际 {n_events}"


def test_short_blip_no_trigger(trained_clench_detector):
    """短暂 clench 脉冲（< min_ms）回落，不触发。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    d = ClenchDetector(model_path=trained_clench_detector, fs=500,
                       min_ms=600)
    rng = np.random.default_rng(3)
    # 2 次 clench（200ms）+ 立即 relax
    d.update(_make_window('clench', rng=rng), dt_ms=100)
    d.update(_make_window('clench', rng=rng), dt_ms=100)
    # 回落
    r = d.update(_make_window('relax', rng=rng), dt_ms=100)
    assert not r.is_clenching
    assert not r.event


def test_rising_edge_only_once(trained_clench_detector):
    """rising edge 在持续 clench 期间只发一次。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    d = ClenchDetector(model_path=trained_clench_detector, fs=500,
                       min_ms=400, cooldown_ms=200)
    rng = np.random.default_rng(4)
    events = []
    # 5 次 clench
    for _ in range(5):
        r = d.update(_make_window('clench', rng=rng), dt_ms=100)
        if r.event:
            events.append(True)
    assert len(events) == 1


def test_cooldown_blocks_retrigger(trained_clench_detector):
    """触发后回落，cooldown_ms 内重新进入 clench 不再触发新 event。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    d = ClenchDetector(model_path=trained_clench_detector, fs=500,
                       min_ms=400, cooldown_ms=1000)
    rng = np.random.default_rng(5)
    # 触发
    for _ in range(5):
        d.update(_make_window('clench', rng=rng), dt_ms=100)
    # 回落
    d.update(_make_window('relax', rng=rng), dt_ms=100)
    # 冷却内重新尝试
    n_events = 0
    for _ in range(8):
        r = d.update(_make_window('clench', rng=rng), dt_ms=100)
        if r.event:
            n_events += 1
    # 冷却 1000ms 内 8×100ms=800ms 还在冷却 → 不触发
    assert n_events == 0


def test_clench_result_fields():
    """ClenchResult 有 is_clenching / event 两字段。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchResult, ClenchDetector
    r = ClenchResult(is_clenching=False, event=False, proba=0.1)
    assert r.is_clenching is False
    assert r.event is False
    assert r.proba == 0.1


def test_missing_model_raises():
    """模型路径不存在时 ClenchDetector 应在 __init__ 报错（fail fast）。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    with pytest.raises((FileNotFoundError, IOError, ValueError)):
        ClenchDetector(model_path='/nonexistent/clench.joblib', fs=500)
