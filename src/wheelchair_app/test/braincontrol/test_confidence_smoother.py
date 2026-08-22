# tests/test_confidence_smoother.py
from wheelchair_app.braincontrol.confidence_smoother import ConfidenceSmoother


def test_high_score_transitions_to_focused():
    sm = ConfidenceSmoother()
    sm.update(p_focus=0.9)
    sm.update(p_focus=0.9)
    sm.update(p_focus=0.9)
    assert sm.state == 'focused'


def test_hysteresis_keeps_state_in_middle_band():
    sm = ConfidenceSmoother()
    for _ in range(5):
        sm.update(0.9)
    assert sm.state == 'focused'
    for _ in range(3):
        sm.update(0.5)
    assert sm.state == 'focused'  # 保持


def test_low_confidence_abstention_after_3_consecutive():
    sm = ConfidenceSmoother(margin=0.15)
    for _ in range(5):
        sm.update(0.9)
    assert sm.state == 'focused'
    sm.update(0.55)  # margin = 0.1 < 0.15
    sm.update(0.52)
    sm.update(0.55)
    assert sm.state == 'neutral'


def test_ewma_smoothing():
    sm = ConfidenceSmoother(alpha=0.3)
    # 初始 score=50（中性起步），连续 0.9/0.1 两次更新：
    # u1: 0.3*90 + 0.7*50 = 62
    # u2: 0.3*10 + 0.7*62 = 46.4
    sm.update(0.9)
    sm.update(0.1)
    assert abs(sm.score - 46.4) < 1.0


def test_starts_at_neutral_50():
    """初始 score 应为 50（中性），不是 90。"""
    sm = ConfidenceSmoother()
    assert sm.score == 50.0
    assert sm.state == 'neutral'


def test_reset_returns_to_neutral_50():
    """reset() 应回到中性 50，清除状态机和不确定计数。"""
    sm = ConfidenceSmoother()
    for _ in range(5):
        sm.update(0.9)
    assert sm.state == 'focused'
    assert sm.score > 60
    sm.reset()
    assert sm.score == 50.0
    assert sm.state == 'neutral'
    assert sm.uncertain_count == 0
