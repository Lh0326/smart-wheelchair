from wheelchair_app.braincontrol.frown_detector import FrownDetector, FrownResult


def test_below_threshold_no_frown():
    d = FrownDetector(threshold=2.0)
    r = d.update(emg_level=1.0, dt_ms=100)
    assert not r.is_frowning
    assert not r.event


def test_above_threshold_immediate_no_event():
    """超阈值但未持续 min_ms，is_frowning=False（尚未确认）。"""
    d = FrownDetector(threshold=2.0)
    r = d.update(emg_level=3.0, dt_ms=100)  # 仅 100ms < 400ms
    assert not r.is_frowning
    assert not r.event


def test_sustained_above_min_ms_triggers_event():
    """超阈值持续 >= 400ms 触发一次事件。"""
    d = FrownDetector(threshold=2.0)
    # 0-300ms：未触发
    for _ in range(3):
        r = d.update(emg_level=3.0, dt_ms=100)
        assert not r.event
    # 400ms：触发
    r = d.update(emg_level=3.0, dt_ms=100)
    assert r.event
    assert r.is_frowning
    # 500ms：保持 is_frowning，不再有 event
    r = d.update(emg_level=3.0, dt_ms=100)
    assert r.is_frowning
    assert not r.event


def test_short_blip_no_trigger():
    """超阈值但 200ms 内回落，不触发。"""
    d = FrownDetector(threshold=2.0)
    d.update(emg_level=3.0, dt_ms=100)
    d.update(emg_level=3.0, dt_ms=100)
    r = d.update(emg_level=1.0, dt_ms=100)  # 回落
    assert not r.is_frowning
    assert not r.event


def test_too_long_resets():
    """超阈值超过 max_ms（800ms）放弃本次。"""
    d = FrownDetector(threshold=2.0)
    for _ in range(7):  # 700ms
        d.update(emg_level=3.0, dt_ms=100)
    r = d.update(emg_level=3.0, dt_ms=200)  # 总计 900ms > 800ms
    assert not r.is_frowning  # 超长放弃


def test_cooldown_blocks_immediate_retrigger():
    """触发后回落，1.5s 冷却期内不再触发。"""
    d = FrownDetector(threshold=2.0)
    # 触发
    for _ in range(4):
        d.update(emg_level=3.0, dt_ms=100)
    # 回落（启动冷却 1500ms）
    d.update(emg_level=1.0, dt_ms=100)
    # 立刻再次超阈值（冷却内 5 步：cd 从 1500 减到 1000）
    for _ in range(5):
        r = d.update(emg_level=3.0, dt_ms=100)  # 500ms 持续
        assert not r.event
    # 冷却期剩余 1000ms（10 步 cd 减到 0，但本步 above_ms 仍不累积；
    # 第 10 步 cd 减到 0 时 above_ms 才开始累积）
    for _ in range(9):
        r = d.update(emg_level=3.0, dt_ms=100)
        assert not r.event
    # 第 10 步：cd=0，above_ms 开始累积（=100）
    r = d.update(emg_level=3.0, dt_ms=100)
    assert not r.event
    # 再累积到 min_ms=400ms 触发（需 3 步：200, 300, 400）
    r = d.update(emg_level=3.0, dt_ms=100)  # above_ms=200
    assert not r.event
    r = d.update(emg_level=3.0, dt_ms=100)  # above_ms=300
    assert not r.event
    r = d.update(emg_level=3.0, dt_ms=100)  # above_ms=400 → 触发
    assert r.event  # 冷却后再次触发
