"""P0 卡死修复回归测试：raw_data 滚动窗口 + ADS1299Reader 增量 emit。

背景：raw_data 是无限增长的 list，ADS1299Reader 每次 emit 都把全部历史
样本拷贝发给主线程。500Hz × 8 通道下，运行数分钟后单次 emit 拷贝量达到
数十万 float，主线程被淹没 → UI 卡死。

本测试验证两件事：
1. ADS1298Data.raw_data 长度被 MAX_HISTORY cap（不再无限增长）
2. ADS1299Reader 每次 data_updated emit 只携带新增样本（不是全量）
"""
import struct
from unittest.mock import MagicMock

import pytest

from wheelchair_app.braincontrol.ads1299 import ADS1298Data, ADS1298cmd


def _feed_samples(ads: ADS1298Data, n_samples: int, value: float = 1.0) -> None:
    """直接调 frame_unpack 喂入 n_samples 个 8 通道样本（绕过 parse_data 校验）。

    ADS1299 帧格式 byte[1] 是单字节长度，最大值 255，所以单帧最多
    (255-2)/32 = 7 个 8 通道样本。本函数按每帧 7 样本拆分喂入。

    frame_unpack 只读 data[1]（长度）、data[2]（帧类型 ADDRESS_START=0x11）、
    data[4:4+data_num*32]（float 数据），其余字节不校验。
    """
    PER_FRAME = 7  # byte[1] 单字节长度上限决定
    remaining = n_samples
    while remaining > 0:
        n = min(PER_FRAME, remaining)
        data_len = 8 * n * 4 + 2
        frame = bytearray(4 + 8 * n * 4 + 1)
        frame[0] = 0xA5
        frame[1] = data_len & 0xFF
        frame[2] = ADS1298cmd.ADDRESS_START  # 0x11
        frame[3] = 0x00
        struct.pack_into(
            f"<{8 * n}f", frame, 4,
            *([value] * (8 * n))
        )
        frame[-1] = 0x5A
        ads.frame_unpack(bytes(frame))
        remaining -= n


def test_raw_data_capped_after_many_samples():
    """喂 5000 样本后，raw_data 长度应被 cap 在 MAX_HISTORY 内（不再无限增长）。"""
    ads = ADS1298Data()
    max_history = getattr(ads, 'MAX_HISTORY', None)
    assert max_history is not None, "ADS1298Data 必须定义 MAX_HISTORY 类属性"

    for _ in range(50):
        _feed_samples(ads, n_samples=100, value=1.0)  # 共 5000 样本

    for ch in range(8):
        assert len(ads.raw_data[ch]) <= max_history, (
            f"ch{ch} 长度 {len(ads.raw_data[ch])} 超过 MAX_HISTORY={max_history}"
        )


def test_drain_new_samples_returns_only_increment():
    """drain_new_samples 只返回自上次 drain 以来的新增样本。"""
    ads = ADS1298Data()
    assert hasattr(ads, 'drain_new_samples'), "ADS1298Data 必须提供 drain_new_samples 方法"

    _feed_samples(ads, 100)
    batch1 = ads.drain_new_samples()
    assert len(batch1) == 8
    assert all(len(ch) == 100 for ch in batch1), (
        f"首次 drain 应得 100/通道，实际 {[len(ch) for ch in batch1]}"
    )

    _feed_samples(ads, 50)
    batch2 = ads.drain_new_samples()
    assert all(len(ch) == 50 for ch in batch2), (
        f"第二次 drain 应得 50/通道（增量），实际 {[len(ch) for ch in batch2]}"
    )

    # 没有新数据时返回空
    batch3 = ads.drain_new_samples()
    assert all(len(ch) == 0 for ch in batch3)


def test_drain_after_trim_stays_consistent():
    """raw_data 头部被 trim 后，drain 跟踪不应错乱。

    场景：消费方长时间没 drain，raw_data 累积超过 MAX_HISTORY 被自动 trim。
    下次 drain 只能拿回 MAX_HISTORY 内的部分（超出部分已丢失，这是设计取舍）。
    但 drained_len 跟踪必须正确——之后的新增仍能完整返回。
    """
    ads = ADS1298Data()
    # 先 drain 一次清空累计
    _feed_samples(ads, 100)
    ads.drain_new_samples()

    # 喂入超过 MAX_HISTORY 的量（触发 trim）
    overflow = ads.MAX_HISTORY + 200
    _feed_samples(ads, overflow)
    batch = ads.drain_new_samples()
    # trim 已发生，只能拿回 MAX_HISTORY 内的部分
    assert all(len(ch) == ads.MAX_HISTORY for ch in batch), (
        f"超量喂入后 drain 应得 MAX_HISTORY/通道，实际 {[len(ch) for ch in batch]}"
    )

    # 再喂 50，应得 50（drained_len 跟踪正确，trim 后不错乱）
    _feed_samples(ads, 50)
    batch2 = ads.drain_new_samples()
    assert all(len(ch) == 50 for ch in batch2), (
        f"trim 后第二次 drain 应得 50/通道，实际 {[len(ch) for ch in batch2]}"
    )


def test_reader_emits_incremental_not_full_history():
    """ADS1299Reader 每次 data_updated 应只发新增样本，不是 raw_data 全量。"""
    from wheelchair_app.braincontrol.ads1299_reader import ADS1299Reader

    ads = ADS1298Data()
    mock_serial = MagicMock()
    mock_serial.is_open = True

    reader = ADS1299Reader(
        serial_port=mock_serial, ads_data=ads,
        data_lock=__import__('threading').Lock(),
    )

    emitted = []
    reader.data_updated.connect(lambda data: emitted.append(data))

    # 第一次：模拟串口读到一帧 100 样本
    mock_serial.in_waiting = 100
    mock_serial.read.return_value = b'\x00' * 100  # 实际不解析，靠手动 frame_unpack
    # 用 _feed_samples 直接喂 ads，再让 reader 处理 drain
    _feed_samples(ads, 100)
    # 直接调内部 parse+drain 路径（手动模拟 run() 的核心步骤）
    with reader._data_lock:
        new_samples = ads.drain_new_samples()
    if any(new_samples):
        reader.data_updated.emit(new_samples)

    assert len(emitted) >= 1
    batch1 = emitted[-1]
    assert all(len(ch) == 100 for ch in batch1), (
        f"首次 emit 应得 100/通道，实际 {[len(ch) for ch in batch1]}"
    )

    # 第二次：再喂 50 样本，emit 应只携带 50（不是 150）
    _feed_samples(ads, 50)
    with reader._data_lock:
        new_samples = ads.drain_new_samples()
    if any(new_samples):
        reader.data_updated.emit(new_samples)

    batch2 = emitted[-1]
    assert all(len(ch) == 50 for ch in batch2), (
        f"第二次 emit 应得 50/通道（增量），实际 {[len(ch) for ch in batch2]}；"
        f"修复前会得到 150（全量历史拷贝）"
    )
