"""PacketParser 字节流状态机单测"""
import struct

import pytest

from rtk_imu.packet_parser import PacketParser, PacketResult
from rtk_imu.jy901_protocol import (
    HEADER, REG_ACC, REG_GYRO, REG_ANGLE, REG_MAG,
    PACKET_LEN, compute_checksum,
)


def _make_packet(reg: int, payload_int16s: list, version: int = 0) -> bytes:
    """构造完整 11 字节包（含校验和）"""
    if reg == REG_ANGLE:
        # 角度包：3 个 int16 + 1 个 uint16
        data_bytes = struct.pack('<hhhH', *payload_int16s[:3], version)
    else:
        # 其他包：3 个 int16 + 2 字节填充
        data_bytes = struct.pack('<hhh', *payload_int16s[:3]) + b'\x00\x00'
    head = bytes([HEADER, reg])
    body = head + data_bytes
    cs = compute_checksum(body)
    return body + bytes([cs])


# ========== 单包解析 ==========

def test_parse_single_acc_packet():
    parser = PacketParser()
    pkt = _make_packet(REG_ACC, [0, 0, 2048])  # az = 9.8 m/s²
    results = parser.feed(pkt)
    assert len(results) == 1
    assert results[0].reg == REG_ACC
    assert results[0].acc[2] == pytest.approx(9.8, abs=0.01)


def test_parse_single_gyro_packet():
    parser = PacketParser()
    pkt = _make_packet(REG_GYRO, [0, 0, 0])
    results = parser.feed(pkt)
    assert len(results) == 1
    assert results[0].reg == REG_GYRO


def test_parse_single_angle_packet():
    parser = PacketParser()
    pkt = _make_packet(REG_ANGLE, [0, 0, 16384], version=0)  # yaw = π/2
    results = parser.feed(pkt)
    assert len(results) == 1
    assert results[0].reg == REG_ANGLE
    assert results[0].angle[2] == pytest.approx(3.14159 / 2, abs=1e-3)


def test_parse_single_mag_packet():
    parser = PacketParser()
    pkt = _make_packet(REG_MAG, [100, -200, 300])
    results = parser.feed(pkt)
    assert len(results) == 1
    assert results[0].mag == [100, -200, 300]


# ========== 多包流 ==========

def test_parse_multiple_packets_one_call():
    """一次性喂入多个包：都应被解析"""
    parser = PacketParser()
    pkt1 = _make_packet(REG_ACC, [0, 0, 0])
    pkt2 = _make_packet(REG_GYRO, [0, 0, 0])
    pkt3 = _make_packet(REG_ANGLE, [0, 0, 0])
    results = parser.feed(pkt1 + pkt2 + pkt3)
    assert len(results) == 3
    assert {r.reg for r in results} == {REG_ACC, REG_GYRO, REG_ANGLE}


def test_parse_split_packet_across_feeds():
    """包跨多次 feed 到达：仍能正确解析"""
    parser = PacketParser()
    pkt = _make_packet(REG_ACC, [0, 0, 0])
    # 分 3 次喂入：4 + 4 + 3 字节
    r1 = parser.feed(pkt[:4])
    r2 = parser.feed(pkt[4:8])
    r3 = parser.feed(pkt[8:11])
    all_results = r1 + r2 + r3
    assert len(all_results) == 1
    assert all_results[0].reg == REG_ACC


# ========== 异常恢复 ==========

def test_recovery_from_garbage_data():
    """字节流中有非 0x55 头数据：应跳过直到找到包头"""
    parser = PacketParser()
    pkt = _make_packet(REG_ACC, [0, 0, 0])
    garbage = b'\x00\xff\xab\xcd'  # 4 字节垃圾
    results = parser.feed(garbage + pkt)
    assert len(results) == 1
    assert results[0].reg == REG_ACC


def test_recovery_from_bad_checksum():
    """校验和错的包：丢弃，统计丢包，继续找下一个"""
    parser = PacketParser()
    pkt_bad = bytes([HEADER, REG_ACC, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF])  # 错的 sum
    pkt_good = _make_packet(REG_GYRO, [0, 0, 0])
    results = parser.feed(pkt_bad + pkt_good)
    assert len(results) == 1
    assert results[0].reg == REG_GYRO
    assert parser.dropped_count == 1


def test_dropped_count_initially_zero():
    parser = PacketParser()
    assert parser.dropped_count == 0


# ========== 状态字段累积 ==========

def test_angle_packet_fills_angle_field():
    """0x53 包后 PacketResult.angle 应有数据，acc/gyro/mag 应为 None"""
    parser = PacketParser()
    pkt = _make_packet(REG_ANGLE, [100, 200, 300])
    results = parser.feed(pkt)
    r = results[0]
    assert r.angle is not None
    assert r.acc is None
    assert r.gyro is None
    assert r.mag is None


def test_partial_state_reset_after_full_packet():
    """完整包解析后，状态机应回到 WAIT_HEADER"""
    parser = PacketParser()
    pkt = _make_packet(REG_ACC, [0, 0, 0])
    parser.feed(pkt)
    # 再喂一个新包，应正常解析（无残留缓冲干扰）
    pkt2 = _make_packet(REG_GYRO, [0, 0, 0])
    results = parser.feed(pkt2)
    assert len(results) == 1
    assert results[0].reg == REG_GYRO
