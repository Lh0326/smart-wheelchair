"""HWT906P 维特标准协议解析单测

测试常量、校验和、字段缩放转换。
"""
import math
import struct

import pytest

from rtk_imu.jy901_protocol import (
    HEADER,
    REG_ACC,
    REG_GYRO,
    REG_ANGLE,
    REG_MAG,
    PACKET_LEN,
    compute_checksum,
    verify_checksum,
    parse_acc_payload,
    parse_gyro_payload,
    parse_angle_payload,
    parse_mag_payload,
    scale_short_to_float,
)


# ========== 常量 ==========

def test_header_value():
    assert HEADER == 0x55


def test_reg_values():
    assert REG_ACC == 0x51
    assert REG_GYRO == 0x52
    assert REG_ANGLE == 0x53
    assert REG_MAG == 0x54


def test_packet_length():
    assert PACKET_LEN == 11


# ========== 校验和 ==========

def test_compute_checksum_known_packet():
    """已知包：0x55 0x51 + 8 字节 + sum"""
    data = bytes([0x55, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    cs = compute_checksum(data)
    assert cs == (0x55 + 0x51) & 0xFF


def test_compute_checksum_overflow():
    """校验和溢出 0xFF 时回绕"""
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    cs = compute_checksum(data)
    assert cs == (0xFF * 10) & 0xFF  # = 0xF6


def test_verify_checksum_valid():
    """正确校验和的包：verify 返回 True"""
    data = bytes([0x55, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    cs = compute_checksum(data)
    full_packet = data + bytes([cs])
    assert verify_checksum(full_packet) is True


def test_verify_checksum_invalid():
    """错误校验和：verify 返回 False"""
    data = bytes([0x55, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    full_packet = data + bytes([0x00])  # 错的 sum
    assert verify_checksum(full_packet) is False


def test_verify_checksum_wrong_length():
    """长度不对：verify 返回 False"""
    assert verify_checksum(bytes([0x55, 0x51])) is False
    assert verify_checksum(bytes(15)) is False


# ========== 缩放函数 ==========

def test_scale_short_to_float_max():
    """int16 最大值 32767 → 接近 +1.0"""
    assert scale_short_to_float(32767, 32768.0) == pytest.approx(32767 / 32768, abs=1e-6)


def test_scale_short_to_float_min():
    """int16 最小值 -32768 → -1.0"""
    assert scale_short_to_float(-32768, 32768.0) == pytest.approx(-1.0, abs=1e-6)


def test_scale_short_to_float_zero():
    assert scale_short_to_float(0, 32768.0) == 0.0


# ========== 加速度解析 ==========

def test_parse_acc_zero_g():
    """静止状态：加速度应为 [0, 0, 0]（不含重力）"""
    payload = struct.pack('<hhh', 0, 0, 0) + b'\x00\x00'  # +2 字节填充
    acc = parse_acc_payload(payload)
    assert acc == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)


def test_parse_acc_z_up_1g():
    """z 轴 +1g：int16 = 32768 × 1 / 16 / 9.8（缩放比例 16g 量程）"""
    payload = struct.pack('<hhh', 0, 0, 2048) + b'\x00\x00'
    acc = parse_acc_payload(payload)
    assert acc[2] == pytest.approx(9.8, abs=0.01)
    assert acc[0] == pytest.approx(0.0, abs=1e-6)
    assert acc[1] == pytest.approx(0.0, abs=1e-6)


# ========== 角速度解析 ==========

def test_parse_gyro_zero():
    payload = struct.pack('<hhh', 0, 0, 0) + b'\x00\x00'
    gyro = parse_gyro_payload(payload)
    assert gyro == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)


def test_parse_gyro_full_scale():
    """满量程 2000°/s：raw=32767 → 约 34.9 rad/s"""
    payload = struct.pack('<hhh', 32767, 0, 0) + b'\x00\x00'
    gyro = parse_gyro_payload(payload)
    expected = 32767 / 32768 * 2000 * math.pi / 180
    assert gyro[0] == pytest.approx(expected, abs=1e-4)


# ========== 角度解析 ==========

def test_parse_angle_zero():
    payload = struct.pack('<hhhHH', 0, 0, 0, 0, 0)  # roll/pitch/yaw + version
    angle_rad, version = parse_angle_payload(payload)
    assert angle_rad == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)
    assert version == 0


def test_parse_angle_yaw_90deg():
    """yaw = 90° → raw = 32768/2 = 16384"""
    payload = struct.pack('<hhhHH', 0, 0, 16384, 0, 0)
    angle_rad, version = parse_angle_payload(payload)
    assert angle_rad[2] == pytest.approx(math.pi / 2, abs=1e-3)


# ========== 磁力计解析 ==========

def test_parse_mag_zero():
    payload = struct.pack('<hhh', 0, 0, 0) + b'\x00\x00'
    mag = parse_mag_payload(payload)
    assert mag == [0, 0, 0]


def test_parse_mag_raw_values():
    """磁力计返回原始 int16（不做单位转换，标定后由上层处理）"""
    payload = struct.pack('<hhh', 100, -200, 300) + b'\x00\x00'
    mag = parse_mag_payload(payload)
    assert mag == [100, -200, 300]
