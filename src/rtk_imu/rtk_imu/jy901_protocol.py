"""JY901 维特标准协议常量、校验和、字段缩放。

参考：维特官方 WitStandardProtocol_JY901 示例(wit_normal_ros.py)
协议文档：https://wit-motion.yuque.com/wumwnr/ltst03/ucpd40hsgx3ymywv

包格式（11 字节）：
    [0]    0x55     包头
    [1]    Reg      寄存器类型（0x51=加速度, 0x52=角速度, 0x53=角度, 0x54=磁场）
    [2..9] Data     8 字节 payload
    [10]   Sum      校验和 = (前 10 字节累加) & 0xFF
"""
import math
import struct

# 包头与长度
HEADER = 0x55
PACKET_LEN = 11

# 寄存器类型
REG_ACC = 0x51       # 加速度 ax/ay/az（int16 × 3，量程 ±16g）
REG_GYRO = 0x52      # 角速度 wx/wy/wz（int16 × 3，量程 ±2000°/s）
REG_ANGLE = 0x53     # 姿态 roll/pitch/yaw（int16 × 3） + 版本（uint16）
REG_MAG = 0x54       # 磁场 mx/my/mz（int16 × 3，原始值）

# 缩放常量
ACC_SCALE_G = 16.0            # 加速度量程 ±16g
ACC_SCALE_MS2 = ACC_SCALE_G * 9.8   # 转 m/s²
GYRO_SCALE_DEG = 2000.0       # 角速度量程 ±2000°/s
ANGLE_SCALE_DEG = 180.0       # 角度量程 ±180°
SHORT_DIVISOR = 32768.0       # int16 → [-1, +1) 比例


def compute_checksum(data: bytes) -> int:
    """计算前 10 字节的校验和（& 0xFF）。

    Args:
        data: 长度为 10 的字节序列（不含 sum 字节）

    Returns:
        0-255 的校验和
    """
    return sum(data) & 0xFF


def verify_checksum(packet: bytes) -> bool:
    """验证 11 字节完整包的校验和。

    Args:
        packet: 长度为 11 的字节序列

    Returns:
        True 如果 packet[10] == compute_checksum(packet[0:10])，否则 False
    """
    if len(packet) != PACKET_LEN:
        return False
    return compute_checksum(packet[:10]) == packet[10]


def scale_short_to_float(raw: int, divisor: float = SHORT_DIVISOR) -> float:
    """int16 原始值 → [-1, +1) 浮点比例。

    Args:
        raw: int16 值（-32768 ~ 32767）
        divisor: 除数（默认 32768）

    Returns:
        浮点比例值
    """
    return raw / divisor


def parse_acc_payload(payload: bytes) -> list:
    """解析加速度 payload（8 字节，3 个 int16 + 2 字节填充）。

    Args:
        payload: 8 字节（前 6 字节为 ax/ay/az int16，后 2 字节忽略）

    Returns:
        [ax, ay, az] 单位 m/s²
    """
    ax_raw, ay_raw, az_raw = struct.unpack('<hhh', payload[:6])
    ax = scale_short_to_float(ax_raw) * ACC_SCALE_MS2
    ay = scale_short_to_float(ay_raw) * ACC_SCALE_MS2
    az = scale_short_to_float(az_raw) * ACC_SCALE_MS2
    return [ax, ay, az]


def parse_gyro_payload(payload: bytes) -> list:
    """解析角速度 payload（8 字节，3 个 int16 + 2 字节填充）。

    Returns:
        [wx, wy, wz] 单位 rad/s
    """
    wx_raw, wy_raw, wz_raw = struct.unpack('<hhh', payload[:6])
    wx = scale_short_to_float(wx_raw) * GYRO_SCALE_DEG * math.pi / 180
    wy = scale_short_to_float(wy_raw) * GYRO_SCALE_DEG * math.pi / 180
    wz = scale_short_to_float(wz_raw) * GYRO_SCALE_DEG * math.pi / 180
    return [wx, wy, wz]


def parse_angle_payload(payload: bytes) -> tuple:
    """解析姿态 payload（8 字节，3 个 int16 + 1 个 uint16 版本）。

    Returns:
        ([roll, pitch, yaw] 单位 rad, version: int)
    """
    roll_raw, pitch_raw, yaw_raw, version = struct.unpack('<hhhH', payload[:8])
    roll = scale_short_to_float(roll_raw) * ANGLE_SCALE_DEG * math.pi / 180
    pitch = scale_short_to_float(pitch_raw) * ANGLE_SCALE_DEG * math.pi / 180
    yaw = scale_short_to_float(yaw_raw) * ANGLE_SCALE_DEG * math.pi / 180
    return ([roll, pitch, yaw], version)


def parse_mag_payload(payload: bytes) -> list:
    """解析磁场 payload（8 字节，3 个 int16 + 2 字节填充）。

    Returns:
        [mx, my, mz] 原始 int16 值（标定后由上层转 µT）
    """
    mx, my, mz = struct.unpack('<hhh', payload[:6])
    return [mx, my, mz]
