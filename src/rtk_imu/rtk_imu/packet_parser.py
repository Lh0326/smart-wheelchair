"""HWT906P 字节流状态机解析器。

把串口读到的字节流喂入 feed()，吐出 PacketResult 列表。
每个 PacketResult 包含 reg + 对应字段（其他字段为 None）。
"""
from dataclasses import dataclass, field
from typing import List, Optional

from rtk_imu.jy901_protocol import (
    HEADER, REG_ACC, REG_GYRO, REG_ANGLE, REG_MAG,
    PACKET_LEN, verify_checksum,
    parse_acc_payload, parse_gyro_payload,
    parse_angle_payload, parse_mag_payload,
)


@dataclass
class PacketResult:
    """单包解析结果。只有 reg 对应字段有值，其他为 None。"""
    reg: int
    acc: Optional[list] = None       # [ax, ay, az] m/s²（仅 reg==0x51）
    gyro: Optional[list] = None      # [wx, wy, wz] rad/s（仅 reg==0x52）
    angle: Optional[list] = None     # [roll, pitch, yaw] rad（仅 reg==0x53）
    mag: Optional[list] = None       # [mx, my, mz] raw int16（仅 reg==0x54）
    version: int = 0


class PacketParser:
    """字节流状态机。

    状态：
        WAIT_HEADER: 寻找 0x55
        READING: 累积剩余 10 字节
    """

    def __init__(self):
        self._buffer = bytearray()
        self._state = 'WAIT_HEADER'
        self.dropped_count = 0  # 校验失败丢包计数

    def feed(self, data: bytes) -> List[PacketResult]:
        """喂入字节流，返回本次解析出的 PacketResult 列表。"""
        results = []
        self._buffer.extend(data)

        while True:
            if self._state == 'WAIT_HEADER':
                # 找包头
                idx = self._find_header(self._buffer)
                if idx == -1:
                    # 没找到包头，丢弃全部缓冲
                    self._buffer.clear()
                    break
                # 丢弃包头前的垃圾
                if idx > 0:
                    del self._buffer[:idx]
                self._state = 'READING'

            if self._state == 'READING':
                if len(self._buffer) < PACKET_LEN:
                    # 数据不够，等下次 feed
                    break
                # 取出一个完整包
                packet = bytes(self._buffer[:PACKET_LEN])
                del self._buffer[:PACKET_LEN]
                self._state = 'WAIT_HEADER'

                # 校验
                if not verify_checksum(packet):
                    self.dropped_count += 1
                    continue  # 继续循环找下一个包头

                # 解析
                result = self._parse_packet(packet)
                if result is not None:
                    results.append(result)

        return results

    @staticmethod
    def _find_header(buffer: bytearray) -> int:
        """在 buffer 中找 0x55 的索引，找不到返回 -1。"""
        for i, b in enumerate(buffer):
            if b == HEADER:
                return i
        return -1

    @staticmethod
    def _parse_packet(packet: bytes) -> Optional[PacketResult]:
        """解析单个 11 字节包。"""
        reg = packet[1]
        payload = packet[2:10]

        if reg == REG_ACC:
            return PacketResult(reg=reg, acc=parse_acc_payload(payload))
        elif reg == REG_GYRO:
            return PacketResult(reg=reg, gyro=parse_gyro_payload(payload))
        elif reg == REG_ANGLE:
            angle, version = parse_angle_payload(payload)
            return PacketResult(reg=reg, angle=angle, version=version)
        elif reg == REG_MAG:
            return PacketResult(reg=reg, mag=parse_mag_payload(payload))
        else:
            # 未知包类型（气压/GPS 等），忽略但不计为丢包
            return None

    def reset(self):
        """重置状态机和缓冲。"""
        self._buffer.clear()
        self._state = 'WAIT_HEADER'
        self.dropped_count = 0
