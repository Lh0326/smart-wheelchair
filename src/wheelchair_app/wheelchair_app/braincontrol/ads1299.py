import struct
import numpy as np
from .eeg_filter import EegFilter


# 数据解析类
class ADS1298Data:
    # raw_data 滚动窗口大小：500Hz × 4s = 2000 样本，比下游 BrainControlTab 的
    # buffer（1000）大 2 倍，足以覆盖一次 emit 间隔内的全部新增。
    # 历史 bug：raw_data 无限 append，配合 ADS1299Reader 每次 emit 全量拷贝，
    # 运行数分钟后单次 emit 拷贝达数十万 float → 主线程被淹没 → UI 卡死。
    MAX_HISTORY = 2000

    def __init__(self):

        self.rx_buff = bytearray()   # 数据包接收缓存区
        self.raw_data = [[] for _ in range(8)]  # 原始数据

        self.sample_rate = 500  # 采样率

        # 内联滤波器（每通道一个，支持连续流）
        self.eeg_filters = [EegFilter(fs=self.sample_rate) for _ in range(8)]

        # 已 drain（消费）的样本索引（相对当前 raw_data 头部）。
        # 每次 drain_new_samples 把 raw_data[drained_len:cur_len] 返回并更新此值。
        # trim 头部时此值同步减去 trim_count，保持跟踪正确。
        self._drained_len = 0

    def clear(self):
        self.rx_buff = bytearray()
        self.raw_data = [[] for _ in range(8)]
        self._drained_len = 0
        for f in self.eeg_filters:
            f.reset()

    def drain_new_samples(self):
        """返回自上次 drain 以来新增的样本（按通道）。

        返回 [[ch0_s0, ch0_s1, ...], [ch1_s0, ...], ...] 共 8 个 list。
        trim 由 frame_unpack 在 append 时自动完成，本方法只负责增量返回。
        调用方（ADS1299Reader）应在 data_lock 内调用。
        """
        if not self.raw_data or not self.raw_data[0]:
            return [[] for _ in range(8)]
        cur_len = len(self.raw_data[0])
        new_samples = [list(ch[self._drained_len:cur_len])
                       for ch in self.raw_data]
        self._drained_len = cur_len
        return new_samples



    # 解包函数
    def parse_data(self, data_in):

        self.rx_buff.extend(data_in)

        while len(self.rx_buff) > 6:

            checkByte = self.rx_buff[1] ^ self.rx_buff[2]

            # 帧头校验
            if (self.rx_buff[0] == 0xA5) and (self.rx_buff[3] == checkByte):

                # 帧长度
                framLen = self.rx_buff[1] + 3

                # 判断剩于数据长度是否大于帧长度
                if framLen <= len(self.rx_buff):

                    # 校验帧尾
                    if self.rx_buff[framLen - 1] == 0x5A:
                        self.frame_unpack(bytes(self.rx_buff[0:framLen]))
                        del self.rx_buff[0:framLen]
                    else:
                        del self.rx_buff[0]
                else:
                    break
            else:
                del self.rx_buff[0]

    def frame_unpack(self, data):

        # 帧地址
        frameType = data[2]

        if frameType == ADS1298cmd.ADDRESS_SAMPLE_PAR:
            sample_rates = [125,250,500,1000,2000]
            idx = data[4] if data[4] < len(sample_rates) else 2
            self.sample_rate = sample_rates[idx]
            self.eeg_filters = [EegFilter(fs=self.sample_rate) for _ in range(8)]

        # 原始信号数据帧
        if frameType == ADS1298cmd.ADDRESS_START:
            data_num = int((data[1] - 2) / (4*8))
            temp = data[4:(4 + data_num * 32)]
            data_temp = struct.unpack(f"<{8 * data_num}f", temp)

            for i in range(data_num):
                ch_data = data_temp[i * 8: i * 8 + 8]
                filtered_ch_data = []
                for ch in range(8):
                    filtered = self.eeg_filters[ch].process_buffer([ch_data[ch]])[0]
                    filtered_ch_data.append(float(filtered))
                for ch in range(8):
                    self.raw_data[ch].append(filtered_ch_data[ch])

            # 滚动窗口：append 后立即 trim 头部，保证 raw_data 长度 <= MAX_HISTORY。
            # _drained_len 同步调整，让 drain_new_samples 仍能正确返回未消费的增量。
            if len(self.raw_data[0]) > self.MAX_HISTORY:
                trim_count = len(self.raw_data[0]) - self.MAX_HISTORY
                for ch in self.raw_data:
                    del ch[:trim_count]
                self._drained_len = max(0, self._drained_len - trim_count)




# 指令集
class ADS1298cmd:
    ADDRESS_HW_VERSION = 0x01  #硬件版本
    ADDRESS_SOFTWARE = 0x02  #软件版本
    ADDRESS_DEVICE_NAME = 0x03  #设备名称
    ADDRESS_DEVICE_MAC = 0x04  # MAC地址
    ADDRESS_POWER = 0x05  #电量信息
    ADDRESS_RESET = 0x06 # 复位
    ADDRESS_SAMPLE_PAR = 0x10 # 采样参数
    ADDRESS_START = 0x11  #开始/停止采集指令

    @staticmethod
    def cmd_data_pack(addr, is_write, data):

        cmd = []

        # 帧头
        cmd.append(0xAA)
        # 帧长度
        cmd.append(len(data) + 3)
        if is_write:
            cmd.append(0x80)  # 写指令
        else:
            cmd.append(0x81)  # 读指令
        # 地址
        cmd.append(addr)

        # 数据内容
        cmd += data

        # 校验码（待定）
        cmd.append(0x00)

        # 帧尾
        cmd.append(0xBB)

        # 计算校验码

        xor = 0x00
        for b in cmd[1:-1]:
            xor ^= b
        cmd[len(cmd) - 2] = xor
        return cmd



    # 开始采集指令
    @staticmethod
    def start_collect_cmd():
        data = [0] * 9
        data[0] = 0x03
        return ADS1298cmd.cmd_data_pack(ADS1298cmd.ADDRESS_START, True, data)

    # 停止采集指令
    @staticmethod
    def stop_collect_cmd():
        data = [0] * 9
        data[0] = 0x00
        return ADS1298cmd.cmd_data_pack(ADS1298cmd.ADDRESS_START, True, data)


