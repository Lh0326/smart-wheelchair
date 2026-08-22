"""ESP32+IMU 串口读取线程。

数据格式自适应（CSV / JSON），失败时通过 error signal 通知。
"""
import json
from typing import Optional, Callable

from PyQt5.QtCore import QThread, pyqtSignal


class ESP32ImuReader(QThread):
    """读取 ESP32+IMU 串口数据。

    信号：
        data_updated(list[float]): 成功解析一帧数据，元素顺序由 format 决定
            - 'euler_csv': [pitch, roll, yaw]（度）
            - 'quaternion_csv': [q0, q1, q2, q3]
            - 'raw_csv': [ax, ay, az, gx, gy, gz]
        error(str): 解析失败或串口异常
        bytes_received(int): 周期性上报接收字节数（用于任务 4 监控）
    """
    data_updated = pyqtSignal(list)
    error = pyqtSignal(str)
    bytes_received = pyqtSignal(int)

    def __init__(self, serial_port, data_format: str = 'auto',
                 on_frame: Optional[Callable] = None):
        """
        Args:
            serial_port: 已打开的 serial.Serial 实例
            data_format: 'auto' / 'euler_csv' / 'quaternion_csv' / 'raw_csv' / 'json'
            on_frame: 可选回调，签名为 (raw_line: str, parsed: list|None)
        """
        super().__init__()
        self.serial_port = serial_port
        self.data_format = data_format
        self.on_frame = on_frame
        self._running = False
        self._line_buffer = b''

    def run(self):
        self._running = True
        while self._running:
            try:
                if self.serial_port is None or not self.serial_port.is_open:
                    self.msleep(50)
                    continue
                n = self.serial_port.in_waiting
                if n == 0:
                    self.msleep(5)
                    continue
                raw = self.serial_port.read(n)
                self.bytes_received.emit(len(raw))
            except Exception as e:
                self.error.emit(f"串口异常: {e}")
                self._running = False
                break
            try:
                self._consume(raw)
            except Exception as e:
                self.error.emit(f"解析失败: {e}")
                self._line_buffer = b''

    def _consume(self, raw: bytes) -> None:
        """逐字节处理，遇到换行视为一帧。"""
        self._line_buffer += raw
        while b'\n' in self._line_buffer:
            line_bytes, self._line_buffer = self._line_buffer.split(b'\n', 1)
            line = line_bytes.decode('ascii', errors='ignore').strip()
            if not line:
                continue
            parsed = self._parse_line(line)
            if self.on_frame is not None:
                self.on_frame(line, parsed)
            if parsed is not None:
                self.data_updated.emit(parsed)

    def _parse_line(self, line: str) -> Optional[list]:
        """自适应解析：auto 模式每帧按 CSV→JSON 顺序尝试（不锁定，每帧重新判定格式）。"""
        if self.data_format == 'euler_csv' or (
                self.data_format == 'auto' and self._looks_like_csv_floats(line, 3)):
            return self._parse_csv(line)

        if self.data_format == 'quaternion_csv' or (
                self.data_format == 'auto' and self._looks_like_csv_floats(line, 4)):
            return self._parse_csv(line)

        if self.data_format == 'raw_csv' or (
                self.data_format == 'auto' and self._looks_like_csv_floats(line, 6)):
            return self._parse_csv(line)

        if self.data_format in ('json', 'auto'):
            try:
                obj = json.loads(line)
                if isinstance(obj, dict):
                    return list(obj.values())
            except json.JSONDecodeError:
                pass

        return None  # 无法解析

    def _looks_like_csv_floats(self, line: str, expected_count: int) -> bool:
        parts = line.split(',')
        if len(parts) != expected_count:
            return False
        try:
            [float(p) for p in parts]
            return True
        except ValueError:
            return False

    def _parse_csv(self, line: str) -> list:
        return [float(p) for p in line.split(',')]

    def stop(self) -> None:
        self._running = False
        self.wait(2000)
