"""ADS1299Reader QThread：从串口持续读取 ADS1299 EEG 数据。

搬迁自 muscles-braincontrol/main_eeg.py（独立抽取，不依赖 MainWindow）。

信号：
    data_updated(list[list[float]]): 8 通道 × N 样本（每通道一个 list）

构造参数：
    serial_port: 已打开的 serial.Serial 实例（不是端口字符串）
    ads_data: ADS1298Data 实例（用于 parse_data）
    data_lock: threading.Lock 实例
"""
import logging

from PyQt5.QtCore import QThread, pyqtSignal

logger = logging.getLogger(__name__)


class ADS1299Reader(QThread):
    """从 ADS1299 串口持续读取 EEG 数据。"""

    data_updated = pyqtSignal(list)

    def __init__(self, serial_port, ads_data, data_lock):
        super().__init__()
        self.serial_port = serial_port
        self.ads_data = ads_data
        self._data_lock = data_lock
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            if self.serial_port is None or not self.serial_port.is_open:
                self.msleep(50)
                continue
            try:
                n = self.serial_port.in_waiting
                if n > 0:
                    com_data = self.serial_port.read(n)
                    with self._data_lock:
                        self.ads_data.parse_data(com_data)
                        # 只发新增样本（增量），不发 raw_data 全量历史。
                        # 历史 bug：每帧 [list(ch) for ch in raw_data] 复制全部累积样本，
                        # 运行数分钟后单次 emit 拷贝数十万 float → Qt 事件队列堆积 → UI 卡死。
                        new_samples = self.ads_data.drain_new_samples()
                    if any(new_samples):
                        self.data_updated.emit(new_samples)
                else:
                    self.msleep(20)
            except Exception as e:
                logger.error(f"ADS1299Reader run error: {e}")
                self.msleep(100)

    def stop(self):
        self._running = False
        self.wait(2000)
