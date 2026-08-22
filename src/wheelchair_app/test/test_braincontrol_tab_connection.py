"""BrainControlTab 自动扫描 + 连接/断开测试。"""
from unittest.mock import MagicMock

import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')
pytest.importorskip('serial')


@pytest.fixture
def qt_app():
    from PyQt5.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def tab(qt_app):
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    return BrainControlTab(ros_node=MagicMock())


class _FakeReader:
    """通用 reader 替身。

    真实 ADS1299Reader/ESP32ImuReader 的 stop() 是终止 QThread 的正确方法
    （内部置 _running=False + wait(2000)），所以本替身只暴露 stop()——
    不提供 quit/terminate，强制被测代码走 stop() 路径，避免再次误用。
    """

    def __init__(self, kind='eeg'):
        self.kind = kind
        self.data_updated = MagicMock()  # PyQt5 pyqtSignal 替身
        self.error = MagicMock()
        self.started = False

    def start(self):
        self.started = True

    def isRunning(self):
        return self.started

    def wait(self, *args, **kwargs):
        return True

    def stop(self):
        self.started = False


class _FakePort:
    def __init__(self, device, vid=None, pid=None, serial_number=None):
        self.device = device
        self.vid = vid
        self.pid = pid
        self.serial_number = serial_number


def _patch_common(monkeypatch):
    """公共 patch：让 ADS1299Reader/ESP32ImuReader/serial.Serial 都不报错。"""
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.serial.Serial',
        lambda *a, **kw: MagicMock()
    )
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.QMessageBox.critical',
        lambda *a, **kw: None
    )


def test_connect_starts_both_qthreads(tab, monkeypatch):
    """点连接后同时启动 ADS1299Reader 和 ESP32ImuReader（自动扫描后）。"""
    started = {'eeg': False, 'imu': False}

    def eeg_factory(*a, **kw):
        r = _FakeReader(kind='eeg')
        orig_start = r.start
        def _s():
            started['eeg'] = True
            orig_start()
        r.start = _s
        return r

    def imu_factory(*a, **kw):
        r = _FakeReader(kind='imu')
        orig_start = r.start
        def _s():
            started['imu'] = True
            orig_start()
        r.start = _s
        return r

    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ADS1299Reader', eeg_factory
    )
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ESP32ImuReader', imu_factory
    )
    _patch_common(monkeypatch)

    # 自动扫描返回已识别的端口（替代旧的下拉选择）
    monkeypatch.setattr(
        tab, '_auto_detect_devices',
        lambda: ('/dev/ttyUSB99', '/dev/ttyACM99')
    )

    tab._on_connect_clicked()

    assert started['eeg'] is True, "ADS1299Reader 未启动"
    assert started['imu'] is True, "ESP32ImuReader 未启动"


def test_non_braincontrol_ports_are_excluded_from_external_scan(tab, monkeypatch):
    """脑控 EEG/IMU 自动扫描不能打开底盘、导航 IMU、雷达等非脑控串口。"""
    monkeypatch.setenv('CHASSIS_SERIAL_PORT', '/dev/wheelchair_chassis')

    assert tab._is_chassis_control_port('/dev/wheelchair_chassis') is True
    assert tab._is_chassis_control_port('/dev/ttyIMU') is True
    assert tab._is_chassis_control_port('/dev/LD14P') is True
    assert tab._is_chassis_control_port('/dev/lidar_n10p') is True
    assert tab._is_chassis_control_port('/dev/ttyBCIMU') is False
    assert tab._is_chassis_control_port('/dev/ttyEEG') is False


def test_auto_detect_prefers_stable_braincontrol_symlinks(tab, monkeypatch):
    """优先使用 /dev/ttyEEG 和 /dev/ttyBCIMU，避免 ttyUSB 编号漂移。"""
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.os.path.exists',
        lambda p: p in {'/dev/ttyEEG', '/dev/ttyBCIMU'},
    )
    monkeypatch.setattr(tab, '_test_eeg_port', lambda p: p == '/dev/ttyEEG')
    monkeypatch.setattr(tab, '_test_imu_port', lambda p: p == '/dev/ttyBCIMU')

    eeg_port, imu_port = tab._auto_detect_devices()

    assert eeg_port == '/dev/ttyEEG'
    assert imu_port == '/dev/ttyBCIMU'


def test_auto_detect_never_falls_back_to_ch340_for_braincontrol_imu(tab, monkeypatch):
    """脑控 IMU 只能是 /dev/ttyBCIMU，不能扫描 raw CH340 端口误碰 HWT906P。"""
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.os.path.exists',
        lambda p: p == '/dev/ttyEEG',
    )
    monkeypatch.setattr(tab, '_test_eeg_port', lambda p: p == '/dev/ttyEEG')
    monkeypatch.setattr(tab, '_test_imu_port', lambda p: True)
    monkeypatch.setattr(
        'serial.tools.list_ports.comports',
        lambda: [_FakePort('/dev/ttyUSB1', vid=0x1a86, pid=0x7523)],
    )

    eeg_port, imu_port = tab._auto_detect_devices()

    assert eeg_port == '/dev/ttyEEG'
    assert imu_port is None


def test_eeg_not_detected_still_starts_imu(tab, monkeypatch):
    """EEG 未检测到（自动扫描 None），IMU 仍能启动。"""
    imu_started = {'flag': False}

    def imu_factory(*a, **kw):
        r = _FakeReader(kind='imu')
        orig_start = r.start
        def _s():
            imu_started['flag'] = True
            orig_start()
        r.start = _s
        return r

    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ADS1299Reader',
        lambda *a, **kw: _FakeReader(kind='eeg')
    )
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ESP32ImuReader', imu_factory
    )
    _patch_common(monkeypatch)

    # EEG 未检测到，IMU 检测到
    monkeypatch.setattr(
        tab, '_auto_detect_devices',
        lambda: (None, '/dev/ttyACM99')
    )

    tab._on_connect_clicked()

    assert imu_started['flag'] is True
    assert '✗' in tab._eeg_status_label.text()
    # 任务 3：连接成功瞬间显示"已连接，等待数据..."（黄色，等数据心跳到位再转绿）
    assert '已连接' in tab._imu_status_label.text()


def test_eeg_failure_does_not_block_imu(tab, monkeypatch):
    """EEG 启动失败时（端口检测到但 Serial 打开失败），IMU 仍应启动。"""
    imu_started = {'flag': False}

    class FailEEG:
        def __init__(self, *a, **kw):
            raise OSError("EEG port busy")
        def start(self): pass
        def isRunning(self): return False
        def wait(self, *a): return True
        def stop(self): pass

    def imu_factory(*a, **kw):
        r = _FakeReader(kind='imu')
        orig_start = r.start
        def _s():
            imu_started['flag'] = True
            orig_start()
        r.start = _s
        return r

    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ADS1299Reader', FailEEG
    )
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ESP32ImuReader', imu_factory
    )
    _patch_common(monkeypatch)

    monkeypatch.setattr(
        tab, '_auto_detect_devices',
        lambda: ('/dev/ttyUSB99', '/dev/ttyACM99')
    )

    tab._on_connect_clicked()

    assert imu_started['flag'] is True
    assert '✗' in tab._eeg_status_label.text()
    # 任务 3：连接成功瞬间显示"已连接，等待数据..."（黄色，等数据心跳到位再转绿）
    assert '已连接' in tab._imu_status_label.text()


def test_disconnect_stops_both(tab, monkeypatch):
    """已连接时再次点击，两个 reader 都应调用 stop()（不是 quit/terminate）。

    ADS1299Reader/ESP32ImuReader 的 run() 是 while _running 自旋循环，不进
    Qt event loop，所以 QThread.quit() 对它们无效。本测试强制断开路径必须
    走 stop()——_FakeReader 故意不提供 quit/terminate，被测代码若误用会直接
    AttributeError 失败。
    """
    stopped = {'eeg': False, 'imu': False}

    def make_factory(kind):
        def factory(*a, **kw):
            r = _FakeReader(kind=kind)
            orig_stop = r.stop
            def _s():
                stopped[kind] = True
                orig_stop()
            r.stop = _s
            return r
        return factory

    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ADS1299Reader',
        make_factory('eeg')
    )
    monkeypatch.setattr(
        'wheelchair_app.tabs.braincontrol_tab.ESP32ImuReader',
        make_factory('imu')
    )
    _patch_common(monkeypatch)

    monkeypatch.setattr(
        tab, '_auto_detect_devices',
        lambda: ('/dev/ttyUSB99', '/dev/ttyACM99')
    )

    # 先连接
    tab._on_connect_clicked()
    # 再断开
    tab._on_connect_clicked()

    assert stopped['eeg'] is True, "EEG reader.stop() 未被调用"
    assert stopped['imu'] is True, "IMU reader.stop() 未被调用"
