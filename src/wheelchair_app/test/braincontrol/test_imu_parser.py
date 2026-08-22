"""imu_reader 纯函数测试（不依赖 PyQt5 / 串口）。"""
import sys
import types


class _StubSignal:
    """模拟 pyqtSignal，仅记录 emit 调用。"""
    def __init__(self, *args, **kwargs):
        self.emits = []

    def emit(self, *args, **kwargs):
        self.emits.append(args)


class _StubQThread:
    """模拟 QThread 基类。"""
    def __init__(self, *args, **kwargs):
        pass

    @staticmethod
    def msleep(ms):
        pass

    @staticmethod
    def wait(timeout):
        pass


# 仅当环境无 PyQt5.QtCore 时才 stub，避免污染真实 PyQt5（其他测试如
# test_package_import 真实 import tilt_indicator 需要 Qt 等符号）。
try:
    from PyQt5.QtCore import QThread, pyqtSignal  # noqa: F401
except ImportError:
    sys.modules.setdefault('PyQt5', types.ModuleType('PyQt5'))
    qtcore = types.ModuleType('PyQt5.QtCore')
    qtcore.QThread = _StubQThread
    qtcore.pyqtSignal = _StubSignal
    sys.modules['PyQt5.QtCore'] = qtcore

from wheelchair_app.braincontrol.imu_reader import ESP32ImuReader  # noqa: E402


def _make_reader(fmt='auto'):
    """构造一个绕过 __init__ 的 reader 实例（不接触串口）。"""
    r = ESP32ImuReader.__new__(ESP32ImuReader)
    r.data_format = fmt
    r._line_buffer = b''
    r.on_frame = None
    r.data_updated = _StubSignal()
    r.error = _StubSignal()
    r.bytes_received = _StubSignal()
    return r


def test_parse_csv_3_fields():
    r = _make_reader()
    assert r._parse_line("1.0,2.0,3.0") == [1.0, 2.0, 3.0]


def test_parse_csv_4_fields():
    r = _make_reader()
    assert r._parse_line("0.1,0.2,0.3,0.4") == [0.1, 0.2, 0.3, 0.4]


def test_parse_csv_6_fields():
    r = _make_reader()
    result = r._parse_line("1.0,2.0,3.0,4.0,5.0,6.0")
    assert result == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_parse_json_dict():
    r = _make_reader()
    result = r._parse_line('{"pitch": 1.0, "roll": 2.0, "yaw": 3.0}')
    assert result == [1.0, 2.0, 3.0]


def test_parse_garbage_returns_none():
    r = _make_reader()
    assert r._parse_line("hello world") is None


def test_parse_empty_line_returns_none():
    r = _make_reader()
    # 空行在 _consume 里被 strip 掉了；_parse_line 本身对空字符串的处理
    # 空字符串 split(',') = [''], float('') 失败，返回 None
    assert r._parse_line("") is None


def test_looks_like_csv_floats_correct_count():
    r = _make_reader()
    assert r._looks_like_csv_floats("1.0,2.0,3.0", 3) is True
    assert r._looks_like_csv_floats("1.0,2.0,3.0", 4) is False
    assert r._looks_like_csv_floats("a,b,c", 3) is False


def test_consume_single_complete_line():
    r = _make_reader()
    r._consume(b"1.0,2.0,3.0\n")
    assert r._line_buffer == b''
    assert len(r.data_updated.emits) == 1
    assert r.data_updated.emits[0] == ([1.0, 2.0, 3.0],)


def test_consume_half_line_buffered_across_chunks():
    """跨 chunk 的半行应该缓冲拼接。"""
    r = _make_reader()
    r._consume(b"1,2,")
    assert r._line_buffer == b"1,2,"
    assert len(r.data_updated.emits) == 0
    r._consume(b"3\n")
    assert r._line_buffer == b''
    assert len(r.data_updated.emits) == 1
    assert r.data_updated.emits[0] == ([1.0, 2.0, 3.0],)


def test_consume_multiple_lines_in_one_chunk():
    r = _make_reader()
    r._consume(b"1,2,3\n4,5,6\n")
    assert r._line_buffer == b''
    assert len(r.data_updated.emits) == 2


def test_consume_garbage_line_skipped_no_emit():
    """无法解析的行不应触发 data_updated。"""
    r = _make_reader()
    r._consume(b"garbage line\n")
    assert r._line_buffer == b''
    assert len(r.data_updated.emits) == 0


def test_consume_on_frame_callback_invoked():
    """on_frame 回调对每行都调用（无论是否解析成功）。"""
    seen = []
    r = _make_reader()
    r.on_frame = lambda raw, parsed: seen.append((raw, parsed))
    r._consume(b"1,2,3\n")
    assert len(seen) == 1
    assert seen[0] == ("1,2,3", [1.0, 2.0, 3.0])

    r._consume(b"garbage\n")
    assert len(seen) == 2
    assert seen[1] == ("garbage", None)


def test_explicit_euler_csv_format():
    """显式 euler_csv 模式直接解析，不做字段数校验。"""
    r = _make_reader(fmt='euler_csv')
    assert r._parse_line("1.0,2.0,3.0") == [1.0, 2.0, 3.0]


def test_explicit_json_format():
    r = _make_reader(fmt='json')
    result = r._parse_line('{"a": 1.0, "b": 2.0}')
    assert result == [1.0, 2.0]
