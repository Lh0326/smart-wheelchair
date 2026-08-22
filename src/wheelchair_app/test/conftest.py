"""pytest 公共 fixtures（参照 rtk_gnss/test/conftest.py）。"""
import os as _os
def _find_ws_root():
    r = _os.environ.get("WS_ROOT")
    if r: return r
    d = _os.path.dirname(_os.path.abspath(__file__))
    for _ in range(6):
        if _os.path.exists(_os.path.join(d, "env.sh")): return d
        d = _os.path.dirname(d)
    return d
_WS_ROOT = _find_ws_root()
_MODELS_ROOT = _os.environ.get("MODELS_ROOT", _os.path.join(_WS_ROOT, "models"))

import sys

import pytest
import rclpy

# ladar_ai.msg 是 rosidl 生成的 Python 包，安装在 ladar-ai workspace 的 dist-packages 下。
# 测试需要订阅 /tts_request (TTSRequest)，因此把该路径加入 sys.path。
_LADAR_AI_DIST = _WS_ROOT + '/third_party/ladar_ai_install/ladar_ai/local/lib/python3.10/dist-packages'
if _LADAR_AI_DIST not in sys.path:
    sys.path.insert(0, _LADAR_AI_DIST)


@pytest.fixture(scope='module')
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()
