"""DK-2500 全局配置（Intel Core Ultra 5 225U + Ubuntu 22.04）

硬件调度策略 (实测):
  NPU  (~11 TOPS INT8) → YOLO 目标检测（OpenVINO Native, ~14ms/帧）
  iGPU (~8 Xe Cores)   → 预留（sherpa-onnx 未编译 OpenVINO 后端，当前无法使用）
  CPU  (12C/14T)        → KWS 唤醒词 + ASR 语音识别 + TTS 合成 + 控制逻辑 + Web 服务
"""

import platform
from pathlib import Path

# ── 路径配置（自动适配 Windows MVP / Linux DK-2500）──────────────────
if platform.system() == "Windows":
    BASE_DIR = Path(r"D:\he-AImodel")
else:
    BASE_DIR = Path(__file__).resolve().parent.parent

MODEL_DIR = BASE_DIR / "models"
VOICE_DIR = MODEL_DIR / "voice"
YOLO_DIR = MODEL_DIR / "yolo"
FRONTEND_DIR = BASE_DIR / "frontend"

# Voice model paths
KWS_MODEL_DIR = VOICE_DIR / "kws" / "sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
ASR_OFFLINE_DIR = VOICE_DIR / "asr-offline" / "manyeyes" / "k2transducer-zipformer-ctc-zh-onnx-offline-20250703"
TTS_MODEL_DIR = VOICE_DIR / "tts" / "kokoro-int8-multi-lang-v1_1"
VAD_MODEL = VOICE_DIR / "vad" / "silero_vad.onnx"
KEYWORDS_FILE = VOICE_DIR / "config" / "keywords.txt"

# ── 硬件设备配置 ──────────────────────────────────────────────────────
# YOLO: yolo26s INT8 量化（NPU 有算子兼容问题，当前用 GPU）
YOLO_DEVICE = "GPU"
# ASR/TTS: sherpa-onnx CPU 推理
ASR_DEVICE = "cpu"
TTS_DEVICE = "cpu"
KWS_DEVICE = "cpu"

# ── OpenVINO 模型路径 ─────────────────────────────────────────────────
YOLO_PT_MODEL = YOLO_DIR / "yolo26s.pt"
YOLO_INT8_MODEL_DIR = YOLO_DIR / "yolo26s_int8_640"
YOLO_INT8_MODEL = YOLO_INT8_MODEL_DIR / "yolo26s_int8.xml"

# ── DK-2500 优化开关 ─────────────────────────────────────────────────
# ASR 常驻内存（DK-2500: True，16GB 足够；Windows MVP: False 按需加载）
ASR_ALWAYS_LOADED = platform.system() != "Windows"
# 深度相机（DK-2500 + Gemini 335L: True；MVP 无深度相机: False）
DEPTH_ENABLED = True
# YOLO 类别过滤（智慧轮椅场景：行人 + 车辆 + 红绿灯）
YOLO_ALLOWED_CLASSES = {
    "person", "car", "bicycle", "motorcycle", "bus", "truck",
    "traffic light",
}
# 各类别置信度阈值覆盖（默认用全局 conf，此处可调整易误检类别）
CLASS_CONF_OVERRIDE = {
    "traffic light": 0.15,            # 红绿灯通常较小，降低阈值提高召回
}

# ── Audio ─────────────────────────────────────────────────────────────
SAMPLE_RATE = 16000
CHANNELS = 1

# Timing (seconds)
ASR_TIMEOUT = 8.0
MAX_RECORDING_DURATION = 10.0
VAD_SILENCE_SEC = 0.8

# TTS
TTS_SAMPLE_RATE = 24000
TTS_SPEAKER_ID = 45  # zf_xiaobei

# ── Command map ───────────────────────────────────────────────────────
COMMAND_MAP = {
    "去客厅": "NAV_LIVING_ROOM",
    "去卧室": "NAV_BEDROOM",
    "去医院": "NAV_HOSPITAL",
    "回家": "NAV_HOME",
    "停止": "STOP",
    "等一下": "STOP",
    "紧急停止": "EMERGENCY_STOP",
    "加速": "SPEED_UP",
    "减速": "SPEED_DOWN",
    "慢点": "SPEED_DOWN",
    "快点": "SPEED_UP",
    "电量多少": "QUERY_BATTERY",
    "我在哪里": "QUERY_LOCATION",
    "前方有什么": "QUERY_AHEAD",
    "确认": "CONFIRM",
    "取消": "CANCEL",
    "好的": "CONFIRM",
    "帮助": "HELP",
}

# Emergency keywords (hot-detection during RECORDING)
EMERGENCY_KEYWORDS = ["紧急停止", "急停", "停"]

# TTS feedback texts
TTS_FEEDBACK = {
    "WAKE": "我在，请说命令",
    "CONFIRM": "好的，{cmd}",
    "UNRECOGNIZED": "抱歉，我没有听清，请再说一次",
    "TIMEOUT": "已超时，如需使用请再次唤醒",
}

# Traffic light advice（国内人行道红绿灯只有红/绿）
TRAFFIC_LIGHT_ADVICE = {
    "红灯": "前方红灯，请停车等待",
    "绿灯": "前方绿灯，可以通行，请注意周围",
}

# Speed levels (m/s)
SPEED_LEVELS = [0.3, 0.5, 0.8]

# Thread counts
KWS_THREADS = 1
ASR_THREADS = 2
TTS_THREADS = 2
