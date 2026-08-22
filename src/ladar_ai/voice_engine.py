"""语音引擎：唤醒词检测 + ASR 语音识别 + 指令解析。
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


使用 ai-model 的模型路径：
  KWS: sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
  ASR: sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30
  VAD: silero_vad.onnx
"""
import os
import time
import logging
from enum import Enum, auto
from typing import Dict, List, Optional

try:
    import sherpa_onnx
except ImportError:
    sherpa_onnx = None

try:
    import numpy as np
except ImportError:
    np = None

logger = logging.getLogger(__name__)

_PROJECT_VOICE_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "models", "voice")
)
_VOICE_ROOTS = [
    os.environ.get("LADAR_AI_VOICE_MODEL_ROOT", ""),
    "" + _MODELS_ROOT + "/models/voice",
    "" + _MODELS_ROOT + "/models/voice",
    _PROJECT_VOICE_DIR,
]


def _first_existing(paths: List[str], default: str = "") -> str:
    for path in paths:
        if path and os.path.exists(path):
            return path
    return default


# ---------- 模型路径 ----------
KWS_DIR = _first_existing([
    os.path.join(root, "kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20")
    for root in _VOICE_ROOTS
])
ASR_DIR = _first_existing([
    os.path.join(root, "asr/sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30")
    for root in _VOICE_ROOTS
])
VAD_MODEL = _first_existing([
    os.path.join(root, "vad/silero_vad.onnx")
    for root in _VOICE_ROOTS
])
KEYWORDS_FILE = _first_existing([
    os.path.join(root, "config/keywords.txt")
    for root in _VOICE_ROOTS
])

MAX_COMMAND_SECONDS = float(os.environ.get("LADAR_AI_COMMAND_TIMEOUT_SEC", "6.0"))
MIN_COMMAND_SECONDS = 0.3
POST_WAKE_IGNORE_SECONDS = float(os.environ.get("LADAR_AI_POST_WAKE_IGNORE_SEC", "1.5"))


class State(Enum):
    """语音引擎状态。"""
    IDLE = auto()
    RECORDING = auto()


# 唤醒词
WAKE_WORDS = ["小智你好", "小志你好", "心语启动"]

# 方位关键词映射
DIRECTION_KEYWORDS: Dict[str, List[str]] = {
    "front":        ["前方", "正前方", "前面", "前边"],
    "front_left":   ["左前方", "左前", "左前面"],
    "front_right":  ["右前方", "右前", "右前面"],
    "left":         ["左边", "左侧", "左面", "左手边"],
    "right":        ["右边", "右侧", "右面", "右手边"],
    "rear":         ["后方", "正后方", "后面", "后边"],
    "rear_left":    ["左后方", "左后", "左后面"],
    "rear_right":   ["右后方", "右后", "右后面"],
    "all":          ["全部", "所有", "周围", "四面"],
}

# 动作关键词
ACTION_KEYWORDS = {
    "query":          ["查询", "问一下", "距离", "情况", "怎么样", "什么", "有没有", "看一下"],
    "stop":           ["停止播报", "暂停播报", "停止说话", "别说了", "安静", "闭嘴", "停止"],
    "emergency_stop": ["紧急停车", "紧急停止", "立即停止", "刹车", "危险"],
    "speed_up":       ["加速", "快一点", "快点", "提高速度"],
    "speed_down":     ["减速", "慢一点", "慢点", "降低速度"],
}

_ACTION_PRIORITY = ["emergency_stop", "stop", "speed_down", "speed_up", "query"]


class VoiceEngine:
    """语音引擎：唤醒词检测 + 流式 ASR + 指令解析。"""

    def __init__(self, model_dir: str = "", sample_rate: int = 16000):
        """
        Parameters
        ----------
        model_dir : str
            语音模型根目录（已弃用，保留接口兼容，实际使用硬编码路径）。
        sample_rate : int
            音频采样率。
        """
        self.sample_rate = sample_rate
        self.state = State.IDLE
        self._audio_buffer: List = []
        self._kws = None
        self._kws_stream = None
        self._asr = None
        self._asr_stream = None
        self._vad = None
        self._recording_started_at = 0.0
        self._recording_samples = 0

        if sherpa_onnx is None:
            logger.error("sherpa_onnx 未安装，语音引擎不可用")
            return

        # ---------- 加载唤醒词检测模型（和 ai-model 完全一致） ----------
        self._init_kws()

        # ---------- 加载离线 ASR 模型（zipformer_ctc 模式） ----------
        self._init_asr()

        # ---------- 加载 VAD ----------
        self._init_vad()

    def _init_kws(self) -> None:
        """初始化关键词检测 (KWS)，与 ai-model 配置一致。"""
        if not os.path.isdir(KWS_DIR):
            logger.warning("KWS 模型目录不存在: %s", KWS_DIR)
            return

        encoder = f"{KWS_DIR}/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx"
        decoder = f"{KWS_DIR}/decoder-epoch-13-avg-2-chunk-16-left-64.onnx"
        joiner = f"{KWS_DIR}/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx"
        tokens = f"{KWS_DIR}/tokens.txt"

        if not all(os.path.isfile(f) for f in [encoder, decoder, joiner, tokens]):
            logger.warning("KWS 模型文件不完整")
            return

        try:
            keywords_score = float(os.environ.get("LADAR_AI_KWS_SCORE", "1.2"))
            keywords_threshold = float(os.environ.get("LADAR_AI_KWS_THRESHOLD", "0.15"))
            self._kws = sherpa_onnx.KeywordSpotter(
                tokens=tokens,
                encoder=encoder,
                decoder=decoder,
                joiner=joiner,
                keywords_file=KEYWORDS_FILE if os.path.isfile(KEYWORDS_FILE) else "",
                num_threads=1,
                max_active_paths=4,
                keywords_score=keywords_score,
                keywords_threshold=keywords_threshold,
                num_trailing_blanks=1,
                provider="cpu",
            )
            self._kws_stream = self._kws.create_stream()
            logger.info(
                "KWS 模型加载成功: keywords=%s score=%.2f threshold=%.2f",
                KEYWORDS_FILE,
                keywords_score,
                keywords_threshold,
            )
        except Exception as e:
            logger.error("KWS 模型加载失败: %s", e)

    def _init_asr(self) -> None:
        """初始化流式 ASR（OnlineRecognizer + endpoint 检测）。

        相比离线 ASR：边收音边出字，endpoint 触发即收尾，无需录完整段才解码。
        """
        if not os.path.isdir(ASR_DIR):
            logger.warning("ASR 模型目录不存在: %s", ASR_DIR)
            return

        encoder = f"{ASR_DIR}/encoder.int8.onnx"
        decoder = f"{ASR_DIR}/decoder.onnx"
        joiner = f"{ASR_DIR}/joiner.int8.onnx"
        tokens = f"{ASR_DIR}/tokens.txt"

        if not all(os.path.isfile(f) for f in [encoder, decoder, joiner, tokens]):
            logger.warning("ASR 流式模型文件不完整")
            return

        try:
            # sherpa_onnx 1.13.2: endpoint 参数直接作为扁平 kwargs 传给 from_transducer
            # 不存在 EndpointConfig / FeatureConfig 类
            # 注意：rule2/rule3 的真实参数名是 rule2_min_trailing_silence /
            # rule3_min_utterance_length（ sherpa_onnx 没有把 utterance_length 命名为 max）
            self._asr = sherpa_onnx.OnlineRecognizer.from_transducer(
                tokens=tokens,
                encoder=encoder,
                decoder=decoder,
                joiner=joiner,
                num_threads=2,
                provider="cpu",
                sample_rate=self.sample_rate,
                feature_dim=80,
                enable_endpoint_detection=True,
                rule1_min_trailing_silence=0.6,
                rule2_min_trailing_silence=0.6,
                rule3_min_utterance_length=0.3,
            )
            logger.info("ASR 流式模型加载成功 (OnlineRecognizer + endpoint)")
        except Exception as e:
            logger.error("ASR 流式模型加载失败: %s", e)

    def _init_vad(self) -> None:
        """初始化 VAD（语音活动检测）。"""
        if not os.path.isfile(VAD_MODEL):
            logger.warning("VAD 模型不存在: %s", VAD_MODEL)
            return

        try:
            vad_config = sherpa_onnx.VadModelConfig(
                silero_vad=sherpa_onnx.SileroVadModelConfig(
                    model=VAD_MODEL,
                    threshold=0.45,
                    min_silence_duration=0.6,
                    min_speech_duration=0.20,
                    max_speech_duration=MAX_COMMAND_SECONDS,
                ),
                sample_rate=self.sample_rate,
                num_threads=1,
                provider="cpu",
            )
            self._vad = sherpa_onnx.VoiceActivityDetector(vad_config, buffer_size_in_seconds=30)
            logger.info("VAD 模型加载成功")
        except Exception as e:
            logger.error("VAD 模型加载失败: %s", e)

    def process_audio(self, samples) -> Optional[Dict]:
        """处理一段音频数据。

        Parameters
        ----------
        samples : numpy.ndarray
            音频采样数据 (float32, 16kHz)。

        Returns
        -------
        dict or None
            识别到指令时返回 {"action": ..., "direction": ..., "text": ...}。
        """
        if sherpa_onnx is None or np is None:
            return None

        if self.state == State.IDLE:
            return self._process_idle(samples)
        elif self.state == State.RECORDING:
            return self._process_recording(samples)
        return None

    def _process_idle(self, samples) -> Optional[Dict]:
        """IDLE 状态：检测唤醒词。"""
        if self._kws is None or self._kws_stream is None:
            return None

        try:
            self._kws_stream.accept_waveform(self.sample_rate, samples.tolist())
            while self._kws.is_ready(self._kws_stream):
                self._kws.decode_stream(self._kws_stream)
                keyword = self._kws.get_result(self._kws_stream)
                if keyword:
                    logger.info("检测到唤醒词: %s", keyword)
                    self._start_recording()
                    return {"action": "wakeup", "direction": None, "text": str(keyword)}
        except Exception as e:
            logger.error("KWS 处理异常: %s", e)

        return None

    def _start_recording(self) -> None:
        """进入唤醒后的指令录音窗口。"""
        self.state = State.RECORDING
        self._audio_buffer = []
        self._recording_started_at = time.monotonic()
        self._recording_samples = 0
        if self._vad is not None:
            self._vad.reset()
        # 创建流式 ASR stream
        if self._asr is not None:
            self._init_recording_stream()
        # 检测到唤醒词后重置 KWS stream，避免同一段音频重复触发。
        if self._kws is not None:
            try:
                self._kws_stream = self._kws.create_stream()
            except Exception:
                self._kws_stream = None

    def _finish_recording(self) -> None:
        self.state = State.IDLE
        self._audio_buffer = []
        self._recording_started_at = 0.0
        self._recording_samples = 0
        self._asr_stream = None
        if self._vad is not None:
            self._vad.reset()

    def _init_recording_stream(self):
        """进入 RECORDING 时创建 OnlineStream。"""
        if self._asr is None:
            return
        try:
            self._asr_stream = self._asr.create_stream()
        except Exception as e:
            logger.error("OnlineStream 创建失败: %s", e)
            self._asr_stream = None

    def _decode_streaming(self, samples, finalize: bool = False) -> str:
        """流式增量解码：把当前 samples 喂入 OnlineStream 并返回最新文本。

        Parameters
        ----------
        samples : np.ndarray
            本帧音频（float32, 16kHz）。
        finalize : bool
            True 表示这是最后一段（应触发 endpoint 收尾），False 表示中间增量。
        """
        if self._asr is None or self._asr_stream is None or np is None:
            return ""
        if len(samples) == 0:
            return ""

        try:
            self._asr_stream.accept_waveform(self.sample_rate, samples.tolist())
            while self._asr.is_ready(self._asr_stream):
                self._asr.decode_stream(self._asr_stream)

            # sherpa_onnx 1.13.2 没有 input_finished；endpoint 触发后 get_result
            # 已能返回完整文本，finalize 时无需额外操作
            result = self._asr.get_result(self._asr_stream)
            if result is None:
                return ""
            return str(result).strip()
        except Exception as e:
            logger.error("ASR 流式解码异常: %s", e)
            return ""

    def _result_from_text(self, text: str) -> Optional[Dict]:
        logger.info("ASR 识别结果: %s", text)
        self._finish_recording()
        if not text:
            return {"action": "timeout", "direction": None, "text": ""}
        result = self._parse_command(text)
        result["text"] = text
        return result

    def _process_recording(self, samples) -> Optional[Dict]:
        """RECORDING 状态：流式增量喂入 OnlineStream，VAD 检测语音段结束即收尾。

        相比 endpoint 方案：VAD 能更可靠地区分"说话停顿"和"说完话"，
        避免唤醒尾音或 TTS 回声静音被误判为指令结束。
        """
        if self._asr is None:
            self._finish_recording()
            return None

        try:
            samples = np.asarray(samples, dtype=np.float32)
            elapsed = time.monotonic() - self._recording_started_at

            if self._asr_stream is None:
                self._init_recording_stream()

            # 起始死区内只更新 VAD 状态，不喂 ASR（避免唤醒尾音进入识别）
            if elapsed < POST_WAKE_IGNORE_SECONDS:
                if self._vad is not None:
                    self._vad.accept_waveform(samples.tolist())
                return None

            # 增量喂 ASR
            partial = self._decode_streaming(samples, finalize=False)
            if partial:
                logger.debug("ASR 流式: %s", partial)

            # VAD 检测语音段结束（说完话静音 0.6s 触发）
            if self._vad is not None:
                self._vad.accept_waveform(samples.tolist())
                if not self._vad.empty():
                    segment = self._vad.front
                    self._vad.pop()
                    # VAD 切出完整段：直接用最近一次 partial（增量识别已包含完整文本）
                    # 不能再调 _decode_streaming(segment)——ASR stream 是累积式，
                    # 重复 accept_waveform 会导致 get_result 返回重复文本
                    speech_samples = np.array(segment.samples, dtype=np.float32)
                    if len(speech_samples) >= self.sample_rate * MIN_COMMAND_SECONDS:
                        logger.info("VAD 触发收尾: %s", partial)
                        return self._result_from_text(partial)

            # 兜底超时
            if elapsed - POST_WAKE_IGNORE_SECONDS >= MAX_COMMAND_SECONDS:
                final_text = self._decode_streaming(samples, finalize=True)
                self._finish_recording()
                if final_text:
                    return self._result_from_text(final_text)
                return {"action": "timeout", "direction": None, "text": ""}

        except Exception as e:
            logger.error("ASR 处理异常: %s", e)
            self._finish_recording()

        return None

    _ASR_FIXES = {
        "方方": "前方",
        "千方": "前方",
        "钱方": "前方",
        "前方有": "前方",
        "方有": "前方有",
        "方前": "前方",
        "左方": "左前方",
        "右方": "右前方",
        "后方有": "后方",
    }

    def _parse_command(self, text: str) -> Dict:
        """解析 ASR 文本为结构化指令。"""
        normalized = "".join(text.split())
        for mark in "，。！？,.!?":
            normalized = normalized.replace(mark, "")

        # 修正 ASR 常见误识别
        for wrong, correct in self._ASR_FIXES.items():
            if wrong in normalized:
                normalized = normalized.replace(wrong, correct)

        direction = None
        action = None

        # 先匹配更长的方位短语，避免"左前方"被"前方"提前截获。
        direction_terms = []
        for dir_name, keywords in DIRECTION_KEYWORDS.items():
            for kw in keywords:
                direction_terms.append((kw, dir_name))
        for kw, dir_name in sorted(direction_terms, key=lambda item: len(item[0]), reverse=True):
            if kw in normalized:
                direction = dir_name
                break

        for act_name in _ACTION_PRIORITY:
            terms = sorted(ACTION_KEYWORDS.get(act_name, []), key=len, reverse=True)
            if any(kw in normalized for kw in terms):
                action = act_name
                break

        if action is None and direction is None:
            return {"action": "unrecognized", "direction": None}

        if action is None:
            action = "query"

        # 有动作关键词但没有方位 → 听不清方位，报 unrecognized
        if action == "query" and direction is None:
            return {"action": "unrecognized", "direction": None}

        return {"action": action, "direction": direction}
