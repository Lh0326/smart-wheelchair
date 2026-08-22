"""Voice pipeline: Two-stage cascade (KWS + Offline ASR).

DK-2500 异构调度:
  KWS: CPU（3.3M 参数轻量模型，~3% CPU，NPU 调度开销反而更大）
  ASR: OpenVINO iGPU 推理（Paraformer/Zipformer，INT8 量化）

ASR 加载策略:
  DK-2500 模式 (ASR_ALWAYS_LOADED=True):
    16GB 内存足够，所有模型常驻，省去加载延迟
  Windows MVP 模式 (ASR_ALWAYS_LOADED=False):
    ASR 按需加载/卸载，节省内存

Architecture:
  IDLE: KWS only → wake detected → ensure ASR loaded
  RECORDING: record 4s → ASR recognize → execute command → (optional) unload ASR → IDLE
"""

import enum
import gc
import numpy as np
import sherpa_onnx
from pathlib import Path

from config import (
    KWS_MODEL_DIR, ASR_OFFLINE_DIR, VAD_MODEL, SAMPLE_RATE,
    COMMAND_MAP, TTS_FEEDBACK, ASR_THREADS, ASR_DEVICE,
    ASR_ALWAYS_LOADED, KEYWORDS_FILE,
)

WAKE_WORDS = ["小智你好", "小志你好", "小吱你好", "心语启动"]


class State(enum.Enum):
    IDLE = 0        # KWS only, low power
    RECORDING = 1   # ASR active, recording command


class VoicePipeline:
    def __init__(self, tts_engine=None):
        self.tts = tts_engine
        self.state = State.IDLE
        self.running = False

        # Stage 1: KWS (always-on on CPU, lightweight ~3% CPU)
        print("Loading KWS model (always-on, low power)...")
        kws_dir = str(KWS_MODEL_DIR)
        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=f"{kws_dir}/tokens.txt",
            encoder=f"{kws_dir}/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx",
            decoder=f"{kws_dir}/decoder-epoch-13-avg-2-chunk-16-left-64.onnx",
            joiner=f"{kws_dir}/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx",
            keywords_file=str(KEYWORDS_FILE),
            num_threads=1,
            max_active_paths=4,
            keywords_score=1.5,
            keywords_threshold=0.25,
            num_trailing_blanks=1,
            provider="cpu",
        )
        print("KWS ready (~3% CPU).")

        # Stage 2: ASR (loaded on demand or always-loaded per config)
        self._asr = None

        # DK-2500: 预加载 ASR（16GB 内存足够常驻）
        if ASR_ALWAYS_LOADED:
            self.load_asr()
            print("[ASR] Always-loaded mode (DK-2500, 16GB DDR5)")

    def load_asr(self):
        """Load ASR model. DK-2500 uses OpenVINO GPU backend for acceleration."""
        if self._asr is not None:
            return
        print(f"[ASR] Loading offline ASR model (device={ASR_DEVICE})...")
        self._asr = sherpa_onnx.OfflineRecognizer.from_zipformer_ctc(
            model=str(ASR_OFFLINE_DIR / "model.int8.onnx"),
            tokens=str(ASR_OFFLINE_DIR / "tokens.txt"),
            num_threads=ASR_THREADS,
            provider=ASR_DEVICE,
            sample_rate=SAMPLE_RATE,
            feature_dim=80,
        )
        print("[ASR] Ready.")

    def unload_asr(self):
        """Unload ASR model, free memory. (Only in Windows MVP mode)"""
        if ASR_ALWAYS_LOADED:
            return  # DK-2500: keep ASR resident
        if self._asr is not None:
            self._asr = None
            gc.collect()
            print("[ASR] Unloaded, memory freed.")

    def recognize(self, samples):
        """Recognize audio with offline ASR. New stream per call, zero buffer accumulation."""
        if self._asr is None:
            self.load_asr()
        stream = self._asr.create_stream()
        stream.accept_waveform(SAMPLE_RATE, samples)
        self._asr.decode_stream(stream)
        text = stream.result.text.strip()
        del stream
        return text

    def _match_command(self, text):
        text = text.strip()
        for keyword in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
            if keyword in text:
                return COMMAND_MAP[keyword], keyword
        # Fuzzy: check first 2 chars
        for keyword in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
            if len(keyword) >= 3 and keyword[:2] in text:
                return COMMAND_MAP[keyword], keyword
        return None, None

    def _speak(self, feedback_key, sid=None, **kwargs):
        if self.tts:
            text = TTS_FEEDBACK.get(feedback_key, "")
            if text:
                self.tts.speak(text.format(**kwargs), speaker_id=sid)

    def stop(self):
        self.running = False
        if not ASR_ALWAYS_LOADED:
            self.unload_asr()
