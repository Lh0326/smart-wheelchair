"""TTS synthesis engine using Kokoro via sherpa-onnx.

DK-2500 异构调度:
  TTS 推理: iGPU (OpenVINO GPU provider) 加速声码器推理
  播放控制: CPU（音频输出）

音频后处理:
  - 低通滤波: 消除 INT8 量化引入的高频噪声
  - 高质量重采样: scipy spline 插值替代 sd.play 内置重采样
  - 淡入淡出: 消除首尾爆音
"""

import threading
from queue import Queue, Empty

import numpy as np
import sounddevice as sd
import sherpa_onnx

from config import TTS_MODEL_DIR, TTS_SPEAKER_ID, TTS_THREADS, TTS_DEVICE

# Output device native sample rate
_OUTPUT_SR = int(sd.query_devices(sd.default.device["output"])["default_samplerate"])


def _postprocess_audio(audio, src_sr, dst_sr):
    """Post-process TTS audio: lowpass filter → resample → fade-in/out → normalize."""
    # 1. Lowpass filter: remove INT8 quantization noise above 8kHz
    cutoff = min(8000.0, src_sr / 2 - 500)
    if cutoff < src_sr / 2:
        try:
            from scipy.signal import butter, sosfilt
            sos = butter(5, cutoff / (src_sr / 2), btype="low", output="sos")
            audio = sosfilt(sos, audio).astype(np.float32)
        except ImportError:
            pass

    # 2. High-quality resample to output device rate
    if dst_sr != src_sr:
        try:
            from scipy.signal import resample_poly
            from math import gcd
            g = gcd(dst_sr, src_sr)
            audio = resample_poly(audio, dst_sr // g, src_sr // g).astype(np.float32)
        except ImportError:
            pass

    # 3. Fade-in / fade-out (5ms) to eliminate clicks at boundaries
    fade_samples = min(int(dst_sr * 0.005), len(audio) // 4)
    if fade_samples > 1:
        fade_in = np.linspace(0, 1, fade_samples, dtype=np.float32)
        fade_out = np.linspace(1, 0, fade_samples, dtype=np.float32)
        audio[:fade_samples] *= fade_in
        audio[-fade_samples:] *= fade_out

    # 4. Soft-limit to prevent any clipping at output
    peak = np.max(np.abs(audio))
    if peak > 0.9:
        audio = audio * (0.9 / peak)

    return audio


class TTSEngine:
    def __init__(self, speaker_id=TTS_SPEAKER_ID):
        tts_dir = str(TTS_MODEL_DIR)

        # DK-2500: 使用 iGPU 加速 TTS 推理
        print(f"[TTS] Loading model (device={TTS_DEVICE})...")
        kokoro_config = sherpa_onnx.OfflineTtsKokoroModelConfig(
            model=f"{tts_dir}/model.onnx",
            voices=f"{tts_dir}/voices.bin",
            tokens=f"{tts_dir}/tokens.txt",
            lexicon=f"{tts_dir}/lexicon-us-en.txt,{tts_dir}/lexicon-zh.txt",
            data_dir=f"{tts_dir}/espeak-ng-data",
            length_scale=1.0,
            lang="cmn",
        )
        model_config = sherpa_onnx.OfflineTtsModelConfig(
            kokoro=kokoro_config,
            num_threads=TTS_THREADS,
            provider=TTS_DEVICE,
        )
        rule_fsts = ",".join([
            f"{tts_dir}/date-zh.fst",
            f"{tts_dir}/phone-zh.fst",
            f"{tts_dir}/number-zh.fst",
        ])
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=model_config,
            rule_fsts=rule_fsts,
        )

        self.tts = sherpa_onnx.OfflineTts(tts_config)
        self.speaker_id = speaker_id
        self.queue = Queue()
        self._stop_event = threading.Event()
        self._speaking = False
        self._generating = False
        self._output_stream = None
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        print("[TTS] Ready.")

    def _worker(self):
        while True:
            try:
                item = self.queue.get(timeout=0.1)
            except Empty:
                continue
            if item is None:
                break
            if self._stop_event.is_set():
                continue

            # Task format: ("generate", text, speaker_id, speed)
            if isinstance(item, tuple) and len(item) == 4 and item[0] == "generate":
                _, text, sid, speed = item
                self._generating = True
                result = self.tts.generate(text, sid=sid, speed=speed)
                self._generating = False
                if self._stop_event.is_set() or not result.samples:
                    continue
                audio = np.array(result.samples, dtype=np.float32)
                audio = _postprocess_audio(audio, result.sample_rate, _OUTPUT_SR)
            elif isinstance(item, tuple) and len(item) == 2:
                # Legacy: pre-generated (audio, sr)
                audio, sr = item
            else:
                continue

            self._speaking = True
            try:
                self._output_stream = sd.OutputStream(
                    samplerate=_OUTPUT_SR, channels=1, dtype="float32",
                )
                self._output_stream.start()
                chunk_size = _OUTPUT_SR // 10  # 100ms chunks
                for i in range(0, len(audio), chunk_size):
                    if self._stop_event.is_set():
                        break
                    chunk = audio[i:i + chunk_size]
                    self._output_stream.write(chunk.reshape(-1, 1))
            except Exception:
                pass
            finally:
                if self._output_stream is not None:
                    try:
                        self._output_stream.stop()
                        self._output_stream.close()
                    except Exception:
                        pass
                    self._output_stream = None
            self._speaking = False

    def speak(self, text, speed=1.0, speaker_id=None):
        """提交语音生成任务（非阻塞）。generate 在 worker 线程执行，不卡调用者。"""
        if not text.strip():
            return
        sid = speaker_id if speaker_id is not None else self.speaker_id
        self._stop_event.clear()
        # Discard pending tasks so the latest command takes priority
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except Empty:
                break
        self.queue.put(("generate", text, sid, speed))

    def stop_speaking(self):
        """立即停止当前 TTS 播放（用于紧急停止打断）。"""
        self._stop_event.set()
        if self._output_stream is not None:
            try:
                self._output_stream.stop()
            except Exception:
                pass
        # 清空队列中待播放的音频
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except Empty:
                break
        self._speaking = False

    def is_speaking(self):
        return self._speaking

    def stop(self):
        self.stop_speaking()
        self.queue.put(None)
