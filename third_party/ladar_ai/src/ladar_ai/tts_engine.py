"""TTS 引擎：基于 sherpa-onnx Kokoro 模型的文本转语音。
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


完全复制 ai-model 的 TTSEngine，唯一改动是路径硬编码。

音频后处理:
  - 低通滤波: 消除 INT8 量化引入的高频噪声
  - 高质量重采样: scipy spline 插值替代 sd.play 内置重采样
  - 淡入淡出: 消除首尾爆音
"""
import os
import time
import logging
import threading
from queue import Queue, Empty
from typing import Optional

try:
    import sherpa_onnx
except ImportError:
    sherpa_onnx = None

try:
    import sounddevice as sd
except ImportError:
    sd = None

try:
    import numpy as np
except ImportError:
    np = None

logger = logging.getLogger(__name__)

from ladar_ai.tts_segment import segment_text

# 模型路径（与 ai-model 一致）
TTS_DIR = "" + _MODELS_ROOT + "/voice/tts/kokoro-int8-multi-lang-v1_1"

# 默认 TTS 参数
_TTS_SPEAKER_ID = 45
_TTS_THREADS = 2
_TTS_PROVIDER = "cpu"
_TTS_SPEED = 1.0

# 固定播报短语，启动时预生成音频实现零延迟播放
_PRECACHE_PHRASES = [
    "前方红灯，请停车",
    "前方绿灯，可以通行",
    "前方黄灯，请注意",
    "我在",
    "没听清，请再说",
    "已紧急停车",
    "已加速",
    "已减速",
    "系统启动中",
]

# 输出采样率：尝试获取设备原生采样率，失败则回退 44100
try:
    if sd is not None:
        _OUTPUT_SR = int(sd.query_devices(sd.default.device["output"])["default_samplerate"])
    else:
        _OUTPUT_SR = 44100
except Exception:
    _OUTPUT_SR = 44100


def _postprocess_audio(audio, src_sr, dst_sr, position="middle"):
    """Post-process TTS audio: lowpass filter -> resample -> fade-in/out -> normalize.

    Parameters
    ----------
    position : str
        段位置标识，控制 fade 行为以避免段间顿挫：
        - "first"：只做 fade-in（开头淡入，消除段首爆音）
        - "last"：只做 fade-out（结尾淡出，消除段尾爆音）
        - "middle"：不做 fade（中段保持原样，避免段间衰减导致的"一顿一顿"）
        - "single"：fade-in + fade-out（整句单段时使用）
    """
    if np is None:
        return audio

    # 1. Lowpass filter: cutoff 提高到 12kHz（保留更多人声高频细节，减少"闷"感）
    #    INT8 量化噪声主要在 14kHz+，12kHz 截止仍能有效抑制
    cutoff = min(12000.0, src_sr / 2 - 500)
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

    # 3. Fade-in / fade-out：根据段位置应用，避免段间衰减导致的顿挫
    fade_samples = min(int(dst_sr * 0.012), len(audio) // 4)
    if fade_samples > 1:
        if position in ("first", "single"):
            fade_in = np.linspace(0, 1, fade_samples, dtype=np.float32)
            audio[:fade_samples] *= fade_in
        if position in ("last", "single"):
            fade_out = np.linspace(1, 0, fade_samples, dtype=np.float32)
            audio[-fade_samples:] *= fade_out

    # 4. Soft-limit to prevent any clipping at output
    peak = np.max(np.abs(audio))
    if peak > 0.9:
        audio = audio * (0.9 / peak)

    return audio


class TTSEngine:
    """Kokoro TTS 引擎，优先级队列 + 后台播放线程。

    完全复制自 ai-model 的 TTSEngine，使用硬编码路径。
    """

    def __init__(self, model_dir: str = "", speaker_id: int = _TTS_SPEAKER_ID,
                 on_speak_complete=None):
        """
        Parameters
        ----------
        model_dir : str
            TTS 模型所在目录（已弃用，保留接口兼容，实际使用硬编码路径）。
        speaker_id : int
            Kokoro 语音 ID。
        on_speak_complete : callable, optional
            播报完成后的回调函数。
        """
        self._queue: Queue = Queue()
        self._stop_event = threading.Event()
        self._speaking = False
        self._generating = False
        self._output_stream = None
        self._tts = None
        self.speaker_id = speaker_id
        self._on_speak_complete = on_speak_complete

        # 流式管线相关默认值（提前初始化，保证加载失败时实例仍处于合法状态）
        self._piper_engine = None
        self._stream_queue: Queue = Queue(maxsize=4)
        self._stream_chunks: list = []
        self._stream_worker = None
        self._stream_player_thread = None

        if sherpa_onnx is None:
            logger.error("sherpa_onnx 未安装，TTS 不可用")
            return

        tts_dir = TTS_DIR

        if not os.path.isfile(f"{tts_dir}/model.onnx"):
            logger.error("TTS 模型文件不存在: %s/model.onnx", tts_dir)
            return

        try:
            logger.info("Loading TTS model (device=%s)...", _TTS_PROVIDER)
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
                num_threads=_TTS_THREADS,
                provider=_TTS_PROVIDER,
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
            self._tts = sherpa_onnx.OfflineTts(tts_config)
            logger.info("TTS 模型加载成功")
        except Exception as e:
            logger.error("TTS 模型加载失败: %s", e)
            return

        # 预缓存固定短语（后台异步，不阻塞节点启动）
        self._cache: dict = {}
        self._cache_lock = threading.Lock()
        # 预缓存暂时禁用：threads=2 下生成慢，会阻塞 worker 导致播报延迟
        # if self._tts is not None:
        #     t = threading.Thread(target=self._precache, daemon=True)
        #     t.start()

        # Piper 引擎（可选，作为主引擎优先使用）
        # 注：_piper_engine 默认值 None 已在 __init__ 开头初始化
        piper_dir = "" + _MODELS_ROOT + "/voice/tts/piper-zh_CN-huayan-medium"
        if os.path.isdir(piper_dir):
            try:
                from ladar_ai.tts_engine_piper import PiperTTSEngine
                self._piper_engine = PiperTTSEngine(piper_dir)
                if self._piper_engine.is_loaded():
                    logger.info("Piper 主引擎已加载")
                else:
                    self._piper_engine = None
                    logger.warning("Piper 加载失败，回退 Kokoro")
            except Exception as e:
                logger.warning("Piper 初始化异常: %s，使用 Kokoro", e)
                self._piper_engine = None

        # 注：_stream_queue / _stream_chunks / _stream_worker / _stream_player_thread
        # 默认值已在 __init__ 开头初始化

        # 注：旧的 _worker 线程已被 _segment_and_stream + _stream_player 替代，不再启动。
        # _worker 方法保留作为参考实现，不调用。
        logger.info("TTS engine ready.")

    def _precache(self):
        """后台线程：预生成固定短语音频缓存。在 worker 空闲时逐条生成。"""
        logger.info("Precaching static phrases (background)...")
        for phrase in _PRECACHE_PHRASES:
            if self._stop_event.is_set():
                break
            # 等待 worker 空闲再预缓存
            while self._speaking or self._generating:
                if self._stop_event.is_set():
                    return
                time.sleep(0.2)
            t0 = time.time()
            with self._cache_lock:
                result = self._tts.generate(phrase, sid=self.speaker_id, speed=_TTS_SPEED)
            if result.samples:
                audio = np.array(result.samples, dtype=np.float32)
                audio = _postprocess_audio(audio, result.sample_rate, _OUTPUT_SR)
                self._cache[phrase] = audio
                ms = (time.time() - t0) * 1000
                logger.info("  cached \"%s...\" (%.0fms)", phrase[:12], ms)
        logger.info("TTS cache ready: %d phrases", len(self._cache))

    def _select_synthesize_fn(self):
        """返回当前使用的合成函数：优先 Piper（实时响应 ~130ms），回退 Kokoro（音质更好但慢 1-3s）。

        实时响应优先于音质——载人轮椅场景下"我在"等短反馈需要 <300ms 才不显迟钝。
        Piper 用 VITS zh_CN-huayan-medium 模型，本地 CPU 合成快 5-10 倍。
        """
        if self._piper_engine is not None and self._piper_engine.is_loaded():
            return self._piper_engine.generate
        if self._tts is not None:
            def kokoro_synthesize(text):
                result = self._tts.generate(text, sid=self.speaker_id, speed=_TTS_SPEED)
                audio = np.array(result.samples, dtype=np.float32)
                return audio, result.sample_rate
            return kokoro_synthesize
        # 无可用引擎
        return lambda text: (np.array([], dtype=np.float32), _OUTPUT_SR)

    def _worker(self):
        """后台线程：从队列取文本 -> 生成音频 -> 播放。"""
        while True:
            try:
                item = self._queue.get(timeout=0.1)
            except Empty:
                continue

            if item is None:
                break
            if self._stop_event.is_set():
                continue

            # Task format: ("generate", text, speaker_id, speed)
            if isinstance(item, tuple) and len(item) == 4 and item[0] == "generate":
                _, text, sid, speed = item
                with self._cache_lock:
                    audio = self._cache.get(text)
                if audio is not None:
                    pass  # 缓存命中，直接播放
                else:
                    self._generating = True
                    with self._cache_lock:
                        result = self._tts.generate(text, sid=sid, speed=speed)
                    self._generating = False
                    if self._stop_event.is_set() or not result.samples:
                        continue
                    audio = np.array(result.samples, dtype=np.float32)
                    audio = _postprocess_audio(audio, result.sample_rate, _OUTPUT_SR)
            elif isinstance(item, tuple) and len(item) == 2:
                # Legacy: (text, priority) format from old API
                text, priority = item
                if not text or self._tts is None:
                    continue
                with self._cache_lock:
                    audio = self._cache.get(text)
                if audio is None:
                    self._generating = True
                    with self._cache_lock:
                        audio_result = self._tts.generate(text, sid=self.speaker_id, speed=_TTS_SPEED)
                    self._generating = False
                    if self._stop_event.is_set() or not audio_result.samples:
                        continue
                    audio = np.array(audio_result.samples, dtype=np.float32)
                    audio = _postprocess_audio(audio, audio_result.sample_rate, _OUTPUT_SR)
            else:
                continue

            if audio is None:
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
            if self._on_speak_complete:
                try:
                    self._on_speak_complete()
                except Exception:
                    pass

    @property
    def speaking(self) -> bool:
        return self._speaking

    def speak(self, text: str, priority: int = 0, speed: float = 1.0,
              speaker_id: Optional[int] = None) -> None:
        """将文本放入队列等待播报。

        Parameters
        ----------
        text : str
            要播报的文本。
        priority : int
            优先级，数值越小越优先（0=普通，1=紧急用负数或更小值）。
        speed : float
            语速倍率。
        speaker_id : int, optional
            语音 ID，不指定则使用默认。
        """
        if not text or not text.strip() or self._tts is None:
            return

        # 先停掉旧一轮，避免双 player 竞争 _stream_queue / _output_stream
        self.stop_speaking()
        self._wait_player_exit()
        self._stop_event.clear()

        # 清流式队列残留
        while not self._stream_queue.empty():
            try:
                self._stream_queue.get_nowait()
            except Empty:
                break

        # 句级流式：切分文本并启动合成/播放双线程
        self._segment_and_stream(text)
        logger.debug("TTS 流式启动: priority=%d text=%s", priority, text[:30])

    def stop_speaking(self) -> None:
        """停止当前播放并清空队列。"""
        self._stop_event.set()
        if self._output_stream is not None:
            try:
                self._output_stream.stop()
            except Exception:
                pass
        # 清空旧任务队列（_worker 已退役，但保留清理以避免残留）
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except Empty:
                break
        # 清空流式队列，避免下一次 speak() 播到上一轮残留音频
        while not self._stream_queue.empty():
            try:
                self._stream_queue.get_nowait()
            except Empty:
                break
        self._speaking = False

    def _wait_player_exit(self, timeout: float = 2.0):
        """等待当前 player 线程退出。speak/stop_speaking 重入时调用。"""
        t = getattr(self, "_stream_player_thread", None)
        if t is not None and t.is_alive():
            t.join(timeout=timeout)

    def _segment_and_stream(self, text: str) -> None:
        """句级流式合成：切分文本 -> 启动合成线程 -> 启动播放线程。

        调用者应在 stop_event 已 clear 后调用。本方法立即返回，
        合成与播放都在后台线程进行。
        """
        segments = segment_text(text)
        if not segments:
            return

        synthesize_fn = self._select_synthesize_fn()

        # 重置共享状态
        self._stream_chunks = []

        # 启动合成 worker（producer）
        self._stream_worker = StreamingTTSWorker(
            synthesize_fn=synthesize_fn,
            output_queue=self._stream_queue,
            output_chunks=self._stream_chunks,
            postprocess_fn=_postprocess_audio,
            output_sample_rate=_OUTPUT_SR,
            stop_event=self._stop_event,
        )
        self._stream_worker.process_request(segments)

        # 启动播放线程（consumer）：边收边播
        self._stream_player_thread = threading.Thread(
            target=self._stream_player, daemon=True
        )
        self._stream_player_thread.start()

    def _stream_player(self) -> None:
        """播放线程：从 _stream_queue 取音频写入 OutputStream。

        第一段到达即播；播放结束后关闭 stream 并发布完成回调。
        """
        self._speaking = True
        try:
            # 增大 blocksize 减小 underrun 概率（blocksize=0 让 PortAudio 自动选择）
            # latency='high' 给 ALSA 更大缓冲，避免 underrun
            self._output_stream = sd.OutputStream(
                samplerate=_OUTPUT_SR, channels=1, dtype="float32",
                blocksize=0, latency='high',
            )
            self._output_stream.start()
            chunk_size = _OUTPUT_SR // 10  # 100ms chunks

            while not self._stop_event.is_set():
                try:
                    audio = self._stream_queue.get(timeout=0.1)
                except Empty:
                    # worker 已退出且 queue 可能还有残留，排空后退出
                    if (self._stream_worker is None
                            or not self._stream_worker.is_running()):
                        self._drain_remaining(chunk_size)
                        break
                    continue

                if self._stop_event.is_set():
                    break
                self._write_audio(self._output_stream, audio, chunk_size)
        except Exception as e:
            logger.error("stream player 异常: %s", e)
        finally:
            if self._output_stream is not None:
                try:
                    self._output_stream.stop()
                    self._output_stream.close()
                except Exception:
                    pass
                self._output_stream = None
            self._speaking = False
            if self._on_speak_complete:
                try:
                    self._on_speak_complete()
                except Exception:
                    pass

    def _drain_remaining(self, chunk_size: int) -> None:
        """worker 已退出，把 _stream_queue 里剩余的音频全部播掉。

        每段之间检查 stop_event，确保中断响应。
        """
        while not self._stop_event.is_set():
            try:
                audio = self._stream_queue.get_nowait()
            except Empty:
                return
            self._write_audio(self._output_stream, audio, chunk_size)

    def _write_audio(self, stream, audio: np.ndarray, chunk_size: int) -> None:
        """把单段音频按 chunk 写入 OutputStream，期间检查 stop_event。"""
        for i in range(0, len(audio), chunk_size):
            if self._stop_event.is_set():
                break
            chunk = audio[i:i + chunk_size]
            stream.write(chunk.reshape(-1, 1))

    def stop(self):
        """完全停止 TTS 引擎。"""
        self.stop_speaking()
        self._queue.put(None)


class StreamingTTSWorker:
    """句级流式合成 + 播放协调器。

    接收一段切好的 segments，启动后台合成线程把每段合成的音频推入 output_queue。
    播放线程（消费者）从 queue 取音频播放，实现首段合成完即播、后续段并行合成。
    """

    def __init__(self, synthesize_fn, output_queue, output_chunks,
                 postprocess_fn, output_sample_rate, stop_event):
        """
        Parameters
        ----------
        synthesize_fn : callable(str) -> (np.ndarray, int)
            单段文本 -> (audio_float32, sample_rate)。
        output_queue : queue.Queue
            播放线程消费的音频队列（共享给 worker 与 player）。
        output_chunks : list
            共享列表，worker 把每段音频 append 进去供测试/调试观察。
        postprocess_fn : callable(np.ndarray, int, int) -> np.ndarray
            (audio, src_sr, dst_sr) -> processed_audio。
        output_sample_rate : int
            输出设备采样率。
        stop_event : threading.Event
            全局停止事件。
        """
        self._synthesize = synthesize_fn
        self._output_queue = output_queue
        self._output_chunks = output_chunks
        self._postprocess = postprocess_fn
        self._dst_sr = output_sample_rate
        self._stop_event = stop_event
        self._thread = None
        self._completion_event = threading.Event()

    def process_request(self, segments: list) -> None:
        """启动后台线程处理一段 TTSRequest 的所有 segments。"""
        if self._thread is not None and self._thread.is_alive():
            return  # 已有处理在跑

        self._completion_event.clear()
        self._thread = threading.Thread(
            target=self._run, args=(list(segments),), daemon=True
        )
        self._thread.start()

    def _run(self, segments: list) -> None:
        # 过滤空段后得到有效段列表，用于判断每段的 position（first/middle/last/single）
        valid_segments = [s for s in segments if s and s.strip()]
        total = len(valid_segments)
        if total == 0:
            self._completion_event.set()
            return

        try:
            idx = 0
            for seg in segments:
                if self._stop_event.is_set():
                    return
                if not seg or not seg.strip():
                    continue
                try:
                    audio, src_sr = self._synthesize(seg)
                    if self._stop_event.is_set():
                        return
                    if audio is None or len(audio) == 0:
                        continue
                    # 段位置：单段=both，首段=fade-in，末段=fade-out，中段=无 fade
                    if total == 1:
                        position = "single"
                    elif idx == 0:
                        position = "first"
                    elif idx == total - 1:
                        position = "last"
                    else:
                        position = "middle"
                    processed = self._postprocess(audio, src_sr, self._dst_sr, position=position)
                    if self._stop_event.is_set():
                        return
                    self._output_chunks.append(processed)
                    self._output_queue.put(processed)
                    idx += 1
                except Exception as e:
                    logger.warning("段合成失败（跳过）: %r -> %s", str(seg)[:20], e)
                    continue
        except Exception as e:
            logger.error("StreamingTTSWorker 致命异常: %s", e)
        finally:
            self._completion_event.set()

    def wait_completion(self, timeout: float = 5.0) -> bool:
        """等待当前请求处理完成。返回是否在超时前完成。"""
        return self._completion_event.wait(timeout=timeout)

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()
