"""语音 ROS2 节点：麦克风采集 -> 唤醒词 + ASR -> 发布指令。"""
import os
import time
import json
import logging
import signal
import threading
from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import sounddevice as sd
except ImportError:
    sd = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    from scipy.signal import resample_poly
    from math import gcd
except ImportError:
    resample_poly = None

from ladar_ai.voice_engine import VoiceEngine, MAX_COMMAND_SECONDS, POST_WAKE_IGNORE_SECONDS

logger = logging.getLogger(__name__)

_SAMPLE_RATE = 16000
_CHUNK_SIZE = 960  # 60ms @ 16kHz

# USB 麦克风（UACDemoV1.0 / Jieli Technology）支持 48kHz/1ch
_HW_DEVICE = "hw:1,0"
_HW_SAMPLE_RATE = 48000
_HW_CHANNELS = 1

# 录音窗口总时长 = 起始死区 + 最大指令时长
_RECORDING_WINDOW_SEC = POST_WAKE_IGNORE_SECONDS + MAX_COMMAND_SECONDS

# processing 状态超时：query/stop 等指令处理后等待 TTS 播报的时间
# 超过此时间强制回到 idle，避免前端"识别中..."永久卡住
_PROCESSING_TIMEOUT_SEC = 8.0


class VoiceNode(Node):
    """语音交互节点：采集音频，检测唤醒词和指令，发布结果。"""

    def __init__(self):
        super().__init__("voice_node")

        # 推导模型目录
        pkg_dir = os.path.dirname(os.path.dirname(__file__))
        model_dir = os.path.join(pkg_dir, "..", "models", "voice")
        model_dir = os.path.abspath(model_dir)

        # 初始化语音引擎
        self._engine = VoiceEngine(model_dir, sample_rate=_SAMPLE_RATE)

        # 发布器
        self._pub_command = self.create_publisher(String, "/voice_command", 10)
        self._pub_state = self.create_publisher(String, "/voice_state", 10)

        # 录音状态：idle / listening / processing
        self._voice_state = "idle"
        self._recording_deadline = 0.0  # Unix 时间戳，0 表示无活跃录音
        self._processing_deadline = 0.0  # processing 状态截止时间
        self._state_lock = threading.Lock()

        # 10Hz 定时器持续发布 voice_state（让前端持续刷新倒计时）
        self._state_timer = self.create_timer(0.1, self._publish_voice_state)

        # 音频采集队列
        self._audio_queue: Queue = Queue()

        # 启动音频采集
        self._running = True
        self._stream = None

        if sd is not None:
            self._stream = self._open_mic()
        else:
            self.get_logger().error("sounddevice 未安装，语音采集不可用")

        # 启动处理线程
        self._process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._process_thread.start()

        self.get_logger().info(f"VoiceNode started, model_dir={model_dir}")

    def _open_mic(self):
        """尝试打开板载声卡（ALSA 直连），失败则回退 PulseAudio 默认。"""
        # 优先使用指定 ALSA 设备，允许通过环境变量适配声卡编号变化。
        for device in [os.environ.get("LADAR_AI_MIC_DEVICE", ""), _HW_DEVICE, "plughw:1,0"]:
            if not device:
                continue
            try:
                stream = sd.InputStream(
                    device=device,
                    samplerate=_HW_SAMPLE_RATE,
                    channels=_HW_CHANNELS,
                    dtype="float32",
                    blocksize=4800,
                    callback=self._audio_callback_hw,
                )
                stream.start()
                self.get_logger().info(
                    f"麦克风采集已启动 (ALSA {device}, {_HW_SAMPLE_RATE} Hz, {_HW_CHANNELS} ch)"
                )
                return stream
            except Exception as e:
                self.get_logger().warn(f"ALSA {device} 打开失败: {e}")

        # 回退 PulseAudio 默认设备
        try:
            stream = sd.InputStream(
                samplerate=_SAMPLE_RATE,
                channels=1,
                dtype="float32",
                blocksize=_CHUNK_SIZE,
                callback=self._audio_callback,
            )
            stream.start()
            self.get_logger().info(f"麦克风采集已启动 (PulseAudio, {_SAMPLE_RATE} Hz)")
            return stream
        except Exception as e:
            self.get_logger().error(f"麦克风启动失败: {e}")
            return None

    def _audio_callback_hw(self, indata, frames, time_info, status):
        """板载声卡回调：最小化处理，只拷贝左声道原始数据入队。"""
        if status:
            logger.warning("音频采集状态: %s", status)
        self._audio_queue.put(("hw", indata[:, 0].copy()))

    def _audio_callback(self, indata, frames, time_info, status):
        """PulseAudio 回调：直接传入。"""
        if status:
            logger.warning("音频采集状态: %s", status)
        self._audio_queue.put(("pa", indata[:, 0].copy()))

    def _process_loop(self) -> None:
        """音频处理线程：从队列取数据 -> 降采样 -> 送入 VoiceEngine。"""
        while self._running:
            try:
                item = self._audio_queue.get(timeout=0.5)
            except Exception:
                continue

            if np is None:
                continue

            src, data = item
            if src == "hw":
                # 板载声卡 48kHz -> 16kHz 降采样（在处理线程中完成，避免阻塞音频回调）
                if resample_poly is not None:
                    g = gcd(_SAMPLE_RATE, _HW_SAMPLE_RATE)
                    samples = resample_poly(data, _SAMPLE_RATE // g, _HW_SAMPLE_RATE // g).astype(np.float32)
                else:
                    samples = data[::_HW_SAMPLE_RATE // _SAMPLE_RATE]
            else:
                samples = data

            samples = np.asarray(samples, dtype=np.float32)
            samples = np.nan_to_num(samples, copy=False)
            peak = float(np.max(np.abs(samples))) if samples.size else 0.0
            if peak > 1.0:
                samples = samples / peak
            samples = np.clip(samples, -1.0, 1.0)

            result = self._engine.process_audio(samples)
            if result is not None:
                self._publish_result(result)

    def _publish_result(self, result: dict) -> None:
        """发布语音识别结果，并同步更新录音状态供前端倒计时。"""
        action = result.get("action", "")
        with self._state_lock:
            if action == "wakeup":
                # 唤醒：进入 listening，计算录音截止时间
                self._voice_state = "listening"
                self._recording_deadline = time.time() + _RECORDING_WINDOW_SEC
            elif action in ("query", "stop", "emergency_stop", "speed_up", "speed_down"):
                # 收到有效指令：进入 processing（fusion 处理 + TTS 播报）
                self._voice_state = "processing"
                self._recording_deadline = 0.0
                self._processing_deadline = time.time() + _PROCESSING_TIMEOUT_SEC
            else:
                # timeout / unrecognized / 其他：回到 idle
                self._voice_state = "idle"
                self._recording_deadline = 0.0

        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self._pub_command.publish(msg)
        action = result.get("action", "")
        if action in ("wakeup", "query", "stop") or action.startswith("speed"):
            self.get_logger().info(f"语音指令: {action} {result.get('text', '')}")
        elif action == "unrecognized":
            self.get_logger().info(f"未识别指令: {result.get('text', '')!r}")
        else:
            self.get_logger().debug(f"语音事件: {msg.data}")

        # 立即推一次状态（不等下一个 10Hz tick）
        self._publish_voice_state()

    def _publish_voice_state(self) -> None:
        """10Hz 发布当前录音状态 + 截止时间，供前端倒计时显示。

        超过 deadline 自动回到 idle（防止前端死等）。
        """
        with self._state_lock:
            now = time.time()
            if self._voice_state == "listening" and now > self._recording_deadline:
                self._voice_state = "idle"
                self._recording_deadline = 0.0
            elif self._voice_state == "processing" and now > self._processing_deadline:
                # processing 超时（TTS 应已播完）：回到 idle
                self._voice_state = "idle"
                self._processing_deadline = 0.0
            state = self._voice_state
            deadline = self._recording_deadline

        payload = {
            "state": state,
            "deadline": deadline,
            "remaining": max(0.0, deadline - now) if deadline else 0.0,
            "window_sec": _RECORDING_WINDOW_SEC,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_state.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
