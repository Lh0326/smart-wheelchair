"""ROS2 voice_node: 严格参照 ai-model main_web.py 的 _voice_thread 重写。
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


关键设计(照搬 main_web.py line 634-770):
1. 麦克风用设备原生采样率录音(int16) + 手动重采样到 16kHz
   → 避免 sounddevice 自动重采样的 artifacts(原版用 float32 直录 16k 失败)
2. 0.1s 块(1600 samples @ 16kHz) → KWS 标准块大小
3. KWS 循环:accept_waveform → while is_ready: decode_stream → get_result
4. 固定录音时长(CMD_SKIP=1s + CMD_RECORD=4s),不使用 VAD
5. boost normalize 到 15000 peak(参照 main_web.py)
6. KWS reset_stream 唤醒后清空 buffer

发布 topic:
  /voice_state   std_msgs/String  JSON: {state, remaining}
  /voice_command std_msgs/String  JSON: {action, text}
"""
import json
import sys
import threading
import time

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.node import Node
from std_msgs.msg import String

# 加入 ai-model + ladar-ai 路径
AI_MODEL_SRC = _MODELS_ROOT + '/src'
LADAR_AI_SRC = _MODELS_ROOT + '/src'
for p in (AI_MODEL_SRC, LADAR_AI_SRC):
    if p not in sys.path:
        sys.path.insert(0, p)

PIPER_MODEL_DIR = _MODELS_ROOT + '/models/voice/tts/piper-zh_CN-huayan-medium'

try:
    from voice_pipeline import VoicePipeline, State
    from config import COMMAND_MAP, SAMPLE_RATE
    from ladar_ai.tts_engine_piper import PiperTTSEngine
    from rtk_msgs.msg import DetectionArray
    AI_MODEL_OK = True
    IMPORT_ERROR = ''
except ImportError as e:
    AI_MODEL_OK = False
    IMPORT_ERROR = str(e)
    VoicePipeline = None
    State = None
    COMMAND_MAP = {}
    SAMPLE_RATE = 16000
    PiperTTSEngine = None
    DetectionArray = None


# === 录音参数(照搬 main_web.py)===
CMD_SKIP_FRAMES = 10       # 1.0s 跳过唤醒尾音
CMD_RECORD_FRAMES = 60     # 6.0s 命令录音(总倒计时 1+6=7s)
CHUNK_SEC = 0.1            # 0.1s 块(标准 KWS 块)
AUDIO_BOOST_TARGET_PEAK = 15000.0
AUDIO_BOOST_MAX_GAIN = 200.0

# === VAD:说完话提前结束 ===
VAD_SILENCE_SEC = 0.8              # 连续静音秒数 → 触发
VAD_ENERGY_THRESHOLD = 0.025       # 原始 16k samples RMS 阈值
VAD_MIN_SPEECH_SEC = 0.3           # 必须先说过话 ≥ 此秒数才允许 VAD 触发
VAD_THRESHOLD_MIN = 0.018
VAD_THRESHOLD_MAX = 0.080
VAD_NOISE_MULTIPLIER = 2.0

# === TTS 反馈 ===
TTS_WAKEUP = '我在，请说命令'
TTS_CONFIRM = '收到'
TTS_UNRECOGNIZED = '抱歉，我无法回答你这个问题'

# === 命令映射 ===
ACTION_MAP = {
    'STOP': 'stop',
    'EMERGENCY_STOP': 'emergency_stop',
    'SPEED_UP': 'speed_up',
    'SPEED_DOWN': 'speed_down',
    'QUERY_BATTERY': 'query',
    'QUERY_LOCATION': 'query',
    'QUERY_AHEAD': 'query',
    'NAV_LIVING_ROOM': 'query',
    'NAV_BEDROOM': 'query',
    'NAV_HOSPITAL': 'query',
    'NAV_HOME': 'query',
    'CONFIRM': 'query',
    'CANCEL': 'query',
    'HELP': 'query',
}

# === 前方查询:YOLO 中文标签 ===
LABEL_ZH = {
    'person': '行人', 'bicycle': '自行车', 'car': '汽车',
    'motorcycle': '摩托车', 'bus': '公交车', 'truck': '卡车',
    'traffic_light': '红绿灯', 'chair': '椅子', 'couch': '沙发',
    'dining_table': '餐桌', 'stop_sign': '停止标志',
    'bench': '长椅', 'potted_plant': '盆栽',
    'cat': '猫', 'dog': '狗',
}
DETECTION_EXPIRY_SEC = 1.5
DETECTION_MIN_CONFIDENCE = 0.5
DETECTION_MAX_REPORT = 3

# 中文数字映射(Piper TTS 直接读 "0.8" 会变成"零八",需要转 "零点八")
_DIGITS_ZH = "零一二三四五六七八九"


def _float_to_zh(num: float) -> str:
    """距离数字 → 中文。

    0.8 → '零点八', 1.5 → '一点五', 2.0 → '二'。
    """
    s = f"{num:.1f}"
    int_str, _, dec_str = s.partition('.')
    int_part = int(int_str)
    dec_part = int(dec_str) if dec_str else 0

    if int_part == 0:
        result = '零'
    else:
        # 整数部分逐位转中文(2 → '二', 10 → '一零', 简化处理)
        result = ''.join(_DIGITS_ZH[int(d)] for d in str(int_part))

    if dec_part > 0:
        result += '点' + _DIGITS_ZH[dec_part]

    return result


def match_command(text: str):
    text = text.strip()
    for kw in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
        if kw in text:
            return COMMAND_MAP[kw], kw
    for kw in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
        if len(kw) >= 3 and kw[:2] in text:
            return COMMAND_MAP[kw], kw
    return None, None


def compute_adaptive_vad_threshold(noise_rms_samples) -> float:
    """根据唤醒尾音阶段的低分位噪声估计本轮静音阈值。"""
    values = [float(v) for v in noise_rms_samples if np.isfinite(v) and v >= 0.0]
    if not values:
        return VAD_ENERGY_THRESHOLD
    noise_floor = float(np.percentile(values, 20))
    return max(
        VAD_THRESHOLD_MIN,
        min(VAD_THRESHOLD_MAX, noise_floor * VAD_NOISE_MULTIPLIER),
    )


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        if not AI_MODEL_OK:
            self.get_logger().error(f'ai-model/ladar-ai 不可用: {IMPORT_ERROR}')
            return

        self._state_pub = self.create_publisher(String, '/voice_state', 10)
        self._cmd_pub = self.create_publisher(String, '/voice_command', 10)

        # YOLO 检测订阅
        self._latest_detections = None
        self._detections_stamp_sec = 0.0
        if DetectionArray is not None:
            self.create_subscription(
                DetectionArray, '/detections', self._on_detections, 10
            )

        # Piper TTS + 预缓存
        self._tts = None
        self._audio_cache = {}
        try:
            self._tts = PiperTTSEngine(PIPER_MODEL_DIR, num_threads=2)
            if self._tts.is_loaded():
                self.get_logger().info('Piper TTS 引擎加载成功')
                for text in (TTS_WAKEUP, TTS_CONFIRM, TTS_UNRECOGNIZED):
                    audio, sr = self._tts.generate(text)
                    if len(audio) > 0:
                        self._audio_cache[text] = (audio, sr)
                self.get_logger().info(f'预缓存 {len(self._audio_cache)} 段反馈')
            else:
                self._tts = None
        except Exception as e:
            self.get_logger().warn(f'Piper 失败: {e}')
            self._tts = None

        # VoicePipeline (KWS + ASR)
        self.get_logger().info('加载 VoicePipeline...')
        self._pipeline = VoicePipeline()
        self._pipeline.running = True
        self.get_logger().info('VoicePipeline ready')

        self._state = 'idle'
        self._state_remaining = 0.0
        self._lock = threading.Lock()

        # 启动 voice 主循环(参照 main_web.py _voice_thread)
        threading.Thread(target=self._voice_supervisor, daemon=True).start()

        # 10Hz 状态发布
        self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            'voice_node 启动,等待唤醒词 "小智你好" / "心语启动"...'
        )

    def _on_detections(self, msg):
        self._latest_detections = msg
        self._detections_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _voice_supervisor(self):
        """监督麦克风会话；USB 声卡掉线或重枚举后自动重连。"""
        while self._pipeline.running:
            try:
                self._voice_loop()
            except Exception as e:
                self.get_logger().error(f'语音会话异常: {e}')
            if not self._pipeline.running:
                break
            with self._lock:
                self._state = 'audio_reconnecting'
                self._state_remaining = 2.0
            self.get_logger().warn('麦克风不可用，2 秒后自动重连')
            time.sleep(2.0)

    def _voice_loop(self):
        """严格参照 ai-model main_web.py _voice_thread 实现。"""
        try:
            mic_info = sd.query_devices(sd.default.device['input'])
            mic_native_sr = int(mic_info['default_samplerate'])
        except Exception:
            mic_native_sr = 44100
        self.get_logger().info(
            f'麦克风: native_sr={mic_native_sr}, default={sd.default.device}'
        )

        chunk_16k = int(SAMPLE_RATE * CHUNK_SEC)            # 1600
        chunk_native = int(mic_native_sr * CHUNK_SEC)        # native 0.1s
        ratio = mic_native_sr / SAMPLE_RATE
        indices = (np.arange(chunk_16k) * ratio).astype(int)

        kws_stream = self._pipeline.kws.create_stream()
        cmd_audio = []
        cmd_frame_counter = 0
        # VAD 状态
        silence_frame_count = 0
        speech_frame_count = 0
        noise_rms_samples = []
        vad_threshold = VAD_ENERGY_THRESHOLD
        frames_per_silence = int(VAD_SILENCE_SEC / CHUNK_SEC)
        frames_per_min_speech = int(VAD_MIN_SPEECH_SEC / CHUNK_SEC)

        try:
            mic = sd.InputStream(
                samplerate=mic_native_sr, channels=1, dtype='int16',
                blocksize=chunk_native, device=sd.default.device['input'],
            )
            mic.start()
        except Exception as e:
            self.get_logger().error(f'麦克风启动失败: {e}')
            return

        self.get_logger().info('麦克风启动,进入主循环')
        read_error_count = 0

        while self._pipeline.running:
            try:
                data, _ = mic.read(chunk_native)
                read_error_count = 0
            except Exception as e:
                read_error_count += 1
                if read_error_count >= 10:
                    self.get_logger().warn(f'麦克风连续读取失败，准备重连: {e}')
                    try:
                        mic.close()
                    except Exception:
                        pass
                    return
                time.sleep(0.05)
                continue

            # 重采样 native → 16kHz(参照 main_web.py 索引重采样)
            native = np.frombuffer(data, dtype=np.int16).astype(np.float32)
            if len(native) >= indices[-1] + 1:
                samples_16k = native[indices]
            else:
                samples_16k = native[:chunk_16k]
            samples = samples_16k / 32768.0

            # boost normalize 到 15000 peak
            raw_peak = int(np.max(np.abs(native))) if len(native) > 0 else 0
            if 0 < raw_peak < AUDIO_BOOST_TARGET_PEAK:
                gain = min(AUDIO_BOOST_TARGET_PEAK / max(raw_peak, 1), AUDIO_BOOST_MAX_GAIN)
                native_boosted = (native * gain).clip(-32767, 32767).astype(np.float32)
            else:
                native_boosted = native
            if len(native_boosted) >= indices[-1] + 1:
                samples_16k_boosted = native_boosted[indices]
            else:
                samples_16k_boosted = native_boosted[:chunk_16k]
            samples_boosted = samples_16k_boosted / 32768.0

            with self._lock:
                current_state = self._state

            # ============ LISTENING: 录 4s 命令(VAD 可提前结束)============
            if current_state == 'listening':
                cmd_frame_counter += 1
                rms = float(np.sqrt(np.mean(samples ** 2)))
                # 跳过唤醒后 1s 尾音
                if cmd_frame_counter <= CMD_SKIP_FRAMES:
                    noise_rms_samples.append(rms)
                    if cmd_frame_counter == CMD_SKIP_FRAMES:
                        vad_threshold = compute_adaptive_vad_threshold(
                            noise_rms_samples
                        )
                        self.get_logger().info(
                            f'VAD 自适应阈值={vad_threshold:.4f} '
                            f'(固定阈值={VAD_ENERGY_THRESHOLD:.4f})'
                        )
                    continue

                cmd_audio.append(samples_boosted.copy())

                # VAD:用【原始 samples】(不是 boosted)算 RMS
                # boost 会放大静音段,RMS 永远高于阈值,VAD 失效
                if rms < vad_threshold:
                    silence_frame_count += 1
                else:
                    silence_frame_count = 0
                    speech_frame_count += 1

                # VAD 提前结束:必须【先说过话】(speech >= 0.3s) + 静音 0.8s
                if (speech_frame_count >= frames_per_min_speech
                        and silence_frame_count >= frames_per_silence):
                    # 去尾静音,只把纯语音送 ASR
                    silence_samples = silence_frame_count * chunk_16k
                    all_samples = np.concatenate(cmd_audio)
                    # 保留最后 1 块(避免词尾爆破音被截掉)
                    cut = max(0, len(all_samples) - silence_samples + chunk_16k)
                    clean_samples = all_samples[:cut]
                    self.get_logger().info(
                        f'VAD 触发(说过 {speech_frame_count*CHUNK_SEC:.1f}s + '
                        f'静音 {VAD_SILENCE_SEC}s),ASR 输入 '
                        f'{len(clean_samples)/SAMPLE_RATE:.2f}s'
                    )
                    self._finish_recording([clean_samples])
                    cmd_audio = []
                    cmd_frame_counter = 0
                    silence_frame_count = 0
                    speech_frame_count = 0
                    noise_rms_samples = []
                    vad_threshold = VAD_ENERGY_THRESHOLD

                # 兜底:固定 6s 录音
                elif cmd_frame_counter >= CMD_SKIP_FRAMES + CMD_RECORD_FRAMES:
                    total_audio = np.concatenate(cmd_audio)
                    self.get_logger().info(
                        f'录音完成(超时 4s),ASR 输入 {len(total_audio)/SAMPLE_RATE:.1f}s'
                    )
                    self._finish_recording([total_audio])
                    cmd_audio = []
                    cmd_frame_counter = 0
                    silence_frame_count = 0
                    speech_frame_count = 0
                    noise_rms_samples = []
                    vad_threshold = VAD_ENERGY_THRESHOLD

            # ============ IDLE: KWS 监听唤醒词 ============
            elif current_state == 'idle':
                kws_stream.accept_waveform(SAMPLE_RATE, samples)
                while self._pipeline.kws.is_ready(kws_stream):
                    self._pipeline.kws.decode_stream(kws_stream)
                result = self._pipeline.kws.get_result(kws_stream).strip()
                if result:
                    self._on_wake(result)
                    cmd_audio = []
                    cmd_frame_counter = 0
                    noise_rms_samples = []
                    vad_threshold = VAD_ENERGY_THRESHOLD
                    self._pipeline.kws.reset_stream(kws_stream)

        try:
            mic.stop()
            mic.close()
        except Exception:
            pass

    def _on_wake(self, kw: str):
        with self._lock:
            self._state = 'listening'
            # 倒计时 = CMD_SKIP 1s + CMD_RECORD 6s
            self._state_remaining = (CMD_SKIP_FRAMES + CMD_RECORD_FRAMES) * CHUNK_SEC
        self.get_logger().info(f'唤醒词触发: {kw}')
        self._publish_cmd('wakeup', kw)
        self._speak(TTS_WAKEUP)

    def _finish_recording(self, buffer):
        samples = np.concatenate(buffer)
        try:
            text = self._pipeline.recognize(samples)
        except Exception as e:
            self.get_logger().warn(f'ASR 推理失败: {e}')
            text = ''

        audio_dur = len(samples) / SAMPLE_RATE
        self.get_logger().info(
            f'ASR 诊断: 音频 {audio_dur:.2f}s, '
            f'识别 {len(text)} 字: {text!r}'
        )

        if text:
            action_id, keyword = match_command(text)
            if action_id:
                front_action = ACTION_MAP.get(action_id)
                if front_action:
                    self._publish_cmd(front_action, text)
                    self.get_logger().info(f'命令: {action_id} → {front_action}')
                    if action_id == 'QUERY_AHEAD':
                        self._describe_ahead()
                    else:
                        self._speak(TTS_CONFIRM)
                    with self._lock:
                        self._state = 'idle'
                        self._state_remaining = 0.0
                    return
            # 未匹配,透传原文
            self._publish_cmd('query', text)
            self.get_logger().info(f'未匹配,ASR 原文已透传: {text!r}')
            self._speak(TTS_UNRECOGNIZED)
        else:
            self.get_logger().warn(f'ASR 空(模型未识别),音频 {audio_dur:.2f}s')
            self._speak(TTS_UNRECOGNIZED)

        with self._lock:
            self._state = 'idle'
            self._state_remaining = 0.0

    def _describe_ahead(self):
        if self._latest_detections is None:
            self._speak('前方没有检测到障碍物')
            with self._lock:
                self._state = 'idle'
                self._state_remaining = 0.0
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        age = now_sec - self._detections_stamp_sec
        if age > DETECTION_EXPIRY_SEC:
            self._speak('前方没有检测到障碍物')
            with self._lock:
                self._state = 'idle'
                self._state_remaining = 0.0
            return

        valid = [
            d for d in self._latest_detections.detections
            if d.confidence >= DETECTION_MIN_CONFIDENCE and d.distance_m > 0
        ]
        if not valid:
            self._speak('前方没有检测到障碍物')
        else:
            valid.sort(key=lambda d: d.distance_m)
            top = valid[:DETECTION_MAX_REPORT]
            parts = [
                f'{_float_to_zh(d.distance_m)}米有一个{LABEL_ZH.get(d.class_name, d.class_name)}'
                for d in top
            ]
            text = '检测到前方' + ','.join(parts)
            self.get_logger().info(f'前方查询播报: {text}')
            self._speak(text)

        with self._lock:
            self._state = 'idle'
            self._state_remaining = 0.0

    def _publish_cmd(self, action: str, text: str):
        msg = String()
        msg.data = json.dumps({'action': action, 'text': text}, ensure_ascii=False)
        self._cmd_pub.publish(msg)

    def _publish_state(self):
        with self._lock:
            state = self._state
            remaining = self._state_remaining
            if state == 'listening':
                self._state_remaining = max(0.0, remaining - 0.1)
                remaining = self._state_remaining
        msg = String()
        msg.data = json.dumps({'state': state, 'remaining': round(remaining, 1)}, ensure_ascii=False)
        self._state_pub.publish(msg)

    def _speak(self, text: str):
        if self._tts is None or not self._tts.is_loaded():
            return
        try:
            if text in self._audio_cache:
                audio, sr = self._audio_cache[text]
                sd.play(audio, sr)
            else:
                def _gen_play():
                    audio, sr = self._tts.generate(text)
                    if len(audio) > 0:
                        sd.play(audio, sr)
                threading.Thread(target=_gen_play, daemon=True).start()
        except Exception as e:
            self.get_logger().warn(f'TTS 失败: {e}')

    def destroy_node(self):
        try:
            self._pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
