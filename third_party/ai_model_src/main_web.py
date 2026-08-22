"""DK-2500 AI模块整合主程序 (Web GUI + 语音 + YOLO NPU)

异构调度:
  NPU  → YOLO 目标检测（INT8 量化，异步推理）
  iGPU → ASR 语音识别 + TTS 声码器
  CPU  → KWS 唤醒词 + 控制逻辑 + Web 服务

语音流程:
  OnlineRecognizer + endpoint detection + reset(stream)
  IDLE: KWS 检查唤醒词
  RECORDING: 等待命令 endpoint（录音阶段支持紧急停止热检测）

用法:
    python src/main_web.py
    python src/main_web.py --camera 1
    python src/main_web.py --no-camera
"""

import argparse
import os
import re
import sys

# 抑制 OpenCV Qt 字体警告，避免每帧 imshow 触发 QFontDatabase 扫描导致卡顿
os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts"
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.fonts=false"
import time
import threading
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from config import (
    SAMPLE_RATE, COMMAND_MAP, MAX_RECORDING_DURATION,
    EMERGENCY_KEYWORDS, TRAFFIC_LIGHT_ADVICE, SPEED_LEVELS,
    YOLO_DEVICE, FRONTEND_DIR, DEPTH_ENABLED,
)
from voice_pipeline import VoicePipeline, State, WAKE_WORDS
from tts_engine import TTSEngine
from yolo_detector import (
    create_detector, draw_detections, estimate_distance,
)
from web_server import state, push_event, run_server


# ── TTS 文本预处理：阿拉伯数字 → 中文 ────────────────────────────────
_ZH_DIGITS = "零一二三四五六七八九"


def _int_to_chinese(n):
    """将 0-99 的整数转为中文口语。"""
    if n == 0:
        return "零"
    if n <= 9:
        return _ZH_DIGITS[n]
    if n == 10:
        return "十"
    if n < 20:
        return "十" + _ZH_DIGITS[n % 10]
    tens, ones = n // 10, n % 10
    if ones == 0:
        return _ZH_DIGITS[tens] + "十"
    return _ZH_DIGITS[tens] + "十" + _ZH_DIGITS[ones]


def _num_to_chinese(num_str):
    """将数字字符串转为中文口语（支持整数和小数）。"""
    if '.' in num_str:
        int_s, dec_s = num_str.split('.', 1)
        int_zh = _int_to_chinese(int(int_s)) if int_s != '0' else "零"
        dec_zh = ''.join(_ZH_DIGITS[int(d)] for d in dec_s if d.isdigit())
        return int_zh + "点" + dec_zh
    return _int_to_chinese(int(num_str))


def _normalize_for_tts(text):
    """将文本中的阿拉伯数字替换为中文，避免 TTS 读出英文数字。"""
    return re.sub(r'\d+\.?\d*', lambda m: _num_to_chinese(m.group()), text)


class AIDemoWeb:
    def __init__(self, camera_source=0, no_camera=False, conf=0.25, port=8080,
                 video_path=None):
        self.no_camera = no_camera
        self.camera_source = camera_source
        self.conf = conf
        self.port = port
        self.running = False
        self.detector = None
        self.camera = None
        self.video_path = video_path
        self.last_frame = None
        self.last_detections = []
        self.frame_lock = threading.Lock()
        self.yolo_fps = 0.0
        self.current_speed_idx = 1  # Default: 0.5 m/s

        # Traffic light tracking (debounce: only speak on state change)
        self._last_tl_state = None       # "红灯" / "绿灯" / None
        self._last_tl_speak_time = 0
        self._tl_cooldown = 8.0          # seconds between repeated alerts

        self.mic_device = 0
        self.mic_native_sr = 44100
        self._find_best_mic()

        print("=" * 60)
        print("  AI Module — DK-2500 (NPU + iGPU + CPU)")
        print("=" * 60)

        # TTS (iGPU)
        print("[INIT] Loading TTS (iGPU)...")
        self.tts = TTSEngine(speaker_id=3)
        state["tts_status"] = "ready"
        state["tts_speaker_id"] = 3

        # YOLO (NPU)
        if not no_camera:
            self._init_yolo()

        # Voice pipeline (CPU KWS + iGPU ASR)
        print("[INIT] Loading voice pipeline...")
        self.pipeline = VoicePipeline(tts_engine=self.tts)

        state["status"] = "RUNNING"
        if self.video_path:
            import os
            state["video_dir"] = os.path.dirname(self.video_path)
        push_event("init", {"log_type": "sys", "message": f"Ready. http://localhost:{self.port}"})
        print(f"\nWeb GUI: http://localhost:{self.port}")

    def _find_best_mic(self):
        import sounddevice as sd
        # Auto-detect best mic: prefer USB mic over line-in
        best_id = 0
        best_name = ""
        for i, d in enumerate(sd.query_devices()):
            if d['max_input_channels'] > 0:
                name = d['name'].lower()
                if 'usb' in name or 'uac' in name or 'mic' in name:
                    best_id = i
                    best_name = d['name']
                    break
        if not best_name:
            best_name = sd.query_devices(0)['name']
        self.mic_device = best_id
        dev_info = sd.query_devices(best_id)
        self.mic_native_sr = int(dev_info['default_samplerate'])
        print(f"[MIC] Device: [{best_id}] {dev_info['name']} @ {self.mic_native_sr}Hz")

    def _init_yolo(self):
        print(f"[INIT] Loading YOLO (device={YOLO_DEVICE})...")
        self.detector = create_detector(conf=self.conf)
        self.detector.warmup()
        state["yolo_running"] = True
        print("[INIT] YOLO ready.")

    def _speak(self, text):
        text = _normalize_for_tts(text)
        sid = state.get("tts_speaker_id", self.tts.speaker_id)
        self.tts.speak(text, speaker_id=sid)
        state["tts_last_text"] = text
        state["tts_status"] = "speaking"

    def _handle_command(self, text):
        """Match and execute command from recognized text."""
        text = text.strip()
        print(f"[CMD] Received: '{text}'")
        push_event("cmd", {"log_type": "cmd", "message": f"Handle: '{text}'"})

        # Fuzzy match: check each keyword, sorted by length (longest first)
        for keyword in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
            if keyword in text:
                cmd = COMMAND_MAP[keyword]
                state["last_command"] = cmd
                state["last_keyword"] = keyword
                self._execute_command(cmd, keyword, text)
                return True

        # No exact match — try partial match (first 2+ chars of keyword in text)
        for keyword in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
            if len(keyword) >= 3 and keyword[:2] in text:
                cmd = COMMAND_MAP[keyword]
                state["last_command"] = cmd
                state["last_keyword"] = keyword
                push_event("cmd", {"log_type": "cmd", "message": f"Partial match: '{keyword}' in '{text}'"})
                self._execute_command(cmd, keyword, text)
                return True

        state["last_command"] = "UNKNOWN"
        self._speak("抱歉，我没有听清，请再说一次")
        return False

    def _execute_command(self, cmd, keyword, text):
        # 紧急停止：最高优先级，立即打断 TTS
        if cmd == "EMERGENCY_STOP":
            self.tts.stop_speaking()
            self._speak("紧急停止已执行，轮椅已锁定")
            return

        if cmd == "QUERY_AHEAD":
            self._handle_query_ahead()
        elif cmd == "QUERY_BATTERY":
            self._speak("当前电量百分之七十五，状态正常")
        elif cmd == "QUERY_LOCATION":
            self._speak("当前位于室内环境，定位正常")
        elif cmd == "STOP":
            self._speak("已停止")
        elif cmd == "SPEED_UP":
            self.current_speed_idx = min(self.current_speed_idx + 1, len(SPEED_LEVELS) - 1)
            speed = SPEED_LEVELS[self.current_speed_idx]
            self._speak(f"已加速至每秒{speed}米")
        elif cmd == "SPEED_DOWN":
            self.current_speed_idx = max(self.current_speed_idx - 1, 0)
            speed = SPEED_LEVELS[self.current_speed_idx]
            self._speak(f"已减速至每秒{speed}米")
        elif cmd.startswith("NAV_"):
            self._speak("正在导航，请注意周围环境")
        elif cmd == "HELP":
            self._speak("可用命令：去客厅、去卧室、停止、紧急停止、加速、减速、前方有什么、电量多少、帮助")
        elif cmd in ("CONFIRM", "CANCEL"):
            pass
        else:
            self._speak(f"好的，{keyword}")

    # YOLO COCO class English → Chinese (only allowed classes)
    YOLO_ZH = {
        "person": "人", "bicycle": "自行车", "car": "汽车", "motorcycle": "摩托车",
        "bus": "公交车", "truck": "卡车", "traffic light": "红绿灯",
    }

    @staticmethod
    def _detect_traffic_light_color(frame, bbox):
        """Crop the traffic light region and detect red/yellow/green."""
        x1, y1, x2, y2 = bbox
        h, w = y2 - y1, x2 - x1
        if h < 15 or w < 8:
            return None
        import cv2
        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        band_h = h // 2
        best_color = None
        best_score = 0

        # 国内人行道红绿灯只有红/绿两色
        for i, (h_lo, h_hi, label) in enumerate([
            ((0, 80, 80), (10, 255, 255), "红灯"),
            ((160, 80, 80), (180, 255, 255), "红灯"),
            ((35, 80, 80), (85, 255, 255), "绿灯"),
        ]):
            mask = cv2.inRange(hsv, h_lo, h_hi)
            if label == "红灯" and i < 2:
                mask_roi = mask[:band_h, :]
            else:
                mask_roi = mask[band_h:, :]
            score = int(cv2.countNonZero(mask_roi))
            if score > best_score:
                best_score = score
                best_color = label

        roi_area = h * w
        if best_score < roi_area * 0.03:
            red_mask = (cv2.inRange(hsv, (0, 80, 80), (10, 255, 255)) |
                        cv2.inRange(hsv, (160, 80, 80), (180, 255, 255)))
            green_mask = cv2.inRange(hsv, (35, 80, 80), (85, 255, 255))
            red_px = int(cv2.countNonZero(red_mask))
            green_px = int(cv2.countNonZero(green_mask))
            if red_px >= green_px and red_px > roi_area * 0.02:
                return "红灯"
            if green_px > red_px and green_px > roi_area * 0.02:
                return "绿灯"
            return None

        return best_color

    @staticmethod
    def _distance_to_chinese(dist_m):
        """Convert distance to natural spoken Chinese.
        e.g. 0.5 → '半米', 1.0 → '一米', 1.5 → '一米半',
             2.3 → '两米多', 3.0 → '三米', 5.0 → '五米'
        """
        _NUM = {0: "零", 1: "一", 2: "两", 3: "三", 4: "四",
                5: "五", 6: "六", 7: "七", 8: "八", 9: "九", 10: "十"}
        if dist_m < 0.1:
            return "极近"
        i = int(dist_m)
        f = round(dist_m - i, 1)
        if i == 0:
            if f <= 0.3:
                return "不到半米"
            else:
                return "半米左右"
        i_zh = _NUM.get(i, _int_to_chinese(i))
        if f == 0:
            return f"{i_zh}米"
        if f == 0.5:
            return f"{i_zh}米半"
        if f < 0.3:
            return f"{i_zh}米出头"
        if f > 0.7:
            next_zh = _NUM.get(i + 1, _int_to_chinese(i + 1))
            return f"将近{next_zh}米"
        return f"{i_zh}米多"

    def _handle_query_ahead(self):
        with self.frame_lock:
            cached_frame = self.last_frame.copy() if self.last_frame is not None else None
            detections = list(self.last_detections)

        print(f"[QUERY_AHEAD] {len(detections)} detections (cached), frame={'yes' if cached_frame is not None else 'no'}")

        if not detections:
            self._speak("前方没有检测到任何物体，可以安全通行")
            return

        # Enrich with traffic light color
        if cached_frame is not None:
            for d in detections:
                if "bbox" not in d:
                    continue
                if d.get("class_name") == "traffic light":
                    color = self._detect_traffic_light_color(cached_frame, d["bbox"])
                    if color:
                        d["light_color"] = color

        # Handle traffic light with advice
        for d in detections:
            if d.get("class_name") == "traffic light" and d.get("light_color"):
                advice = TRAFFIC_LIGHT_ADVICE.get(d["light_color"])
                if advice:
                    self._speak(advice)
                    state["traffic_light_state"] = d["light_color"].replace("灯", "")
                    return

        if not detections:
            self._speak("前方没有检测到任何物体，可以安全通行")
        else:
            from collections import Counter
            _zh_num = {1: "一", 2: "二", 3: "三", 4: "四", 5: "五",
                       6: "六", 7: "七", 8: "八", 9: "九", 10: "十"}
            # Sort by distance (nearest first)
            sorted_dets = sorted(
                detections,
                key=lambda d: d.get("distance_m") or 999
            )

            parts = []
            for d in sorted_dets[:5]:
                en_name = d.get("class_name", "物体")
                zh_name = self.YOLO_ZH.get(en_name, en_name)
                if en_name == "traffic light" and d.get("light_color"):
                    zh_name = d["light_color"]
                dist = d.get("distance", "未知距离")
                parts.append(f"一个{zh_name}，距离{dist}")

            categories = Counter(
                self.YOLO_ZH.get(d.get("class_name", ""), d.get("class_name", ""))
                for d in detections
            )
            summary_parts = []
            for name, count in categories.most_common(5):
                cn = _zh_num.get(count, str(count))
                summary_parts.append(f"{cn}个{name}")

            msg = f"前方检测到" + "、".join(summary_parts) + "，"
            msg += "最近的" + parts[0]
            if len(parts) > 1:
                msg += "，其次" + parts[1]

            print(f"[QUERY_AHEAD] Speaking: {msg}")
            self._speak(msg)

    def _capture_and_detect(self):
        if self.detector is None:
            return []
        import cv2

        # Get depth frame from ROS 2 (thread-safe)
        with self.frame_lock:
            depth_frame = self.last_depth_frame

        # Prefer OpenCV camera (reliable), ROS 2 as supplement for depth
        if self.camera is not None:
            if self.video_path:
                # Video mode: read sequential frames, loop on end
                ret, frame = self.camera.read()
                if not ret:
                    self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.camera.read()
            else:
                # Live camera: drain buffer to get the latest frame
                for _ in range(2):
                    ret, _ = self.camera.read()
                ret, frame = self.camera.read()
            if ret:
                with self.frame_lock:
                    self.last_frame = frame
            else:
                with self.frame_lock:
                    frame = self.last_frame
        else:
            with self.frame_lock:
                frame = self.last_frame

        if frame is None:
            return []

        with self.frame_lock:
            self.last_frame = frame

        detections = self.detector.detect(frame)

        # Depth-based distance measurement (Gemini 335L)
        depth_h, depth_w = (depth_frame.shape[:2] if depth_frame is not None else (0, 0))
        frame_h, frame_w = frame.shape[:2]
        use_depth = DEPTH_ENABLED and depth_frame is not None and depth_w > 0 and depth_h > 0

        for d in detections:
            if d.get("class_name") == "traffic light":
                color = self._detect_traffic_light_color(frame, d["bbox"])
                if color:
                    d["light_color"] = color

            x1, y1, x2, y2 = d["bbox"]
            if use_depth:
                # Map bbox from color frame to depth frame
                sx1 = max(0, int(x1 * depth_w / frame_w))
                sy1 = max(0, int(y1 * depth_h / frame_h))
                sx2 = min(depth_w - 1, int(x2 * depth_w / frame_w))
                sy2 = min(depth_h - 1, int(y2 * depth_h / frame_h))
                bw, bh = sx2 - sx1, sy2 - sy1
                if bw < 2 or bh < 2:
                    d["distance"] = estimate_distance(d["bbox"], frame.shape)
                    continue

                # Sample the lower-center region (object base, more stable)
                # Use bottom 40% horizontally centered 60%
                region_left = sx1 + bw // 5
                region_right = sx2 - bw // 5
                region_top = sy1 + int(bh * 0.6)
                region_bot = sy2
                region = depth_frame[region_top:region_bot, region_left:region_right]
                valid = region[region > 0]

                # Fallback to full bbox center strip if base region is empty
                if len(valid) < 3:
                    cx = (sx1 + sx2) // 2
                    strip_left = max(0, cx - max(bw // 6, 2))
                    strip_right = min(depth_w - 1, cx + max(bw // 6, 2))
                    region = depth_frame[sy1:sy2, strip_left:strip_right]
                    valid = region[region > 0]

                if len(valid) >= 3:
                    # Use 30th percentile for robustness against mixed depths
                    dist_mm = float(np.percentile(valid, 30))
                    dist_m = dist_mm / 1000.0
                    d["distance"] = self._distance_to_chinese(dist_m)
                    d["distance_m"] = dist_m
                else:
                    d["distance"] = "未知距离"
                    d["distance_m"] = None
            else:
                d["distance"] = estimate_distance(d["bbox"], frame.shape)

        with self.frame_lock:
            if detections:
                self.last_detections = detections
        return detections

    def _check_traffic_light_alert(self, detections):
        """Track traffic light state changes and proactively alert the user.

        Debounce rules:
          - Only speak when state changes (e.g. 绿灯→红灯) or on first detection.
          - Same-state repeat alert after _tl_cooldown seconds.
          - Only alert for red/green (yellow is brief and less actionable).
        """
        current_color = None
        for d in detections:
            if d.get("class_name") == "traffic light" and d.get("light_color"):
                current_color = d["light_color"]
                break

        if current_color is None:
            return

        state["traffic_light_state"] = current_color.replace("灯", "")
        now = time.time()

        # State changed, or cooldown expired for same state
        if (current_color != self._last_tl_state or
                now - self._last_tl_speak_time > self._tl_cooldown):
            advice = TRAFFIC_LIGHT_ADVICE.get(current_color)
            if advice:
                print(f"[TL-ALERT] {current_color} → {advice}")
                push_event("traffic_light", {
                    "log_type": "tl",
                    "message": advice,
                })
                self._speak(advice)
                self._last_tl_state = current_color
                self._last_tl_speak_time = now

    def _yolo_thread(self):
        import cv2
        print("[YOLO-THREAD] Starting...")
        debug_count = 0

        fps_counter = 0
        fps_timer = time.time()
        fail_count = 0
        prev_frame = None
        prev_detections = []
        while self.running:
            t0 = time.time()

            # Check for video switch request from web UI
            new_video = state.get("switch_video")
            if new_video:
                state["switch_video"] = None
                if self.camera is not None:
                    self.camera.release()
                self.camera = cv2.VideoCapture(new_video)
                if self.camera.isOpened():
                    self.video_path = new_video
                    prev_frame = None
                    prev_detections = []
                    print(f"[YOLO] Switched to: {new_video}")
                else:
                    print(f"[YOLO] Failed to open: {new_video}")

            # Check for camera switch request from web UI
            if state.get("switch_to_camera"):
                state["switch_to_camera"] = False
                if self.camera is not None:
                    self.camera.release()
                    self.camera = None
                self.video_path = None
                import cv2
                for src in [self.camera_source, 1, 3, 4, 6, 2, 5, 7, 0]:
                    cap = cv2.VideoCapture(src)
                    if cap.isOpened():
                        ret, test = cap.read()
                        if ret and len(test.shape) == 3 and test.shape[2] == 3:
                            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                            self.camera = cap
                            print(f"[YOLO] Switched to camera: source={src}")
                            break
                        cap.release()
                prev_frame = None
                prev_detections = []

            # Spin ROS 2 to get latest frames (init done in main thread)
            if self._ros2_ok:
                try:
                    import rclpy
                    rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                except Exception:
                    pass

            dets = self._capture_and_detect()
            debug_count += 1
            if debug_count <= 5:
                cam_ok = self.camera is not None and self.camera.isOpened()
                print(f"[YOLO-DEBUG] #{debug_count} dets={len(dets)} cam={cam_ok} frame={'yes' if self.last_frame is not None else 'NO'}")

            if not dets:
                fail_count += 1
            else:
                fail_count = 0

            elapsed = time.time() - t0

            with self.frame_lock:
                frame = self.last_frame
                detections = list(self.last_detections)

            # Reuse previous frame/detections for display if current is empty
            if frame is None and prev_frame is not None:
                frame = prev_frame
                detections = prev_detections

            if frame is not None:
                prev_frame = frame
                prev_detections = detections

                frame = draw_detections(frame, detections)
                device_info = getattr(self.detector, 'device', 'cpu')
                cv2.putText(frame, f"FPS: {self.yolo_fps:.1f} | Dets: {len(detections)} | {device_info}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Scale display to fit screen (leave room for title bar + taskbar)
                import tkinter as _tk
                _root = _tk.Tk()
                screen_w, screen_h = _root.winfo_screenwidth(), _root.winfo_screenheight()
                _root.destroy()
                max_w, max_h = screen_w - 40, screen_h - 80
                fh, fw = frame.shape[:2]
                scale = min(max_w / fw, max_h / fh, 1.0)
                if scale < 1.0:
                    frame = cv2.resize(frame, (int(fw * scale), int(fh * scale)))
                cv2.imshow("AI Module - YOLO + Voice (DK-2500 NPU)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.running = False
                    break

            if detections:
                state["yolo_detections"] = [
                    {"class_name": d.get("class_name", "?"),
                     "confidence": d.get("confidence", 0),
                     "light_color": d.get("light_color"),
                     "distance": d.get("distance")}
                    for d in detections
                ]
                state["yolo_detection_count"] = len(detections)
                self._check_traffic_light_alert(detections)

            fps_counter += 1
            if fps_counter % 10 == 0:
                now = time.time()
                self.yolo_fps = 10.0 / (now - fps_timer)
                fps_timer = now
                state["yolo_fps"] = self.yolo_fps

            time.sleep(max(0.001, 0.020 - elapsed))
        cv2.destroyAllWindows()

    def _voice_thread(self):
        """Two-stage cascade: KWS (CPU, ~3% CPU) + ASR (iGPU, on-demand).

        IDLE:    KWS streams audio → detects wake word → load ASR (if needed)
        RECORDING: Skip TTS → record 4s → ASR recognize → execute → unload ASR → IDLE

        紧急停止优化:
          RECORDING 阶段对每帧音频做轻量关键词热检测，
          检测到紧急关键词立即执行停止。
        """
        import sounddevice as sd

        pipeline = self.pipeline
        kws = pipeline.kws

        chunk_16k = int(SAMPLE_RATE * 0.1)
        chunk_native = int(self.mic_native_sr * 0.1)
        ratio = self.mic_native_sr / SAMPLE_RATE
        indices = (np.arange(chunk_16k) * ratio).astype(int)

        level_counter = 0
        kws_stream = kws.create_stream()

        cmd_audio = []
        cmd_frame_counter = 0
        CMD_SKIP = 10       # 1.0s skip after wake
        CMD_RECORD = 40     # 4s command recording

        push_event("sys", {"log_type": "sys", "message": "Voice: KWS (CPU) + ASR (iGPU)"})

        with sd.InputStream(samplerate=self.mic_native_sr, channels=1, dtype="int16",
                            blocksize=chunk_native, device=self.mic_device) as mic:
            while self.running:
                try:
                    data, _ = mic.read(chunk_native)
                except Exception:
                    time.sleep(0.05)
                    continue

                # Resample 44100 → 16000
                native = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                if len(native) >= indices[-1] + 1:
                    samples_16k = native[indices]
                else:
                    samples_16k = native[::int(ratio)][:chunk_16k]
                samples = samples_16k / 32768.0

                # Boosted version for ASR (normalize to ~15000 peak)
                raw_peak = int(np.max(np.abs(native)))
                if 0 < raw_peak < 15000:
                    gain = min(15000.0 / max(raw_peak, 1), 200.0)
                    native_boosted = (native * gain).clip(-32767, 32767).astype(np.float32)
                else:
                    native_boosted = native
                if len(native_boosted) >= indices[-1] + 1:
                    samples_16k_boosted = native_boosted[indices]
                else:
                    samples_16k_boosted = native_boosted[::int(ratio)][:chunk_16k]
                samples_boosted = samples_16k_boosted / 32768.0

                # Mic level display
                boosted_rms = float(np.sqrt(np.mean(native_boosted ** 2)))
                level_counter += 1
                if level_counter % 10 == 0:
                    state["mic_level"] = boosted_rms
                    state["mic_peak"] = int(np.max(np.abs(native_boosted)))
                    state["mic_working"] = state["mic_peak"] > 50

                # ============ RECORDING: ASR active, record command ============
                if pipeline.state == State.RECORDING:
                    cmd_frame_counter += 1

                    if cmd_frame_counter <= CMD_SKIP:
                        state["asr_partial"] = f"[等待 {2.5 - cmd_frame_counter*0.1:.1f}s]"
                        continue

                    cmd_audio.append(samples_boosted.copy())
                    rec_elapsed = (cmd_frame_counter - CMD_SKIP) * 0.1
                    state["asr_partial"] = f"[命令录音 {rec_elapsed:.1f}s / 4.0s]"

                    # 紧急停止热检测：每累积 0.5s 音频做一次轻量 ASR
                    if cmd_frame_counter % 5 == 0 and len(cmd_audio) >= 5:
                        hot_audio = np.concatenate(cmd_audio[-5:])
                        hot_text = pipeline.recognize(hot_audio)
                        if hot_text:
                            for kw in EMERGENCY_KEYWORDS:
                                if kw in hot_text:
                                    print(f"[EMERGENCY] Hot-detected: '{kw}'")
                                    push_event("emergency", {"log_type": "emergency", "message": f"Hot: '{kw}'"})
                                    self.tts.stop_speaking()
                                    self._execute_command("EMERGENCY_STOP", kw, hot_text)
                                    cmd_audio = []
                                    cmd_frame_counter = 0
                                    pipeline.state = State.IDLE
                                    state["kws_state"] = "IDLE"
                                    break

                    if cmd_frame_counter >= CMD_SKIP + CMD_RECORD:
                        total_audio = np.concatenate(cmd_audio)
                        state["asr_partial"] = f"[识别中 {len(total_audio)/SAMPLE_RATE:.1f}s...]"

                        text = pipeline.recognize(total_audio)
                        print(f"[ASR] Command recognized: '{text}'")
                        cmd_audio = []
                        cmd_frame_counter = 0
                        state["asr_partial"] = ""

                        push_event("asr", {"log_type": "asr", "message": f"Cmd: '{text}'"})

                        if text:
                            state["asr_final"] = text
                            self._handle_command(text)
                        else:
                            self._speak("抱歉，我没有听清")

                        pipeline.state = State.IDLE
                        pipeline.unload_asr()
                        state["kws_state"] = "IDLE"

                # ============ IDLE: KWS only (~3% CPU) ============
                elif pipeline.state == State.IDLE:
                    kws_stream.accept_waveform(SAMPLE_RATE, samples)

                    while kws.is_ready(kws_stream):
                        kws.decode_stream(kws_stream)

                    result = kws.get_result(kws_stream)
                    if result:
                        push_event("wake", {"log_type": "wake", "message": f"KWS: '{result}'"})

                        state["wake_count"] = state.get("wake_count", 0) + 1
                        state["last_wake_time"] = time.strftime("%H:%M:%S")

                        pipeline.load_asr()
                        pipeline.state = State.RECORDING
                        cmd_audio = []
                        cmd_frame_counter = 0
                        state["kws_state"] = "RECORDING"
                        self._speak("我在，请说命令")

                        kws.reset_stream(kws_stream)

                state["kws_state"] = pipeline.state.name

    def run(self):
        self.running = True

        # Open video file or camera as frame source
        if not self.no_camera and self.camera is None:
            import cv2
            if self.video_path:
                cap = cv2.VideoCapture(self.video_path)
                if cap.isOpened():
                    self.camera = cap
                    print(f"[INIT] Video: {self.video_path}")
                else:
                    print(f"[INIT] Failed to open video: {self.video_path}")
            if self.camera is None:
                for src in [self.camera_source, 1, 3, 4, 6, 2, 5, 7, 0]:
                    cap = cv2.VideoCapture(src)
                    if cap.isOpened():
                        ret, test = cap.read()
                        if ret and len(test.shape) == 3 and test.shape[2] == 3:
                            b, g, r = cv2.split(test)
                            is_gray = (b == g).all() and (g == r).all()
                            if not is_gray:
                                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                                self.camera = cap
                                print(f"[INIT] Camera: source={src} ({test.shape[1]}x{test.shape[0]} RGB)")
                                break
                        cap.release()

        # Initialize ROS 2 in main thread (required by rclpy)
        self.last_depth_frame = None
        self._ros_node = None
        self._ros2_ok = False
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            rclpy.init()
            self._ros_node = Node('ai_yolo_depth')
            self._cv_bridge = CvBridge()

            def on_depth(msg):
                try:
                    depth = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                    with self.frame_lock:
                        self.last_depth_frame = depth
                except Exception:
                    pass

            def on_color(msg):
                try:
                    color = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    with self.frame_lock:
                        self.last_frame = color
                except Exception:
                    pass

            self._ros_node.create_subscription(Image, '/camera/depth/image_raw', on_depth, 1)
            self._ros_node.create_subscription(Image, '/camera/color/image_raw', on_color, 1)
            self._ros2_ok = True
            print("[INIT] ROS 2 depth subscriber active (Gemini 335L)")
        except Exception as e:
            print(f"[INIT] ROS 2 not available: {e}")
            self._ros_node = None

        # Web server
        threading.Thread(target=run_server, kwargs={"port": self.port}, daemon=True).start()
        push_event("sys", {"log_type": "sys", "message": f"Web server started on port {self.port}"})

        # YOLO continuous thread (uses ROS 2 depth + OpenVINO)
        if not self.no_camera and self.detector is not None:
            threading.Thread(target=self._yolo_thread, daemon=True).start()

        # Welcome
        self._speak("智能轮椅AI模块已启动，请说小智你好唤醒")

        # Voice thread (blocking)
        try:
            self._voice_thread()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.pipeline.stop()
        self.tts.stop()
        if self.camera is not None:
            self.camera.release()
        if self._ros_node is not None:
            try:
                import rclpy
                self._ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--video", type=str, default=None, help="Use video file instead of camera")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    demo = AIDemoWeb(camera_source=args.camera, no_camera=args.no_camera, conf=args.conf,
                     port=args.port, video_path=args.video)
    demo.run()


if __name__ == "__main__":
    main()
