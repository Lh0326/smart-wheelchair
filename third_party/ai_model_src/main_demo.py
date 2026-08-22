"""Windows MVP — AI模块整合演示主程序

功能（符合项目方案功能定位）:
  1. 语音交互: KWS唤醒词 → ASR命令识别 → 命令匹配 → TTS语音反馈
  2. YOLO实时检测: 摄像头实时检测行人、车辆等
  3. 跨模块联动: 语音问"前方有什么" → 触发YOLO → TTS播报

用法:
    python src/main_demo.py
    python src/main_demo.py --camera 1        # USB深度相机
    python src/main_demo.py --no-camera       # 无摄像头模式
"""

import argparse
import sys
import time
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from config import SAMPLE_RATE, COMMAND_MAP
from tts_engine import TTSEngine
from voice_pipeline import VoicePipeline


class AIDemo:
    def __init__(self, camera_source=0, no_camera=False, conf=0.25):
        self.no_camera = no_camera
        self.camera_source = camera_source
        self.conf = conf
        self.running = False
        self.detector = None
        self.camera = None
        self.last_frame = None
        self.last_detections = []
        self.frame_lock = threading.Lock()

        # TTS
        print("=" * 60)
        print("  智能轮椅 AI 模块 — Windows MVP 演示")
        print("=" * 60)
        print()
        self.tts = TTSEngine()

        # YOLO
        if not no_camera:
            self._init_yolo()
            self._init_camera()

        # Voice pipeline
        self.pipeline = VoicePipeline(tts_engine=self.tts)
        self.pipeline.on_command(self._on_command)

        print()
        print("所有模块加载完成。")
        print("唤醒词: \"小智你好\" 或 \"心语启动\"")
        print("可用命令:")
        for kw, cmd in sorted(COMMAND_MAP.items(), key=lambda x: x[1]):
            print(f"  {kw:10s} → {cmd}")
        print()

    def _init_yolo(self):
        from yolo_detector import YOLODetectorUltralytics
        print("Loading YOLO detector...")
        self.detector = YOLODetectorUltralytics("yolo26s.pt", conf=self.conf)
        print("YOLO warmup...")
        self.detector.warmup()
        print("YOLO ready.")

    def _init_camera(self):
        import cv2
        self.camera = cv2.VideoCapture(self.camera_source)
        if not self.camera.isOpened():
            print(f"WARNING: Cannot open camera {self.camera_source}, running without camera")
            self.no_camera = True
            self.camera = None
            return
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        w = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera opened: source={self.camera_source}, {w}x{h}")

    def _capture_and_detect(self):
        if self.no_camera or self.camera is None or self.detector is None:
            return []
        import cv2
        ret, frame = self.camera.read()
        if not ret:
            return []
        with self.frame_lock:
            self.last_frame = frame.copy()
        detections = self.detector.detect(frame)
        with self.frame_lock:
            self.last_detections = detections
        return detections

    def _yolo_thread(self):
        while self.running:
            self._capture_and_detect()
            time.sleep(0.01)

    def _on_command(self, text, cmd):
        if cmd == "QUERY_AHEAD":
            self._handle_query_ahead()
        elif cmd == "QUERY_BATTERY":
            self.tts.speak("当前电量百分之七十五，状态正常")
        elif cmd == "QUERY_LOCATION":
            self.tts.speak("当前位于室内环境，定位正常")
        elif cmd == "EMERGENCY_STOP":
            self.tts.speak("紧急停止已执行，轮椅已锁定")
        elif cmd == "STOP":
            self.tts.speak("已停止")
        elif cmd == "SPEED_UP":
            self.tts.speak("已加速至每秒零点八米")
        elif cmd == "SPEED_DOWN":
            self.tts.speak("已减速至每秒零点三米")
        elif cmd.startswith("NAV_"):
            with self.frame_lock:
                dets = self.last_detections
            self.tts.speak(f"正在导航，请注意周围环境")

    def _handle_query_ahead(self):
        print("[YOLO] Triggering detection for QUERY_AHEAD...")
        detections = self._capture_and_detect()
        if not detections:
            self.tts.speak("前方没有检测到障碍物，可以安全通行")
            return
        # Count by category
        from collections import Counter
        categories = Counter()
        for d in detections:
            name = d.get("class_name", "object")
            categories[name] += 1
        # Build TTS message
        parts = []
        for name, count in categories.most_common(5):
            if count == 1:
                parts.append(f"一个{name}")
            else:
                parts.append(f"{count}个{name}")
        msg = "前方检测到" + "、".join(parts) + "，请注意安全"
        print(f"[TTS] {msg}")
        self.tts.speak(msg)

    def _display_thread(self):
        import cv2
        from yolo_detector import draw_detections
        cv2.namedWindow("AI Module - YOLO + Voice", cv2.WINDOW_NORMAL)
        while self.running:
            with self.frame_lock:
                frame = self.last_frame.copy() if self.last_frame is not None else None
                dets = list(self.last_detections)
            if frame is not None:
                frame = draw_detections(frame, dets)
                det_names = [d.get("class_name", "?") for d in dets]
                cv2.putText(frame, f"Detections: {len(dets)}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if det_names:
                    cv2.putText(frame, f"  ".join(det_names[:5]), (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imshow("AI Module - YOLO + Voice", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    self.running = False
                    break
            else:
                time.sleep(0.05)
        cv2.destroyAllWindows()

    def run(self):
        self.running = True

        # Start YOLO detection thread
        if not self.no_camera:
            t_yolo = threading.Thread(target=self._yolo_thread, daemon=True)
            t_yolo.start()
            t_display = threading.Thread(target=self._display_thread, daemon=True)
            t_display.start()

        # Welcome message
        self.tts.speak("智能轮椅AI模块已启动，请说小智你好唤醒")
        print(">>> 系统已就绪，请说唤醒词 \"小智你好\" <<<")
        print()

        # Run voice pipeline (blocking, main thread)
        try:
            self.pipeline.run()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.pipeline.stop()
        self.tts.stop()
        if self.camera is not None:
            self.camera.release()


def main():
    parser = argparse.ArgumentParser(description="AI模块 Windows MVP 整合演示")
    parser.add_argument("--camera", type=int, default=0,
                        help="摄像头索引 (0=内置, 1=USB深度相机)")
    parser.add_argument("--no-camera", action="store_true",
                        help="不使用摄像头（仅语音交互）")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="YOLO置信度阈值")
    args = parser.parse_args()

    demo = AIDemo(camera_source=args.camera, no_camera=args.no_camera, conf=args.conf)
    demo.run()


if __name__ == "__main__":
    main()
