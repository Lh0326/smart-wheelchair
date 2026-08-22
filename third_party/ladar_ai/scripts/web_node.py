"""Web 展示节点。

Flask 服务器，通过 SSE 推送系统状态到前端，MJPEG 流推送摄像头画面。
摄像头画面从 ROS2 /annotated_image 话题获取（camera_detect_node 发布的 YOLO 标注图）。
"""
import os
import json
import time
import math
import threading
import logging
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from flask import Flask, Response, send_from_directory, jsonify

from ladar_ai import hardware_monitor

logger = logging.getLogger(__name__)


def _system_info_impl():
    """构造 /api/system 返回 dict。模块级便于单测。"""
    info = {
        "cpu_pct": 0,
        "cpu_freq_mhz": 0,
        "cpu_max_mhz": 0,
        "cpu_temp_c": None,
        "mem_used_gb": 0,
        "mem_total_gb": 16,
        "battery_pct": -1,
        "gpu_pct": 0,
        "gpu_freq_mhz": 0,
        "gpu_max_mhz": 0,
        "gpu_vram_used_gb": 0,
        "gpu_vram_total_gb": 0,
        "npu_pct": 0,
        "npu_freq_mhz": 0,
        "npu_active": False,
    }

    # MEM 从 /proc/meminfo（保持原有路径，比 psutil 兼容性好）
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    info["mem_total_gb"] = round(int(line.split()[1]) / 1048576, 1)
                elif line.startswith("MemAvailable:"):
                    avail = int(line.split()[1]) / 1048576
                    info["mem_used_gb"] = round(info["mem_total_gb"] - avail, 1)
    except Exception:
        pass

    # CPU + GPU + NPU 从 hardware_monitor
    try:
        per_core, total, freq_mhz, temp = hardware_monitor.get_cpu_info()
        info["cpu_pct"] = round(total, 1) if total else 0
        info["cpu_freq_mhz"] = int(freq_mhz or 0)
        info["cpu_temp_c"] = round(temp, 1) if temp is not None else None
    except Exception as e:
        logger.warning("CPU info failed: %s", e)

    try:
        g_act, g_max, g_util, (vram_u, vram_t) = hardware_monitor.get_gpu_info()
        info["gpu_pct"] = int(g_util)
        info["gpu_freq_mhz"] = int(g_act)
        info["gpu_max_mhz"] = int(g_max)
        info["gpu_vram_used_gb"] = round(vram_u, 2)
        info["gpu_vram_total_gb"] = round(vram_t, 2)
    except Exception as e:
        logger.warning("GPU info failed: %s", e)

    try:
        n_util, n_freq, n_max = hardware_monitor.get_npu_info()
        info["npu_pct"] = int(n_util)
        info["npu_freq_mhz"] = int(n_freq)
        info["npu_active"] = n_util > 0
    except Exception as e:
        logger.warning("NPU info failed: %s", e)

    # CPU 最大频率
    try:
        import psutil
        freq = psutil.cpu_freq()
        if freq and freq.max:
            info["cpu_max_mhz"] = int(freq.max)
    except Exception:
        pass

    # 电池
    try:
        bat_dir = "/sys/class/power_supply/BAT0"
        if os.path.isdir(bat_dir):
            with open(f"{bat_dir}/capacity") as f:
                info["battery_pct"] = int(f.read().strip())
    except Exception:
        pass

    return info


class WebNode(Node):
    def __init__(self):
        super().__init__("web_node")

        self.declare_parameter("web_port", 5001)

        self._latest_zones = None
        self._latest_detections = []
        self._latest_scan_points = []
        self._latest_ld14p_points = []
        self._last_scan_push_at = 0.0
        self._last_ld14p_push_at = 0.0
        self._latest_voice = {"action": "idle", "text": ""}
        self._latest_tts = ""
        self._latest_frame = None
        self._latest_frame_id = None
        self._frame_lock = threading.Lock()
        self._sse_clients = []
        self._sse_lock = threading.Lock()

        # CvBridge 用于将 ROS2 Image 转为 OpenCV
        self._bridge = CvBridge() if CvBridge is not None else None

        frontend_dir = os.path.join(
            os.path.dirname(__file__), "..", "..", "frontend"
        )
        if not os.path.exists(os.path.join(frontend_dir, "index.html")):
            from ament_index_python.packages import get_package_share_directory
            try:
                frontend_dir = os.path.join(
                    get_package_share_directory("ladar_ai"), "frontend"
                )
            except Exception:
                pass

        # 订阅 ROS2 话题
        try:
            from ladar_ai.msg import ZoneDistances, Detections
            from sensor_msgs.msg import Image, LaserScan
            self.create_subscription(ZoneDistances, "/lidar_zones", self._zones_cb, 10)
            self.create_subscription(Detections, "/front_detections", self._dets_cb, 10)
            self.create_subscription(Image, "/annotated_image", self._image_cb, 10)
            self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
            self.create_subscription(LaserScan, "/scan_ld14p", self._scan_ld14p_cb, 10)
            self.create_subscription(String, "/voice_command", self._voice_cb, 10)
            self.create_subscription(String, "/voice_state", self._voice_state_cb, 10)
            self.create_subscription(String, "/tts_event", self._tts_cb, 10)
            self.create_subscription(String, "/video_player/status", self._video_status_cb, 10)
            # 视频切换控制发布器（前端 POST /api/video/switch → 这里转发到 ROS2 话题）
            self._video_control_pub = self.create_publisher(String, "/video_player/control", 10)
        except ImportError:
            self.get_logger().warn("Messages not built yet, running without subscriptions")

        # Flask
        app = Flask(__name__, static_folder=frontend_dir)
        self._frontend_dir = frontend_dir

        @app.route("/")
        def index():
            return send_from_directory(frontend_dir, "index.html")

        @app.route("/<path:filename>")
        def static_files(filename):
            return send_from_directory(frontend_dir, filename)

        @app.route("/video_feed")
        def video_feed():
            """MJPEG 流端点：从 ROS2 /annotated_image 话题获取最新帧。使用 UMat GPU 加速编码。"""
            use_umat = cv2.ocl.haveOpenCL() and cv2.ocl.useOpenCL()

            def generate():
                last_sent_id = None
                while True:
                    with self._frame_lock:
                        frame = self._latest_frame
                        frame_id = self._latest_frame_id
                    if frame is None:
                        time.sleep(0.02)
                        continue
                    if frame_id == last_sent_id:
                        time.sleep(0.033)
                        continue
                    last_sent_id = frame_id
                    if use_umat:
                        umat = cv2.UMat(frame)
                        ret, buf = cv2.imencode(
                            '.jpg', umat,
                            [cv2.IMWRITE_JPEG_QUALITY, 65],
                        )
                    else:
                        ret, buf = cv2.imencode(
                            '.jpg', frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 65],
                        )
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' +
                               buf.tobytes() + b'\r\n')
                    time.sleep(0.033)

            return Response(
                generate(),
                mimetype='multipart/x-mixed-replace; boundary=frame',
            )

        @app.route("/api/state")
        def state():
            return jsonify({
                "zones": self._zones_to_dict(),
                "detections": self._dets_to_list(),
                "scan": self._latest_scan_points,
                "scan_ld14p": self._latest_ld14p_points,
                "voice": self._latest_voice,
                "tts": self._latest_tts,
            })

        @app.route("/api/events")
        def events():
            def generate():
                import queue
                q = queue.Queue()
                with self._sse_lock:
                    self._sse_clients.append(q)
                try:
                    while True:
                        try:
                            data = q.get(timeout=30)
                            yield f"data: {data}\n\n"
                        except Exception:
                            yield ": keepalive\n\n"
                except GeneratorExit:
                    pass
                finally:
                    with self._sse_lock:
                        if q in self._sse_clients:
                            self._sse_clients.remove(q)
            return Response(generate(), mimetype="text/event-stream")

        @app.route("/api/weather")
        def weather():
            import urllib.request
            try:
                url = "https://wttr.in/Kunming?format=%C+%t&lang=zh"
                req = urllib.request.Request(url, headers={"User-Agent": "curl/7.68.0"})
                with urllib.request.urlopen(req, timeout=5) as resp:
                    text = resp.read().decode().strip()
                    return jsonify({"weather": text})
            except Exception as e:
                logger.warning("Weather fetch failed: %s", e)
                return jsonify({"weather": "昆明 --"})

        @app.route("/api/system")
        def system_info():
            return jsonify(_system_info_impl())

        @app.route("/api/video/switch", methods=["POST"])
        def video_switch():
            """前端切换视频：POST JSON {"action": "next"|"prev", "file": "xxx.mp4"}."""
            from flask import request
            try:
                data = request.get_json(silent=True) or {}
                msg = String()
                msg.data = json.dumps(data, ensure_ascii=False)
                self._video_control_pub.publish(msg)
                return jsonify({"ok": True, "echo": data})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500

        self._app = app
        port = self.get_parameter("web_port").value
        threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=port, threaded=True, debug=False),
            daemon=True,
        ).start()
        self.get_logger().info(f"WebNode started, http://localhost:{port}")

    def _image_cb(self, msg):
        """接收 YOLO 标注后的图像。"""
        if self._bridge is None or cv2 is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_id = id(frame)
        except Exception as e:
            logger.warning("Image conversion failed in _image_cb: %s", e)

    def _zones_cb(self, msg):
        self._latest_zones = msg
        self._push_sse("zones", self._zones_to_dict())

    def _dets_cb(self, msg):
        self._latest_detections = msg.detections
        self._push_sse("detections", self._dets_to_list())

    def _scan_cb(self, msg):
        now = time.time()
        if now - self._last_scan_push_at < 0.2:
            return
        self._last_scan_push_at = now

        from ladar_ai.scan_to_points import scan_to_points
        # N10P 原始 5400 点/帧（角分辨率 0.067°），保留更多点以保证边缘清晰度
        points = scan_to_points(msg, max_points=2160)
        self._latest_scan_points = points
        self._push_sse("scan", points, "n10p")

    def _scan_ld14p_cb(self, msg):
        now = time.time()
        if now - self._last_ld14p_push_at < 0.2:
            return
        self._last_ld14p_push_at = now

        from ladar_ai.scan_to_points import scan_to_points
        # LD14P 原始 667 点/帧，默认 max_points=720 不触发降采样
        points = scan_to_points(msg)
        self._latest_ld14p_points = points
        self._push_sse("scan", points, "ld14p")

    def _voice_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"action": "unknown", "text": msg.data}
        self._latest_voice = data
        self._push_sse("voice", data)

    def _voice_state_cb(self, msg):
        """录音状态变化（idle/listening/processing）+ 截止时间，推送给前端倒计时。"""
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"state": "idle", "deadline": 0, "remaining": 0}
        self._push_sse("voice_state", data)

    def _tts_cb(self, msg):
        text = msg.data
        if text.startswith("speaking:"):
            text = text[len("speaking:"):]
        self._latest_tts = text
        self._push_sse("tts", {"text": text})

    def _video_status_cb(self, msg):
        """video_player 推送当前播放状态（当前文件/索引/视频列表），转发给前端。"""
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"current": msg.data, "videos": []}
        self._push_sse("video_status", data)

    def _push_sse(self, event_type, data, source=None):
        payload_dict = {"type": event_type, "data": data}
        if source is not None:
            payload_dict["source"] = source
        payload = json.dumps(payload_dict, ensure_ascii=False)
        with self._sse_lock:
            dead = []
            for q in self._sse_clients:
                try:
                    q.put_nowait(payload)
                except Exception:
                    dead.append(q)
            for q in dead:
                if q in self._sse_clients:
                    self._sse_clients.remove(q)

    def _zones_to_dict(self):
        if self._latest_zones is None:
            return {}
        return {
            "front_left": self._latest_zones.front_left,
            "front": self._latest_zones.front,
            "front_right": self._latest_zones.front_right,
            "right": self._latest_zones.right,
            "rear_right": self._latest_zones.rear_right,
            "rear": self._latest_zones.rear,
            "rear_left": self._latest_zones.rear_left,
            "left": self._latest_zones.left,
        }

    def _dets_to_list(self):
        return [
            {"class": d.class_name, "confidence": d.confidence,
             "distance": d.distance, "extra": d.extra_info}
            for d in self._latest_detections
        ]


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
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
