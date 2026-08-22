"""视频回放节点：从本地视频文件读帧，发布到 /camera/color/image_raw。
import os

用途：用路况视频素材替代 Orbbec 真实相机，测试 YOLO 检测算法
在真实道路场景下对红绿灯/车辆/行人的识别能力。

用法：
    ros2 run ladar_ai video_player_node --ros-args -p video_path:=/path/to/video.mp4
    ros2 run ladar_ai video_player_node --ros-args -p video_dir:=<视频目录>

特性：
- 默认从 video_dir 选第一个 mp4 文件
- 解码后立即 resize 到 target_width x target_height（默认 640x480），大幅提升 FPS
- 循环播放（视频结束后从头开始）
- 支持通过 /video_player/control 话题手动切换视频（next/prev/指定文件名）
- 不发布深度图，camera_detect_node 会自动 fallback 到 bbox 高度估算
"""
import os
import glob
import json
import logging
import signal
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

logger = logging.getLogger(__name__)

_DEFAULT_VIDEO_DIR = os.environ.get("VIDEO_DIR", "~/videos")
_DEFAULT_FPS = 30.0
_DEFAULT_LONG_SIDE = 1280  # 长边缩放到此值，保持宽高比（小目标仍可分辨）


class VideoPlayerNode(Node):
    """视频回放节点，替代 Orbbec 真实相机驱动。"""

    def __init__(self):
        super().__init__("video_player_node")

        self.declare_parameter("video_path", "")
        self.declare_parameter("video_dir", _DEFAULT_VIDEO_DIR)
        self.declare_parameter("fps", _DEFAULT_FPS)
        self.declare_parameter("loop", True)
        self.declare_parameter("long_side", _DEFAULT_LONG_SIDE)  # 长边目标尺寸（保持宽高比）

        if cv2 is None or CvBridge is None:
            self.get_logger().error("cv2 或 cv_bridge 未安装，video_player 不可用")
            return

        self._bridge = CvBridge()

        video_dir = self.get_parameter("video_dir").value
        fps = self.get_parameter("fps").value
        self._loop = self.get_parameter("loop").value
        self._long_side = int(self.get_parameter("long_side").value)

        # 扫描视频目录，得到所有可选文件（按文件名排序）
        self._video_dir = video_dir
        self._mp4_files = sorted(glob.glob(os.path.join(video_dir, "*.mp4")))
        if not self._mp4_files:
            self.get_logger().error(f"video_dir 中没有 mp4 文件: {video_dir}")
            return

        # 选定当前视频
        video_path = self.get_parameter("video_path").value
        if video_path:
            if video_path not in self._mp4_files:
                # 用户传的是文件名而非完整路径，尝试匹配
                matches = [f for f in self._mp4_files if os.path.basename(f) == video_path or f == video_path]
                if matches:
                    video_path = matches[0]
                else:
                    self.get_logger().warning(f"video_path 不在目录中: {video_path}，使用第一个")
                    video_path = self._mp4_files[0]
        else:
            video_path = self._mp4_files[0]

        self._current_idx = self._mp4_files.index(video_path) if video_path in self._mp4_files else 0
        self._cap = None
        self._cap_lock = threading.Lock()
        self._open_video(video_path)

        # 发布器
        self._pub_image = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self._pub_status = self.create_publisher(String, "/video_player/status", 10)

        # 控制订阅：接收 next/prev/switch 指令
        self._sub_control = self.create_subscription(
            String, "/video_player/control", self._control_cb, 10
        )

        # 定时器控制发布频率
        interval = 1.0 / fps if fps > 0 else 0.1
        self._timer = self.create_timer(interval, self._publish_frame)

        # 状态发布定时器（2Hz，告诉前端当前播放的视频）
        self._status_timer = self.create_timer(0.5, self._publish_status)

        self._frame_count = 0

    def _open_video(self, video_path):
        """打开指定视频文件。线程安全（调用方持有 _cap_lock）。"""
        if self._cap is not None:
            self._cap.release()
        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            self.get_logger().error(f"无法打开视频文件: {video_path}")
            return False

        # 减少 OpenCV 内部 buffer（让 read 尽快返回最新帧）
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        total_frames = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        src_fps = self._cap.get(cv2.CAP_PROP_FPS)
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 计算需要跳过的帧数：让视频按原速播放（60FPS源 + 30FPS发布 → 跳1帧）
        target_fps = self.get_parameter("fps").value
        self._frames_to_skip = max(0, int(round(src_fps / target_fps)) - 1) if target_fps > 0 and src_fps > 0 else 0

        self.get_logger().info(
            f"打开视频: {os.path.basename(video_path)}\n"
            f"  原始: {width}x{height}@{src_fps:.1f}FPS, 总帧数: {total_frames}\n"
            f"  发布: 长边缩放到 {self._long_side}（保持宽高比）@{target_fps:.1f}FPS, 每帧跳 {self._frames_to_skip} 帧"
        )
        return True

    def _control_cb(self, msg):
        """接收前端切换指令。

        消息格式（JSON 字符串或纯文本）：
        - "next"：下一个视频
        - "prev"：上一个视频
        - 文件名（如 "video_20260523_131313.mp4"）：直接切换
        """
        raw = msg.data.strip()
        action = ""
        target_file = ""
        try:
            payload = json.loads(raw)
            action = payload.get("action", "")
            target_file = payload.get("file", "")
        except (json.JSONDecodeError, AttributeError):
            # 纯文本
            if raw in ("next", "prev"):
                action = raw
            else:
                target_file = raw

        with self._cap_lock:
            old_idx = self._current_idx
            if action == "next":
                self._current_idx = (self._current_idx + 1) % len(self._mp4_files)
            elif action == "prev":
                self._current_idx = (self._current_idx - 1) % len(self._mp4_files)
            elif target_file:
                matches = [f for f in self._mp4_files
                           if os.path.basename(f) == target_file or f == target_file]
                if matches:
                    self._current_idx = self._mp4_files.index(matches[0])
                else:
                    self.get_logger().warning(f"未找到视频文件: {target_file}")
                    return
            else:
                return

            if self._current_idx != old_idx:
                new_path = self._mp4_files[self._current_idx]
                self._frame_count = 0
                self._open_video(new_path)
                self._publish_status()

    def _publish_frame(self):
        with self._cap_lock:
            if self._cap is None or not self._cap.isOpened():
                return

            ret, frame = self._cap.read()
            if not ret:
                if self._loop:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                else:
                    self._timer.cancel()
                return

            # 跳帧保持视频原速（60FPS源 + 30FPS发布 → 每帧跳1帧）
            for _ in range(getattr(self, "_frames_to_skip", 0)):
                if not self._cap.read()[0]:
                    break

            # 保持宽高比缩放：长边缩到 _long_side（保留小目标细节，避免红绿灯被压没）
            h, w = frame.shape[:2]
            scale = self._long_side / max(w, h)
            if scale < 1.0:
                new_w = max(1, int(round(w * scale)))
                new_h = max(1, int(round(h * scale)))
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_color_optical_frame"
            self._pub_image.publish(img_msg)
            self._frame_count += 1
        except Exception as e:
            self.get_logger().warning(f"帧发布失败: {e}")

    def _publish_status(self):
        """2Hz 发布当前播放状态，供前端显示视频列表和当前选中。"""
        payload = {
            "current": os.path.basename(self._mp4_files[self._current_idx]) if self._mp4_files else "",
            "current_idx": self._current_idx,
            "total": len(self._mp4_files),
            "videos": [os.path.basename(f) for f in self._mp4_files],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_status.publish(msg)

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPlayerNode()
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
