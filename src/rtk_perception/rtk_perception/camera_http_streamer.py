"""轻量 HTTP JPEG 流节点：突破 web_video_server snapshot 的 10 FPS 上限。

背景:
  web_video_server 的 /snapshot 端点每次请求都会等下一帧 + 重新 JPEG 编码,
  实测串行 30 次请求耗时 3008ms (100ms/帧 = 10 FPS 上限),
  无法满足前端 30 FPS fetch 间隔。

策略:
  订阅 /camera/color/image_raw, 在订阅回调中按相机源频率(~15Hz)做一次 JPEG 编码,
  把最新 JPEG 缓存到内存; HTTP 端点(默认 8086)直接返回缓存字节, 不等待不重编码。
  这样:
    - JPEG 编码次数 = 相机源帧率(而非 fetch 次数), CPU 成本固定
    - 前端 33ms fetch 即可拿到一帧(内容刷新率 = 源帧率, fetch 多余时返回相同帧)
    - 浏览器 FPS 显示 = fetch 频率(可达 30)
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise SystemExit(f'cv_bridge 不可用: {e}')

try:
    import cv2
except ImportError as e:
    raise SystemExit(f'cv2 不可用: {e}')


class CameraHttpStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_http_streamer')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('port', 8086)
        self.declare_parameter('jpeg_quality', 80)

        topic = self.get_parameter('image_topic').value
        port = int(self.get_parameter('port').value)
        quality = int(self.get_parameter('jpeg_quality').value)

        self._bridge = CvBridge()
        self._quality = quality
        self._latest_jpeg = bytes()
        self._lock = threading.Lock()
        self._frame_count = 0

        self.create_subscription(Image, topic, self._on_image, 10)

        # 单进程多线程 HTTP, 每个请求独立线程, 立即返回缓存
        self._httpd = ThreadingHTTPServer(('0.0.0.0', port), self._make_handler())
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

        self.get_logger().info(
            f'camera_http_streamer ready: 订阅 {topic}, HTTP 端口 {port}, JPEG quality {quality}'
        )

    def _on_image(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, jpg = cv2.imencode(
                '.jpg', cv_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._quality],
            )
            if not ok:
                return
            with self._lock:
                self._latest_jpeg = jpg.tobytes()
                self._frame_count += 1
        except Exception as e:
            self.get_logger().warn(f'JPEG 编码失败: {e}')

    def _make_handler(self):
        node = self
        lock = self._lock

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                with lock:
                    jpg = node._latest_jpeg
                if not jpg:
                    self.send_response(503)
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpg)))
                self.send_header('Cache-Control', 'no-store')
                # 前端页面在 :8000,本服务在 :8086,跨域必须显式允许
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                try:
                    self.wfile.write(jpg)
                except BrokenPipeError:
                    pass

            def do_OPTIONS(self):
                # 预检请求(若浏览器发起)
                self.send_response(204)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', '*')
                self.end_headers()

            def log_message(self, *args, **kwargs):
                pass

        return Handler

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraHttpStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
