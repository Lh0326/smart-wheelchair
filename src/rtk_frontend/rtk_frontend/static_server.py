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

#!/usr/bin/env python3
"""前端静态文件 HTTP 服务（基于 Python 标准库）

提供 Leaflet 前端 HTML/CSS/JS/POI GeoJSON 的 HTTP 服务。
- 默认端口 8000
- 自动定位到 share/rtk_frontend/frontend 目录（ROS2 安装路径）
- 启用 CORS 允许跨域请求（瓦片从 mbtiles-server 拉取）
"""
import http.server
import os
import socketserver
import threading

import rclpy
from rclpy.node import Node


class Handler(http.server.SimpleHTTPRequestHandler):
    """带 CORS 支持的 HTTP 请求处理器"""

    # 路径前缀 → 实际目录映射(直接指向 src,开发时无需 rebuild)
    PATH_MAP = {
        '/nav/': _WS_ROOT + '/src/wheelchair_app/web/nav/',
        '/companion/': _WS_ROOT + '/src/wheelchair_app/web/companion/',
        '/shared/': _WS_ROOT + '/src/wheelchair_app/web/shared/',
    }

    def __init__(self, *args, directory=None, **kwargs):
        # 不再依赖调用方传 directory,改由 send_head 内根据 self.path 选
        super().__init__(*args, directory=directory or os.getcwd(), **kwargs)

    def translate_path(self, path):
        """根据 URL path 前缀选择 directory(PATH_MAP),并 strip 前缀后 join。

        例:GET /nav/index.html → directory=PATH_MAP['/nav/'],path='index.html'。
        """
        for prefix, dir_path in self.PATH_MAP.items():
            if path.startswith(prefix):
                self.directory = dir_path
                # 去掉前缀,只保留相对文件名
                path = '/' + path[len(prefix):]
                break
        else:
            try:
                self.directory = find_frontend_dir()
            except FileNotFoundError:
                pass  # 保留 cwd(默认 __init__ 设的 os.getcwd())
        return super().translate_path(path)

    def end_headers(self):
        # 允许跨域（瓦片从 mbtiles-server 拉取）
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

    def log_message(self, format, *args):
        # 静默默认日志（避免刷屏），仅错误用 log_error
        pass


def find_frontend_dir() -> str:
    """定位前端静态文件目录（安装模式 vs 开发模式）

    优先用 ament 的 get_package_share_directory（安装模式），
    回退到源码同级 frontend 目录（开发模式）。
    """
    from ament_index_python.packages import get_package_share_directory
    candidates = [
        # 1. 安装模式：share/rtk_frontend/frontend
        os.path.join(get_package_share_directory('rtk_frontend'), 'frontend'),
        # 2. 开发模式：源码目录（本文件所在目录的上一级 frontend）
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'frontend')),
    ]
    for path in candidates:
        if os.path.isdir(path) and os.path.exists(os.path.join(path, 'index.html')):
            return path
    raise FileNotFoundError(
        f"前端目录未找到，尝试过：{candidates}"
    )


def main():
    rclpy.init()
    node = Node('frontend_static_server')
    node.declare_parameter('port', 8000)
    node.declare_parameter('host', '0.0.0.0')

    port = int(node.get_parameter('port').value)
    host = node.get_parameter('host').value

    try:
        frontend_dir = find_frontend_dir()
    except FileNotFoundError as e:
        node.get_logger().warning(f"frontend 安装目录未找到(根路径 / 兜底不可用):{e}")
        frontend_dir = None

    # 不传 directory:让 Handler.PATH_MAP 根据 URL path 选择 /nav/ /companion/ /shared/,
    # 未命中前缀的根路径由父类 SimpleHTTPRequestHandler 回退到 cwd
    handler = Handler

    try:
        httpd = socketserver.TCPServer((host, port), handler)
    except OSError as e:
        node.get_logger().error(f"端口 {port} 启动失败：{e}")
        rclpy.shutdown()
        return

    node.get_logger().info(f'前端静态服务: http://{host}:{port} (dir={frontend_dir})')

    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    finally:
        httpd.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
