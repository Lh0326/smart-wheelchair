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
"""
mbtiles 瓦片图 HTTP 服务。

提供：
    GET /tiles/{z}/{x}/{y}.png  → 瓦片图二进制
    GET /metadata                → JSON 元数据
    GET /health                  → 健康检查

mbtiles 文件用 TMS 坐标系存储（y 轴翻转），前端用 XYZ 坐标系请求。
"""
import argparse
import os
import sqlite3
from pathlib import Path

from flask import Flask, Response, jsonify, abort


class MbtilesReader:
    """读取 mbtiles 文件中的瓦片"""

    def __init__(self, mbtiles_path: str):
        if not os.path.exists(mbtiles_path):
            raise FileNotFoundError(f"mbtiles 文件不存在: {mbtiles_path}")
        self.path = mbtiles_path

    def _connect(self):
        # SQLite 连接不能跨线程，每次新建
        return sqlite3.connect(self.path)

    def get_metadata(self, name: str):
        with self._connect() as conn:
            c = conn.cursor()
            c.execute("SELECT value FROM metadata WHERE name=?", (name,))
            row = c.fetchone()
            return row[0] if row else None

    def get_all_metadata(self) -> dict:
        with self._connect() as conn:
            c = conn.cursor()
            c.execute("SELECT name, value FROM metadata")
            return dict(c.fetchall())

    def get_tile(self, z: int, x: int, y: int):
        """读取 TMS 坐标（tile_row）的瓦片

        注意：参数 y 是 mbtiles 内部存储的 TMS 行号（tile_row），
        不是前端 XYZ 坐标系的 y。HTTP 路由会先做 xyz_to_tms_y 转换。
        """
        with self._connect() as conn:
            c = conn.cursor()
            c.execute(
                "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                (z, x, y),
            )
            row = c.fetchone()
            return row[0] if row else None


def tms_to_xyz_y(tms_y: int, z: int) -> int:
    """TMS y 转 XYZ y"""
    return (2 ** z - 1) - tms_y


def xyz_to_tms_y(xyz_y: int, z: int) -> int:
    """XYZ y 转 TMS y"""
    return (2 ** z - 1) - xyz_y


def create_app(mbtiles_path: str) -> Flask:
    app = Flask(__name__)
    reader = MbtilesReader(mbtiles_path)
    app.config["mbtiles_reader"] = reader

    # 允许跨域（前端 localhost:8000 请求瓦片 localhost:8080）
    @app.after_request
    def add_cors_headers(resp):
        resp.headers["Access-Control-Allow-Origin"] = "*"
        return resp

    @app.route("/health")
    def health():
        return jsonify({"status": "ok", "mbtiles": mbtiles_path})

    @app.route("/metadata")
    def metadata():
        return jsonify(reader.get_all_metadata())

    @app.route("/tiles/<int:z>/<int:x>/<int:y>.png")
    def get_tile(z: int, x: int, y: int):
        # 前端用 XYZ y，mbtiles 存 TMS y，先转换再查
        tms_y = xyz_to_tms_y(y, z)
        tile_data = reader.get_tile(z, x, tms_y)
        if tile_data is None:
            abort(404)
        return Response(tile_data, mimetype="image/png")

    return app


def main():
    """主入口：优先使用 ROS2 参数，fallback 到 argparse"""
    try:
        import rclpy
        from rclpy.node import Node as RosNode

        rclpy.init()
        node = RosNode('mbtiles_server_launcher')
        node.declare_parameter('mbtiles', _WS_ROOT + '/data/region.mbtiles')
        node.declare_parameter('host', '0.0.0.0')
        node.declare_parameter('port', 8080)

        mbtiles = node.get_parameter('mbtiles').value
        host = node.get_parameter('host').value
        port_value = node.get_parameter('port').value
        # port 可能是 int 或 str（launch 传入），统一转 int
        port = int(port_value)

        app = create_app(mbtiles)
        node.get_logger().info(f"启动: {mbtiles} → http://{host}:{port}")

        # Flask 在主线程跑，ROS2 spin 在另一个线程
        import threading
        flask_thread = threading.Thread(
            target=app.run,
            kwargs={'host': host, 'port': port, 'debug': False, 'use_reloader': False},
            daemon=True,
        )
        flask_thread.start()

        rclpy.spin(node)
        rclpy.shutdown()
    except Exception as e:
        # Fallback：rclpy 不可用或直接运行，用 argparse
        print(f"[WARN] ROS2 模式失败 ({e})，回退到 argparse 模式")
        parser = argparse.ArgumentParser()
        parser.add_argument("--mbtiles", default="data/region.mbtiles")
        parser.add_argument("--host", default="0.0.0.0")
        parser.add_argument("--port", type=int, default=8080)
        parser.add_argument("--debug", action="store_true")
        args = parser.parse_args()

        app = create_app(args.mbtiles)
        print(f"[mbtiles-server] 启动: {args.mbtiles} → http://{args.host}:{args.port}")
        app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
