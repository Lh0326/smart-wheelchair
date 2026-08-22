"""Tab 1:自主导航 — QWebEngineView 嵌入 Leaflet 地图。"""
import os
import shutil
import time

from PyQt5.QtCore import QUrl, QTimer
from PyQt5.QtWebEngineWidgets import QWebEnginePage, QWebEngineProfile, QWebEngineView


class NavigationTab(QWebEngineView):
    """Tab 1:加载 http://localhost:8000/nav/index.html。

    Web 内容由 rtk frontend(static_server.py)提供,100% 复用现有 app.js。

    防御层:renderProcessTerminated 信号触发时延迟 500ms 自动 reload。
    在 XWayland + Intel Arc 环境下,渲染进程偶发崩溃(尤其页面刚加载 + JS
    启动密集时),自动 reload 可避免永久黑屏直到下次重启。

    缓存策略：启动时清 HTTP 缓存 + URL 带 cache-busting 时间戳，
    确保每次启动都拉最新 HTML/JS/CSS（QtWebEngine 默认缓存激进，
    否则前端代码改动用户看不到）。
    """

    BASE_URL = "http://localhost:8000/nav/index.html"

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self._ros = ros_node
        self.renderProcessTerminated.connect(self._on_render_terminated)

        # 彻底清掉 QtWebEngine 硬盘缓存（避免改前端代码后看不到变化）。
        # setHttpCacheType(NoCache) 只控内存策略，硬盘缓存目录仍可能存旧 HTML。
        # 启动时 rm -rf 硬盘缓存目录，强制下次启动重新下载。
        for cache_dir in [
            os.path.expanduser('~/.cache/QtWebEngine'),
            os.path.expanduser('~/.local/share/QtWebEngine'),
        ]:
            try:
                if os.path.isdir(cache_dir):
                    shutil.rmtree(cache_dir, ignore_errors=True)
            except Exception:
                pass

        # 内存层：禁用 HTTP 缓存 + 清当前内存缓存
        try:
            profile = QWebEngineProfile.defaultProfile()
            profile.setHttpCacheType(QWebEngineProfile.NoCache)
            profile.clearHttpCache()
        except Exception:
            pass

        # URL 加 cache-busting 时间戳，双保险绕过任何残留缓存层
        url = f"{self.BASE_URL}?_t={int(time.time())}"
        self.load(QUrl(url))

    def _on_render_terminated(self, _status, _exit_code):
        """渲染进程崩溃后 500ms 自动 reload(给 zygote/sandbox 清理时间)。"""
        QTimer.singleShot(500, lambda: self.reload())
