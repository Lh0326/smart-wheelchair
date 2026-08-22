"""Tab 2:小智陪伴 — QWebEngineView 嵌入 companion web。

网页由 src/wheelchair_app/web/companion/{index.html,app.js,styles.css} 提供,
经 rtk_frontend/static_server.py 在 :8000 暴露。

Web 内容:
  - 左:相机视频流(web_video_server MJPEG,端口 8085)
  - 右上:三色点云融合(N10P/LD14P/Gemini,Canvas + roslibjs)
  - 右下:ASR 语音识别(麦克风状态 + 历史日志)
  - 底部:硬件监控条(CPU/GPU/NPU/内存/Load)

防御层:renderProcessTerminated 信号触发时延迟 500ms 自动 reload
(详见 navigation_tab.py 同名方法)。
"""
from PyQt5.QtCore import QUrl, QTimer
from PyQt5.QtWebEngineWidgets import QWebEngineView


class CompanionTab(QWebEngineView):
    """Tab 2:加载 http://localhost:8000/companion/index.html。"""

    URL = "http://localhost:8000/companion/index.html"

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self._ros = ros_node
        self.renderProcessTerminated.connect(self._on_render_terminated)

        self.load(QUrl(self.URL))

    def _on_render_terminated(self, _status, _exit_code):
        """渲染进程崩溃后 500ms 自动 reload(给 zygote/sandbox 清理时间)。"""
        QTimer.singleShot(500, lambda: self.reload())
