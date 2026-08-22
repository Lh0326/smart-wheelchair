"""TTS ROS2 节点：接收 TTSRequest 消息并调用 TTS 引擎播报。"""
import os
import logging
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ladar_ai.msg import TTSRequest
except ImportError:
    TTSRequest = None

from ladar_ai.tts_engine import TTSEngine

logger = logging.getLogger(__name__)


class TTSNode(Node):
    """订阅 /tts_request，调用 TTSEngine 播报，发布 /tts_event 状态。"""

    def __init__(self):
        super().__init__("tts_node")

        # 推导模型目录：src/ladar_ai/../../models
        pkg_dir = os.path.dirname(os.path.dirname(__file__))
        model_dir = os.path.join(pkg_dir, "..", "models")
        model_dir = os.path.abspath(model_dir)

        # 初始化 TTS 引擎
        self._engine = TTSEngine(model_dir, on_speak_complete=self._on_speak_complete)

        # 延迟导入消息类型
        from ladar_ai.msg import TTSRequest as TTSReqMsg

        self._sub = self.create_subscription(
            TTSReqMsg, "/tts_request", self._tts_callback, 10
        )
        self._pub_event = self.create_publisher(String, "/tts_event", 10)

        engine_type = "Kokoro (高质量本地)" if self._engine._tts is not None else (
            "Piper (轻量本地)" if (
                self._engine._piper_engine is not None
                and self._engine._piper_engine.is_loaded()
            ) else "None"
        )
        self.get_logger().info(f"TTSNode started, engine={engine_type}")

        # USB Hub OCP 错峰：启动时即预热 Kokoro ONNX 推理图。
        # 首次 generate 会触发 ONNX 计算图编译 + 内存分配（CPU/内存峰值），
        # 把这个峰值从"用户点终点瞬间"提前到"节点启动阶段"，避免和电机 inrush、
        # TEB 启动、networkx 规划叠加触发 Hub OCP。仅推理 1 字符不播放声音。
        if self._engine._tts is not None:
            try:
                t0 = self.get_clock().now().nanoseconds * 1e-9
                self._engine._tts.generate("。", sid=self._engine.speaker_id, speed=1.0)
                dt = (self.get_clock().now().nanoseconds * 1e-9 - t0) * 1000.0
                self.get_logger().info(f"TTS 模型预热完成 ({dt:.0f}ms)")
            except Exception as e:
                self.get_logger().warn(f"TTS 模型预热失败: {e}")

    def _on_speak_complete(self):
        """TTS 播报完成回调，发布 finished 事件。"""
        event = String()
        event.data = "finished"
        self._pub_event.publish(event)

    def _tts_callback(self, msg):
        text = msg.text
        priority = msg.priority
        self.get_logger().info(f"TTS 请求: priority={priority} text={text[:50]}")

        if text == "__stop__":
            self._engine.stop_speaking()
            event = String()
            event.data = "stopped"
            self._pub_event.publish(event)
            return

        self._engine.speak(text, priority=priority)

        # 发布事件
        event = String()
        event.data = f"speaking:{text[:50]}"
        self._pub_event.publish(event)


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
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
