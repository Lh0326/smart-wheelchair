"""voice_command → tts_request 桥接节点。

订阅 /voice_command(JSON String),解析 action + text,
调用 tts_node 发布 /tts_request,实现语音反馈。

例如:
  /voice_command: {"action":"wakeup","text":"小智你好"}
  → /tts_request: text="唤醒成功" priority=2
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ladar_ai.msg import TTSRequest
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False
    TTSRequest = None


# action → TTS 反馈文本
ACTION_FEEDBACK = {
    'wakeup': '我在',
    'stop': '已停止',
    'emergency_stop': '紧急停止',
    'speed_up': '已加速',
    'speed_down': '已减速',
}


class VoiceToTTSBridge(Node):
    def __init__(self):
        super().__init__('voice_to_tts_bridge')

        if not TTS_AVAILABLE:
            self.get_logger().error('ladar_ai.msg.TTSRequest 不可用,无法桥接')
            return

        self._sub_voice = self.create_subscription(
            String, '/voice_command', self._on_voice, 10
        )
        self._pub_tts = self.create_publisher(TTSRequest, '/tts_request', 10)
        self.get_logger().info('voice_to_tts_bridge 启动,等 voice_command...')

    def _on_voice(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'非 JSON voice_command: {msg.data}')
            return

        action = cmd.get('action', '')
        text = cmd.get('text', '')

        # 生成 TTS 反馈
        if action == 'wakeup':
            tts_text = '我在'  # 唤醒后简洁回应
            priority = 2
        elif action == 'query' and text:
            tts_text = f'收到:{text}'  # 询问类,回声确认
            priority = 2
        elif action in ACTION_FEEDBACK:
            tts_text = ACTION_FEEDBACK[action]
            priority = 1  # 控制类指令高优先级
        else:
            return  # 未知 action,不播报

        req = TTSRequest()
        req.text = tts_text
        req.priority = priority
        self._pub_tts.publish(req)
        self.get_logger().info(f'voice→tts: action={action} → "{tts_text}"')


def main():
    rclpy.init()
    node = VoiceToTTSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
