"""核心融合决策节点。

融合 8 方位雷达距离和前方 YOLO 检测结果，
根据播报策略生成 TTS 请求，同时发布系统状态。
"""
import time
import json
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ladar_ai.utils import num_to_chinese, direction_to_chinese, ZONE_NAMES

try:
    from geometry_msgs.msg import Twist
except ImportError:
    Twist = None


def voice_action_to_twist(action: str):
    """把语音动作名翻译为 Twist。未知动作返回零 Twist。"""
    if Twist is None:
        return None
    t = Twist()
    if action == "forward":
        t.linear.x = 0.4
    elif action == "back":
        t.linear.x = -0.2
    elif action == "left":
        t.angular.z = 0.6
    elif action == "right":
        t.angular.z = -0.6
    # stop / emergency_stop / unknown → 全零 Twist
    return t


_BROADCAST_TEMPLATES = {
    ("person", ""): "前方{dist}米有行人",
    ("traffic_light", "ped_red"): "前方红灯，请停车",
    ("traffic_light", "ped_green"): "前方绿灯，可以通行",
    ("traffic_light", "ped_yellow"): "前方黄灯，请注意",
    # 兼容旧版 camera_detect_node 的 red/green/yellow 字段
    ("traffic_light", "red"): "前方红灯，请注意",
    ("traffic_light", "green"): "前方绿灯",
    ("traffic_light", "yellow"): "前方黄灯，请注意",
    ("car", ""): "前方{dist}米有汽车",
    ("bus", ""): "前方{dist}米有公交车",
    ("truck", ""): "前方{dist}米有卡车",
    ("bicycle", ""): "前方{dist}米有自行车",
    ("motorcycle", ""): "前方{dist}米有摩托车",
}

_URGENT_TEMPLATE = "{direction}{dist}米有障碍物，请小心"
_DEFAULT_TEMPLATE = "{direction}{dist}米有障碍物"
_WAKEUP_PROMPT = "我在"
_ASR_TIMEOUT_PROMPT = "没听清，请再说"
_ASR_UNRECOGNIZED_PROMPT = "抱歉，我无法回复这个问题"
_INTERACTION_HOLD_SEC = 10.0
_WAKEUP_COOLDOWN_SEC = 3.0  # 唤醒去重：3 秒内的重复唤醒忽略，避免连说两次"小智你好"导致重复播报"我在"

_CLASS_CN_UNIT = {
    "person": ("行人", "名"),
    "car": ("汽车", "辆"),
    "bus": ("公交车", "辆"),
    "truck": ("卡车", "辆"),
    "bicycle": ("自行车", "辆"),
    "motorcycle": ("摩托车", "辆"),
    "traffic_light": ("人行道红绿灯", "个"),
}

_PED_LIGHT_TEXT = {
    "ped_red": "前方红灯，请停车",
    "ped_green": "前方绿灯，可以通行",
    "ped_yellow": "前方黄灯，请注意",
    # 兼容旧版 camera_detect_node 的 red/green/yellow 字段
    "red": "前方红灯，请停车",
    "green": "前方绿灯，可以通行",
    "yellow": "前方黄灯，请注意",
}

# 所有红绿灯（含车行灯 road_*）都触发播报——用户希望及时反应
_ALL_LIGHT_TEXT = {
    "ped_red": "前方红灯，请停车",
    "ped_green": "前方绿灯，可以通行",
    "ped_yellow": "前方黄灯，请注意",
    "road_red": "前方红灯，请停车",
    "road_green": "前方绿灯，可以通行",
    "road_yellow": "前方黄灯，请注意",
    "red": "前方红灯，请停车",
    "green": "前方绿灯，可以通行",
    "yellow": "前方黄灯，请注意",
}


def _count_to_chinese(count: int) -> str:
    if count == 2:
        return "两"
    return num_to_chinese(count)


def generate_broadcast_text(zone, distance, class_name=None, extra_info="", urgent=False):
    dist_cn = num_to_chinese(round(distance, 1))
    direction_cn = direction_to_chinese(zone)

    if urgent:
        return _URGENT_TEMPLATE.format(direction=direction_cn, dist=dist_cn)

    if class_name and zone == "front":
        key = (class_name, extra_info or "")
        template = _BROADCAST_TEMPLATES.get(key)
        if template:
            return template.format(dist=dist_cn)

    return _DEFAULT_TEMPLATE.format(direction=direction_cn, dist=dist_cn)


def should_broadcast(zone, distance, history, cooldown=3.0, now=None, threshold=0.5):
    if now is None:
        now = time.time()
    if zone not in history:
        return True
    last_dist, last_time = history[zone]
    elapsed = now - last_time
    if elapsed >= cooldown:
        return True
    if abs(distance - last_dist) >= threshold:
        return True
    return False


class FusionDecisionNode(Node):
    def __init__(self):
        super().__init__("fusion_decision_node")

        self.declare_parameter("danger_threshold", 3.0)
        self.declare_parameter("urgent_threshold", 1.0)
        self.declare_parameter("broadcast_cooldown_sec", 15.0)
        self.declare_parameter("global_broadcast_cooldown_sec", 12.0)
        self.declare_parameter("distance_change_threshold", 0.5)

        self._zones = {name: 25.0 for name in ZONE_NAMES}
        self._front_detections = []
        self._broadcast_history = {}
        self._last_auto_broadcast_time = 0.0
        self._last_ped_light_state = ""
        self._last_ped_light_time = 0.0
        self._interaction_hold_until = 0.0
        self._voice_state = "idle"
        self._last_tts_text = ""
        self._tts_busy = False
        self._last_tts_send_time = 0.0
        self._last_wakeup_time = 0.0  # wakeup 去重：记录上次唤醒时间
        self._status_dirty = True  # publish once at startup

        self._zones_sub = self.create_subscription(
            # 延迟导入消息类型
            self._get_zone_msg_type(), "/lidar_zones", self._zones_cb, 10
        )
        self._det_sub = self.create_subscription(
            self._get_detections_msg_type(), "/front_detections", self._detections_cb, 10
        )
        self._voice_sub = self.create_subscription(
            String, "/voice_command", self._voice_cb, 10
        )
        self._tts_event_sub = self.create_subscription(
            String, "/tts_event", self._tts_event_cb, 10
        )

        self._tts_pub = None
        self._status_pub = None
        self._motion_pub = self.create_publisher(String, "/motion_command", 10)
        self._voice_intent_pub = (
            self.create_publisher(Twist, "/voice_motion_intent", 10)
            if Twist is not None else None
        )

        self._status_timer = self.create_timer(0.2, self._publish_status)
        self._auto_timer = self.create_timer(0.5, self._auto_broadcast_check)

        self.get_logger().info("FusionDecisionNode started")

    def _get_zone_msg_type(self):
        from ladar_ai.msg import ZoneDistances
        return ZoneDistances

    def _get_detections_msg_type(self):
        from ladar_ai.msg import Detections
        return Detections

    def _ensure_publishers(self):
        if self._tts_pub is None:
            from ladar_ai.msg import TTSRequest, SystemStatus
            self._tts_pub = self.create_publisher(TTSRequest, "/tts_request", 10)
            self._status_pub = self.create_publisher(SystemStatus, "/system_status", 10)

    def _zones_cb(self, msg):
        self._zones = {
            "front_left": msg.front_left,
            "front": msg.front,
            "front_right": msg.front_right,
            "right": msg.right,
            "rear_right": msg.rear_right,
            "rear": msg.rear,
            "rear_left": msg.rear_left,
            "left": msg.left,
        }
        self._status_dirty = True

    def _detections_cb(self, msg):
        self._front_detections = msg.detections
        self._status_dirty = True

    def _tts_event_cb(self, msg: String):
        if msg.data == "finished":
            self._tts_busy = False
        elif msg.data.startswith("speaking:"):
            self._tts_busy = True

    def _voice_cb(self, msg: String):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        action = command.get("action", "")
        self._voice_state = action or "idle"

        if action == "wakeup":
            # 去重：cooldown 内的重复唤醒忽略，避免连说两次"小智你好"导致重复播报"我在"
            now = time.time()
            elapsed_since_wakeup = now - self._last_wakeup_time
            if elapsed_since_wakeup < _WAKEUP_COOLDOWN_SEC:
                self.get_logger().debug(
                    f"忽略重复 wakeup（距上次 {elapsed_since_wakeup:.1f}s < cooldown {_WAKEUP_COOLDOWN_SEC}s）"
                )
                # 仍进入交互保持（防止自动播报打断），但不重新播报"我在"
                self._begin_interaction()
                self._voice_state = "listening"  # 标记为聆听中，期间禁止自动播报
                return
            self._last_wakeup_time = now
            self._begin_interaction()
            self._voice_state = "listening"  # 标记为聆听中，期间禁止自动播报
            self._send_tts("__stop__", priority=1, source=1)
            self._send_tts(_WAKEUP_PROMPT, priority=1, source=1)
            return
        if action == "timeout":
            self._begin_interaction()
            self._voice_state = "idle"
            self._send_tts("__stop__", priority=1, source=1)
            self._send_tts(_ASR_TIMEOUT_PROMPT, priority=1, source=1)
            return
        if action == "unrecognized":
            self._begin_interaction()
            self._voice_state = "idle"
            self._send_tts("__stop__", priority=1, source=1)
            self._send_tts(_ASR_UNRECOGNIZED_PROMPT, priority=1, source=1)
            return
        if action == "query":
            self._begin_interaction()
            self._send_tts("__stop__", priority=1, source=1)
            self._handle_query(command.get("direction") or "all")
            self._voice_state = "idle"
            return
        if action == "stop":
            self._begin_interaction()
            self._send_tts("__stop__", priority=1, source=1)
            self._voice_state = "idle"
            return
        if action in ("emergency_stop", "speed_up", "speed_down"):
            self._begin_interaction()
            self._send_tts("__stop__", priority=1, source=1)
            self._send_motion(action)
            replies = {
                "emergency_stop": "已紧急停车",
                "speed_up": "已加速",
                "speed_down": "已减速",
            }
            self._send_tts(replies[action], priority=1 if action == "emergency_stop" else 0, source=1)
            # 把动作翻译为 Twist，发布到 /voice_motion_intent 给避障节点
            if self._voice_intent_pub is not None:
                voice_twist_map = {
                    "emergency_stop": "stop",
                    "speed_up": "forward",
                    "speed_down": "forward",
                }
                mapped = voice_twist_map.get(action)
                if mapped:
                    self._voice_intent_pub.publish(voice_action_to_twist(mapped))
            self._voice_state = "idle"

    def _begin_interaction(self):
        self._interaction_hold_until = time.time() + _INTERACTION_HOLD_SEC
        # 重置定时播报计时器，防止 hold 结束后立即被插队
        self._last_auto_broadcast_time = time.time()

    def _front_context(self, fallback_dist):
        if not self._front_detections:
            return fallback_dist, None, ""

        valid = [
            det for det in self._front_detections
            if getattr(det, "distance", -1.0) > 0
            and not (det.class_name == "traffic_light" and det.extra_info not in _PED_LIGHT_TEXT)
        ]
        if valid:
            closest = min(valid, key=lambda det: det.distance)
            return closest.distance, closest.class_name, closest.extra_info

        # 没有有效视觉距离时仍使用雷达正前方距离播报目标类别。
        candidates = [
            det for det in self._front_detections
            if not (det.class_name == "traffic_light" and det.extra_info not in _PED_LIGHT_TEXT)
        ]
        if candidates:
            closest = max(candidates, key=lambda det: det.confidence)
            return fallback_dist, closest.class_name, closest.extra_info
        return fallback_dist, None, ""

    def _pedestrian_light_context(self):
        for det in self._front_detections:
            if det.class_name == "traffic_light" and det.extra_info in _PED_LIGHT_TEXT:
                return det.extra_info, _PED_LIGHT_TEXT[det.extra_info]
        return None, ""

    def _any_traffic_light_context(self):
        """任意红绿灯（人行灯 ped_* 或车行灯 road_*）的播报上下文。

        比 _pedestrian_light_context 范围更广，用于自动优先播报。
        """
        # 取置信度最高的红绿灯（可能有多个，选最显著的）
        best = None
        best_conf = 0.0
        for det in self._front_detections:
            if det.class_name == "traffic_light" and det.extra_info in _ALL_LIGHT_TEXT:
                if det.confidence > best_conf:
                    best = det
                    best_conf = det.confidence
        if best is not None:
            return best.extra_info, _ALL_LIGHT_TEXT[best.extra_info]
        return None, ""

    def _front_detection_summary(self, fallback_dist, direction="front"):
        label = direction_to_chinese(direction)
        if not self._front_detections:
            dist_cn = num_to_chinese(round(fallback_dist, 1))
            return f"{label}没有障碍物，{label}{dist_cn}米内安全"

        # 红绿灯只在 direction == "front" 时播报（侧前方红绿灯用户难看清）
        if direction == "front":
            ped_state, ped_text = self._pedestrian_light_context()
            if ped_state:
                return ped_text

        counts = {}
        valid_distances = []
        for det in self._front_detections:
            if det.class_name == "traffic_light" and not det.extra_info.startswith("ped_"):
                continue
            if det.class_name not in _CLASS_CN_UNIT:
                continue
            counts[det.class_name] = counts.get(det.class_name, 0) + 1
            if det.distance > 0:
                valid_distances.append(det.distance)

        if not counts:
            dist_cn = num_to_chinese(round(fallback_dist, 1))
            return f"{label}没有明确目标，{label}{dist_cn}米有障碍"

        dist = min(valid_distances) if valid_distances else fallback_dist
        parts = []
        for class_name, count in sorted(counts.items()):
            cls_label, unit = _CLASS_CN_UNIT[class_name]
            parts.append(f"{_count_to_chinese(count)}{unit}{cls_label}")
        dist_cn = num_to_chinese(round(dist, 1))
        return f"{label}检测到{'、'.join(parts)}，大约{dist_cn}米"

    def _effective_zones(self):
        zones = dict(self._zones)
        front_dist, _, _ = self._front_context(zones["front"])
        if 0 < front_dist < zones["front"]:
            zones["front"] = front_dist
        return zones

    def _handle_query(self, direction):
        """8 方位查询：前 3 方位（front/front_left/front_right）融合 YOLO；
        其他 5 方位仅播报雷达距离。"""
        if direction not in ZONE_NAMES:
            direction = "front"

        zones = self._effective_zones()
        dist = zones.get(direction, 25.0)
        label = direction_to_chinese(direction)

        if direction in ("front", "front_left", "front_right"):
            self._send_tts(self._front_detection_summary(dist, direction), priority=0, source=1)
            return

        urgent = dist < self.get_parameter("urgent_threshold").value
        if dist >= 25.0:
            self._send_tts(f"{label}没有障碍物", priority=0, source=1)
        else:
            dist_cn = num_to_chinese(round(dist, 1))
            template = _URGENT_TEMPLATE if urgent else _DEFAULT_TEMPLATE
            self._send_tts(template.format(direction=label, dist=dist_cn), priority=0, source=1)

    def _send_motion(self, action):
        msg = String()
        msg.data = json.dumps({"action": action, "source": "voice"}, ensure_ascii=False)
        self._motion_pub.publish(msg)

    def _auto_broadcast_check(self):
        danger = self.get_parameter("danger_threshold").value
        urgent = self.get_parameter("urgent_threshold").value
        cooldown = self.get_parameter("broadcast_cooldown_sec").value
        global_cooldown = self.get_parameter("global_broadcast_cooldown_sec").value
        threshold = self.get_parameter("distance_change_threshold").value
        now = time.time()
        zones = self._effective_zones()

        # 用户在 listening（唤醒后等指令）期间完全停止自动播报，避免干扰用户说指令
        # 此检查优先于 hold 机制，确保 listening 期间绝对不会播报
        if self._voice_state == "listening":
            return

        if now < self._interaction_hold_until:
            return

        # ---- 优先级 2: 红绿灯识别（人行灯+车行灯都触发，打断当前播报） ----
        light_state, light_text = self._any_traffic_light_context()
        if light_state and now - self._last_auto_broadcast_time >= global_cooldown:
            if light_state != self._last_ped_light_state or now - self._last_ped_light_time >= cooldown:
                self._send_tts_immediate(light_text, priority=1, source=0)
                self._last_ped_light_state = light_state
                self._last_ped_light_time = now
                self._last_auto_broadcast_time = now
                return

        # TTS 正在生成/播报时跳过低优先级播报
        if self._tts_busy and now - self._last_tts_send_time < 5.0:
            return

        # ---- 优先级 3: 定时障碍物播报（不打断） ----
        min_zone = min(zones, key=zones.get)
        min_dist = zones[min_zone]

        if min_dist >= danger:
            return

        is_urgent = min_dist < urgent

        class_name, extra = None, ""
        if min_zone == "front":
            # "谁最近谁优先"：比较 YOLO 最近距离 vs 雷达正前方距离
            yolo_dist, yolo_class, yolo_extra = self._front_context(min_dist)
            radar_dist = self._zones.get("front", 25.0)
            if 0 < yolo_dist < radar_dist:
                # YOLO 胜出（相机看到更近的物体）：带语义播报
                min_dist = yolo_dist
                class_name, extra = yolo_class, yolo_extra
            else:
                # 雷达胜出（雷达看到更近的物体，或 YOLO 无检测）：纯距离播报
                min_dist = radar_dist
                class_name, extra = None, ""
            is_urgent = min_dist < urgent

        if now - self._last_auto_broadcast_time < global_cooldown:
            return

        if not should_broadcast(min_zone, min_dist, self._broadcast_history,
                                cooldown, now, threshold):
            return

        text = generate_broadcast_text(min_zone, min_dist, class_name, extra, urgent=is_urgent)

        # 去重：与上次播报内容完全相同则跳过，防止 TTS 队列积压导致连播
        if text == self._last_tts_text:
            self._broadcast_history[min_zone] = (min_dist, now)
            self._last_auto_broadcast_time = now
            return

        self._send_tts(text, priority=1 if is_urgent else 0, source=0)
        self._broadcast_history[min_zone] = (min_dist, now)
        self._last_auto_broadcast_time = now

    def _send_tts(self, text, priority=0, source=0):
        self._ensure_publishers()
        from ladar_ai.msg import TTSRequest
        msg = TTSRequest()
        msg.text = text
        msg.priority = priority
        msg.source = source
        self._tts_pub.publish(msg)
        if text != "__stop__":
            self._last_tts_text = text
            self._last_tts_send_time = time.time()
            self._tts_busy = True

    def _send_tts_immediate(self, text, priority=1, source=0):
        """高优先级播报：打断当前播报，立即播放新内容。"""
        self._send_tts("__stop__", priority=priority, source=source)
        self._send_tts(text, priority=priority, source=source)

    def _publish_status(self):
        if not self._status_dirty:
            return
        self._status_dirty = False
        self._ensure_publishers()
        from ladar_ai.msg import SystemStatus
        msg = SystemStatus()
        msg.zones.front_left = self._zones["front_left"]
        msg.zones.front = self._zones["front"]
        msg.zones.front_right = self._zones["front_right"]
        msg.zones.right = self._zones["right"]
        msg.zones.rear_right = self._zones["rear_right"]
        msg.zones.rear = self._zones["rear"]
        msg.zones.rear_left = self._zones["rear_left"]
        msg.zones.left = self._zones["left"]
        msg.front_detections.detections = self._front_detections
        msg.last_tts_text = self._last_tts_text
        msg.voice_state = self._voice_state
        msg.node_status = [True, True, True, True, True]
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionDecisionNode()
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
