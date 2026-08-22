"""voice_announce_node: 路径语音播报节点。
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


订阅 /global_plan + /fix + /clear_goal，发布 /tts_request (ladar_ai.msg.TTSRequest)。
- 路径概览：规划成功后播报"路径规划完毕，全程 X 米，..."
- turn-by-turn：路名变化前 5m 播报"前方 5 米进入 X 路"
- 到达终点：播报"已到达终点，导航结束"

架构：本节点只负责"什么时候说什么话"，文本发到 /tts_request；
现有 tts_node（ladar-ai 包）订阅 /tts_request 并调 TTSEngine 发声。
"""
import math
import logging

_logger = logging.getLogger('voice_announce_node')

# 无名道路的统一标签（边无 name 字段或反查失败时使用）
_UNNAMED_ROAD_LABEL = '小路'


EARTH_RADIUS_M = 6371000.0


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """WGS84 两点间球面距离（米）。"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def _latlon_to_xy(lat: float, lon: float, ref_lat: float, ref_lon: float):
    """经纬度 → 平面米（equirectangular approximation，以 ref 为原点）。

    小范围（< 1km）误差 < 0.5%，足够做点-段投影。
    """
    x = math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat)) * EARTH_RADIUS_M
    y = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    return x, y


def point_to_segment_distance(
    p_lat: float, p_lon: float,
    a_lat: float, a_lon: float,
    b_lat: float, b_lon: float,
) -> float:
    """点 P 到线段 AB 的最短距离（米）。

    用 equirectangular approximation 转平面坐标，做点积投影。
    """
    ref_lat = (p_lat + a_lat + b_lat) / 3
    ref_lon = (p_lon + a_lon + b_lon) / 3
    px, py = _latlon_to_xy(p_lat, p_lon, ref_lat, ref_lon)
    ax, ay = _latlon_to_xy(a_lat, a_lon, ref_lat, ref_lon)
    bx, by = _latlon_to_xy(b_lat, b_lon, ref_lat, ref_lon)

    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-9:
        # A == B，退化成点
        return math.sqrt(apx * apx + apy * apy)
    t = (apx * abx + apy * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))

    cx = ax + t * abx
    cy = ay + t * aby
    dx = px - cx
    dy = py - cy
    return math.sqrt(dx * dx + dy * dy)


def extract_road_segments(path_wgs84, G):
    """从 path_wgs84 反查每段边的道路名。

    path_wgs84[0] 是起点投影，path_wgs84[-1] 是终点投影，中间是真实 OSM node。
    返回 [{name, length, from_node, to_node}, ...]，段数 = len(path_wgs84) - 3
    （去掉起终点投影后 n 个内部 node 产生 n-1 段）。

    反查失败（osmnx 异常）时所有段标"小路"并日志告警。
    """
    if len(path_wgs84) < 3:
        return []

    interior = list(path_wgs84[1:-1])
    try:
        node_ids = _nearest_node_ids(G, interior)
    except Exception as e:
        _logger.warning(f'_nearest_node_ids 反查失败，所有段标"小路": {e}')
        node_ids = list(range(len(interior)))

    segments = []
    for i in range(len(node_ids) - 1):
        u = node_ids[i]
        v = node_ids[i + 1]
        try:
            edge_data = G.edges[u, v, 0]
            name = edge_data.get('name', None)
            length = float(edge_data.get('length', 0.0))
        except (KeyError, AttributeError, TypeError):
            name = None
            length = 0.0
        if isinstance(name, list):
            name = name[0] if name else None
        segments.append({
            'name': name if name else _UNNAMED_ROAD_LABEL,
            'length': length,
            'from_node': u,
            'to_node': v,
        })
    return segments


def _nearest_node_ids(G, points):
    """用 osmnx 批量反查 node_id（生产用）。

    测试时用 monkeypatch 替换本函数，避免依赖 osmnx 重依赖加载。
    """
    import osmnx as ox
    lons = [p.x for p in points]
    lats = [p.y for p in points]
    return ox.distance.nearest_nodes(G, X=lons, Y=lats)


def merge_consecutive_same(segments):
    """合并连续同名段，记录每段在 path_wgs84 中的起止 index。

    segments[i] 对应 path_wgs84[i+1] → path_wgs84[i+2]（因为 path_wgs84[0] 是起点投影）。
    返回 [{name, length, start_path_idx, end_path_idx}, ...]
    """
    if not segments:
        return []
    merged = [{
        'name': segments[0]['name'],
        'length': segments[0]['length'],
        'start_path_idx': 1,
        'end_path_idx': 2,
    }]
    for i, seg in enumerate(segments[1:], start=1):
        if seg['name'] == merged[-1]['name']:
            merged[-1]['length'] += seg['length']
            merged[-1]['end_path_idx'] = i + 2
        else:
            merged.append({
                'name': seg['name'],
                'length': seg['length'],
                'start_path_idx': i + 1,
                'end_path_idx': i + 2,
            })
    return merged


def build_overview_text(distance_m: float, duration_s: float, merged_segments) -> str:
    """组装路径概览文本（高德风格）。

    主路按顺序报 + 末尾追加"沿途经过 N 段小路"（N 是合并后段数）。
    """
    if distance_m < 3.0:
        return '终点就在附近，无需导航。'

    dist_str = f"{int(round(distance_m))} 米"

    minutes = duration_s / 60.0
    if minutes < 1.0:
        time_str = '不到 1 分钟'
    else:
        time_str = f"{int(round(minutes))} 分钟"

    # 主路列表（按顺序，去重）
    main_roads = []
    for seg in merged_segments:
        name = seg['name']
        if name != _UNNAMED_ROAD_LABEL and name not in main_roads:
            main_roads.append(name)

    # 小路段计数（合并后段数）
    small_road_count = sum(1 for seg in merged_segments if seg['name'] == _UNNAMED_ROAD_LABEL)

    text = f"路径规划完毕，全程 {dist_str}，预计用时 {time_str}"
    if main_roads:
        text += '，途经' + '、'.join(main_roads)
    if small_road_count > 0:
        text += f'，沿途经过 {small_road_count} 段小路'
    text += '。'
    return text


def build_turn_text(next_name: str, ahead_meters: float) -> str:
    """组装 turn-by-turn 文本。"""
    ahead_int = int(round(ahead_meters))
    return f'前方 {ahead_int} 米进入{next_name}。'


def find_current_segment(wheelchair_lat: float, wheelchair_lon: float, path_wgs84):
    """找 wheelchair 当前所在段的 index（path_wgs84 中的 segment 起点 index）。

    返回 (segment_start_idx, min_distance_to_path)。
    """
    if len(path_wgs84) < 2:
        return 0, float('inf')
    min_dist = float('inf')
    best_idx = 0
    for i in range(len(path_wgs84) - 1):
        p1 = path_wgs84[i]
        p2 = path_wgs84[i + 1]
        d = point_to_segment_distance(
            wheelchair_lat, wheelchair_lon,
            p1.y, p1.x, p2.y, p2.x,
        )
        if d < min_dist:
            min_dist = d
            best_idx = i
    return best_idx, min_dist


def find_next_turn(current_path_idx: int, merged_segments):
    """从 current_path_idx 向后找第一个路名变化点。

    返回 (turn_node_path_idx, next_name) 或 (None, None)。
    turn_node_path_idx 是下一个 merged_segment 的 start_path_idx。
    """
    if not merged_segments:
        return None, None
    current_seg_idx = None
    for i, seg in enumerate(merged_segments):
        if seg['start_path_idx'] <= current_path_idx < seg['end_path_idx']:
            current_seg_idx = i
            break
    if current_seg_idx is None:
        # current_path_idx 超出所有段范围，取最后一段
        current_seg_idx = len(merged_segments) - 1
    if current_seg_idx + 1 >= len(merged_segments):
        return None, None
    next_seg = merged_segments[current_seg_idx + 1]
    return next_seg['start_path_idx'], next_seg['name']


import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty

from rtk_msgs.msg import GlobalPlan

try:
    from ladar_ai.msg import TTSRequest
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False
    TTSRequest = None


# === 常量 ===
TTS_TEXT_STOP = '__stop__'
ARRIVAL_TEXT = '已到达终点，导航结束。'
CANCEL_TEXT = '已取消导航。'
OFFROUTE_TEXT = '已偏离路径，请重新规划。'
FAILURE_TEXT = '路径规划失败，请稍后重试。'


def _load_graph(graphml_path):
    """加载 osmnx graphml（生产用）。测试时被 mock。"""
    import osmnx as ox
    return ox.load_graphml(graphml_path)


class VoiceAnnounceNode(Node):
    """路径语音播报节点。

    订阅 /global_plan + /fix + /clear_goal，发布 /tts_request。
    """

    def __init__(self):
        super().__init__('voice_announce_node')

        # 参数
        self.declare_parameter('graphml_path', _WS_ROOT + '/data/region.graphml')
        self.declare_parameter('turn_ahead_meters', 5.0)
        self.declare_parameter('arrival_meters', 3.0)
        self.declare_parameter('offroute_meters', 25.0)
        self.declare_parameter('offroute_hold_seconds', 3.0)
        self.declare_parameter('fix_stale_seconds', 5.0)
        self.declare_parameter('turn_dedup_seconds', 5.0)
        self.declare_parameter('goal_dedup_seconds', 5.0)
        self.declare_parameter('announce_delay_sec', 3.0)
        self.declare_parameter('tts_priority', 0)

        self.graphml_path = self.get_parameter('graphml_path').value
        self.turn_ahead_meters = float(self.get_parameter('turn_ahead_meters').value)
        self.arrival_meters = float(self.get_parameter('arrival_meters').value)
        self.offroute_meters = float(self.get_parameter('offroute_meters').value)
        self.offroute_hold_seconds = float(self.get_parameter('offroute_hold_seconds').value)
        self.fix_stale_seconds = float(self.get_parameter('fix_stale_seconds').value)
        self.turn_dedup_seconds = float(self.get_parameter('turn_dedup_seconds').value)
        self.goal_dedup_seconds = float(self.get_parameter('goal_dedup_seconds').value)
        self.announce_delay_sec = float(self.get_parameter('announce_delay_sec').value)
        self.tts_priority = int(self.get_parameter('tts_priority').value)

        # 加载图（后台线程，避免阻塞 init）
        self._G = None
        self._graph_loaded = False
        if os.path.exists(self.graphml_path):
            threading.Thread(target=self._load_graph_async, daemon=True).start()
        else:
            self.get_logger().warn(f'graphml 不存在: {self.graphml_path}')

        # /tts_request 发布者
        self._tts_pub = self.create_publisher(
            TTSRequest if TTS_AVAILABLE else type('FakeMsg', (), {}),
            '/tts_request', 10
        ) if TTS_AVAILABLE else None

        # 状态
        self._last_path_wgs84 = None
        self._last_merged_segments = None
        self._last_goal_key = None
        self._last_goal_time = 0.0
        self._announced_goal_keys: set = set()  # 已播报过的 goal_key，永久去重（防 networkx 定时刷新重复触发）
        self._announced_turn_indices = set()
        self._turn_announce_time = {}
        self._arrived = False
        self._offroute = False
        self._offroute_start_time = None
        self._last_fix_lat = None
        self._last_fix_lon = None
        self._last_fix_time = 0.0
        # USB Hub OCP 错峰：节点启动后第一次概览播报额外延迟 5s，
        # 让 chassis + TEB + TTS 模型预热完全进入稳态后再发首次 TTS 推理，
        # 避免冷启动叠加多重 inrush 触发 Hub OCP。
        self._first_announce_done = False
        self._first_announce_extra_delay_sec = 5.0

        # 订阅
        self.create_subscription(GlobalPlan, '/global_plan', self._on_global_plan, 10)
        self.create_subscription(NavSatFix, '/fix', self._on_fix, 10)
        self.create_subscription(Empty, '/clear_goal', self._on_clear_goal, 10)

        self.get_logger().info('voice_announce_node 启动')

    def _load_graph_async(self):
        try:
            self._G = _load_graph(self.graphml_path)
            self._graph_loaded = True
            self.get_logger().info('region.graphml 加载完成')
        except Exception as e:
            self.get_logger().warn(f'加载 graphml 失败: {e}')

    def _publish_tts(self, text: str):
        """发一条 TTSRequest。"""
        if not TTS_AVAILABLE or self._tts_pub is None:
            self.get_logger().info(f'[TTS dry-run] {text}')
            return
        msg = TTSRequest()
        msg.text = text
        msg.priority = self.tts_priority
        self._tts_pub.publish(msg)
        self.get_logger().info(f'TTS: {text}')

    def _on_global_plan(self, msg: GlobalPlan):
        """路径规划结果到来。"""
        # 同终点永久去重：相同 goal_key 只播报一次。
        # 必须在发 __stop__ 之前判断，否则 networkx 3s 定时刷新的同终点 /global_plan
        # 会先发 __stop__ 打断正在播的语音（即使新消息被去重跳过，stop 已经发出）。
        # 现象：用户听到"路径规划完毕"就被截断，因为下一秒刷新的 __stop__ 打断了 TTS。
        goal_key = (round(msg.goal_lat, 6), round(msg.goal_lon, 6))
        if (msg.status == 'OK'
                and msg.distance_meters >= 3.0
                and len(msg.path_wgs84) >= 3
                and goal_key in self._announced_goal_keys):
            self.get_logger().info(
                f'同终点 {goal_key} 已播报过，跳过（networkx 定时刷新不重复播报，不打断 TTS）'
            )
            return

        # 打断当前播报（只在真正需要新播报时才打断）
        self._publish_tts(TTS_TEXT_STOP)

        # 重置状态
        self._announced_turn_indices.clear()
        self._turn_announce_time.clear()
        self._arrived = False
        self._offroute = False
        self._offroute_start_time = None

        # 失败场景
        if msg.status != 'OK':
            self._last_path_wgs84 = None
            self._last_merged_segments = None
            self._publish_tts(FAILURE_TEXT)
            return

        # 距离太近
        if msg.distance_meters < 3.0 or len(msg.path_wgs84) < 3:
            self._last_path_wgs84 = None
            self._last_merged_segments = None
            self._publish_tts('终点就在附近，无需导航。')
            return

        # 记录已播报（去重 set）
        self._announced_goal_keys.add(goal_key)
        self._last_goal_key = goal_key
        self._last_goal_time = self.get_clock().now().nanoseconds * 1e-9

        # 反查路名
        segments = extract_road_segments(msg.path_wgs84, self._G)
        merged = merge_consecutive_same(segments)

        self._last_path_wgs84 = list(msg.path_wgs84)
        self._last_merged_segments = merged

        # 概览播报（错峰：延迟 announce_delay_sec 再发，避免和 chassis 启动 + TEB 初始化
        # + USB 设备电流叠加触发主板 USB 总线保护性断开。
        # 现象：用户点击终点 → 同时触发 networkx 算路径 + TEB 启动 + 电机启动 + TTS 推理
        # → USB 总线瞬时电流过载 → 主板 USB 端口整体 disconnect → 系统假死。
        # 用 one-shot timer：rclpy.create_timer 默认是周期 timer，必须在 callback 里 cancel
        # 否则会每 announce_delay_sec 重复发一次播报。）
        text = build_overview_text(msg.distance_meters, msg.duration_seconds, merged)

        # 首次播报额外延迟 5s，让整个导航栈（chassis + TEB + TTS 模型预热）完全进入稳态
        extra = self._first_announce_extra_delay_sec if not self._first_announce_done else 0.0
        delay_sec = self.announce_delay_sec + extra

        def _delayed_announce():
            timer.cancel()
            self._first_announce_done = True
            self._publish_tts(text)

        timer = self.create_timer(delay_sec, _delayed_announce)
        self.get_logger().info(
            f'概览播报已排队，延迟 {delay_sec}s 发送（USB 供电错峰'
            f'{"，首次额外 +5s 冷启动避让" if extra > 0 else ""}）'
        )

    def _on_fix(self, msg: NavSatFix):
        """wheelchair 位置更新 → turn-by-turn 逻辑。"""
        self._last_fix_lat = msg.latitude
        self._last_fix_lon = msg.longitude
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_fix_time = now

        if self._last_path_wgs84 is None or self._arrived:
            return

        current_idx, min_dist = find_current_segment(
            msg.latitude, msg.longitude, self._last_path_wgs84
        )

        if min_dist > self.offroute_meters:
            if self._offroute_start_time is None:
                self._offroute_start_time = now
            elif now - self._offroute_start_time >= self.offroute_hold_seconds:
                if not self._offroute:
                    self._offroute = True
                    self._publish_tts(OFFROUTE_TEXT)
            return
        else:
            self._offroute_start_time = None
            self._offroute = False

        end_point = self._last_path_wgs84[-1]
        dist_to_end = haversine(msg.latitude, msg.longitude, end_point.y, end_point.x)
        if dist_to_end < self.arrival_meters:
            self._arrived = True
            self._publish_tts(ARRIVAL_TEXT)
            return

        if self._last_merged_segments is None:
            return
        turn_idx, next_name = find_next_turn(current_idx, self._last_merged_segments)
        if turn_idx is None:
            return

        turn_point = self._last_path_wgs84[turn_idx]
        dist_to_turn = haversine(msg.latitude, msg.longitude, turn_point.y, turn_point.x)

        if dist_to_turn < self.turn_ahead_meters:
            last_time = self._turn_announce_time.get(turn_idx, 0.0)
            if now - last_time < self.turn_dedup_seconds:
                return
            self._turn_announce_time[turn_idx] = now
            self._announced_turn_indices.add(turn_idx)
            text = build_turn_text(next_name, self.turn_ahead_meters)
            self._publish_tts(text)

    def _on_clear_goal(self, msg: Empty):
        """清除终点。"""
        self._publish_tts(TTS_TEXT_STOP)
        self._last_path_wgs84 = None
        self._last_merged_segments = None
        self._announced_turn_indices.clear()
        self._turn_announce_time.clear()
        self._announced_goal_keys.clear()  # 清空去重记录，让用户重设同终点时能再次播报
        self._last_goal_key = None
        self._arrived = False
        self._offroute = False
        self._offroute_start_time = None
        self._publish_tts(CANCEL_TEXT)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAnnounceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
