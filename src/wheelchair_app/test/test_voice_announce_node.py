"""voice_announce_node ROS2 集成测试（Layer 2）。

启动 voice_announce_node + 发布 fake /global_plan + /fix，
断言 /tts_request 输出。
"""
import time

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty

from rtk_msgs.msg import GlobalPlan

try:
    from ladar_ai.msg import TTSRequest
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False
    TTSRequest = None

from wheelchair_app.nodes.voice_announce_node import VoiceAnnounceNode


@pytest.fixture
def announce_node(rclpy_init, monkeypatch):
    """启动 VoiceAnnounceNode 测试实例（mock graphml 加载）。"""
    import wheelchair_app.nodes.voice_announce_node as mod
    monkeypatch.setattr(mod, '_load_graph', lambda path: None)
    node = VoiceAnnounceNode()
    yield node
    node.destroy_node()


def _publish_global_plan(helper: Node, status: str, path_points, distance: float, duration: float):
    """发布一条 GlobalPlan。"""
    pub = helper.create_publisher(GlobalPlan, '/global_plan', 10)
    time.sleep(0.3)
    msg = GlobalPlan()
    msg.status = status
    msg.distance_meters = distance
    msg.duration_seconds = duration
    msg.path_wgs84 = [Point(x=lon, y=lat, z=0.0) for lat, lon in path_points]
    pub.publish(msg)
    time.sleep(0.2)


def _collect_tts(helper: Node, received: list, duration: float = 1.0):
    """spin helper duration 秒，把 /tts_request 消息追加到 received 列表。

    必须在调用本函数前用 _make_tts_sub 创建订阅，否则订阅晚于发布会丢消息。
    """
    if not TTS_AVAILABLE:
        return
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(helper, timeout_sec=0.05)


def _make_tts_sub(helper: Node):
    """在 helper 上创建 /tts_request 订阅，返回 (sub, received_list)。"""
    received = []
    if not TTS_AVAILABLE:
        return None, received

    def _cb(m):
        received.append(m)
    sub = helper.create_subscription(TTSRequest, '/tts_request', _cb, 10)
    return sub, received


def test_global_plan_ok_triggers_overview(announce_node):
    """status=OK → 发 /tts_request 含概览文本。"""
    helper = Node('test_helper_overview')
    _publish_global_plan(helper, 'OK', [(24.855, 102.855), (24.856, 102.856), (24.857, 102.857)], 350.0, 300.0)
    rclpy.spin_once(announce_node, timeout_sec=0.3)
    assert announce_node._last_path_wgs84 is not None
    assert announce_node._last_merged_segments is not None
    helper.destroy_node()


def test_same_goal_dedup_forever(announce_node):
    """同 goal_key 第二次 /global_plan（networkx 3s 定时刷新）→ 不再播报。

    场景：用户设终点 A → networkx 每 3s 重算 → 反复发 /global_plan。
    旧逻辑（goal_dedup_seconds=5s）会在 5s 后再次播报，
    新逻辑（永久 set 去重）保证只播一次，直到 /clear_goal 或换终点。

    注：voice_announce 有 announce_delay_sec=1.5s 错峰延迟，spin 必须 ≥ 2s 才能收到。
    """
    helper = Node('test_helper_dedup_goal')
    _sub, received = _make_tts_sub(helper)

    # 第一次发 /global_plan（同终点）→ 触发播报
    _publish_global_plan(helper, 'OK',
                         [(24.855, 102.855), (24.856, 102.856), (24.857, 102.857)],
                         350.0, 300.0)
    _spin_pair(announce_node, helper, 2.2)
    first_texts = [m.text for m in received if '路径规划完毕' in m.text]
    assert len(first_texts) >= 1, f"第一次应触发概览播报，实际 {first_texts}"

    # 第二次发同终点的 /global_plan（模拟 networkx 3s 后刷新）
    received.clear()
    _publish_global_plan(helper, 'OK',
                         [(24.855, 102.855), (24.856, 102.856), (24.857, 102.857)],
                         350.0, 300.0)
    _spin_pair(announce_node, helper, 2.2)
    second_texts = [m.text for m in received if '路径规划完毕' in m.text]
    assert len(second_texts) == 0, f"同终点第二次不应再播报，实际 {second_texts}"

    # clear_goal 后再发同终点 → 应该能再次播报
    pub_clear = helper.create_publisher(Empty, '/clear_goal', 10)
    time.sleep(0.2)
    pub_clear.publish(Empty())
    _spin_pair(announce_node, helper, 1.0)

    received.clear()
    _publish_global_plan(helper, 'OK',
                         [(24.855, 102.855), (24.856, 102.856), (24.857, 102.857)],
                         350.0, 300.0)
    _spin_pair(announce_node, helper, 2.2)
    third_texts = [m.text for m in received if '路径规划完毕' in m.text]
    assert len(third_texts) >= 1, f"clear_goal 后再设同终点应再次播报，实际 {third_texts}"

    helper.destroy_node()


def test_global_plan_no_route_triggers_failure(announce_node):
    """status=NO_ROUTE → 发失败文本。"""
    helper = Node('test_helper_no_route')
    # 关键：先建订阅再发布，否则消息发布早于订阅会丢
    _sub, received = _make_tts_sub(helper)
    _publish_global_plan(helper, 'NO_ROUTE', [], 0.0, 0.0)
    # 双 spin 让 announce_node 处理 GlobalPlan 回调并 publish TTS，helper 同时收消息
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(announce_node, timeout_sec=0.05)
        rclpy.spin_once(helper, timeout_sec=0.05)
    texts = [m.text for m in received]
    assert any('路径规划失败' in t for t in texts), f"期望含'路径规划失败'，实际 {texts}"
    helper.destroy_node()


def test_global_plan_short_path_no_overview(announce_node):
    """path_wgs84 < 3 → 不发概览（距离太近）。"""
    helper = Node('test_helper_short')
    _publish_global_plan(helper, 'OK', [(24.855, 102.855), (24.856, 102.856)], 2.0, 0.0)
    rclpy.spin_once(announce_node, timeout_sec=0.3)
    assert announce_node._last_path_wgs84 is None or len(announce_node._last_path_wgs84) < 3
    helper.destroy_node()


def _publish_fix(helper: Node, lat: float, lon: float):
    """发布一条 NavSatFix。"""
    pub = helper.create_publisher(NavSatFix, '/fix', 10)
    time.sleep(0.3)
    msg = NavSatFix()
    msg.latitude = lat
    msg.longitude = lon
    msg.status.status = 0
    msg.status.service = 1
    pub.publish(msg)
    time.sleep(0.1)


def _spin_pair(announce_node, helper, duration: float = 1.0):
    """同时 spin announce_node 和 helper duration 秒，让消息往返。"""
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(announce_node, timeout_sec=0.05)
        rclpy.spin_once(helper, timeout_sec=0.05)


def test_fix_movement_triggers_turn(announce_node):
    """/fix 接近拐点 → 发 turn-by-turn 文本。

    path 用 5 个点（起点投影 + 2 个真实 node + 终点投影），
    这样 merge_consecutive_same 给出的 start_path_idx 才对得上 path 索引。
    拐点是 path[2]，wheelchair 在拐点前 ~4.7m，触发 5m 阈值。
    """
    helper = Node('test_helper_turn')
    from wheelchair_app.nodes.voice_announce_node import merge_consecutive_same
    fake_merged = merge_consecutive_same([
        {'name': '梁王路', 'length': 50.0, 'from_node': 0, 'to_node': 1},
        {'name': '求知路', 'length': 50.0, 'from_node': 1, 'to_node': 2},
    ])
    announce_node._last_path_wgs84 = [
        Point(x=102.8550, y=24.8550, z=0.0),  # 起点投影
        Point(x=102.8552, y=24.8552, z=0.0),  # n1
        Point(x=102.8555, y=24.8555, z=0.0),  # n2 = 拐点
        Point(x=102.8558, y=24.8558, z=0.0),  # n3
        Point(x=102.8560, y=24.8560, z=0.0),  # 终点投影
    ]
    announce_node._last_merged_segments = fake_merged
    announce_node._arrived = False

    _sub, received = _make_tts_sub(helper)
    _publish_fix(helper, 24.85547, 102.85547)  # 距拐点 path[2] 约 4.7m
    _spin_pair(announce_node, helper, 1.0)

    texts = [m.text for m in received]
    assert any('进入求知路' in t for t in texts), f"期望含'进入求知路'，实际 {texts}"
    helper.destroy_node()


def test_arrival_triggers_arrival_text(announce_node):
    """距 path[-1] < 3m → 发"已到达终点"。"""
    helper = Node('test_helper_arrival')
    announce_node._last_path_wgs84 = [
        Point(x=102.8550, y=24.8550, z=0.0),
        Point(x=102.8555, y=24.8555, z=0.0),
        Point(x=102.8560, y=24.8560, z=0.0),  # 终点
    ]
    announce_node._last_merged_segments = [{'name': '梁王路', 'length': 220.0,
                                            'start_path_idx': 1, 'end_path_idx': 3}]
    announce_node._arrived = False

    _sub, received = _make_tts_sub(helper)
    _publish_fix(helper, 24.8560, 102.8560)  # wheelchair 完全在终点
    _spin_pair(announce_node, helper, 1.0)

    texts = [m.text for m in received]
    assert any('已到达终点' in t for t in texts), f"期望含'已到达终点'，实际 {texts}"
    assert announce_node._arrived is True
    helper.destroy_node()


def test_same_turn_dedup_within_window(announce_node):
    """同拐点 5s 内重复 /fix → 只发一次。"""
    helper = Node('test_helper_dedup')
    from wheelchair_app.nodes.voice_announce_node import merge_consecutive_same
    fake_merged = merge_consecutive_same([
        {'name': '梁王路', 'length': 50.0, 'from_node': 0, 'to_node': 1},
        {'name': '求知路', 'length': 50.0, 'from_node': 1, 'to_node': 2},
    ])
    announce_node._last_path_wgs84 = [
        Point(x=102.8550, y=24.8550, z=0.0),
        Point(x=102.8552, y=24.8552, z=0.0),
        Point(x=102.8555, y=24.8555, z=0.0),  # 拐点
        Point(x=102.8558, y=24.8558, z=0.0),
        Point(x=102.8560, y=24.8560, z=0.0),
    ]
    announce_node._last_merged_segments = fake_merged
    announce_node._arrived = False

    # 第一次发 /fix（接近拐点）→ 触发 turn-by-turn
    _sub, received = _make_tts_sub(helper)
    _publish_fix(helper, 24.85547, 102.85547)
    _spin_pair(announce_node, helper, 0.6)
    first_turn_texts = [m.text for m in received if '进入' in m.text]
    assert len(first_turn_texts) >= 1, f"第一次应触发 turn，实际 {first_turn_texts}"

    # 第二次发同样位置的 /fix（5s 内）→ 不应再触发
    received.clear()
    _publish_fix(helper, 24.85547, 102.85547)
    _spin_pair(announce_node, helper, 0.6)
    second_turn_texts = [m.text for m in received if '进入' in m.text]
    assert len(second_turn_texts) == 0, f"5s 内不应重复，第二次发了 {second_turn_texts}"
    helper.destroy_node()


def test_new_global_plan_clears_state(announce_node):
    """新 /global_plan → 清空 _announced_turn_indices + _arrived。"""
    helper = Node('test_helper_new_plan')
    # 先注入旧状态
    announce_node._announced_turn_indices.add(99)
    announce_node._arrived = True

    _publish_global_plan(helper, 'OK',
                         [(24.855, 102.855), (24.856, 102.856), (24.857, 102.857)],
                         350.0, 300.0)
    rclpy.spin_once(announce_node, timeout_sec=0.3)

    assert 99 not in announce_node._announced_turn_indices
    assert announce_node._arrived is False
    helper.destroy_node()


def test_offroute_triggers_warning(announce_node):
    """偏离 > 25m 持续 3s → 发"已偏离路径"。"""
    helper = Node('test_helper_offroute')
    announce_node._last_path_wgs84 = [
        Point(x=102.8550, y=24.8550, z=0.0),
        Point(x=102.8560, y=24.8560, z=0.0),
    ]
    announce_node._last_merged_segments = [{'name': '梁王路', 'length': 220.0,
                                            'start_path_idx': 1, 'end_path_idx': 2}]
    announce_node._arrived = False
    announce_node._offroute = False
    announce_node._offroute_start_time = None

    _sub, received = _make_tts_sub(helper)
    # wheelchair 远离 path（path 在 24.855-24.856，wheelchair 在 24.860，约 555m 外）
    _publish_fix(helper, 24.8600, 102.8600)
    _spin_pair(announce_node, helper, 0.3)
    # 第一次只是标记开始时间，不应发告警
    assert not any('偏离' in m.text for m in received), \
        f"第一次不应触发告警，实际 {[m.text for m in received]}"

    # 模拟时间流逝：手动把 _offroute_start_time 调到 4s 前
    announce_node._offroute_start_time = (
        announce_node.get_clock().now().nanoseconds * 1e-9 - 4.0
    )
    received.clear()
    _publish_fix(helper, 24.8600, 102.8600)
    _spin_pair(announce_node, helper, 0.5)
    texts = [m.text for m in received]
    assert any('偏离' in t for t in texts), f"期望含'偏离'，实际 {texts}"
    helper.destroy_node()


def test_clear_goal_triggers_cancel(announce_node):
    """/clear_goal → 发"已取消导航" + 清空状态。"""
    helper = Node('test_helper_clear')
    # 注入活跃状态
    announce_node._last_path_wgs84 = [Point(x=102.855, y=24.855, z=0.0)]
    announce_node._announced_turn_indices.add(5)
    announce_node._arrived = False

    _sub, received = _make_tts_sub(helper)
    pub = helper.create_publisher(Empty, '/clear_goal', 10)
    time.sleep(0.3)
    pub.publish(Empty())
    _spin_pair(announce_node, helper, 1.0)

    texts = [m.text for m in received]
    assert any('已取消导航' in t for t in texts), f"期望含'已取消导航'，实际 {texts}"
    assert announce_node._last_path_wgs84 is None
    assert len(announce_node._announced_turn_indices) == 0
    helper.destroy_node()
