"""voice_announce_node 纯函数单元测试（Layer 1）。

不需要 rclpy，直接 import 模块级函数测试。
"""
import math

import pytest


def test_haversine_known_distance():
    """haversine 已知距离验证：昆工呈贡校区两点 ~100m。"""
    from wheelchair_app.nodes.voice_announce_node import haversine
    # 24.8551, 102.8553 到 24.8560, 102.8553（纬度差 0.0009° ≈ 100m）
    d = haversine(24.8551, 102.8553, 24.8560, 102.8553)
    assert 95 < d < 105, f"期望 ~100m，实际 {d:.1f}m"


def test_haversine_same_point_zero():
    """同点距离 0。"""
    from wheelchair_app.nodes.voice_announce_node import haversine
    assert haversine(24.8551, 102.8553, 24.8551, 102.8553) == 0.0


def test_haversine_lon_delta():
    """经度差 0.001° ≈ 101m（在纬度 24.8551 处，cos(24.8551°) × 111.32m）。"""
    from wheelchair_app.nodes.voice_announce_node import haversine
    d = haversine(24.8551, 102.8553, 24.8551, 102.8543)
    assert 100 < d < 102, f"期望 ~101m，实际 {d:.1f}m"


def test_point_to_segment_distance_on_line():
    """点在线段上时距离为 0。"""
    from wheelchair_app.nodes.voice_announce_node import point_to_segment_distance
    # 线段 (24.8550, 102.8550) → (24.8560, 102.8550)，点在线段中点
    d = point_to_segment_distance(24.8555, 102.8550, 24.8550, 102.8550, 24.8560, 102.8550)
    assert d < 0.5, f"点在线段上应 < 0.5m，实际 {d:.2f}m"


def test_point_to_segment_distance_off_line():
    """点在线段外 50m。"""
    from wheelchair_app.nodes.voice_announce_node import point_to_segment_distance
    # 线段 (24.8550, 102.8550) → (24.8560, 102.8550)（东西向）
    # 点 (24.8555, 102.8555)（线段北侧 ~50m）
    d = point_to_segment_distance(24.8555, 102.8555, 24.8550, 102.8550, 24.8560, 102.8550)
    assert 45 < d < 55, f"期望 ~50m，实际 {d:.2f}m"


def test_point_to_segment_distance_beyond_endpoint():
    """点在线段延长线上（应算到端点距离）。"""
    from wheelchair_app.nodes.voice_announce_node import point_to_segment_distance
    # 线段 (24.8550, 102.8550) → (24.8555, 102.8550)
    # 点 (24.8560, 102.8550)（在线段北端延长线上）
    d = point_to_segment_distance(24.8560, 102.8550, 24.8550, 102.8550, 24.8555, 102.8550)
    # 应该是到 (24.8555, 102.8550) 的距离 ≈ 55m
    assert 50 < d < 60, f"期望 ~55m，实际 {d:.2f}m"


def test_extract_road_segments_all_named(monkeypatch):
    """全主路路径反查：每段都有 name。"""
    from wheelchair_app.nodes import voice_announce_node as mod
    monkeypatch.setattr(mod, '_nearest_node_ids', lambda G, points: list(range(len(points))))

    from wheelchair_app.nodes.voice_announce_node import extract_road_segments
    from geometry_msgs.msg import Point

    class _FakeEdge:
        def __init__(self, name, length):
            self._d = {'name': name, 'length': length}
        def get(self, k, default=None):
            return self._d.get(k, default)

    class _FakeEdges:
        def __getitem__(self, key):
            return _FakeEdge('梁王路', 50.0)

    class _FakeGraph:
        def __init__(self):
            self.edges = _FakeEdges()

    G = _FakeGraph()
    # path 长度 5：起投影 + 3 node + 终投影 → interior 长度 3 → 2 段
    path = [
        Point(x=102.855, y=24.855, z=0.0),
        Point(x=102.856, y=24.856, z=0.0),
        Point(x=102.857, y=24.857, z=0.0),
        Point(x=102.858, y=24.858, z=0.0),
        Point(x=102.859, y=24.859, z=0.0),
    ]
    segs = extract_road_segments(path, G)
    assert len(segs) == 2
    assert segs[0]['name'] == '梁王路'
    assert segs[0]['length'] == 50.0


def test_extract_road_segments_with_small_road(monkeypatch):
    """含无名边的"小路"标记。"""
    from wheelchair_app.nodes import voice_announce_node as mod
    monkeypatch.setattr(mod, '_nearest_node_ids', lambda G, points: list(range(len(points))))

    from wheelchair_app.nodes.voice_announce_node import extract_road_segments
    from geometry_msgs.msg import Point

    class _FakeEdge:
        def __init__(self, name, length):
            self._d = {'name': name, 'length': length}
        def get(self, k, default=None):
            return self._d.get(k, default)

    class _FakeEdges:
        def __init__(self, edges_data):
            self._edges = edges_data
        def __getitem__(self, key):
            u, v, k = key
            name, length = self._edges[(u, v)]
            return _FakeEdge(name, length)

    class _FakeGraph:
        def __init__(self, edges_data):
            self.edges = _FakeEdges(edges_data)

    # 边 0→1 有名字，1→2 无名字
    G = _FakeGraph({(0, 1): ('梁王路', 50.0), (1, 2): (None, 30.0)})
    path = [
        Point(x=102.855, y=24.855, z=0.0),
        Point(x=102.856, y=24.856, z=0.0),
        Point(x=102.857, y=24.857, z=0.0),
        Point(x=102.858, y=24.858, z=0.0),
        Point(x=102.859, y=24.859, z=0.0),
    ]
    segs = extract_road_segments(path, G)
    assert len(segs) == 2
    assert segs[0]['name'] == '梁王路'
    assert segs[1]['name'] == '小路'


def test_extract_road_segments_short_path():
    """path_wgs84 长度 < 3 返回空。"""
    from wheelchair_app.nodes.voice_announce_node import extract_road_segments
    from geometry_msgs.msg import Point
    path = [Point(x=102.855, y=24.855, z=0.0), Point(x=102.856, y=24.856, z=0.0)]
    assert extract_road_segments(path, None) == []


def test_extract_road_segments_list_name(monkeypatch):
    """name 是 list 时取第一个。"""
    from wheelchair_app.nodes import voice_announce_node as mod
    monkeypatch.setattr(mod, '_nearest_node_ids', lambda G, points: list(range(len(points))))

    from wheelchair_app.nodes.voice_announce_node import extract_road_segments
    from geometry_msgs.msg import Point

    class _FakeEdge:
        def __init__(self, name, length):
            self._d = {'name': name, 'length': length}
        def get(self, k, default=None):
            return self._d.get(k, default)

    class _FakeEdges:
        def __getitem__(self, key):
            return _FakeEdge(['梁王路', '梁王路辅路'], 50.0)

    class _FakeGraph:
        def __init__(self):
            self.edges = _FakeEdges()

    G = _FakeGraph()
    path = [
        Point(x=102.855, y=24.855, z=0.0),
        Point(x=102.856, y=24.856, z=0.0),
        Point(x=102.857, y=24.857, z=0.0),
        Point(x=102.858, y=24.858, z=0.0),
        Point(x=102.859, y=24.859, z=0.0),
    ]
    segs = extract_road_segments(path, G)
    assert segs[0]['name'] == '梁王路'


def test_merge_consecutive_same_basic():
    """连续同名段合并：长度累加。"""
    from wheelchair_app.nodes.voice_announce_node import merge_consecutive_same
    segs = [
        {'name': '梁王路', 'length': 50.0, 'from_node': 0, 'to_node': 1},
        {'name': '梁王路', 'length': 80.0, 'from_node': 1, 'to_node': 2},
        {'name': '求知路', 'length': 100.0, 'from_node': 2, 'to_node': 3},
        {'name': '图书馆路', 'length': 60.0, 'from_node': 3, 'to_node': 4},
    ]
    merged = merge_consecutive_same(segs)
    assert len(merged) == 3
    assert merged[0]['name'] == '梁王路'
    assert merged[0]['length'] == 130.0
    assert merged[0]['start_path_idx'] == 1
    assert merged[0]['end_path_idx'] == 3
    assert merged[1]['name'] == '求知路'
    assert merged[1]['start_path_idx'] == 3
    assert merged[1]['end_path_idx'] == 4


def test_merge_consecutive_same_small_road():
    """连续小路合并。"""
    from wheelchair_app.nodes.voice_announce_node import merge_consecutive_same
    segs = [
        {'name': '梁王路', 'length': 50.0, 'from_node': 0, 'to_node': 1},
        {'name': '小路', 'length': 20.0, 'from_node': 1, 'to_node': 2},
        {'name': '小路', 'length': 30.0, 'from_node': 2, 'to_node': 3},
        {'name': '图书馆路', 'length': 60.0, 'from_node': 3, 'to_node': 4},
    ]
    merged = merge_consecutive_same(segs)
    assert len(merged) == 3
    assert merged[1]['name'] == '小路'
    assert merged[1]['length'] == 50.0
    assert merged[1]['start_path_idx'] == 2
    assert merged[1]['end_path_idx'] == 4


def test_merge_consecutive_same_empty():
    """空输入返回空。"""
    from wheelchair_app.nodes.voice_announce_node import merge_consecutive_same
    assert merge_consecutive_same([]) == []


def test_merge_consecutive_same_single():
    """单段直接返回，start_path_idx=1。"""
    from wheelchair_app.nodes.voice_announce_node import merge_consecutive_same
    segs = [{'name': '梁王路', 'length': 50.0, 'from_node': 0, 'to_node': 1}]
    merged = merge_consecutive_same(segs)
    assert len(merged) == 1
    assert merged[0]['start_path_idx'] == 1
    assert merged[0]['end_path_idx'] == 2


def test_build_overview_text_all_main():
    """全主路文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    segs = [
        {'name': '梁王路', 'length': 130.0, 'start_path_idx': 1, 'end_path_idx': 3},
        {'name': '求知路', 'length': 100.0, 'start_path_idx': 3, 'end_path_idx': 4},
        {'name': '图书馆路', 'length': 60.0, 'start_path_idx': 4, 'end_path_idx': 5},
    ]
    text = build_overview_text(350.0, 300.0, segs)
    assert text == '路径规划完毕，全程 350 米，预计用时 5 分钟，途经梁王路、求知路、图书馆路。'


def test_build_overview_text_main_and_small():
    """主路 + 小路文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    segs = [
        {'name': '梁王路', 'length': 130.0, 'start_path_idx': 1, 'end_path_idx': 3},
        {'name': '求知路', 'length': 100.0, 'start_path_idx': 3, 'end_path_idx': 4},
        {'name': '小路', 'length': 50.0, 'start_path_idx': 4, 'end_path_idx': 6},
    ]
    text = build_overview_text(350.0, 300.0, segs)
    assert text == '路径规划完毕，全程 350 米，预计用时 5 分钟，途经梁王路、求知路，沿途经过 1 段小路。'


def test_build_overview_text_main_small_main_small():
    """主路 + 小路 + 主路 + 小路：小路段数合计。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    segs = [
        {'name': '梁王路', 'length': 130.0, 'start_path_idx': 1, 'end_path_idx': 3},
        {'name': '小路', 'length': 50.0, 'start_path_idx': 3, 'end_path_idx': 4},
        {'name': '求知路', 'length': 100.0, 'start_path_idx': 4, 'end_path_idx': 5},
        {'name': '小路', 'length': 30.0, 'start_path_idx': 5, 'end_path_idx': 6},
    ]
    text = build_overview_text(350.0, 300.0, segs)
    assert text == '路径规划完毕，全程 350 米，预计用时 5 分钟，途经梁王路、求知路，沿途经过 2 段小路。'


def test_build_overview_text_all_small():
    """全小路文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    segs = [{'name': '小路', 'length': 150.0, 'start_path_idx': 1, 'end_path_idx': 4}]
    text = build_overview_text(50.0, 60.0, segs)
    assert text == '路径规划完毕，全程 50 米，预计用时 1 分钟，沿途经过 1 段小路。'


def test_build_overview_text_single_main():
    """单主路文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    segs = [{'name': '梁王路', 'length': 100.0, 'start_path_idx': 1, 'end_path_idx': 2}]
    text = build_overview_text(100.0, 60.0, segs)
    assert text == '路径规划完毕，全程 100 米，预计用时 1 分钟，途经梁王路。'


def test_build_overview_text_too_close():
    """距离 < 3m 文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    text = build_overview_text(2.0, 0.0, [])
    assert text == '终点就在附近，无需导航。'


def test_build_overview_text_less_than_one_minute():
    """时长 < 1 分钟报"不到 1 分钟"。"""
    from wheelchair_app.nodes.voice_announce_node import build_overview_text
    segs = [{'name': '梁王路', 'length': 50.0, 'start_path_idx': 1, 'end_path_idx': 2}]
    text = build_overview_text(50.0, 30.0, segs)
    assert '不到 1 分钟' in text


def test_build_turn_text_main_road():
    """进入主路文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_turn_text
    assert build_turn_text('求知路', 5.0) == '前方 5 米进入求知路。'


def test_build_turn_text_small_road():
    """进入小路文本。"""
    from wheelchair_app.nodes.voice_announce_node import build_turn_text
    assert build_turn_text('小路', 5.0) == '前方 5 米进入小路。'


def test_build_turn_text_custom_distance():
    """自定义提前量。"""
    from wheelchair_app.nodes.voice_announce_node import build_turn_text
    assert build_turn_text('梁王路', 8.0) == '前方 8 米进入梁王路。'


def test_find_current_segment_first():
    """wheelchair 在第一段。"""
    from wheelchair_app.nodes.voice_announce_node import find_current_segment
    from geometry_msgs.msg import Point
    # path: 3 个点，2 段
    path = [
        Point(x=102.8550, y=24.8550, z=0.0),
        Point(x=102.8555, y=24.8555, z=0.0),
        Point(x=102.8560, y=24.8560, z=0.0),
    ]
    # wheelchair 在第一段中点附近
    idx, dist = find_current_segment(24.8552, 102.8552, path)
    assert idx == 0
    assert dist < 5.0


def test_find_current_segment_second():
    """wheelchair 在第二段。"""
    from wheelchair_app.nodes.voice_announce_node import find_current_segment
    from geometry_msgs.msg import Point
    path = [
        Point(x=102.8550, y=24.8550, z=0.0),
        Point(x=102.8555, y=24.8555, z=0.0),
        Point(x=102.8560, y=24.8560, z=0.0),
    ]
    idx, dist = find_current_segment(24.8558, 102.8558, path)
    assert idx == 1
    assert dist < 5.0


def test_find_next_turn_returns_next_seg_start():
    """拐点 = 下一段的 start_path_idx。"""
    from wheelchair_app.nodes.voice_announce_node import find_next_turn
    merged = [
        {'name': '梁王路', 'length': 130.0, 'start_path_idx': 1, 'end_path_idx': 3},
        {'name': '求知路', 'length': 100.0, 'start_path_idx': 3, 'end_path_idx': 4},
        {'name': '图书馆路', 'length': 60.0, 'start_path_idx': 4, 'end_path_idx': 5},
    ]
    # 当前在第一段（path_idx=2）
    turn_idx, name = find_next_turn(2, merged)
    assert turn_idx == 3
    assert name == '求知路'


def test_find_next_turn_last_segment_no_turn():
    """已在最后一段，无下一个拐点。"""
    from wheelchair_app.nodes.voice_announce_node import find_next_turn
    merged = [
        {'name': '梁王路', 'length': 130.0, 'start_path_idx': 1, 'end_path_idx': 3},
        {'name': '求知路', 'length': 100.0, 'start_path_idx': 3, 'end_path_idx': 4},
    ]
    turn_idx, name = find_next_turn(3, merged)
    assert turn_idx is None
    assert name is None


def test_find_next_turn_empty_merged():
    """空 merged 返回 (None, None)。"""
    from wheelchair_app.nodes.voice_announce_node import find_next_turn
    turn_idx, name = find_next_turn(0, [])
    assert turn_idx is None
    assert name is None
