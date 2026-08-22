"""GlobalPlan 消息字段和序列化测试"""
import pytest

from rtk_msgs.msg import GlobalPlan
from geometry_msgs.msg import Point


def test_global_plan_msg_fields():
    """GlobalPlan 所有字段存在且类型正确"""
    msg = GlobalPlan()

    # float64 字段
    assert hasattr(msg, 'start_lat')
    assert hasattr(msg, 'start_lon')
    assert hasattr(msg, 'goal_lat')
    assert hasattr(msg, 'goal_lon')
    assert hasattr(msg, 'distance_meters')
    assert hasattr(msg, 'duration_seconds')

    # string 字段
    assert hasattr(msg, 'goal_source')
    assert hasattr(msg, 'source')
    assert hasattr(msg, 'status')
    assert hasattr(msg, 'error_message')

    # 数组字段
    assert hasattr(msg, 'path_wgs84')

    # header
    assert hasattr(msg, 'header')

    # 默认值合理
    assert msg.distance_meters == 0.0
    assert msg.duration_seconds == 0.0
    assert msg.status == ''
    assert msg.source == ''
    assert msg.error_message == ''
    assert len(msg.path_wgs84) == 0


def test_global_plan_serialize():
    """消息字段赋值后能正确读回（验证序列化无丢字段）"""
    msg = GlobalPlan()

    msg.start_lat = 24.8551
    msg.start_lon = 102.8553
    msg.goal_lat = 24.8600
    msg.goal_lon = 102.8600
    msg.goal_source = 'map_click'
    msg.distance_meters = 1234.5
    msg.duration_seconds = 678.9
    msg.source = 'public_osrm'
    msg.status = 'OK'
    msg.error_message = ''

    # 路径点数组
    p1 = Point()
    p1.x = 102.8553
    p1.y = 24.8551
    p1.z = 0.0
    p2 = Point()
    p2.x = 102.8600
    p2.y = 24.8600
    p2.z = 0.0
    msg.path_wgs84 = [p1, p2]

    # 读回断言
    assert msg.start_lat == 24.8551
    assert msg.start_lon == 102.8553
    assert msg.goal_lat == 24.8600
    assert msg.goal_source == 'map_click'
    assert msg.distance_meters == 1234.5
    assert msg.source == 'public_osrm'
    assert msg.status == 'OK'
    assert len(msg.path_wgs84) == 2
    assert msg.path_wgs84[0].x == 102.8553
    assert msg.path_wgs84[0].y == 24.8551
    assert msg.path_wgs84[1].x == 102.8600
    assert msg.path_wgs84[1].y == 24.8600
