"""测试 /goal_gps 话题类型和发布功能"""
import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_goal_gps_topic_type_exists():
    """验证 rtk_msgs/msg/GoalGPS 类型可以导入"""
    from rtk_msgs.msg import GoalGPS
    msg = GoalGPS()
    assert hasattr(msg, 'latitude')
    assert hasattr(msg, 'longitude')
    assert hasattr(msg, 'source')
    assert hasattr(msg, 'poi_name')
    assert hasattr(msg, 'altitude')
    assert hasattr(msg, 'header')


def test_can_create_publisher(rclpy_init):
    """验证可以在 ROS2 中创建 GoalGPS publisher 并发布消息"""
    from rtk_msgs.msg import GoalGPS
    node = rclpy.create_node('test_publisher')
    pub = node.create_publisher(GoalGPS, '/goal_gps_test', 10)
    assert pub is not None

    msg = GoalGPS()
    msg.latitude = 24.8551
    msg.longitude = 102.8553
    msg.source = 'poi'
    msg.poi_name = '校区中心广场'
    msg.altitude = 0.0
    pub.publish(msg)
    node.destroy_node()


def test_goal_gps_field_types(rclpy_init):
    """验证 GoalGPS 字段类型正确"""
    from rtk_msgs.msg import GoalGPS
    msg = GoalGPS()
    # float64 字段
    msg.latitude = 24.8551
    msg.longitude = 102.8553
    msg.altitude = 1950.0  # 昆明海拔约 1950m
    # string 字段
    msg.source = 'map_click'
    msg.poi_name = ''
    # 验证类型
    assert isinstance(msg.latitude, float)
    assert isinstance(msg.source, str)
