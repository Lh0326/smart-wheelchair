"""pytest 公共 fixtures（rtk_imu 包）"""
import pytest


@pytest.fixture(scope='module')
def rclpy_init():
    """rclpy 初始化 fixture（消息装配测试用）"""
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()
