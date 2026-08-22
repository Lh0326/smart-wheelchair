"""pytest 公共 fixtures"""
import pytest
import rclpy


@pytest.fixture(scope='module')
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()
