"""自主导航绝对航向保持测试。"""

from rtk_perception.chassis_serial_node import NavHeadingHold


def test_straight_motion_holds_absolute_heading_after_drift():
    hold = NavHeadingHold()
    first = hold.apply(0.0, 0.0, 0.3, 0.0, 0.01)
    after_drift = hold.apply(8.0, 8.0, 0.3, 0.0, 0.01)
    assert first == 0.0
    assert after_drift == 0.0


def test_planner_turn_releases_heading_hold():
    hold = NavHeadingHold()
    hold.apply(0.0, 0.0, 0.3, 0.0, 0.01)
    output = hold.apply(-20.0, 0.0, 0.3, 0.2, 0.01)
    assert output == -20.0


def test_mechanical_trim_is_applied_when_moving():
    hold = NavHeadingHold(steering_trim_deg=-3.0)
    assert hold.apply(10.0, 10.0, 0.3, 0.0, 0.01) == 7.0


def test_stop_resets_previous_heading_lock():
    hold = NavHeadingHold()
    hold.apply(0.0, 0.0, 0.3, 0.0, 0.01)
    hold.apply(12.0, 12.0, 0.0, 0.0, 0.01)
    assert hold.apply(12.0, 12.0, 0.3, 0.0, 0.01) == 12.0
