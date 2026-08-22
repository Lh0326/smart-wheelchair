"""TiltIndicator 高灵敏度显示映射测试。"""
import math

import pytest

from wheelchair_app.braincontrol.tilt_indicator import map_tilt_for_display


@pytest.mark.parametrize(
    ("angle_deg", "expected_radius_ratio"),
    [(3.0, 0.31), (6.0, 0.49), (12.0, 0.77), (18.0, 1.0)],
)
def test_small_head_tilts_are_visually_amplified(angle_deg, expected_radius_ratio):
    pitch_display, roll_display = map_tilt_for_display(angle_deg, 0.0)
    assert roll_display == 0.0
    assert pitch_display / 30.0 == pytest.approx(
        expected_radius_ratio, abs=0.015
    )


def test_mapping_preserves_diagonal_direction():
    pitch_display, roll_display = map_tilt_for_display(6.0, 8.0)
    assert pitch_display / roll_display == pytest.approx(6.0 / 8.0)
    assert math.hypot(pitch_display, roll_display) > 10.0


def test_mapping_saturates_continuously_at_outer_ring():
    at_edge = math.hypot(*map_tilt_for_display(18.0, 0.0))
    past_edge = math.hypot(*map_tilt_for_display(36.0, 0.0))
    assert at_edge == pytest.approx(30.0)
    assert past_edge == pytest.approx(30.0)
