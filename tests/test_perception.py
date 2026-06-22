"""Pure-trig perception: compass bearings, relative bearing + camera FOV."""
import math

from agents.perception import bearing_word, heading_word, rel_bearing, yaw_deg_to, FOV_HALF_DEG


def test_bearing_word_cardinals():
    assert bearing_word(0, 10) == "N"
    assert bearing_word(10, 0) == "E"
    assert bearing_word(0, -10) == "S"
    assert bearing_word(-10, 0) == "W"
    assert bearing_word(10, 10) == "NE"


def test_heading_word_faces_compass():
    assert heading_word(0.0) == "N"
    assert heading_word(math.radians(90)) == "E"


def test_rel_bearing_target_dead_ahead_is_in_view():
    word, in_view, rel = rel_bearing(0, 10, heading_rad=0.0)
    assert word == "ahead" and in_view is True and math.isclose(rel, 0.0, abs_tol=1e-6)


def test_rel_bearing_just_outside_fov_not_in_view():
    # 45deg off-axis target is beyond the ~35deg half-FOV
    word, in_view, rel = rel_bearing(10, 10, heading_rad=0.0)
    assert in_view is False and word == "ahead-right" and abs(rel) > FOV_HALF_DEG


def test_rel_bearing_is_relative_to_heading():
    # target due east; if the drone faces east it is dead ahead
    word, in_view, _ = rel_bearing(10, 0, heading_rad=math.radians(90))
    assert word == "ahead" and in_view is True


def test_yaw_deg_to_points_at_world_point():
    assert math.isclose(yaw_deg_to(0, 0, 0, 10), 0.0, abs_tol=1e-6)
    assert math.isclose(yaw_deg_to(0, 0, 10, 0), 90.0, abs_tol=1e-6)
