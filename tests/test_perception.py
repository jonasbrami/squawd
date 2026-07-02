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


def test_scan_reports_building_footprint_geometry():
    """Obstacle navigation needs the footprint, not just an edge distance and a
    compass word: a 5m-clearance route around a rectangle you cannot locate is
    unplannable (sonnet planned a sensible detour and still hit obs_0 because
    scan withheld where the wall was)."""
    from agents.perception.perception import scan_text

    class W:
        buildings = [{"name": "obs_0", "x": 45.0, "y": 0.0, "w": 14.0, "d": 14.0, "h": 20.0}]

        def drone_state(self, bridge, i):
            return (0.0, 0.0, 12.0, 0.0)   # at home, facing north

        def world_xy(self, bridge, j):
            return None

    out = scan_text(W(), None, 0, 1)
    assert "obs_0" in out
    assert "E45" in out and "N0" in out          # world-frame centre
    assert "14x14" in out                        # footprint extents
