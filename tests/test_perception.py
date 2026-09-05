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

    out = scan_text(W(), None)
    assert "obs_0" in out
    assert "E45" in out and "N0" in out          # world-frame centre
    assert "14x14" in out                        # footprint extents


def test_scan_covers_all_six_obstacle_world_buildings():
    """k=4 truncation made a drone plan around 4 known buildings and fly into the
    5th — scan must cover every building in the obstacles world."""
    from agents.perception.perception import scan_text

    class W:
        buildings = [{"name": f"obs_{j}", "x": 40.0 + 15 * j, "y": 0.0,
                      "w": 10.0, "d": 10.0, "h": 20.0} for j in range(6)]

        def drone_state(self, bridge, i):
            return (0.0, 0.0, 12.0, 0.0)

        def world_xy(self, bridge, j):
            return None

    out = scan_text(W(), None)
    for j in range(6):
        assert f"obs_{j}" in out


class _FlatWorld:
    buildings = []

    def drone_state(self, bridge, i):
        return (0.0, 0.0, 12.0, 0.0)   # at origin, facing north

    def world_xy(self, bridge, j):
        return None


def test_scan_reports_mover_contacts_position_only():
    """Contacts carry name, distance, bearing, and absolute position — but NO
    velocity or trajectory hints: deriving a contact's course by differencing
    two scans is the capability the dynamic ladder probes."""
    from agents.perception.perception import scan_text

    out = scan_text(_FlatWorld(), None,
                    mover_poses={"mov_1": (0.0, 60.0, 1.2)})
    assert "contact mov_1 60m ahead [IN VIEW]" in out
    assert "E0 N60" in out and "alt 1m" in out
    for leak in ("speed", "m/s", "loop", "circle", "route"):
        assert leak not in out


def test_scan_omits_movers_beyond_range():
    from agents.perception.perception import MOVER_SCAN_RANGE_M, scan_text

    far = MOVER_SCAN_RANGE_M + 30.0
    out = scan_text(_FlatWorld(), None,
                    mover_poses={"mov_0": (far, 0.0, 10.0),
                                 "mov_1": (30.0, 30.0, 5.0)})
    assert "mov_0" not in out
    assert "contact mov_1" in out


def test_scan_without_movers_unchanged():
    from agents.perception.perception import scan_text

    assert "contact" not in scan_text(_FlatWorld(), None)
