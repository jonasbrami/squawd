import math
from types import SimpleNamespace

from agents.observatory import metrics


def test_is_armed():
    assert metrics.is_armed(2) is True
    assert metrics.is_armed(1) is False
    assert metrics.is_armed(None) is None


def test_mode_name_known_and_unknown():
    assert metrics.mode_name(14) == "OFFBOARD"
    assert metrics.mode_name(4) == "HOLD"
    assert metrics.mode_name(99) == "#99"


def test_heading_deg_wraps_to_0_360():
    assert metrics.heading_deg(0.0) == 0
    assert metrics.heading_deg(math.pi / 2) == 90
    assert metrics.heading_deg(-math.pi / 2) == 270   # -90 -> 270
    assert metrics.heading_deg(None) is None


def test_build_drone_state_full():
    pos = SimpleNamespace(x=10.04, y=-5.06, z=-12.4, vx=3.0, vy=4.0, vz=-1.0, heading=0.0)
    status = SimpleNamespace(arming_state=2, nav_state=14)
    batt = SimpleNamespace(remaining=0.78, voltage_v=15.62, warning=0)
    d = metrics.build_drone_state(1, pos, status, batt, "survey north", "done", True)
    assert d["id"] == 1
    assert d["north"] == 10.0 and d["east"] == -5.1 and d["alt"] == 12.4
    assert d["speed"] == 5.0          # hypot(3, 4)
    assert d["vspeed"] == 1.0         # -vz
    assert d["heading"] == 0
    assert d["armed"] is True and d["mode"] == "OFFBOARD"
    assert d["batt_pct"] == 78 and d["voltage"] == 15.6 and d["warn"] == 0
    assert d["task"] == "survey north" and d["report"] == "done"
    assert d["cam"] is True


def test_build_drone_state_all_none_is_null_safe():
    d = metrics.build_drone_state(0, None, None, None, None, None, False)
    for k in ("north", "east", "alt", "speed", "vspeed", "heading",
              "armed", "mode", "batt_pct", "voltage", "warn", "task", "report"):
        assert d[k] is None
    assert d["id"] == 0 and d["cam"] is False


def test_battery_unknown_values_become_none():
    batt = SimpleNamespace(remaining=-1.0, voltage_v=0.0, warning=0)
    d = metrics.build_drone_state(0, None, None, batt, None, None, False)
    assert d["batt_pct"] is None and d["voltage"] is None and d["warn"] == 0


def test_battery_none_fields_do_not_crash():
    # px4_msgs always sends floats, but stay defensive: None must not raise.
    batt = SimpleNamespace(remaining=None, voltage_v=None, warning=0)
    d = metrics.build_drone_state(0, None, None, batt, None, None, False)
    assert d["batt_pct"] is None and d["voltage"] is None and d["warn"] == 0
