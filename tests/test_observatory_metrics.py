import math
from types import SimpleNamespace

from agents.observatory import metrics, overlay


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


def test_rpy_from_quat_identity_and_yaw_wrap():
    assert metrics.rpy_from_quat([1.0, 0.0, 0.0, 0.0]) == (0.0, 0.0, 0.0)
    # 90 deg yaw about z (w=cos45, z=sin45)
    s = math.sin(math.pi / 4)
    roll, pitch, yaw = metrics.rpy_from_quat([s, 0.0, 0.0, s])
    assert (roll, pitch) == (0.0, 0.0)
    assert yaw == 90.0
    # +90 deg roll about x (w=cos45, x=sin45); pitch clamped safely
    r2, p2, _ = metrics.rpy_from_quat([s, s, 0.0, 0.0])
    assert r2 == 90.0 and p2 == 0.0
    assert metrics.rpy_from_quat(None) is None


def _snapshot(**kw):
    snap = {"schema_version": 1, "sim_stamp": 42.0, "seq": 7,
            "detector": {"healthy": True, "latency_ms": 33.0},
            "beam": {"status": "IDLE", "target": None, "range_m": None},
            "track": {"state": "IDLE", "target": None, "gap_m": None},
            "contacts": []}
    snap.update(kw)
    return snap


def test_build_state_full():
    pos = SimpleNamespace(x=10.04, y=-5.06, z=-12.4, vx=3.0, vy=4.0, vz=-1.0, heading=0.0)
    status = SimpleNamespace(arming_state=2, nav_state=14)
    batt = SimpleNamespace(remaining=0.78, voltage_v=15.62, warning=0)
    snap = _snapshot(track={"state": "RANGE_LOCKED", "target": "vis_target_0", "gap_m": 3.2})
    d = metrics.build_state(pos, status, batt, att=(1.0, -2.0, 271.0),
                            cam_seq=57, cam_stamp=42.1, snapshot=snap)
    assert d["north"] == 10.0 and d["east"] == -5.1 and d["alt"] == 12.4
    assert d["speed"] == 5.0          # hypot(3, 4)
    assert d["vspeed"] == 1.0         # -vz
    assert d["heading"] == 0
    assert (d["roll"], d["pitch"], d["yaw"]) == (1.0, -2.0, 271.0)
    assert d["armed"] is True and d["mode"] == "OFFBOARD"
    assert d["batt_pct"] == 78 and d["voltage"] == 15.6 and d["warn"] == 0
    assert d["cam"] is True and d["cam_seq"] == 57 and d["cam_stamp"] == 42.1
    assert d["detector"]["healthy"] is True
    assert d["track"]["state"] == "RANGE_LOCKED"
    assert d["beam"]["status"] == "IDLE"
    assert d["sim_stamp"] == 42.0
    assert d["banner"] is None        # healthy + ranged/idle => no banner


def test_build_state_all_none_is_null_safe():
    d = metrics.build_state(None, None, None)
    for k in ("north", "east", "alt", "speed", "vspeed", "heading",
              "roll", "pitch", "yaw", "armed", "mode", "batt_pct", "voltage",
              "warn", "cam_stamp", "sim_stamp", "detector", "beam", "track",
              "contacts", "banner"):
        assert d[k] is None
    assert d["cam"] is False and d["cam_seq"] == 0


def test_build_state_surfaces_detector_down_banner():
    snap = _snapshot(detector={"healthy": False, "latency_ms": 0.0})
    d = metrics.build_state(None, None, None, snapshot=snap)
    assert d["detector"]["healthy"] is False
    assert d["banner"] == overlay.SENSING_DEGRADED


def test_battery_unknown_values_become_none():
    batt = SimpleNamespace(remaining=-1.0, voltage_v=0.0, warning=0)
    d = metrics.build_state(None, None, batt)
    assert d["batt_pct"] is None and d["voltage"] is None and d["warn"] == 0


def test_battery_none_fields_do_not_crash():
    # px4_msgs always sends floats, but stay defensive: None must not raise.
    batt = SimpleNamespace(remaining=None, voltage_v=None, warning=0)
    d = metrics.build_state(None, None, batt)
    assert d["batt_pct"] is None and d["voltage"] is None and d["warn"] == 0
