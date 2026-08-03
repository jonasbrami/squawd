"""M4 contract, load-bearing: the 0.5 s overlay match/staleness guard and the
first-class degraded banners (design §3.7, ICD §8.4). Pure logic — the browser
mirrors these exact rules in JS."""
from agents.observatory import overlay


def _snap(sim_stamp=100.0, **kw):
    s = {"sim_stamp": sim_stamp,
         "detector": {"healthy": True, "latency_ms": 30.0},
         "beam": {"status": "IDLE", "target": None, "range_m": None},
         "track": {"state": "IDLE", "target": None, "gap_m": None},
         "contacts": [], "dets": []}
    s.update(kw)
    return s


# ---- match / staleness (gate: >0.5 s overlays are dropped) ----

def test_matching_snapshot_is_drawn():
    assert overlay.overlay_fresh(100.0, _snap(100.0)) is True
    assert overlay.overlay_fresh(100.3, _snap(100.0)) is True
    assert overlay.overlay_fresh(100.0, _snap(100.4)) is True   # snap ahead of frame


def test_boundary_half_second_is_still_fresh():
    assert overlay.overlay_fresh(100.5, _snap(100.0)) is True


def test_stale_snapshot_is_dropped():
    assert overlay.overlay_fresh(100.51, _snap(100.0)) is False
    assert overlay.overlay_fresh(105.0, _snap(100.0)) is False


def test_missing_or_stamp_less_inputs_never_match():
    assert overlay.overlay_fresh(100.0, None) is False
    assert overlay.overlay_fresh(0.0, _snap(100.0)) is False     # pre-first-frame
    assert overlay.overlay_fresh(100.0, _snap(0.0)) is False
    assert overlay.overlay_fresh(100.0, {"detector": {}}) is False


def test_overlay_age_s_reports_none_without_stamps():
    assert abs(overlay.overlay_age_s(100.0, 99.6) - 0.4) < 1e-9
    assert overlay.overlay_age_s(0.0, 99.6) is None
    assert overlay.overlay_age_s(100.0, None) is None


# ---- degraded banners (gate: banner on killed detector) ----

def test_no_snapshot_no_banner():
    assert overlay.hud_banner(None) is None


def test_detector_down_raises_sensing_degraded():
    snap = _snap(detector={"healthy": False, "latency_ms": 0.0})
    assert overlay.hud_banner(snap) == overlay.SENSING_DEGRADED


def test_healthy_detector_idle_track_no_banner():
    assert overlay.hud_banner(_snap()) is None


def test_active_track_without_range_reads_range_unavailable():
    contacts = [{"name": "vis_target_0", "cls": "target", "range_m": None,
                 "range_src": "bearing", "health": "ACQUIRING"}]
    snap = _snap(track={"state": "ACQUIRING", "target": "vis_target_0",
                        "gap_m": None},
                 contacts=contacts)
    assert overlay.hud_banner(snap) == overlay.RANGE_UNAVAILABLE


def test_active_track_with_range_has_no_banner():
    contacts = [{"name": "vis_target_0", "cls": "target", "range_m": 12.4,
                 "range_src": "tof", "health": "MEASURED"}]
    snap = _snap(track={"state": "WORLD_TRACKED", "target": "vis_target_0",
                        "gap_m": 3.1},
                 contacts=contacts)
    assert overlay.hud_banner(snap) is None


def test_detector_down_beats_range_unavailable():
    contacts = [{"name": "vis_target_0", "cls": "target", "range_m": None}]
    snap = _snap(detector={"healthy": False, "latency_ms": 0.0},
                 track={"state": "COASTING", "target": "vis_target_0",
                        "gap_m": None},
                 contacts=contacts)
    assert overlay.hud_banner(snap) == overlay.SENSING_DEGRADED


# ---- click hit-test (W0.3: crowd-safe, server-side, bbox authoritative) ----

def _contacts(*named_boxes):
    return [{"name": n, "cls": "car", "bbox_xyxy": b} for n, b in named_boxes]


def test_hit_test_single_containing_box_wins():
    s = _snap(contacts=_contacts(("vis_car_0", [10, 10, 50, 50]),
                                 ("vis_car_1", [100, 100, 150, 150])))
    r = overlay.hit_test(s, 30.0, 30.0, frame_stamp=100.2)
    assert r == {"contact": "vis_car_0", "bbox_xyxy": [10, 10, 50, 50]}
    r = overlay.hit_test(s, 50.0, 50.0, frame_stamp=100.0)   # edge inclusive
    assert r["contact"] == "vis_car_0"


def test_hit_test_overlapping_crowd_is_ambiguous_never_guessed():
    s = _snap(contacts=_contacts(("vis_car_0", [10, 10, 60, 60]),
                                 ("vis_car_1", [20, 20, 70, 70]),
                                 ("vis_car_2", [200, 200, 250, 250])))
    r = overlay.hit_test(s, 30.0, 30.0, frame_stamp=100.0)
    assert r == {"contact": None, "reason": "ambiguous"}


def test_hit_test_no_containing_box_is_a_miss():
    s = _snap(contacts=_contacts(("vis_car_0", [10, 10, 50, 50]),
                                 ("vis_coasting_0", None)))  # no bbox: never hit
    assert overlay.hit_test(s, 300.0, 300.0, frame_stamp=100.0) == \
        {"contact": None, "reason": "miss"}
    assert overlay.hit_test(s, 30.0, 30.0, frame_stamp=100.0)["contact"] == \
        "vis_car_0"


def test_hit_test_stale_snapshot_is_rejected():
    s = _snap(100.0, contacts=_contacts(("vis_car_0", [10, 10, 50, 50])))
    assert overlay.hit_test(s, 30.0, 30.0, frame_stamp=100.6) == \
        {"contact": None, "reason": "stale"}
    assert overlay.hit_test(s, 30.0, 30.0, frame_stamp=None)["reason"] == "stale"
    assert overlay.hit_test(None, 30.0, 30.0, frame_stamp=100.0)["reason"] == \
        "stale"
