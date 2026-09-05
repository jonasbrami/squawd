"""Vision contacts (ICD §6.4/§6.5): CvEkf gates, birth/rebind/NIS rules,
bearing-only semantics, source-change confirmation, coast/lost on SIM time,
designate seam, deep-copy readers, the designated-vehicle corner-maneuver
mode (codex R7). Synthetic InferenceResults over a fake World (fixed
pose/attitude) — drone at (0, 0, 12) heading north throughout."""
import dataclasses
import math

import pytest

from agents.core.contact import Frame
from agents.core.rangefinder import RangeSample
from agents.perception.projection import vfov_deg
from agents.vision.contacts import CvEkf, TrackerConfig, VisionContacts
from agents.vision.types import AssociationHit, Detection, InferenceResult

W, H = 640, 360
T0, DT = 100.0, 0.2
ALT = 12.0


class FakeWorld:
    """Duck-typed World: fixed pose (0, 0, ALT, heading 0) + level attitude."""

    def pose_at(self, t):
        return (0.0, 0.0, ALT, 0.0)

    def attitude_at(self, t):
        return (0.0, 0.0, 0.0)


def _px(ax, ay):
    """Inverse of projection.pixel_to_angles for a 640x360 / hfov-69 frame."""
    fx = (W / 2) / math.tan(math.radians(69.0) / 2)
    fy = (H / 2) / math.tan(math.radians(vfov_deg(W, H)) / 2)
    return (W / 2 + fx * math.tan(ax), H / 2 + fy * math.tan(ay))


def det_geom(cls, e, n, conf=0.9, alt=ALT):
    """Detection whose footpoint the projection composition (ray_support_range
    -> contact_world, the v1 slant-range convention of projection.py) maps to
    world (e, n) on the mover's base plane (z=0.6 for "target", z=0 else):
    the ray needs SLANT range == horizontal distance, so depression is
    asin((alt-support)/g), not atan2(alt, g)."""
    sup = 0.6 if cls == "target" else 0.0
    g = math.hypot(e, n)
    ax = math.atan2(e, n)
    ay = math.asin(min((alt - sup) / g, 1.0))
    u, v = _px(ax, ay)
    return Detection(cls, conf, (u - 3.0, v - 6.0, u + 3.0, v))


def det_horizon(cls, bearing_deg, conf=0.9):
    """Bearing-only detection: footpoint on the horizon (angle_y=0 -> the
    support-plane ray never converges)."""
    u, v = _px(math.radians(bearing_deg), 0.0)
    return Detection(cls, conf, (u - 3.0, v - 6.0, u + 3.0, v))


_seq = [0]


def result(t, dets, hit=None):
    _seq[0] += 1
    return InferenceResult(Frame(_seq[0], t, W, H, b""), list(dets), 0.0, hit)


def make_vc(rangefinder=None, config=None):
    return VisionContacts(FakeWorld(), rangefinder=rangefinder, config=config)


def birth_geom(vc, e=0.0, n=40.0, cls="target", t0=T0):
    """Two consecutive gated hits -> one born, positioned track; returns name."""
    vc.update(result(t0, [det_geom(cls, e, n)]))
    vc.update(result(t0 + DT, [det_geom(cls, e, n)]))
    views = vc.all_views()
    assert len(views) == 1
    return views[0].name


# ---- TrackerConfig: the defaults ARE test vectors (ICD §6.5) ----

def test_tracker_config_defaults_are_the_contract():
    c = TrackerConfig()
    assert (c.dt_nominal_s, c.v_max_mps, c.gate_m, c.nis_max) == (0.2, 12.0, 5.0, 9.21)
    assert (c.confirm_hits, c.birth_hits) == (2, 2)
    assert (c.coast_s, c.lost_s, c.rebind_window_s) == (1.0, 2.0, 2.0)
    assert (c.sigma_geom_m, c.sigma_tof_m, c.sigma_bearing_deg) == (2.0, 0.15, 1.5)
    assert c.accel_max_mps2 == 4.0
    # codex R7 corner-maneuver mode: DISABLED by default (maneuver_key None)
    # — the mover 2-class path is byte-identical; the knobs below are the
    # contract the COCO profile activates with maneuver_key="vehicle"
    assert c.maneuver_key is None
    assert (c.maneuver_gate_m, c.maneuver_trigger_m,
            c.maneuver_trigger_hits) == (8.0, 1.0, 2)
    assert (c.maneuver_window_s, c.maneuver_accel_mps2,
            c.maneuver_nis_scale) == (2.0, 20.0, 4.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        c.gate_m = 9.0


# ---- CvEkf unit contracts ----

def test_ekf_predict_advances_cv_state_and_grows_covariance():
    ekf = CvEkf(0.0, 0.0, ve=2.0, vn=0.0)
    p0 = ekf.P[0, 0]
    ekf.predict(0.5)
    assert ekf.x[0] == pytest.approx(1.0)          # e += ve*dt
    assert ekf.P[0, 0] > p0                        # process noise injected


def test_ekf_update_xy_rejects_outlier_without_moving_state():
    ekf = CvEkf(0.0, 40.0)
    nis = ekf.update_xy(0.0, 40.5, 2.0)
    assert nis < 9.21 and ekf.x[1] != 40.0         # consistent hit applied
    before = ekf.x.copy(), ekf.P.copy()
    nis = ekf.update_xy(30.0, 80.0, 2.0)
    assert nis > 9.21                              # chi2 2dof 99% gate
    assert (ekf.x == before[0]).all() and (ekf.P == before[1]).all()


def test_ekf_update_range_moves_only_along_the_ray():
    ekf = CvEkf(0.0, 40.0)
    ekf.set_origin(0.0, 0.0)
    nis = ekf.update_range(42.0, 0.0, 0.15)        # beam due north
    assert nis < 9.21
    assert ekf.x[1] == pytest.approx(42.0, abs=0.1)
    assert ekf.x[0] == pytest.approx(0.0, abs=1e-6)
    nis = ekf.update_range(60.0, 0.0, 0.15)
    assert nis > 9.21                              # rejected: no teleport
    assert ekf.x[1] == pytest.approx(42.0, abs=0.1)


def test_ekf_update_bearing_holds_range_moves_angle():
    ekf = CvEkf(0.0, 40.0)
    ekf.set_origin(0.0, 0.0)
    nis = ekf.update_bearing(2.0, 1.5)
    assert nis < 9.21
    assert ekf.x[0] > 0.0                          # angle moved toward +2 deg
    assert ekf.x[1] == pytest.approx(40.0, abs=0.5)  # range ~held


# ---- birth / gating rules ----

def test_birth_needs_two_consecutive_gated_hits():
    vc = make_vc()
    vc.update(result(T0, [det_geom("target", 3.0, 40.0)]))
    assert vc.all_views() == [] and vc.poses() == {}     # one hit births nothing
    vc.update(result(T0 + DT, []))                        # miss breaks the streak
    vc.update(result(T0 + 2 * DT, [det_geom("target", 3.0, 40.0)]))
    assert vc.all_views() == []                           # still only one hit
    vc.update(result(T0 + 3 * DT, [det_geom("target", 3.0, 40.0)]))
    views = vc.all_views()
    assert len(views) == 1 and views[0].name == "vis_target_0"
    e, n, z = vc.poses()["vis_target_0"]
    assert e == pytest.approx(3.0, abs=0.75)
    assert n == pytest.approx(40.0, abs=0.75)
    assert z == 0.6                                       # "target" base plane
    assert views[0].health == "MEASURED"
    assert views[0].position_src == "measured"


def test_outlier_is_rejected_and_does_not_move_the_estimate():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    e0, n0, _ = vc.poses()[name]
    vc.update(result(T0 + 2 * DT, [det_geom("target", 25.0, 60.0)]))  # ~32 m away
    e1, n1, _ = vc.poses()[name]
    assert e1 == pytest.approx(e0, abs=1e-6)
    assert n1 == pytest.approx(n0, abs=1e-6)
    assert len(vc.all_views()) == 1                      # no visible twin track


# ---- bearing-only semantics ----

def test_bearing_only_birth_is_acquiring_and_not_in_poses():
    vc = make_vc()
    vc.update(result(T0, [det_horizon("target", 5.0)]))
    vc.update(result(T0 + DT, [det_horizon("target", 5.0)]))
    assert vc.poses() == {} and vc.velocities() == {}
    views = vc.all_views()
    assert len(views) == 1
    v = views[0]
    assert (v.e, v.n, v.z) == (None, None, None)
    assert v.position_src == "none" and v.range_src == "bearing"
    assert v.range_m is None and v.health == "ACQUIRING"
    assert v.bearing_deg == pytest.approx(5.0, abs=0.1)


def test_positioned_track_slipping_to_bearing_only_keeps_prediction():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    t = T0 + 2 * DT
    for k in range(6):                                    # bearing-only interval
        t += DT
        vc.update(result(t, [det_horizon("target", 0.0)]))
    # t - t_meas = 1.2 s > coast_s: predicted position, COASTING, still in poses
    assert name in vc.poses()
    assert vc.health(name) == "COASTING"
    v = vc.observation(name)
    assert v.position_src == "predicted"
    assert v.range_src == "bearing" and v.z is None and v.range_m is None
    assert v.e == pytest.approx(0.0, abs=1.0)
    assert v.n == pytest.approx(40.0, abs=1.0)


# ---- source-change confirmation ----

class FakeRangefinder:
    """Duck-typed RangeProvider: one scripted VALID sample, robustly joined."""

    def __init__(self, range_m, sample_time):
        self.set(range_m, sample_time)

    def set(self, range_m, sample_time):
        self._s = RangeSample(sample_time, 0.0, range_m, 0.2, 100.0, 0.0087,
                              1.0, "VALID", 1)

    def latest(self):
        return self._s

    def robust_at(self, t, **kw):
        return self._s if abs(self._s.sample_time - t) <= 0.05 else None


class LowWorld(FakeWorld):
    """Hover LOW (3 m) so a ground track sits inside the fusion envelope
    (|dz| = |0.6 - 3| <= 3) — M3b's envelope blocks geom->tof transitions
    from 12 m alt by design (the forward beam is for co-altitude targets)."""

    def pose_at(self, t):
        return (0.0, 0.0, 3.0, 0.0)


def test_first_tof_fuses_with_incumbent_sigma_and_does_not_flip_src():
    """M3b path (designation + reserved det + envelope + beam association):
    the FIRST tof hit on a geom-ranged track fuses with the INCUMBENT geom
    sigma (no teleport) and the source does not change hands yet (ICD §6.5
    confirm rule, safety half)."""
    rf = FakeRangefinder(24.0, T0 + 2 * DT)              # beam point (0, 24)
    vc = VisionContacts(LowWorld(), rangefinder=rf)
    vc.set_beam_context(mode="shadow", own_speed_mps=0.0)
    # geom-born track at (0, 20), bearing 0 == boresight: dep = asin(2.4/20)
    fy = (H / 2) / math.tan(math.radians(vfov_deg(W, H)) / 2)
    v = H / 2 + fy * math.tan(math.asin((3.0 - 0.6) / 20.0))
    det = Detection("target", 0.9, (317.0, v - 6, 323.0, v))
    vc.update(result(T0, [det]))
    vc.update(result(T0 + DT, [det]))
    (name,) = vc.poses()
    assert vc.observation(name).range_src == "geom"
    vc.designate(name)
    t = T0 + 2 * DT
    # the reserved det under the beam + a fresh VALID sample
    hit = AssociationHit(0, (310.0, 170.0, 330.0, 190.0), (320.0, 190.0),
                         0.8)
    vc.update(result(t, [Detection("target", 0.9,
                                   (310.0, 170.0, 330.0, 190.0))], hit=hit))
    n1 = vc.poses()[name][1]
    assert 20.5 < n1 < 23.0        # fused with INCUMBENT geom sigma: no teleport
    assert vc.observation(name).range_src == "geom"       # not confirmed yet
    assert vc.ranges()[name][1:] == ("geom", 2.0)


def test_source_change_confirm_rule_unit():
    """The confirm rule itself (ICD §6.5), exercised directly: confirm_hits
    CONSECUTIVE hits of a challenging source switch the covariance; an
    interleaved hit of any other kind breaks the streak. (Post-M3b the
    geom->tof flip is unreachable via update() — the envelope + reserved-det
    gates mean co-altitude tracks are BORN tof and ground tracks are
    OUT_OF_ENVELOPE — so the rule's reachable surface is this one.)"""
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    tr = vc._tracks[name]
    assert vc._peek_sigma(tr, "tof") == 2.0       # incumbent defends covariance
    vc._commit_source(tr, "tof")
    assert tr.src == "geom" and tr.pending_n == 1
    vc._commit_source(tr, "tof")                  # second CONSECUTIVE tof
    assert tr.src == "tof" and vc._peek_sigma(tr, "tof") == 0.15
    tr2 = vc._tracks[name]
    tr2.src, tr2.pending_src, tr2.pending_n = "geom", None, 0
    vc._commit_source(tr2, "tof")
    vc._commit_source(tr2, "bearing")             # interleaved: streak broken
    vc._commit_source(tr2, "tof")
    assert tr2.src == "geom" and tr2.pending_n == 1


# ---- name rebind ----

def starve(vc, t_from, t_to):
    t = t_from
    while t < t_to:
        t += DT
        vc.update(result(t, []))
    return t


def test_rebind_inside_window_resumes_the_same_name():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    t = starve(vc, T0 + DT, T0 + 12 * DT)                 # age 2.2 s -> dropped
    assert vc.poses() == {} and vc.health(name) == "LOST"
    t += 4 * DT                                           # 0.8 s after the drop
    vc.update(result(t, [det_geom("target", 1.0, 41.0)]))
    vc.update(result(t + DT, [det_geom("target", 1.0, 41.0)]))
    assert "vis_target_0" in vc.poses()                   # same name resumes
    assert "vis_target_1" not in vc.poses()
    e, n, _ = vc.poses()["vis_target_0"]
    assert e == pytest.approx(1.0, abs=0.75)
    assert n == pytest.approx(41.0, abs=0.75)


def test_rebind_after_window_gets_a_new_name():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    t = starve(vc, T0 + DT, T0 + 12 * DT)                 # dropped at age > 2 s
    t += 14 * DT                                          # 2.8 s later: window shut
    vc.update(result(t, [det_geom("target", 1.0, 41.0)]))
    vc.update(result(t + DT, [det_geom("target", 1.0, 41.0)]))
    assert "vis_target_1" in vc.poses()
    assert "vis_target_0" not in vc.poses()


# ---- coast / lost on SIM time ----

def test_coast_then_lost_uses_sim_time_from_the_frame_stamp():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    t = starve(vc, T0 + DT, T0 + 4 * DT)                  # age 0.6 <= coast_s
    assert vc.health(name) == "MEASURED"
    t = starve(vc, t, T0 + 7 * DT)                        # age 1.2 > coast_s
    assert vc.health(name) == "COASTING"
    assert name in vc.poses()
    t = starve(vc, t, T0 + 12 * DT)                       # age 2.2 > lost_s
    assert vc.poses() == {}
    assert vc.health(name) == "LOST"
    assert vc.all_views() == []
    assert vc.sim_time() == pytest.approx(T0 + 12 * DT)   # sim, never wall clock


# ---- velocities ----

def test_velocities_track_a_constant_velocity_target():
    vc = make_vc()
    for k in range(12):
        t = T0 + k * DT
        vc.update(result(t, [det_geom("target", 2.0 * (t - T0), 40.0)]))
    (name,) = vc.velocities()
    ve, vn = vc.velocities()[name]
    assert ve == pytest.approx(2.0, abs=0.75)
    assert vn == pytest.approx(0.0, abs=0.75)


def test_velocities_empty_for_bearing_only():
    vc = make_vc()
    vc.update(result(T0, [det_horizon("target", 5.0)]))
    vc.update(result(T0 + DT, [det_horizon("target", 5.0)]))
    assert vc.velocities() == {}


# ---- ranges / designate / reset / readers ----

def test_ranges_reports_fused_ranges_per_source():
    """M3b: ranges() is filled (ICD §5.1 extended read) — {name: (range_m,
    src, sigma)} for positioned tracks holding a fused range."""
    vc = make_vc()
    name = birth_geom(vc)
    rng, src, sigma = vc.ranges()[name]
    assert rng == pytest.approx(40.0, abs=0.75)
    assert (src, sigma) == ("geom", 2.0)
    # bearing-only tracks hold no fused range: absent from ranges()
    vc2 = make_vc()
    vc2.update(result(T0, [det_horizon("target", 5.0)]))
    vc2.update(result(T0 + DT, [det_horizon("target", 5.0)]))
    assert vc2.ranges() == {}


class FakeDetector:
    def __init__(self):
        self.calls = []

    def request_lock(self, seed_xy=None, seed_index=None):
        self.calls.append(("request_lock", seed_xy, seed_index))

    def clear_lock(self):
        self.calls.append(("clear_lock",))


def test_designate_never_crashes_unwired_and_drives_detector_hooks():
    vc = make_vc()
    vc.designate("vis_target_0", support_z=0.5)           # no detector: no-op
    vc.clear_designation()
    name = birth_geom(vc, 0.0, 40.0)
    det = FakeDetector()
    vc.attach_detector(det)
    vc.designate(name, support_z=0.5)
    assert det.calls[0][0] == "request_lock"
    assert det.calls[0][1] is not None                    # seed footpoint px
    assert vc._tracks[name].support_z == 0.5
    vc.clear_designation()
    assert det.calls[-1] == ("clear_lock",)


def test_designated_hit_feeds_the_designated_track():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    vc.designate(name)
    ax = math.atan2(0.0, 41.0)
    ay = math.asin((ALT - 0.6) / 41.0)   # cls "target" base plane z=0.6
    hit = AssociationHit(None, None, _px(ax, ay), 0.8)
    vc.update(result(T0 + 2 * DT, [], hit=hit))
    # one geom-sigma fusion step toward (0, 41): K = 5.48/9.48 -> +0.58 m
    assert vc.poses()[name][1] == pytest.approx(40.58, abs=0.1)
    assert vc.health(name) == "MEASURED"


def test_reset_clears_everything():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    vc.designate(name)
    vc.reset()
    assert vc.poses() == {} and vc.all_views() == [] and vc.velocities() == {}
    assert vc.sim_time() == 0.0
    assert vc.health(name) == "LOST"
    assert birth_geom(vc, 0.0, 40.0) == "vis_target_0"    # counters reset too


def test_readers_return_independent_copies():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    p = vc.poses()
    p.clear()
    assert name in vc.poses()
    views = vc.all_views()
    views.pop()
    assert len(vc.all_views()) == 1
    v = vc.observation(name)
    with pytest.raises(dataclasses.FrozenInstanceError):
        v.e = 99.0                                        # frozen read model
    assert vc.observation(name).e != 99.0


# ---- cockpit snapshot seam: beam_view/track_view (ICD §6.7) ----

def _beam_last(vc, status):
    from agents.vision.beam import BeamAssociation
    vc._beam_last = BeamAssociation(status, 0, None, 4.0, "test")


def test_beam_track_views_idle_without_designation():
    vc = make_vc()
    assert vc.beam_view() == {"status": "IDLE", "target": None,
                              "range_m": None}
    assert vc.track_view() == {"state": "IDLE", "target": None,
                               "gap_m": None}


def test_beam_view_searches_then_locks_with_fused_range():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    vc.designate(name)
    # no association cycle yet -> SEARCHING
    assert vc.beam_view()["status"] == "SEARCHING"
    from agents.vision.beam import (AMBIGUOUS, ASSOCIATED, EDGE, NO_SAMPLE,
                                    OUT_OF_ENVELOPE)
    _beam_last(vc, ASSOCIATED)
    b = vc.beam_view()
    assert b["status"] == "LOCKED" and b["target"] == name
    assert b["range_m"] == pytest.approx(40.0, abs=1.0)   # the fused range
    _beam_last(vc, AMBIGUOUS)
    assert vc.beam_view()["status"] == "EDGE-MIX"
    _beam_last(vc, EDGE)
    assert vc.beam_view()["status"] == "EDGE-MIX"
    _beam_last(vc, OUT_OF_ENVELOPE)
    assert vc.beam_view()["status"] == "OUT-OF-ENVELOPE"
    _beam_last(vc, NO_SAMPLE)
    assert vc.beam_view()["status"] == "NO-RETURN"
    vc.clear_designation()
    assert vc.beam_view()["status"] == "IDLE"


def test_track_view_sm_states_and_lost():
    vc = make_vc()
    name = birth_geom(vc, 0.0, 40.0)
    vc.designate(name)
    tv = vc.track_view()
    assert tv["state"] == "ACQUIRING" and tv["target"] == name  # DESIGNATED folds
    assert tv["gap_m"] == pytest.approx(40.0, abs=1.0)
    vc._sm_state = "RANGE_LOCKED"
    assert vc.track_view()["state"] == "RANGE_LOCKED"
    vc._sm_state = "COASTING"
    assert vc.track_view()["state"] == "COASTING"
    vc._tracks.pop(name)                                   # track died
    assert vc.track_view()["state"] == "LOST"
    vc.clear_designation()
    assert vc.track_view()["state"] == "IDLE"


def _bearing_only_designated(vc, name_seed=0.0):
    """Birth a bearing-only track (two horizon hits), designate it."""
    vc.update(result(T0, [det_horizon("target", name_seed)]))
    vc.update(result(T0 + DT, [det_horizon("target", name_seed)]))
    name = vc.all_views()[0].name
    assert vc.observation(name).position_src == "none"
    vc.designate(name)
    return name


def test_tof_birth_crosschecked_against_bbox_height_cue():
    """M3b v8.2 guard: the first (bearing-only) ToF lock has no EKF prediction
    for the 3σ consistency gate — the bbox-height cue is the plausibility
    check. A VALID sample agreeing with the cue births a tof position; one
    disagreeing >3σ (background through the mask) is a slip with NO birth."""
    fy = (H / 2) / math.tan(math.radians(vfov_deg(W, H)) / 2)
    px_h = 66.0
    rng_h = 1.2 * fy / px_h                      # ~8.5 m for the 1.2 m box
    t = T0 + 2 * DT
    # tall box whose ERODED region contains the boresight disc (footpoint
    # v=200: 22% erosion of 66 px = 14.5 px -> the bottom edge sits at 185.5)
    box = (312.0, 200.0 - px_h, 328.0, 200.0)

    # cue-consistent sample -> BIRTH
    rf = FakeRangefinder(rng_h + 0.2, t)
    vc = VisionContacts(LowWorld(), rangefinder=rf)
    vc.set_beam_context(mode="shadow", own_speed_mps=0.0)
    name = _bearing_only_designated(vc)
    hit = AssociationHit(0, box, (320.0, 200.0), 0.9)
    vc.update(result(t, [Detection("target", 0.9, box)], hit=hit))
    assert name in vc.poses()
    assert vc.poses()[name][1] == pytest.approx(rng_h + 0.2, abs=0.3)
    assert vc.ranges()[name][1] == "tof"

    # cue-DISAGREEING sample (40 m background through the mask) -> NO birth
    rf2 = FakeRangefinder(40.0, t)
    vc2 = VisionContacts(LowWorld(), rangefinder=rf2)
    vc2.set_beam_context(mode="shadow", own_speed_mps=0.0)
    name2 = _bearing_only_designated(vc2)
    vc2.update(result(t, [Detection("target", 0.9, box)], hit=hit))
    assert name2 not in vc2.poses()              # no ghost position
    assert vc2._sm_state != "RANGE_LOCKED"


def test_support_plane_geom_rejected_below_min_drop():
    """M3b v8.6: below _MIN_DROP_M of alt-support drop, the EKF alt bias
    multiplies through the 1/sin(dep) lever into a >100% range bias — the
    honest product is bearing-only, never a ghost position (the pursuit
    flies ghosts)."""
    class VeryLowWorld(FakeWorld):
        def pose_at(self, t):
            return (0.0, 0.0, 1.5, 0.0)          # drop 0.9 vs support 0.6

    # steep depression that WOULD be geom from altitude: aim the footpoint
    # at ~20 deg below boresight (well past the 6 deg pitch lever)
    vc = VisionContacts(VeryLowWorld())
    fy = (H / 2) / math.tan(math.radians(vfov_deg(W, H)) / 2)
    v = H / 2 + fy * math.tan(math.radians(20.0))
    det = Detection("target", 0.9, (317.0, v - 6.0, 323.0, v))
    vc.update(result(T0, [det]))
    vc.update(result(T0 + DT, [det]))
    views = vc.all_views()
    assert len(views) == 1
    assert views[0].position_src == "none"       # bearing-only, no ghost
    assert vc.poses() == {}


# ---- W2: admission allowlist + two-hit confirm (design 2026-07-28 §4) ----

def test_two_hit_confirm_gates_trackable_contacts():
    """One sighting is a candidate, not a contact; the second consecutive
    gated hit births it (TrackerConfig.birth_hits=2)."""
    vc = VisionContacts(FakeWorld(), admit_classes=("car",))
    vc.update(result(T0, [det_geom("car", 0.0, 40.0)]))
    assert vc.all_views() == []                    # first sighting: no contact
    vc.update(result(T0 + DT, [det_geom("car", 0.0, 40.0)]))
    views = vc.all_views()
    assert len(views) == 1 and views[0].name == "vis_car_0"


def test_allowlist_never_admits_static_classes():
    """The §4 dynamic-class allowlist: a static class (the W0.1 "chair"
    look-down artifact) never becomes a contact, however often seen; an
    admitted class births normally."""
    vc = VisionContacts(FakeWorld(), admit_classes=("car", "person"))
    for k in range(4):
        vc.update(result(T0 + k * DT, [det_geom("chair", 0.0, 40.0)]))
    assert vc.all_views() == []
    name = birth_geom(vc, cls="car")
    assert name == "vis_car_0"


def test_no_allowlist_admits_every_class():
    """Legacy default (admit_classes=None): no filtering — the M0→M6 mover
    path and direct constructions keep their pre-W2 behavior."""
    vc = VisionContacts(FakeWorld())
    name = birth_geom(vc, cls="chair")
    assert name == "vis_chair_0"


# ---- W3 (codex §1/§2): COCO superclass association keys + 5 s grace ----

COCO_CFG = TrackerConfig(lost_s=5.0, rebind_window_s=5.0,
                         assoc_keys={"car": "vehicle", "truck": "vehicle",
                                     "bus": "vehicle"})


def test_coco_vehicle_class_flap_keeps_one_contact_id():
    """The W3 demo-world churn, fixed: with the COCO superclass map a
    car<->truck flap on ONE physical vehicle keeps ONE contact id — through
    the candidate confirm AND the live association — and the contact keeps
    the BIRTH class/name (vis_car_0, never vis_truck_*)."""
    vc = make_vc(config=COCO_CFG)
    vc.update(result(T0, [det_geom("car", 0.0, 40.0)]))
    vc.update(result(T0 + DT, [det_geom("truck", 0.2, 40.1)]))  # flap pre-birth
    views = vc.all_views()
    assert len(views) == 1 and views[0].name == "vis_car_0"    # birth class wins
    t = T0 + DT
    for _ in range(3):                                          # flap while live
        t += DT
        vc.update(result(t, [det_geom("truck", 0.3, 40.2)]))
    views = vc.all_views()
    assert len(views) == 1
    assert views[0].name == "vis_car_0" and views[0].cls == "car"
    assert vc.health("vis_car_0") == "MEASURED"


def test_default_mover_classes_remain_distinct():
    """The empty-map default is the mover contract, byte-identical: class
    gates stay STRICT — co-located target + obstacle dets never cross-feed a
    candidate or a track (two contacts, per-class names)."""
    vc = make_vc()
    for k in range(2):
        vc.update(result(T0 + k * DT, [det_geom("target", 0.0, 40.0),
                                       det_geom("obstacle", 0.1, 40.1)]))
    assert sorted(vc.poses()) == ["vis_obstacle_0", "vis_target_0"]


def test_coco_profile_survives_four_second_flicker_then_drops_after_five():
    """The COCO 5 s grace (codex §2): a 4 s detection gap leaves the contact
    alive (COASTING, still posed); past 5 s it drops to the graveyard as
    before. The mover default (2.0) is pinned by
    test_tracker_config_defaults_are_the_contract above."""
    vc = make_vc(config=COCO_CFG)
    name = birth_geom(vc, 0.0, 40.0, cls="car")           # last seen T0 + DT
    t = starve(vc, T0 + DT, T0 + 21 * DT)                 # age 4.0 s
    assert name in vc.poses()
    assert vc.health(name) == "COASTING"
    t = starve(vc, t, T0 + 27 * DT)                       # age 5.2 s > lost_s
    assert vc.poses() == {}
    assert vc.health(name) == "LOST"


def test_coco_rebind_across_vehicle_classes_resumes_the_name():
    """The second half of the live churn: the car track drops after the 5 s
    grace and the SAME object re-detected as a TRUCK inside the 5 s rebind
    window resumes vis_car_0 (the graveyard gate compares association keys)
    instead of birthing a fresh id."""
    vc = make_vc(config=COCO_CFG)
    birth_geom(vc, 0.0, 40.0, cls="car")
    t = starve(vc, T0 + DT, T0 + 27 * DT)                 # dropped at age 5.2 s
    assert vc.poses() == {}
    t += 4 * DT                                           # 0.8 s after the drop
    vc.update(result(t, [det_geom("truck", 1.0, 41.0)]))
    vc.update(result(t + DT, [det_geom("truck", 1.0, 41.0)]))
    assert "vis_car_0" in vc.poses()                      # lineage resumed
    assert "vis_truck_0" not in vc.poses()
    e, n, _ = vc.poses()["vis_car_0"]
    assert e == pytest.approx(1.0, abs=0.75)
    assert n == pytest.approx(41.0, abs=0.75)


# ---- codex R7: designated-vehicle corner-maneuver mode (w3-run6) ----

COCO_MAN_CFG = TrackerConfig(lost_s=5.0, rebind_window_s=5.0,
                             assoc_keys={"car": "vehicle", "truck": "vehicle",
                                         "bus": "vehicle"},
                             maneuver_key="vehicle")


def east_mover(vc, t0=T0, n=12, e0=8.0, n0=44.0):
    """Birth + converge a DESIGNATED 4 m/s eastbound car track (the corner
    scenarios' starting point); returns (name, t_of_last_frame)."""
    t = t0
    for k in range(n):
        t = t0 + k * DT
        vc.update(result(t, [det_geom("car", e0 + 4.0 * (t - t0), n0)]))
    (name,) = vc.poses()
    vc.designate(name)
    return name, t


def corner_north(vc, name, t, frames):
    """Feed a 90 deg east->north waypoint corner from (current e, 44) at
    4 m/s; returns the corner's e."""
    e = 8.0 + 4.0 * (t - T0)
    for k in range(1, frames + 1):
        vc.update(result(t + k * DT,
                         [det_geom("car", e, 44.0 + 4.0 * k * DT)]))
    return e


def test_ekf_predict_q_scale_scales_process_noise_not_state():
    """The codex-R7 predict hook: q_scale multiplies ONLY the Q injection —
    1.0 by default (the byte-identical mover path), 25 = (20/4)^2 armed."""
    a = CvEkf(0.0, 40.0, ve=4.0)
    b = CvEkf(0.0, 40.0, ve=4.0)
    a.predict(0.2)                                # default: exactly 1
    b.predict(0.2, q_scale=25.0)
    assert (a.x == b.x).all()                     # state propagation untouched
    shared = 0.2 ** 2 * 36.0                      # F P F^T: dt^2 * sigma_vel^2
    dp_a = a.P[0, 0] - 4.0                        # minus sigma_pos^2 init
    dp_b = b.P[0, 0] - 4.0
    assert dp_b - shared == pytest.approx(25.0 * (dp_a - shared))


def test_maneuver_trigger_arms_on_sign_consistent_lateral_innovations():
    """The arming half (R7 §2): two consecutive frames, <=0.35 s apart, whose
    UNIQUE same-superclass geom hit departs SIDEWAYS from the CV prediction
    (|cross_m| >= 1 m, same nonzero sign) — a turn, not range noise."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    assert tr.man_armed_t is None
    e = corner_north(vc, name, t, 2)
    assert tr.man_armed_t is None                 # one qualifying frame only
    vc.update(result(t + 3 * DT, [det_geom("car", e, 44.0 + 4.0 * 3 * DT)]))
    assert tr.man_armed_t == pytest.approx(t + 3 * DT)   # armed on the 2nd hit
    assert vc.health(name) == "MEASURED"


def test_maneuver_trigger_stays_disarmed_on_alternating_signs():
    """Sign consistency: lateral innovations alternating +/-/+ restart the
    streak every frame — no arming (a jittering box is not a turn)."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    e = 8.0 + 4.0 * (t - T0)
    jogs = [(e - 0.8, 45.6), (e + 0.4, 43.6), (e - 0.6, 46.4)]
    for k, (je, jn) in enumerate(jogs, start=1):
        vc.update(result(t + k * DT, [det_geom("car", je, jn)]))
    assert tr.man_armed_t is None
    assert tr.man_trig_n == 1                     # streak never passed one


def test_maneuver_trigger_stays_disarmed_on_ambiguous_pair():
    """Two qualifying vehicle hits in the frame = ambiguity: an explicit
    no-op (the three-car swap risk dominates) — the streak resets."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    e = 8.0 + 4.0 * (t - T0)
    for k in range(1, 3):
        vc.update(result(t + k * DT,
                         [det_geom("car", e - 0.8, 44.0 + 4.0 * k * DT),
                          det_geom("truck", e + 0.6, 44.3 + 4.0 * k * DT)]))
    assert tr.man_armed_t is None
    assert tr.man_trig_n == 0


def test_corner_recapture_accepts_unique_car_to_truck_measurement():
    """The recapture half (R7 §3): while armed, the UNIQUE trigger-qualified
    hit — here a car->truck flap 6 m off the prediction, past the 5 m NN
    gate — is reserved for the designated track and admitted through
    distance<=8 m; the contact keeps its birth id/class and NO candidate or
    new id is born from the consumed measurement. The default profile
    (maneuver_key None) rejects the same hit to the candidate pool."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    e = corner_north(vc, name, t, 3)              # arms on the 3rd frame
    assert tr.man_armed_t is not None
    pe, pn, _ = vc.poses()[name]
    ve, vn = vc.velocities()[name]
    px, py = pe + ve * DT, pn + vn * DT           # next prediction
    tk = t + 4 * DT
    d = math.hypot(-1.0, 6.0)
    assert 5.0 < d <= 8.0                         # past NN gate, inside maneuver
    vc.update(result(tk, [det_geom("truck", px - 1.0, py + 6.0)]))
    assert tr.t_meas == pytest.approx(tk)         # fused through the widen
    assert sorted(vc.poses()) == [name]
    assert vc.observation(name).cls == "car"      # birth class/id kept
    assert vc._candidates == []                   # consumed: no new id born
    # same play, default profile: maneuver never arms, the 6 m hit is refused
    vc2 = make_vc()
    name2, t2 = east_mover(vc2)
    corner_north(vc2, name2, t2, 3)
    tr2 = vc2._tracks[name2]
    assert tr2.man_armed_t is None
    pe, pn, _ = vc2.poses()[name2]
    ve, vn = vc2.velocities()[name2]
    px2, py2 = pe + ve * DT, pn + vn * DT
    t_meas2 = tr2.t_meas
    vc2.update(result(t2 + 4 * DT, [det_geom("truck", px2 - 1.0, py2 + 6.0)]))
    assert tr2.t_meas == pytest.approx(t_meas2)   # rejected: nothing fused
    assert len(vc2._candidates) == 1              # feeds the candidate pool


def test_corner_recapture_refuses_two_plausible_vehicles():
    """Two qualifying vehicles while armed: recover NEITHER (R7 §3) — both
    stay out of the designated track (no fusion this frame) and flow to the
    candidate pool; the window stays armed (ambiguity is not a reset)."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    corner_north(vc, name, t, 3)                  # armed
    pe, pn, _ = vc.poses()[name]
    ve, vn = vc.velocities()[name]
    px, py = pe + ve * DT, pn + vn * DT
    tk = t + 4 * DT
    t_seen, t_meas = tr.t_seen, tr.t_meas
    vc.update(result(tk, [det_geom("truck", px - 1.0, py + 6.2),
                          det_geom("car", px + 0.5, py + 6.6)]))
    assert tr.t_seen == pytest.approx(t_seen)     # neither admitted...
    assert tr.t_meas == pytest.approx(t_meas)
    assert len(vc.all_views()) == 1               # ...no new id born...
    assert len(vc._candidates) == 2               # ...both parked as candidates
    assert tr.man_armed_t is not None             # window still running


def test_armed_window_admits_designated_hit_through_widened_nis():
    """The widen covers the designated force-associated path (R7 §3): that
    path bypasses the NN distance gate and is NIS-gated ALONE, so the armed
    cap is nis_scale x nis_max. A designated hit 10 m off the prediction
    (NIS ~17: above 9.21, below 4x9.21) fuses ONLY while armed; unarmed, the
    same hit is geom-rejected (bearing fallback keeps the angle, no range)."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    corner_north(vc, name, t, 3)                  # armed
    pe, pn, _ = vc.poses()[name]
    ve, vn = vc.velocities()[name]
    px, py = pe + ve * DT, pn + vn * DT
    me, mn = px, py + 10.0
    g = math.hypot(me, mn)
    hit = AssociationHit(None, None,
                         _px(math.atan2(me, mn), math.asin(ALT / g)),
                         0.8)
    tk = t + 4 * DT
    vc.update(result(tk, [], hit=hit))
    assert tr.t_meas == pytest.approx(tk)         # widened NIS admitted it
    assert vc.poses()[name][1] > py + 3.0         # fused toward the hit
    # same hit, window NOT armed (one corner frame only): geom-rejected
    vc2 = make_vc(config=COCO_MAN_CFG)
    name2, t2 = east_mover(vc2)
    corner_north(vc2, name2, t2, 1)
    tr2 = vc2._tracks[name2]
    assert tr2.man_armed_t is None
    pe, pn, _ = vc2.poses()[name2]
    ve, vn = vc2.velocities()[name2]
    px2, py2 = pe + ve * DT, pn + vn * DT
    me2, mn2 = px2, py2 + 10.0
    g2 = math.hypot(me2, mn2)
    hit2 = AssociationHit(None, None,
                          _px(math.atan2(me2, mn2), math.asin(ALT / g2)),
                          0.8)
    t_meas2 = tr2.t_meas
    vc2.update(result(t2 + 2 * DT, [], hit=hit2))
    assert tr2.t_meas == pytest.approx(t_meas2)   # no range fused
    assert vc2.poses()[name2][1] < mn2 - 5.0      # ghost did not jump


def test_maneuver_q_resets_after_three_nominal_hits():
    """The reset half (R7 §2): three NORMAL-gate accepted hits = the filter
    re-converged after the turn — disarm to the proven straight-line filter;
    from then on a 6 m hit is past the (restored) 5 m gate again."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    e = corner_north(vc, name, t, 3)
    assert tr.man_armed_t == pytest.approx(t + 3 * DT)
    assert tr.man_normal == 1                     # the arming frame's own hit
    vc.update(result(t + 4 * DT, [det_geom("car", e, 44.0 + 4.0 * 4 * DT)]))
    assert tr.man_armed_t is not None and tr.man_normal == 2
    vc.update(result(t + 5 * DT, [det_geom("car", e, 44.0 + 4.0 * 5 * DT)]))
    assert tr.man_armed_t is None                 # third normal hit: disarmed
    pe, pn, _ = vc.poses()[name]
    ve, vn = vc.velocities()[name]
    px, py = pe + ve * DT, pn + vn * DT
    t_seen = tr.t_seen
    vc.update(result(t + 6 * DT, [det_geom("car", px, py + 6.0)]))
    assert tr.t_seen == pytest.approx(t_seen)     # normal gates resumed
    assert len(vc._candidates) == 1


def test_maneuver_window_expires_after_two_seconds():
    """The hard timeout (R7 §2): window_s without re-convergence disarms on
    its own — a late 6 m qualifier is then refused like any other outlier."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    tr = vc._tracks[name]
    corner_north(vc, name, t, 3)
    armed_t = tr.man_armed_t
    t2 = starve(vc, t + 3 * DT, armed_t + 1.8)
    assert tr.man_armed_t is not None             # window still running
    t2 = starve(vc, t2, armed_t + 2.2)
    assert tr.man_armed_t is None                 # hard timeout fired
    pe, pn, _ = vc.poses()[name]
    ve, vn = vc.velocities()[name]
    px, py = pe + ve * DT, pn + vn * DT
    t_seen = tr.t_seen
    vc.update(result(t2 + DT, [det_geom("car", px - 1.0, py + 6.0)]))
    assert tr.t_seen == pytest.approx(t_seen)     # refused: window shut
    assert len(vc._candidates) == 1


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def test_armed_maneuver_rotates_velocity_through_the_turn():
    """The payoff (R7 §2/§3): fed through the armed window, the turned
    measurements rotate the CV velocity ~90 deg (eastbound -> northbound at
    4 m/s) instead of ghosting straight on — the heading change is the point."""
    vc = make_vc(config=COCO_MAN_CFG)
    name, t = east_mover(vc)
    ve0, vn0 = vc.velocities()[name]
    assert ve0 > 3.5 and abs(vn0) < 0.5           # eastbound
    corner_north(vc, name, t, 12)                 # corner + the north leg
    ve1, vn1 = vc.velocities()[name]
    assert vn1 > 3.0 and abs(ve1) < 1.0           # rotated to northbound
    h0 = math.degrees(math.atan2(ve0, vn0))
    h1 = math.degrees(math.atan2(ve1, vn1))
    assert abs(_wrap(h1 - h0)) > 60.0


# The corner end-to-end at fixture level (R7 §6's sub-gate shape). LowWorld
# (3 m) puts the lap at demo ranges; the far leg's (18, 26) corner region
# trips the projection's own 6 deg depression honesty lever (g > 28.6 m ->
# bearing-only) — the same ground-plane degradation the demo's receding
# boxes produced (w3-run6 §3), which is what makes the corner FATAL for the
# default profile here: with continuous geom the CV-EKF + bearing fallback
# re-captures any single corner at fixture level (verified).
LAP_CORNERS = ((8.0, 16.0), (18.0, 16.0), (18.0, 26.0), (8.0, 26.0))


def lap_points(t0=T0, legs=5, speed=4.0):
    """Waypoints of the square lap at constant speed with instantaneous 90
    deg corners (the demo mover's waypoint model): (t, e, n) every DT."""
    pts = []
    t = t0
    for k in range(legs):
        (e0, n0), (e1, n1) = (LAP_CORNERS[k % 4], LAP_CORNERS[(k + 1) % 4])
        de, dn = e1 - e0, n1 - n0
        steps = max(int(round(math.hypot(de, dn) / (speed * DT))), 1)
        for s in range(steps):
            f = s / steps
            pts.append((t, e0 + f * de, n0 + f * dn))
            t += DT
    return pts


def lap_corner_frames(pts):
    """Frame indices where the heading changes (the four corners)."""
    out = []
    for i in range(2, len(pts)):
        d0 = (round(pts[i - 1][1] - pts[i - 2][1], 9),
              round(pts[i - 1][2] - pts[i - 2][2], 9))
        d1 = (round(pts[i][1] - pts[i - 1][1], 9),
              round(pts[i][2] - pts[i - 1][2], 9))
        if d1 != d0:
            out.append(i)
    return out


def test_coco_designated_track_survives_four_right_angle_corners_with_one_id():
    """R7's corner sub-gate at fixture level: the 4 m/s cornering mover
    keeps ONE contact id on the COCO profile through all four 90 deg corners
    — never LOST, no second id born — with MEASURED recovery within 2 s of
    each corner (the maneuver window's widen + Q inflation carry the track
    through the degraded far corner)."""
    pts = lap_points()
    corners = lap_corner_frames(pts)
    assert len(corners) == 4
    vc = VisionContacts(LowWorld(), config=COCO_MAN_CFG)
    name0 = None
    health_at = {}
    for i, (t, e, n) in enumerate(pts):
        vc.update(result(t, [det_geom("car", e, n, alt=3.0)]))
        if i == 1:
            (name0,) = vc.poses()
            vc.designate(name0)
        if name0 is None:
            continue
        health_at[i] = vc.health(name0)
        assert health_at[i] != "LOST"             # one contiguous engagement
        assert sorted(vc.poses()) == [name0]      # same id, no re-lock birth
        assert len(vc.all_views()) == 1
    for i_c in corners:
        i_chk = min(i_c + 10, len(pts) - 1)       # 2.0 s after the corner
        assert health_at[i_chk] == "MEASURED"
    assert vc.velocities()[name0][0] == pytest.approx(4.0, abs=0.75)


def test_default_profile_ghosts_off_the_far_corner_and_loses_the_mover():
    """The w3-run6 failure, replayed on the SAME lap with the default
    (maneuverless) TrackerConfig — documents the demo-path scope of the fix:
    the CV ghost through the degraded far corner rejects the turned
    measurements, the 2 s grace expires, and the mover resurfaces under a
    NEW id."""
    pts = lap_points()
    vc = VisionContacts(LowWorld())               # contractual defaults
    name0 = None
    lost_frame = None
    for i, (t, e, n) in enumerate(pts):
        vc.update(result(t, [det_geom("car", e, n, alt=3.0)]))
        if i == 1:
            (name0,) = vc.poses()
            vc.designate(name0)
        if name0 is not None and lost_frame is None \
                and vc.health(name0) == "LOST":
            lost_frame = i
    assert lost_frame is not None                 # the corner ghost killed it
    assert name0 not in vc.poses()                # no same-id recovery...
    assert len(vc.poses()) == 1                   # ...a NEW id tracks the mover
