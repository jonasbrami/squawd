"""Vision contacts (ICD §6.4/§6.5): CvEkf gates, birth/rebind/NIS rules,
bearing-only semantics, source-change confirmation, coast/lost on SIM time,
designate seam, deep-copy readers. Synthetic InferenceResults over a fake
World (fixed pose/attitude) — drone at (0, 0, 12) heading north throughout."""
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


def det_geom(cls, e, n, conf=0.9):
    """Detection whose footpoint the projection composition (ray_support_range
    -> contact_world, the v1 slant-range convention of projection.py) maps to
    world (e, n) on the mover's base plane (z=0.6 for "target", z=0 else):
    the ray needs SLANT range == horizontal distance, so depression is
    asin((alt-support)/g), not atan2(alt, g)."""
    sup = 0.6 if cls == "target" else 0.0
    g = math.hypot(e, n)
    ax = math.atan2(e, n)
    ay = math.asin(min((ALT - sup) / g, 1.0))
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
    return InferenceResult(Frame(_seq[0], t, W, H, b""), list(dets), 0.0, 0, hit)


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
                         0.8, None)
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
    hit = AssociationHit(None, None, _px(ax, ay), 0.8, None)
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
    hit = AssociationHit(0, box, (320.0, 200.0), 0.9, None)
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
