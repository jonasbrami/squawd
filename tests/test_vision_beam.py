"""Beam association + ToF fusion (ICD §6.6, design §3.10) — the M3b set.

BeamAssociator gates: footprint inside one mask => ASSOCIATED; inside two
regions => AMBIGUOUS; straddling an edge (incl. the 22% erosion margin) =>
EDGE; background => OFF_TARGET; |residual| > 3σ => consistency reject;
NO_SAMPLE / OUT_OF_ENVELOPE precede geometry. VisionContacts: the acquisition
SM (DESIGNATED -> ACQUIRING -> RANGE_LOCKED -> WORLD_TRACKED; slip x3 ->
COASTING, never LOST; dropout -> LOST), envelope gating, and the deterministic
consumption order (the designated det is RESERVED; one sample, once, into the
designated track only). 640x360 / hfov 69deg frames; beam = principal point
(320, 180), disc radius ~2.02 px (0.25 deg half-angle at fx~465.6).
"""
import math

import pytest

from agents.core.contact import Frame
from agents.core.rangefinder import RangeSample
from agents.perception.projection import vfov_deg
from agents.vision.beam import (ASSOCIATED, AMBIGUOUS, EDGE, NO_SAMPLE,
                                OFF_TARGET, OUT_OF_ENVELOPE, BeamAssociator,
                                in_fusion_envelope)
from agents.vision.contacts import VisionContacts
from agents.vision.types import AssociationHit, Detection, InferenceResult
from agents.vision.types import rle_encode

W, H = 640, 360
T0, DT = 100.0, 0.2
CU, CV = 320.0, 180.0                     # principal point = beam pixel
R_PX = 2.02                               # 0.25 deg half-angle at fx~465.6
CENTER_BOX = (310.0, 170.0, 330.0, 190.0)  # covers the beam w/ margin
FAR_BOX = (40.0, 40.0, 60.0, 60.0)


def frame(t, seq=1):
    return Frame(seq, t, W, H, b"")


def sample(rng, t, status="VALID"):
    return RangeSample(t, 0.0, rng, 0.2, 100.0, 0.0087, 1.0, status, 1)


def det_box(xyxy=CENTER_BOX, cls="target"):
    return Detection(cls, 0.9, xyxy)


def det_mask(x1, y1, x2, y2):
    rows = [[False] * W for _ in range(H)]
    for y in range(y1, y2):
        for x in range(x1, x2):
            rows[y][x] = True
    return Detection("target", 0.9, (x1, y1, x2, y2), mask=rle_encode(rows))


def assoc(**kw):
    return BeamAssociator(**kw)


# ---- BeamAssociator: association gates (ICD §6.6) ----

def test_footprint_disc_geometry_zero_and_rigid_offset():
    a = assoc()
    (cu, cv), r = a.footprint(frame(T0), 30.0)
    assert (cu, cv) == (CU, CV)                       # co-boresighted, no offset
    assert r == pytest.approx(R_PX, abs=0.05)
    a2 = assoc(cam_to_beam_offset_m=(0.10, 0.0, 0.0))  # 10 cm right of camera
    (cu2, cv2), _ = a2.footprint(frame(T0), 10.0)
    assert cu2 == pytest.approx(CU + 465.6 * 0.10 / 10.0, abs=0.2)
    assert cv2 == pytest.approx(CV, abs=1e-9)


def test_associated_when_footprint_inside_exactly_one_mask():
    a = assoc()
    d = det_mask(300, 160, 340, 200)
    ba = a.associate(frame(T0), [d], sample(30.0, T0), None, 0, None, None)
    assert ba.status == ASSOCIATED and ba.detection_index == 0
    assert ba.footprint_px == pytest.approx(R_PX, abs=0.05)


def test_associated_box_fallback_uses_eroded_box():
    a = assoc()
    ba = a.associate(frame(T0), [det_box()], sample(30.0, T0), None, 0,
                     None, None)
    assert ba.status == ASSOCIATED and ba.detection_index == 0


def test_ambiguous_when_footprint_inside_two_regions():
    a = assoc()
    d1 = det_box((310.0, 170.0, 330.0, 190.0))
    d2 = det_box((312.0, 172.0, 332.0, 192.0))   # also contains the disc
    ba = a.associate(frame(T0), [d1, d2], sample(30.0, T0), None, 0,
                     None, None)
    assert ba.status == AMBIGUOUS and ba.detection_index is None
    assert "2" in ba.reason


def test_edge_when_footprint_straddles_a_mask_edge():
    a = assoc()
    d = det_mask(322, 160, 360, 200)             # cuts the disc's right half
    ba = a.associate(frame(T0), [d], sample(30.0, T0), None, 0, None, None)
    assert ba.status == EDGE and ba.detection_index == 0


def test_erosion_margin_inside_raw_but_not_eroded_box_is_edge():
    a = assoc()
    # raw box contains the disc; the 22%-eroded box does not (left margin)
    d = det_box((316.0, 170.0, 340.0, 190.0))
    ba = a.associate(frame(T0), [d], sample(30.0, T0), None, 0, None, None)
    assert ba.status == EDGE and ba.detection_index == 0


def test_off_target_when_footprint_on_background():
    a = assoc()
    ba = a.associate(frame(T0), [det_box(FAR_BOX)], sample(30.0, T0), None,
                     0, None, None)
    assert ba.status == OFF_TARGET and ba.detection_index is None
    ba = a.associate(frame(T0), [], sample(30.0, T0), None, None, None, None)
    assert ba.status == OFF_TARGET                # no dets at all == background


def test_designated_det_is_reserved_never_a_silent_fallback():
    a = assoc()
    # the beam sits on det 1 but det 0 is the designated one: OFF_TARGET,
    # not ASSOCIATED-with-1 (§3.10 deterministic consumption)
    ba = a.associate(frame(T0), [det_box(FAR_BOX), det_box()],
                     sample(30.0, T0), None, 0, None, None)
    assert ba.status == OFF_TARGET and "reserved" in ba.reason
    # with nothing reserved (designated_index=None) the same geometry associates
    ba = a.associate(frame(T0), [det_box(FAR_BOX), det_box()],
                     sample(30.0, T0), None, None, None, None)
    assert ba.status == ASSOCIATED and ba.detection_index == 1


def test_consistency_gate_rejects_multipath_outlier():
    a = assoc()
    ba = a.associate(frame(T0), [det_box()], sample(60.0, T0), None, 0,
                     40.0, 1.0)                  # residual 20 > 3*1.0
    assert ba.status == OFF_TARGET and "consistency" in ba.reason
    assert ba.residual_m == pytest.approx(20.0)
    ba = a.associate(frame(T0), [det_box()], sample(42.0, T0), None, 0,
                     40.0, 1.0)                  # residual 2 <= 3σ: accepted
    assert ba.status == ASSOCIATED and ba.residual_m == pytest.approx(2.0)


def test_no_sample_variants_precede_geometry():
    a = assoc()
    d = [det_box()]
    assert a.associate(frame(T0), d, None, None, 0, None, None).status \
        == NO_SAMPLE
    assert a.associate(frame(T0), d, sample(None, T0), None, 0, None,
                       None).status == NO_SAMPLE          # no return
    assert a.associate(frame(T0), d, sample(30.0, T0, "LOW_SIGNAL"), None, 0,
                       None, None).status == NO_SAMPLE    # non-VALID status
    assert a.associate(frame(T0), d, sample(30.0, T0, "EDGE_MIX"), None, 0,
                       None, None).status == NO_SAMPLE
    stale = sample(30.0, T0 - 1.0)                        # vs stale_s = 0.2
    ba = a.associate(frame(T0), d, stale, None, 0, None, None)
    assert ba.status == NO_SAMPLE and "stale" in ba.reason


def test_out_of_envelope_flag_precedes_geometry():
    a = assoc()
    ba = a.associate(frame(T0), [det_box()], sample(30.0, T0), None, 0,
                     None, None, in_envelope=False)
    assert ba.status == OUT_OF_ENVELOPE


def test_in_fusion_envelope_contract():
    assert in_fusion_envelope("shadow", 2.0, 1.0, 0.1)
    assert not in_fusion_envelope("intercept", 1.0, 1.0, 0.1)   # mode
    assert not in_fusion_envelope("shadow", 4.0, 0.0, 0.0)      # >3 m/s
    assert not in_fusion_envelope("shadow", 2.0, 3.5, 0.0)      # |Δz| > 3
    assert not in_fusion_envelope("shadow", 2.0, 0.0, 0.3)      # > half-width
    assert not in_fusion_envelope(None, 0.0, 0.0, 0.0)          # unset ctx


# ---- VisionContacts: fusion + acquisition SM (design §3.10) ----

class FakeWorld:
    """Duck World: hover at (0, 0, 12), heading north, level attitude."""

    def pose_at(self, t):
        return (0.0, 0.0, 12.0, 0.0)

    def attitude_at(self, t):
        return (0.0, 0.0, 0.0)


class ScriptRange:
    """Duck RangeProvider: one scripted sample; counts robust_at calls."""

    def __init__(self):
        self.sample = None
        self.calls = 0

    def set(self, rng, t):
        self.sample = sample(rng, t)

    def latest(self):
        return self.sample

    def robust_at(self, t, **kw):
        self.calls += 1
        s = self.sample
        if s is None or abs(s.sample_time - t) > 0.05:
            return None
        return s


def det_horizon(cls="target", bearing_deg=0.0):
    """Bearing-only detection: footpoint on the horizon (angle_y = 0)."""
    fx = (W / 2) / math.tan(math.radians(69.0) / 2)
    u = W / 2 + fx * math.tan(math.radians(bearing_deg))
    return Detection(cls, 0.9, (u - 3.0, 174.0, u + 3.0, 180.0))


_seq = [0]


def result(t, dets, hit=None):
    _seq[0] += 1
    return InferenceResult(Frame(_seq[0], t, W, H, b""), list(dets), 0.0,
                           0, hit)


def center_hit(k=0):
    return AssociationHit(k, CENTER_BOX, (320.0, 190.0), 0.8, None)


def make_vc(rf=None):
    vc = VisionContacts(FakeWorld(), rangefinder=rf)
    vc.set_beam_context(mode="shadow", own_speed_mps=0.0)
    return vc


def birth_bearing_only(vc, t0=T0):
    """Two consecutive horizon dets -> one bearing-only track; returns name."""
    vc.update(result(t0, [det_horizon()]))
    vc.update(result(t0 + DT, [det_horizon()]))
    views = vc.all_views()
    assert len(views) == 1 and views[0].position_src == "none"
    return views[0].name


def fusion_frame(vc, rf, t, rng=30.0):
    """One cycle with the reserved det under the beam + a fresh VALID sample."""
    rf.set(rng, t)
    vc.update(result(t, [det_box()], hit=center_hit()))


def test_acquisition_sm_bearing_only_to_world_tracked():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    assert vc.track_state(name) == "IDLE"            # pre-designation
    vc.designate(name)
    assert vc.track_state(name) == "DESIGNATED"
    t = T0 + 2 * DT
    fusion_frame(vc, rf, t)                          # 1st lock: EKF born
    assert vc.track_state(name) == "ACQUIRING"       # lock_hits 1 < confirm 2
    e, n, z = vc.poses()[name]
    assert (e, n) == (pytest.approx(0.0, abs=1e-6),
                      pytest.approx(30.0, abs=1e-6))  # range 30 along bearing 0
    assert z == pytest.approx(12.0)                  # co-altitude beam lock
    rng, src, sigma = vc.ranges()[name]
    assert (rng, src, sigma) == (pytest.approx(30.0), "tof",
                                 pytest.approx(0.15))
    fusion_frame(vc, rf, t + DT)                     # 2nd consecutive lock
    assert vc.track_state(name) == "RANGE_LOCKED"
    fusion_frame(vc, rf, t + 2 * DT)                 # locked and world-fed
    assert vc.track_state(name) == "WORLD_TRACKED"
    assert vc.health(name) == "MEASURED"


def test_track_state_vocabulary():
    vc = make_vc(ScriptRange())
    name = birth_bearing_only(vc)
    assert vc.track_state("vis_nope_9") == "LOST"    # unknown name
    assert vc.track_state(name) == "IDLE"            # never designated
    vc.designate(name)
    vc.clear_designation()
    assert vc.track_state(name) == "IDLE"            # off the ladder again


def test_beam_slip_from_locked_coasts_and_recovers_never_lost():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    vc.designate(name)
    t = T0 + 2 * DT
    for k in range(3):                               # -> WORLD_TRACKED
        fusion_frame(vc, rf, t + k * DT)
    assert vc.track_state(name) == "WORLD_TRACKED"
    t += 3 * DT
    for k in range(3):                               # 3 consecutive slips
        vc.update(result(t + k * DT, []))            # (no reserved det)
    assert vc.track_state(name) == "COASTING"        # retry/backoff state
    assert vc.health(name) != "LOST"
    assert name in vc.poses()                        # prediction retained
    t += 3 * DT
    fusion_frame(vc, rf, t)                          # re-lock: confirm again
    assert vc.track_state(name) == "ACQUIRING"
    fusion_frame(vc, rf, t + DT)
    assert vc.track_state(name) == "RANGE_LOCKED"
    assert vc.health(name) != "LOST"


def test_beam_slip_while_acquiring_retries_and_never_lost_cycles():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    vc.designate(name)
    t = T0 + 2 * DT
    for k in range(3):                               # 3 slips before any lock
        vc.update(result(t + k * DT, []))
    assert vc.track_state(name) == "ACQUIRING"       # still holding for lock
    assert vc.health(name) != "LOST"
    fusion_frame(vc, rf, t + 3 * DT)                 # acquisition resumes
    assert vc.track_state(name) == "ACQUIRING"       # lock 1 < confirm 2
    fusion_frame(vc, rf, t + 4 * DT)
    assert vc.track_state(name) == "RANGE_LOCKED"


def test_dropout_of_the_contact_coasts_then_lost():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    vc.designate(name)
    t = T0 + 2 * DT
    for k in range(3):                               # -> WORLD_TRACKED
        fusion_frame(vc, rf, t + k * DT)
    t += 3 * DT
    for k in range(12):                              # silence > lost_s = 2 s
        vc.update(result(t + k * DT, []))
    assert vc.track_state(name) == "LOST"
    assert vc.health(name) == "LOST"
    assert vc.poses() == {} and vc.ranges() == {}


def test_envelope_blocks_fusion_at_chase_mode_and_speed():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    vc.designate(name)
    vc.set_beam_context(mode="intercept", own_speed_mps=4.0)  # chase: 4 m/s
    t = T0 + 2 * DT
    for k in range(3):
        fusion_frame(vc, rf, t + k * DT)
    # OUT_OF_ENVELOPE: the sample never touches the EKF (§3.10 opportunistic)
    assert vc.poses() == {} and vc.ranges() == {}
    assert vc._beam_last.status == OUT_OF_ENVELOPE
    assert vc.track_state(name) == "ACQUIRING"       # neutral: no slip decay
    vc.set_beam_context(mode="shadow", own_speed_mps=0.0)
    fusion_frame(vc, rf, t + 3 * DT)
    assert name in vc.poses()                        # fusion resumes


def test_envelope_blocks_fusion_for_ground_movers():
    """|Δz| > 3 m (ground plane vs 12 m alt) => OUT_OF_ENVELOPE even in shadow
    — the forward beam is for near-co-altitude targets (§3.10)."""
    rf = ScriptRange()
    vc = make_vc(rf)
    # geom-born ground track at (0, 40) on the z=0.6 support plane
    fy = (H / 2) / math.tan(math.radians(vfov_deg(W, H)) / 2)
    ay = math.asin((12.0 - 0.6) / 40.0)
    v = H / 2 + fy * math.tan(ay)
    for k in range(2):
        vc.update(result(T0 + k * DT, [Detection("target", 0.9,
                                                 (317.0, v - 6, 323.0, v))]))
    (name,) = vc.poses()
    vc.designate(name)
    t = T0 + 2 * DT
    n0 = vc.poses()[name][1]
    fusion_frame(vc, rf, t)
    assert vc._beam_last.status == OUT_OF_ENVELOPE   # dz = 0.6 − 12
    assert vc.poses()[name][1] == pytest.approx(n0)  # estimate untouched
    assert vc.ranges()[name][1] == "geom"            # src never challenged


def test_out_of_envelope_is_neutral_not_a_slip():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    vc.designate(name)
    t = T0 + 2 * DT
    for k in range(2):                               # -> RANGE_LOCKED
        fusion_frame(vc, rf, t + k * DT)
    assert vc.track_state(name) == "RANGE_LOCKED"
    vc.set_beam_context(mode="intercept", own_speed_mps=8.0)
    fusion_frame(vc, rf, t + 2 * DT)                 # fresh VALID sample…
    assert vc.track_state(name) == "RANGE_LOCKED"    # unchanged
    assert vc._beam_last.status == OUT_OF_ENVELOPE
    vc.set_beam_context(mode="shadow", own_speed_mps=0.0)
    fusion_frame(vc, rf, t + 3 * DT)                 # lock count preserved…
    assert vc.track_state(name) == "WORLD_TRACKED"   # …advances immediately


def test_consistency_reject_leaves_the_estimate_unmoved():
    rf = ScriptRange()
    vc = make_vc(rf)
    name = birth_bearing_only(vc)
    vc.designate(name)
    t = T0 + 2 * DT
    for k in range(2):                               # -> RANGE_LOCKED @ 30 m
        fusion_frame(vc, rf, t + k * DT)
    e0, n0, _ = vc.poses()[name]
    fusion_frame(vc, rf, t + 2 * DT, rng=45.0)       # multipath: 15 m jump
    assert vc._beam_last.status == OFF_TARGET
    assert "consistency" in vc._beam_last.reason
    e1, n1, _ = vc.poses()[name]
    assert (e1, n1) == (pytest.approx(e0), pytest.approx(n0))
    assert vc.track_state(name) == "RANGE_LOCKED"    # one slip, still locked


def test_deterministic_consumption_designated_reserved_and_once_per_cycle():
    rf = ScriptRange()
    vc = make_vc(rf)
    # two bearing-only tracks 30 deg apart; the first born is the designated one
    vc.update(result(T0, [det_horizon(bearing_deg=0.0),
                          det_horizon(bearing_deg=30.0)]))
    vc.update(result(T0 + DT, [det_horizon(bearing_deg=0.0),
                               det_horizon(bearing_deg=30.0)]))
    names = [v.name for v in vc.all_views()]
    assert names == ["vis_target_0", "vis_target_1"]
    vc.designate("vis_target_0")
    # the reserved det (index 0) is NOT under the beam; a second, identical
    # on-beam det exists — it must NOT be used as a fallback. (The hit's aim
    # stays at the frame center so the track's bearing meas is unchanged —
    # this frame isolates the BEAM reservation, not the world gate.)
    t = T0 + 2 * DT
    rf.set(30.0, t)
    vc.update(result(t, [det_box(FAR_BOX), det_box()], hit=AssociationHit(
        0, FAR_BOX, (320.0, 190.0), 0.8, None)))
    assert "vis_target_0" not in vc.poses()          # no fallback fusion
    # an off-beam designated det is rejected by the envelope's off-boresight
    # gate (image-space), before the footprint path — the no-fallback
    # property is what matters and holds either way
    assert vc._beam_last.status in (OFF_TARGET, OUT_OF_ENVELOPE)
    # now the reserved det is on the beam: fusion lands ONLY in vis_target_0
    calls0 = rf.calls
    for k in range(1, 4):
        t = T0 + 2 * DT + k * DT
        rf.set(30.0, t)
        vc.update(result(t, [det_box(), det_horizon(bearing_deg=30.0)],
                         hit=center_hit(0)))
    assert rf.calls - calls0 == 3                    # one robust_at per cycle
    assert "vis_target_0" in vc.poses()
    assert vc.ranges()["vis_target_0"][1] == "tof"
    assert "vis_target_1" not in vc.poses()          # the second track never
    assert "vis_target_1" not in vc.ranges()         # touched the sample


def test_mask_hole_inside_eroded_box_still_associates():
    """v14: the blob's mask HOLES on the shadowed face — a disc on a hole is
    on the object (mask, OR eroded-box region semantics, ICD §6.6)."""
    a = assoc()
    rows = [[False] * W for _ in range(H)]
    for y in range(150, 210):
        for x in range(280, 360):
            rows[y][x] = True
    for y in range(172, 188):                  # the dark-face hole at the disc
        for x in range(310, 330):
            rows[y][x] = False
    d = Detection("target", 0.9, (280.0, 150.0, 360.0, 210.0),
                  mask=rle_encode(rows))
    ba = a.associate(frame(T0), [d], sample(30.0, T0), None, 0, None, None)
    assert ba.status == ASSOCIATED and ba.detection_index == 0
