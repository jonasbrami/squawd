"""vision/contacts.py — the vision-fed contact tracker (ICD §6.4/§6.5).

SOLE owner of per-track CV-EKF fusion + track health: detections are projected
to the support plane (perception/projection), NN-gated at gate_m, NIS-gated at
nis_max, and fused into one CvEkf per contact. asyncio-loop-confined except the
deep-copy readers (§0.2c); numpy confined to agents/vision (§0.1). ToF feeding
(M3b, design §3.10): the BeamAssociator (§6.6) associates the beam footprint
against the RESERVED designated detection — no nearest-track fallback — and an
ASSOCIATED, in-envelope sample fuses into the designated track via
update_range; the acquisition state machine (DESIGNATED → ACQUIRING →
RANGE_LOCKED → WORLD_TRACKED, beam-slip ×slip_n → COASTING, dropout →
COASTING → LOST) rides on the association outcomes. Time is SIM time from the
InferenceResult's frame stamp — never wall clock.
"""
import math
import threading
from dataclasses import dataclass

import numpy as np

from agents.core.contact import ContactView  # noqa: F401
from agents.perception.projection import (HFOV_DEG, contact_world,
                                          pixel_to_angles, ray_support_range,
                                          vfov_deg)
from agents.vision.beam import (AMBIGUOUS, ASSOCIATED, EDGE, NO_SAMPLE,
                                OFF_TARGET, OUT_OF_ENVELOPE, BeamAssociation,
                                BeamAssociator, in_fusion_envelope)
from agents.vision.types import InferenceResult  # noqa: F401

MEASURED, COASTING, ACQUIRING, LOST = "MEASURED", "COASTING", "ACQUIRING", "LOST"
# acquisition-SM states beyond the health vocabulary (§3.10; the snapshot's
# track dict, ICD §1): DESIGNATED/ACQUIRING/RANGE_LOCKED/WORLD_TRACKED, slip ->
# COASTING, dropout -> COASTING -> LOST, never-designated -> IDLE
DESIGNATED, RANGE_LOCKED, WORLD_TRACKED, IDLE = (
    "DESIGNATED", "RANGE_LOCKED", "WORLD_TRACKED", "IDLE")
_RANGED = ("geom", "tof")
_BEARING_GATE_REF_M = 20.0     # bearing-only NN gate: gate_m cross-range at 20 m
_MIN_DROP_M = 2.0             # support-plane geom needs alt-support ≥ this —
                              # below it the EKF alt bias poisons the range


@dataclass(frozen=True)
class TrackerConfig:
    """Fusion knobs (ICD §6.5) — every default is a contractual test vector."""
    dt_nominal_s: float = 0.2            # 5 Hz detector
    v_max_mps: float = 12.0
    gate_m: float = 5.0                  # NN gate on projected ground points
    nis_max: float = 9.21                # chi2(2 dof, 99%) innovation gate
    confirm_hits: int = 2                # source changes apply after this many
    birth_hits: int = 2                  # consecutive gated hits to open a track
    coast_s: float = 1.0                 # sim-time
    lost_s: float = 2.0                  # sim-time — THE single lost constant
    rebind_window_s: float = 2.0         # name-rebind gate after drop
    sigma_geom_m: float = 2.0            # support-plane measurement noise
    sigma_tof_m: float = 0.15            # ToF measurement noise (in-envelope)
    sigma_bearing_deg: float = 1.5
    accel_max_mps2: float = 4.0          # process-model white-accel clamp


class CvEkf:
    """Constant-velocity EKF, state (e, n, ve, vn), numpy 4x4 (ICD §6.5).

    Measurement models: XY = full 2D position; RANGE = 1D along a known world
    bearing from `origin` (set_origin before the call); BEARING = 1D angle with
    the range held at the predicted value. Every update returns the NIS and is
    APPLIED ONLY when NIS <= nis_max — a rejected measurement never touches the
    state. Process noise is white acceleration clamped at accel_max_mps2."""

    def __init__(self, e: float, n: float, *, ve: float = 0.0, vn: float = 0.0,
                 sigma_pos_m: float = 2.0, sigma_vel_mps: float = 6.0,
                 accel_max_mps2: float = 4.0, nis_max: float = 9.21) -> None:
        self.x = np.array([e, n, ve, vn], dtype=float)
        self.P = np.diag([sigma_pos_m ** 2, sigma_pos_m ** 2,
                          sigma_vel_mps ** 2, sigma_vel_mps ** 2])
        self.nis_max = float(nis_max)
        self._q = float(accel_max_mps2) ** 2
        self._origin = (0.0, 0.0)     # sensor (drone) world position, m

    def set_origin(self, e: float, n: float) -> None:
        self._origin = (float(e), float(n))

    def predict(self, dt: float) -> None:
        dt = min(max(float(dt), 1e-3), 1.0)
        F = np.array([[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt],
                      [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        d2, d3 = dt * dt, dt * dt * dt
        Q = self._q * np.array(
            [[d3 / 3, 0.0, d2 / 2, 0.0], [0.0, d3 / 3, 0.0, d2 / 2],
             [d2 / 2, 0.0, dt, 0.0], [0.0, d2 / 2, 0.0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def _apply(self, y, H, R):
        """One gated linear update (Joseph form). y innovation, H (k,4), R (k,k)."""
        H = np.atleast_2d(H)
        R = np.atleast_2d(R)
        S = H @ self.P @ H.T + R
        y = np.atleast_1d(np.asarray(y, dtype=float))
        nis = float(y @ np.linalg.solve(S, y))
        if nis > self.nis_max:
            return nis                            # REJECTED: state untouched
        K = np.linalg.solve(S.T, (self.P @ H.T).T).T
        self.x = self.x + K @ y
        IKH = np.eye(4) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return nis

    def update_xy(self, e: float, n: float, sigma_m: float) -> float:
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        z = np.array([e, n]) - self.x[:2]
        return self._apply(z, H, np.eye(2) * float(sigma_m) ** 2)

    def update_range(self, rng: float, bearing_deg: float,
                     sigma_m: float) -> float:
        b = math.radians(bearing_deg)
        u = np.array([math.sin(b), math.cos(b)])   # (e, n) unit ray
        pred = float(u @ (self.x[:2] - np.asarray(self._origin)))
        H = np.array([[u[0], u[1], 0.0, 0.0]])
        return self._apply(rng - pred, H, [[float(sigma_m) ** 2]])

    def update_bearing(self, bearing_deg: float, sigma_deg: float) -> float:
        rel = self.x[:2] - np.asarray(self._origin)
        r2 = float(rel @ rel)
        if r2 < 1e-6:
            return math.inf                        # origin == track: no angle
        pred = math.atan2(rel[0], rel[1])
        y = (math.radians(bearing_deg) - pred + math.pi) % (2 * math.pi) - math.pi
        H = np.array([[rel[1] / r2, -rel[0] / r2, 0.0, 0.0]])
        return self._apply(y, H, [[math.radians(float(sigma_deg)) ** 2]])


def _wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


class _Meas:
    """One detection (or designated hit) projected into the world for a tick."""
    __slots__ = ("cls", "conf", "kind", "e", "n", "z", "rng", "bearing_deg",
                 "elev_deg", "origin", "foot_px", "det_index", "support_z",
                 "xyxy",
                 "designated_for", "rng_h", "sigma_h")

    def __init__(self, cls, conf, kind, e, n, z, rng, bearing_deg, elev_deg,
                 origin, foot_px, det_index, support_z):
        self.cls, self.conf, self.kind = cls, conf, kind      # kind geom|bearing
        self.e, self.n, self.z, self.rng = e, n, z, rng
        self.bearing_deg, self.elev_deg = bearing_deg, elev_deg
        self.origin, self.foot_px = origin, foot_px
        self.det_index, self.support_z = det_index, support_z
        self.designated_for = None
        self.rng_h, self.sigma_h = None, None   # bbox angular-height range cue
        self.designated_for = None          # track name: force-association


class _Track:
    __slots__ = ("name", "cls", "conf", "ekf", "src", "pending_src",
                 "pending_n", "z", "support_z", "bearing_deg", "elev_deg",
                 "origin", "range_m", "range_conf", "t_seen", "t_meas",
                 "t_state", "foot_px", "det_index", "xyxy")

    def __init__(self, name, cls):
        self.name, self.cls = name, cls
        self.conf = 0.0
        self.ekf = None                   # None while bearing-only newborn
        self.src = "bearing"              # RangeSource; ranged sigma defends it
        self.pending_src, self.pending_n = None, 0
        self.z = 0.0                      # support plane of the last ranged lock
        self.support_z = 0.0
        self.bearing_deg = None
        self.elev_deg = None
        self.origin = (0.0, 0.0)
        self.range_m = None
        self.range_conf = 0.0
        self.t_seen = 0.0                 # last accepted measurement of any kind
        self.t_meas = 0.0                 # last accepted RANGE-carrying one
        self.t_state = 0.0                # EKF time base
        self.foot_px = None
        self.det_index = None
        self.xyxy = None


class _Candidate:
    """Tentative track gathering birth_hits CONSECUTIVE gated hits (ICD §6.5)."""
    __slots__ = ("cls", "hits", "t_last", "e", "n", "z", "rng", "src_kind",
                 "bearing_deg", "elev_deg", "origin", "conf", "foot_px",
                 "det_index", "support_z", "rebind", "xyxy")

    def __init__(self, m, t, rebind):
        self.cls = m.cls
        self.hits = 1
        self.t_last = t
        self.e, self.n, self.z, self.rng = m.e, m.n, m.z, m.rng
        self.src_kind = m.kind
        self.bearing_deg, self.elev_deg = m.bearing_deg, m.elev_deg
        self.origin = m.origin
        self.conf = m.conf
        self.foot_px, self.det_index = m.foot_px, m.det_index
        self.xyxy = getattr(m, "xyxy", None)
        self.support_z = m.support_z
        self.rebind = rebind              # (name, ekf, t_state) or None


class VisionContacts:
    """ContactProvider + TargetDesignator (ICD §5.1/§6.4)."""

    def __init__(self, world, rangefinder=None, i: int = 0,
                 config: "TrackerConfig | None" = None,
                 beam: "BeamAssociator | None" = None,
                 slip_n: int = 3) -> None:
        self._world = world
        self._rangefinder = rangefinder
        # M3b: the beam associator owns ToF association (§6.6); default-built
        # whenever a rangefinder is present. slip_n = consecutive non-ASSOCIATED
        # outcomes that drop a locked beam to COASTING (documented default 3).
        self._beam = beam if beam is not None else (
            BeamAssociator() if rangefinder is not None else None)
        self._slip_n = int(slip_n)
        self._i = i
        self.cfg = config or TrackerConfig()
        self._lock = threading.Lock()
        self._tracks: dict[str, _Track] = {}
        self._candidates: list[_Candidate] = []
        self._graveyard: dict[str, tuple] = {}   # name -> (cls, ekf, z, supp_z, t_state, t_drop)
        self._counters: dict[str, int] = {}
        self._sim_t = 0.0
        self._designated: str | None = None
        self._designated_support_z: float | None = None
        self._designated_context = None
        self._detector = None                     # designate seam (attach_detector)
        # acquisition SM (one designated target => one SM, §3.10) + the
        # flight-layer envelope context (set_beam_context; unset => OUT_OF_ENVELOPE)
        self._sm_state: str | None = None
        self._lock_hits = 0
        self._slip_count = 0
        self._beam_mode: str | None = None
        self._beam_speed = 0.0
        self._beam_last: BeamAssociation | None = None

    # ---- ticking (asyncio loop only; called by VisionPipeline) ----

    def update(self, result: InferenceResult) -> None:
        with self._lock:
            t = float(result.frame.sim_stamp)
            self._sim_t = t
            pose = self._call("pose_at", t)
            att = self._call("attitude_at", t)
            for tr in self._tracks.values():
                if tr.ekf is not None and t > tr.t_state:
                    tr.ekf.predict(t - tr.t_state)
                    tr.t_state = t
            meas = self._measure(result, pose, att)
            self._feed_tof(t, result, pose, att)
            self._associate(t, meas)
            self._drop_stale(t)
            self._tick_sm_dropout(t)

    def _call(self, meth, t):
        fn = getattr(self._world, meth, None)
        return fn(t) if callable(fn) else None

    # ---- projection ----

    def _project_px(self, cls, conf, u, v, frame, pose, att, det_index,
                    support_z):
        """pixel footpoint -> _Meas (geom when the support plane converges,
        else bearing-only). None when the drone pose is unknown."""
        if pose is None:
            return None
        ax, ay = pixel_to_angles(u, v, frame.width, frame.height)
        heading = pose[3]
        bearing_deg = math.degrees(heading + ax)
        origin = (pose[0], pose[1])
        # honest fallbacks (fable-Q3-7/4): a missing attitude, or a footpoint
        # clipped by the frame's bottom/top edge (the box base is out of view —
        # its row is fabricated), both get bearing-only, never a geom guess
        if att is None:
            return _Meas(cls, conf, "bearing", None, None, None, None,
                         bearing_deg, None, origin, (u, v), det_index,
                         support_z)
        if v >= frame.height - 4 or v <= 1:
            # frame-edge clip: the box base is (partly) out of view and the
            # reported bottom row is fabricated — bearing-only, never geom.
            # ~4 px margin: the color threshold erodes the shadowed base, so
            # the fake row lands a few px INSIDE the frame too (fable-R2-6)
            return _Meas(cls, conf, "bearing", None, None, None, None,
                         bearing_deg, math.degrees(att[1] - ay), origin,
                         (u, v), det_index, support_z)
        elev_deg = math.degrees(att[1] - ay)    # up-positive (camera measure)
        # support-plane envelope: below ~6° depression the 1/sin(dep) lever
        # turns sub-degree pitch noise into 100–400 m range garbage (observed:
        # chase-pitch windows birthing tracks 200 m off — the poisoned-birth
        # churn). Bearing-only there is honest; geom is for the steep zone
        # (M2's 8–20° verified band). dep = ay·cos(roll) − pitch + ax·sin(roll).
        if ay * math.cos(att[0]) - att[1] + ax * math.sin(att[0]) \
                < math.radians(6.0):
            return _Meas(cls, conf, "bearing", None, None, None, None,
                         bearing_deg, elev_deg, origin, (u, v), det_index,
                         support_z)
        if pose[2] - support_z < _MIN_DROP_M:
            # the alt-bias lever (M3b v8.6): at a drop below ~2 m the EKF's
            # ±1 m altitude bias multiplies through the 1/sin(dep) lever into
            # a >100% support-plane range BIAS — a born position the pursuit
            # then flies as a ghost (observed: hd 17->61 m runaway). Same
            # honesty rule as the 6° pitch lever, second axis: bearing-only
            # is honest here; geom is for drops the alt bias can't poison.
            return _Meas(cls, conf, "bearing", None, None, None, None,
                         bearing_deg, elev_deg, origin, (u, v), det_index,
                         support_z)
        rng = ray_support_range(ax, ay, roll=att[0], pitch=att[1],
                                alt=pose[2], support_z=support_z)
        if rng is None:
            return _Meas(cls, conf, "bearing", None, None, None, None,
                         bearing_deg, elev_deg, origin, (u, v), det_index,
                         support_z)
        e, n = contact_world(pose[0], pose[1], heading, ax, rng)
        return _Meas(cls, conf, "geom", e, n, support_z, rng,
                     bearing_deg, elev_deg, origin, (u, v), det_index,
                     support_z)

    def _measure(self, result, pose, att):
        out = []
        hit = result.designated_hit
        hit_idx = hit.detection_index if hit is not None else None
        for k, d in enumerate(result.detections):
            if hit_idx is not None and k == hit_idx:
                continue                          # consumed as the designated hit
            supp = self._support_z_for(d)
            m = self._project_px(d.cls, d.conf, d.footpoint[0], d.footpoint[1],
                                 result.frame, pose, att, k, supp)
            if m is not None:
                m.xyxy = d.xyxy
                self._add_height_range(m, d, result.frame)
                out.append(m)
        if hit is not None and self._designated in self._tracks:
            tr = self._tracks[self._designated]
            m = self._project_px(tr.cls, hit.conf, hit.aim_px[0], hit.aim_px[1],
                                 result.frame, pose, att, hit_idx, tr.support_z)
            if m is not None:
                if hit.xyxy is not None:
                    m.xyxy = hit.xyxy
                m.designated_for = tr.name        # force-association marker
                out.append(m)
        return out

    # bbox angular height as an independent range cue (fable-R2-Q2-2): the
    # support plane's weak axis is ALONG the ray; the box's pixel height gives
    # range with sigma ≈ R·2px/px_h — better than sigma_geom at the operating
    # geometry and, fused via update_range, it shrinks the EKF's orbit-phase
    # error oscillation that keeps breaking the dwell contiguity.
    _H_BOX = {"target": 1.2}                      # class -> physical height (m)

    def _height_range_for(self, d, frame):
        """bbox angular height as an independent range cue: (rng_h, sigma_h),
        (None, None) when unusable (small/clipped box, unknown class).
        R = h·fy/px_h (small-angle; cos(dep) ≈ 1 at the operating depression)."""
        h_box = self._H_BOX.get(d.cls)
        if h_box is None:
            return None, None
        px_h = d.xyxy[3] - d.xyxy[1]
        if px_h < 8 or d.xyxy[3] >= frame.height - 4:
            return None, None                        # too noisy / edge-clipped
        fy = (frame.height / 2.0) / math.tan(
            math.radians(vfov_deg(frame.width, frame.height)) / 2.0)
        rng_h = h_box * fy / px_h
        return rng_h, max(0.5, rng_h * 2.0 / px_h)

    def _add_height_range(self, m, d, frame):
        if m.kind != "geom":
            return
        # the support plane's weak axis is ALONG the ray; the box's pixel
        # height gives range with sigma ≈ R·2px/px_h — fused via update_range
        # it shrinks the EKF's orbit-phase error oscillation (fable-R2-Q2-2)
        m.rng_h, m.sigma_h = self._height_range_for(d, frame)

    def _support_z_for(self, d):
        # the dynamic-world movers are boxes CENTERED at their pose z, so their
        # base (the blob footpoint) floats half a box-height up: mov_1's 1.2 m
        # box rests at z=0.6 — measured live (it floats over its shadow); z=0
        # overshoots range +18–25% and eats the 5 m NN gate (fable-Q3-3)
        return 0.6 if d.cls == "target" else 0.0

    # ---- gating primitives ----

    def _sigma_for(self, kind):
        return {"geom": self.cfg.sigma_geom_m, "tof": self.cfg.sigma_tof_m,
                "bearing": self.cfg.sigma_bearing_deg}[kind]

    def _commit_source(self, tr, kind):
        """Source-change rule (ICD §6.5): while a RANGED source holds the track,
        a different source needs confirm_hits consecutive gated hits before the
        new source's covariance applies; from bearing/None a first ranged lock
        switches immediately (the NN/NIS gates do the guarding)."""
        if kind == tr.src:
            tr.pending_src, tr.pending_n = None, 0
            return
        if tr.src in _RANGED:
            if tr.pending_src != kind:
                tr.pending_src, tr.pending_n = kind, 0
            tr.pending_n += 1
            if tr.pending_n >= self.cfg.confirm_hits:
                tr.src, tr.pending_src, tr.pending_n = kind, None, 0
            return
        tr.src, tr.pending_src, tr.pending_n = kind, None, 0

    def _peek_sigma(self, tr, kind):
        """Covariance that applies to THIS hit, pre-confirmation (ICD §6.5)."""
        if kind == tr.src or kind == "bearing":
            return self._sigma_for(kind)
        if tr.src in _RANGED:
            n = (tr.pending_n if tr.pending_src == kind else 0) + 1
            return self._sigma_for(kind if n >= self.cfg.confirm_hits else tr.src)
        return self._sigma_for(kind)

    def _gate_dist(self, tr, m):
        """NN distance (m) of a measurement to a born track, or None when not
        gateable. Ranged measurements gate on projected ground points (§6.5);
        bearing-only ones gate on cross-range at the predicted range (or at
        _BEARING_GATE_REF_M for a never-positioned track)."""
        ref_deg = math.degrees(math.atan2(self.cfg.gate_m, _BEARING_GATE_REF_M))
        if m.kind == "geom":
            if tr.ekf is not None:
                return math.hypot(m.e - tr.ekf.x[0], m.n - tr.ekf.x[1])
            if tr.bearing_deg is None:
                return None
            db = abs(_wrap_deg(m.bearing_deg - tr.bearing_deg))
            return (math.radians(db) * _BEARING_GATE_REF_M
                    if db <= ref_deg else None)
        if tr.ekf is not None:
            oe, on = m.origin
            rel_e, rel_n = tr.ekf.x[0] - oe, tr.ekf.x[1] - on
            pred = math.degrees(math.atan2(rel_e, rel_n))
            db = abs(_wrap_deg(m.bearing_deg - pred))
            return math.radians(db) * max(math.hypot(rel_e, rel_n), 1.0)
        if tr.bearing_deg is None:
            return None
        db = abs(_wrap_deg(m.bearing_deg - tr.bearing_deg))
        return (math.radians(db) * _BEARING_GATE_REF_M
                if db <= ref_deg else None)

    # ---- association ----

    def _associate(self, t, meas):
        cfg = self.cfg
        pairs = []
        for mi, m in enumerate(meas):
            if m.designated_for is not None and m.designated_for in self._tracks:
                # the designated contact's hit (registry tracker or detector
                # association) still passes the world-space NN/NIS gates (§6.8);
                # on rejection it must NOT be consumed — let it feed the
                # candidate/rebind logic below instead (codex-R2)
                if self._apply_to_track(self._tracks[m.designated_for], m, t):
                    meas[mi] = None
                continue
            for name, tr in self._tracks.items():
                if m.cls != tr.cls:
                    continue
                d = self._gate_dist(tr, m)
                if d is not None and d <= cfg.gate_m:
                    pairs.append((d, mi, name))
        pairs.sort(key=lambda p: p[0])
        used_t, used_m = set(), set()
        for _d, mi, name in pairs:
            if mi in used_m or name in used_t or meas[mi] is None:
                continue
            if self._apply_to_track(self._tracks[name], meas[mi], t):
                used_t.add(name)
                used_m.add(mi)
        for mi, m in enumerate(meas):
            if m is not None and mi not in used_m:
                self._candidate_hit(t, m)
        # strict consecutiveness: a candidate that missed this frame dies
        self._candidates = [c for c in self._candidates if c.t_last == t]

    def _apply_to_track(self, tr, m, t):
        """Fuse one gated measurement; False when the NIS gate rejects it."""
        if m.kind == "bearing":
            if tr.ekf is not None:
                tr.ekf.set_origin(*m.origin)
                nis = tr.ekf.update_bearing(m.bearing_deg,
                                            self._sigma_for("bearing"))
                if nis > self.cfg.nis_max:
                    return False
            self._commit_source(tr, "bearing")
        else:  # geom
            if tr.ekf is None:
                tr.ekf = CvEkf(m.e, m.n, sigma_pos_m=self._sigma_for(m.kind),
                               sigma_vel_mps=self.cfg.v_max_mps / 2.0,
                               accel_max_mps2=self.cfg.accel_max_mps2,
                               nis_max=self.cfg.nis_max)
                tr.src = m.kind
                tr.pending_src, tr.pending_n = None, 0
                tr.t_state = t
                tr.z, tr.support_z = m.z, m.support_z
            else:
                sigma = self._peek_sigma(tr, m.kind)
                nis = tr.ekf.update_xy(m.e, m.n, sigma)
                if nis > self.cfg.nis_max:
                    # bearing fallback (fable-Q3-2): a geom hit the range gate
                    # rejects still carries a good ANGLE — apply it as a
                    # bearing update so the detector firing every frame can
                    # never starve the track to LOST through a bad-range window.
                    # Refresh the full observation metadata too (codex-R2) —
                    # t_seen alone left consumers reading a stale bearing.
                    tr.ekf.set_origin(*m.origin)
                    nis_b = tr.ekf.update_bearing(m.bearing_deg,
                                                  self._sigma_for("bearing"))
                    if nis_b > self.cfg.nis_max:
                        return False
                    tr.bearing_deg, tr.elev_deg = m.bearing_deg, m.elev_deg
                    tr.origin = m.origin
                    tr.conf = m.conf
                    tr.foot_px, tr.det_index = m.foot_px, m.det_index
                    tr.xyxy = getattr(m, "xyxy", None)
                    tr.t_seen = t
                    return True
                self._commit_source(tr, m.kind)
                tr.z, tr.support_z = m.z, m.support_z
                # fuse the bbox-height range too (second, independent channel —
                # tightens the along-ray error the support plane is blind to)
                if m.rng_h is not None:
                    tr.ekf.set_origin(*m.origin)
                    tr.ekf.update_range(m.rng_h, m.bearing_deg, m.sigma_h)
            tr.range_m, tr.range_conf = m.rng, m.conf
            tr.t_meas = t
        tr.bearing_deg, tr.elev_deg = m.bearing_deg, m.elev_deg
        tr.origin = m.origin
        tr.conf = m.conf
        tr.foot_px, tr.det_index = m.foot_px, m.det_index
        tr.xyxy = getattr(m, "xyxy", None)
        tr.t_seen = t
        return True

    # ---- candidates / birth / rebind ----

    def _candidate_hit(self, t, m):
        ref = math.degrees(math.atan2(self.cfg.gate_m, _BEARING_GATE_REF_M))
        best, best_d = None, math.inf
        for c in self._candidates:
            if c.cls != m.cls or c.t_last == t:
                continue
            if c.e is not None and m.e is not None:
                d = math.hypot(m.e - c.e, m.n - c.n)
            elif c.e is not None:
                oe, on = m.origin
                pred = math.degrees(math.atan2(c.e - oe, c.n - on))
                db = abs(_wrap_deg(m.bearing_deg - pred))
                d = math.radians(db) * _BEARING_GATE_REF_M if db <= ref else None
            else:
                db = abs(_wrap_deg(m.bearing_deg - (c.bearing_deg or 0.0)))
                d = math.radians(db) * _BEARING_GATE_REF_M if db <= ref else None
            if d is not None and d < best_d:
                best, best_d = c, d
        if best is None or best_d > self.cfg.gate_m:
            # re-evaluate rebind on EVERY candidate hit (codex-R2): the first
            # hit commonly lands while the stale track is still active, so a
            # one-shot first-hit decision never rebinds. Gate bearing-only
            # candidates against the predicted dead EKF's bearing corridor
            # too — the blind close pass rebirths bearing-only, and without
            # this the lineage is lost and the pursuit loses the name.
            rebind = self._graveyard_match(t, m)
            self._candidates.append(_Candidate(m, t, rebind))
            return
        best.hits += 1
        best.t_last = t
        best.conf = m.conf
        best.bearing_deg, best.elev_deg = m.bearing_deg, m.elev_deg
        best.origin = m.origin
        best.foot_px, best.det_index = m.foot_px, m.det_index
        best.xyxy = getattr(m, "xyxy", None)
        if m.kind == "geom":
            best.e, best.n, best.z, best.rng = m.e, m.n, m.z, m.rng
            best.src_kind = m.kind
            best.support_z = m.support_z
        if best.hits >= self.cfg.birth_hits:
            self._birth(t, best)
            self._candidates.remove(best)

    def _graveyard_match(self, t, m):
        # unique-best match (codex-R2): the first qualifying track is NOT
        # chosen — score them all, take the minimum, reject ties (ambiguity
        # means hold/reacquire, not a wrong identity). Bearing-only candidates
        # gate on the cross-range corridor to the PREDICTED position.
        best, second = None, None
        for name, (cls, ekf, z, supp, t_state, t_drop) in self._graveyard.items():
            if cls != m.cls or t - t_drop > self.cfg.rebind_window_s:
                continue
            # predict the dead EKF to the match time BEFORE gating (fable-Q3-5):
            # a 3.5 m/s mover exits the 5 m gate in ~1.4 s against the frozen
            # position — well inside the 2 s rebind window, so the naive gate
            # never fired and every rebirth got a fresh name
            if t > t_state:
                ekf.predict(t - t_state)
                self._graveyard[name] = (cls, ekf, z, supp, t, t_drop)
                t_state = t
            if m.e is not None:
                d = math.hypot(m.e - ekf.x[0], m.n - ekf.x[1])
            else:
                oe, on = m.origin
                rel_e, rel_n = ekf.x[0] - oe, ekf.x[1] - on
                pred = math.degrees(math.atan2(rel_e, rel_n))
                db = abs(_wrap_deg(m.bearing_deg - pred))
                d = math.radians(db) * max(math.hypot(rel_e, rel_n), 1.0)
            cand = (name, ekf, t_state)
            if d <= self.cfg.gate_m and (best is None or d < best[0]):
                second, best = best, (d, cand)
            elif d <= self.cfg.gate_m and (second is None or d < second[0]):
                second = (d, cand)
        if best is None:
            return None
        if second is not None and abs(second[0] - best[0]) < 1.0:
            return None                          # ambiguous lineage: hold
        return best[1]

    def _birth(self, t, c):
        if c.rebind is not None:
            name, ekf, t_state = c.rebind
            ekf.predict(max(t - t_state, 1e-3))
            if c.e is not None:
                # re-lock: the rebinding measurement fuses into the resumed EKF
                ekf.update_xy(c.e, c.n, self._sigma_for(c.src_kind))
            self._graveyard.pop(name, None)
        else:
            k = self._counters.get(c.cls, 0)
            self._counters[c.cls] = k + 1
            name, ekf = f"vis_{c.cls}_{k}", None
        tr = _Track(name, c.cls)
        tr.conf, tr.bearing_deg, tr.elev_deg = c.conf, c.bearing_deg, c.elev_deg
        tr.origin = c.origin
        tr.foot_px, tr.det_index = c.foot_px, c.det_index
        tr.xyxy = getattr(c, "xyxy", None)
        tr.t_seen = t
        tr.support_z = c.support_z
        if c.e is not None:                       # positioned at birth
            if ekf is None:
                ekf = CvEkf(c.e, c.n, sigma_pos_m=self._sigma_for(c.src_kind),
                            sigma_vel_mps=self.cfg.v_max_mps / 2.0,
                            accel_max_mps2=self.cfg.accel_max_mps2,
                            nis_max=self.cfg.nis_max)
            tr.ekf = ekf
            tr.src = c.src_kind
            tr.z, tr.range_m, tr.range_conf = c.z, c.rng, c.conf
            tr.t_meas = t
            tr.t_state = t
        self._tracks[name] = tr

    # ---- ToF seam (M3b: beam association + acquisition SM, §3.10/§6.6) ----

    def set_beam_context(self, *, mode: "str | None" = None,
                         own_speed_mps: "float | None" = None) -> None:
        """The flight layer's envelope input (design §3.10): the track loop
        reports its mode ("shadow"|"intercept"|…) and own speed per call.
        Unset (mode None) => never in the reliable envelope => OUT_OF_ENVELOPE
        => no fusion (ToF is opportunistic, never required). own_speed None
        reads as hover (0.0)."""
        with self._lock:
            self._beam_mode = mode
            self._beam_speed = float(own_speed_mps or 0.0)

    def track_state(self, name: str) -> str:
        """Acquisition-SM state for the snapshot's track dict (ICD §1/§3.10):
        the designated contact's DESIGNATED|ACQUIRING|RANGE_LOCKED|
        WORLD_TRACKED|COASTING; LOST for unknown names; IDLE for tracks that
        never entered the acquisition ladder (design's DETECTED_BEARING_ONLY
        maps onto the snapshot's IDLE vocabulary)."""
        with self._lock:
            if name != self._designated:
                return IDLE if name in self._tracks else LOST
            if name not in self._tracks:
                return LOST
            return self._sm_state or DESIGNATED

    # ---- cockpit snapshot seam (ICD §6.7; consumed by VisionPipeline) ----

    def beam_view(self) -> dict:
        """The cockpit beam chip's authoritative state. IDLE with no
        designation; SEARCHING while the SM hunts (incl. OFF_TARGET — the
        beam is on background/other); LOCKED on the last ASSOCIATED cycle
        with the track's fused range; else the legible failure —
        NO-RETURN / EDGE-MIX (AMBIGUOUS or boundary-straddling) /
        OUT-OF-ENVELOPE — so the operator sees WHY fusion isn't happening."""
        with self._lock:
            if self._designated is None:
                return {"status": "IDLE", "target": None, "range_m": None}
            status, rng = "SEARCHING", None
            last = self._beam_last
            if last is not None:
                if last.status == ASSOCIATED:
                    status = "LOCKED"
                    tr = self._tracks.get(self._designated)
                    rng = tr.range_m if tr is not None else None
                elif last.status in (AMBIGUOUS, EDGE):
                    status = "EDGE-MIX"
                elif last.status == OUT_OF_ENVELOPE:
                    status = "OUT-OF-ENVELOPE"
                elif last.status == NO_SAMPLE:
                    status = "NO-RETURN"
            return {"status": status, "target": self._designated,
                    "range_m": rng}

    def track_view(self) -> dict:
        """The cockpit track banner: the acquisition SM verbatim
        (DESIGNATED folds into ACQUIRING — it is the pre-first-attempt
        instant), LOST when the designated track is gone, IDLE with no
        designation; gap_m is the designated track's latest range."""
        with self._lock:
            if self._designated is None:
                return {"state": "IDLE", "target": None, "gap_m": None}
            if self._designated not in self._tracks:
                return {"state": LOST, "target": self._designated,
                        "gap_m": None}
            st = self._sm_state or DESIGNATED
            tr = self._tracks[self._designated]
            return {"state": "ACQUIRING" if st == DESIGNATED else st,
                    "target": self._designated, "gap_m": tr.range_m}

    def _feed_tof(self, t, result, pose, att):
        """One beam-association cycle per update (§3.10's per-cycle algorithm).

        Deterministic consumption order: at most ONE designated target; the
        designated detection is RESERVED (a beam on any other det is
        OFF_TARGET, never a fallback); the robust_at sample is consumed at
        most once, into the designated track only."""
        if self._designated is None:
            return
        tr = self._tracks.get(self._designated)
        if tr is None:
            return                            # designation ahead of birth
        if self._sm_state == DESIGNATED:
            self._sm_state = ACQUIRING        # first attempt: acquiring
        rf, assoc = self._rangefinder, self._beam
        if rf is None or assoc is None or pose is None:
            return
        heading_deg = math.degrees(pose[3])
        # the reserved designated det in THIS frame: the fresh tracker hit's
        # index first, else the track's last accepted index (stale by one)
        dets = result.detections
        hit = result.designated_hit
        det_idx = hit.detection_index if hit is not None else None
        if det_idx is not None and not (0 <= det_idx < len(dets)):
            det_idx = None
        if det_idx is None and tr.det_index is not None \
                and 0 <= tr.det_index < len(dets):
            det_idx = tr.det_index
        robust = getattr(rf, "robust_at", None)
        s = robust(t) if callable(robust) else (
            rf.latest() if callable(getattr(rf, "latest", None)) else None)
        if det_idx is None:
            self._beam_last = BeamAssociation(
                OFF_TARGET, None, None, 0.0,
                "designated det absent from frame")
            self._beam_outcome(OFF_TARGET, fused=False)
            return
        pred_r, pred_s = self._predicted_beam_range(tr, pose, heading_deg)
        # off-boresight = the beam's angle OFF THE BOX (0 when the footprint
        # overlaps): |ax of the det's footpoint from the principal point|
        # minus the box's own half-angle. NOT the det's center vs boresight
        # (±0.25° — unattainable under PX4 yaw hold, 100% OUT_OF_ENVELOPE
        # observed) nor the world-frame yaw error (conflates flight tracking
        # with image-space alignment).
        d = dets[det_idx]
        ax_off, _ay_off = pixel_to_angles(d.footpoint[0], d.footpoint[1],
                                          result.frame.width,
                                          result.frame.height)
        fx = (result.frame.width / 2.0) / math.tan(
            math.radians(HFOV_DEG) / 2.0)
        box_half_deg = math.degrees(
            math.atan(((d.xyxy[2] - d.xyxy[0]) / 2.0) / fx))
        off = max(0.0, abs(math.degrees(ax_off)) - box_half_deg)
        # |Δz| gate: a never-positioned (bearing-only) target's altitude is
        # unknown — read as co-altitude; the gate bites once a z lock exists
        dz = (tr.z - pose[2]) if tr.ekf is not None else 0.0
        in_env = in_fusion_envelope(self._beam_mode, self._beam_speed, dz, off)
        ba = assoc.associate(result.frame, dets, s, att, det_idx, pred_r,
                             pred_s, in_envelope=in_env)
        self._beam_last = ba
        if ba.status != ASSOCIATED:
            self._beam_outcome(ba.status, fused=False)
            return
        # ASSOCIATED: fuse the one reserved sample (the EKF's NIS gate is the
        # second, independent guard — a rejection here is a slip, not a lock)
        if tr.ekf is None:
            # bearing-only track is BORN from the beam lock (§3.10: range +
            # bearing -> position — the acquisition payoff). The 3σ
            # consistency gate has nothing to check a FIRST lock against
            # (prediction is None), so cross-check the range against the
            # box's bbox-height cue instead: a >3σ disagreement means the
            # return is not the box (background through the mask / noise) —
            # a birth at that range is a ghost position the pursuit will
            # fly INTO (observed live: min gap 0.1 m, v8.2). Slip, no lock.
            rng_h, sigma_h = self._height_range_for(d, result.frame)
            if rng_h is not None \
                    and abs(s.range_m - rng_h) > 3.0 * sigma_h:
                self._beam_outcome(ASSOCIATED, fused=False)
                return
            b = math.radians(tr.bearing_deg if tr.bearing_deg is not None
                             else heading_deg)
            tr.ekf = CvEkf(pose[0] + s.range_m * math.sin(b),
                           pose[1] + s.range_m * math.cos(b),
                           sigma_pos_m=self.cfg.sigma_tof_m,
                           sigma_vel_mps=self.cfg.v_max_mps / 2.0,
                           accel_max_mps2=self.cfg.accel_max_mps2,
                           nis_max=self.cfg.nis_max)
            tr.src, tr.pending_src, tr.pending_n = "tof", None, 0
            tr.t_state = t
            # the lock's z is the beam altitude itself (level beam ⇒ the hit
            # sits at the drone's height, design §3.10's near-co-altitude
            # envelope) — NOT the ground-mover support plane, or the |Δz|
            # envelope gate would slam shut on the very next cycle
            tr.z = pose[2]
            fused = True
        else:
            tr.ekf.set_origin(pose[0], pose[1])
            nis = tr.ekf.update_range(s.range_m, heading_deg,
                                      self._peek_sigma(tr, "tof"))
            fused = nis <= self.cfg.nis_max
            if fused:
                self._commit_source(tr, "tof")
        if fused:
            tr.range_m = s.range_m
            tr.range_conf = float(getattr(s, "quality", 1.0))
            tr.t_meas = tr.t_seen = t
        self._beam_outcome(ASSOCIATED, fused=fused)

    def _predicted_beam_range(self, tr, pose, heading_deg):
        """EKF prediction along the beam for the 3σ consistency gate (§3.10
        step 4): (u·(x − origin), sqrt(uᵀPu + σ_tof²)). (None, None) for a
        bearing-only track — nothing to be consistent with on the first lock."""
        if tr.ekf is None:
            return None, None
        b = math.radians(heading_deg)
        ue, un = math.sin(b), math.cos(b)
        rel = tr.ekf.x[:2] - np.asarray(pose[:2], dtype=float)
        pred = float(ue * rel[0] + un * rel[1])
        P = tr.ekf.P
        var = (ue * ue * P[0, 0] + 2.0 * ue * un * P[0, 1]
               + un * un * P[1, 1] + self.cfg.sigma_tof_m ** 2)
        return pred, math.sqrt(max(var, 1e-12))

    def _beam_outcome(self, status, *, fused):
        """SM bookkeeping per association outcome (§3.10): ASSOCIATED+fused
        advances the lock ladder (confirm_hits consecutive => RANGE_LOCKED,
        next lock => WORLD_TRACKED); OUT_OF_ENVELOPE is NEUTRAL (a declined
        fusion, not a verification failure); every other outcome is a slip —
        slip_n consecutive drops a LOCKED beam to COASTING, never LOST."""
        if status == ASSOCIATED and fused:
            self._slip_count = 0
            self._lock_hits += 1
            if self._sm_state == ACQUIRING \
                    and self._lock_hits >= self.cfg.confirm_hits:
                self._sm_state = RANGE_LOCKED
            elif self._sm_state == RANGE_LOCKED:
                self._sm_state = WORLD_TRACKED
            elif self._sm_state == COASTING:
                self._sm_state = ACQUIRING    # re-lock: confirm again
            return
        if status == OUT_OF_ENVELOPE:
            return
        self._lock_hits = 0
        self._slip_count += 1
        if self._slip_count >= self._slip_n \
                and self._sm_state in (RANGE_LOCKED, WORLD_TRACKED):
            self._sm_state = COASTING

    def _tick_sm_dropout(self, t):
        """Dropout leg of the SM (§3.10 diagram): the designated contact
        silent > coast_s => COASTING; > lost_s the track itself drops
        (_drop_stale) and track_state reads LOST — never a LOST-cycle."""
        if self._designated is None or self._sm_state in (None, DESIGNATED):
            return
        tr = self._tracks.get(self._designated)
        if tr is not None and t - tr.t_seen > self.cfg.coast_s \
                and self._sm_state != COASTING:
            self._sm_state = COASTING

    # ---- health / drops ----

    def _drop_stale(self, t):
        for name, tr in list(self._tracks.items()):
            if t - tr.t_seen <= self.cfg.lost_s:
                continue
            if tr.ekf is not None:
                self._graveyard[name] = (tr.cls, tr.ekf, tr.z, tr.support_z,
                                         tr.t_state, t)
            del self._tracks[name]
        for name, entry in list(self._graveyard.items()):
            if t - entry[5] > self.cfg.rebind_window_s:
                del self._graveyard[name]

    def _health(self, tr, now):
        if tr.ekf is None:
            return ACQUIRING
        return MEASURED if now - tr.t_meas <= self.cfg.coast_s else COASTING

    def _view(self, tr, now):
        positioned = tr.ekf is not None
        if positioned:
            e, n = float(tr.ekf.x[0]), float(tr.ekf.x[1])
            ve, vn = float(tr.ekf.x[2]), float(tr.ekf.x[3])
            age = now - tr.t_meas
            fresh = age <= self.cfg.coast_s and tr.src in _RANGED
            z = None if tr.src == "bearing" else tr.z
            rng = None if tr.src == "bearing" else tr.range_m
            rconf = tr.range_conf if tr.src in _RANGED else 0.0
            return ContactView(
                name=tr.name, cls=tr.cls, conf=tr.conf, e=e, n=n, z=z,
                position_src="measured" if fresh else "predicted",
                ve=ve, vn=vn, bearing_deg=tr.bearing_deg,
                elevation_deg=tr.elev_deg, range_m=rng, range_src=tr.src,
                range_conf=rconf, health=self._health(tr, now), age_s=age,
                foot_px=tr.foot_px, bbox_xyxy=tr.xyxy)
        return ContactView(
            name=tr.name, cls=tr.cls, conf=tr.conf, e=None, n=None, z=None,
            position_src="none", ve=0.0, vn=0.0, bearing_deg=tr.bearing_deg,
            elevation_deg=tr.elev_deg, range_m=None, range_src="bearing",
            range_conf=0.0, health=ACQUIRING, age_s=now - tr.t_seen,
            foot_px=tr.foot_px, bbox_xyxy=tr.xyxy)

    # ---- ContactProvider (deep-copy readers; any thread) ----

    def poses(self) -> dict:
        """Contacts WITH a numeric position (measured or predicted, §6.4)."""
        with self._lock:
            return {name: (float(tr.ekf.x[0]), float(tr.ekf.x[1]), tr.z)
                    for name, tr in self._tracks.items() if tr.ekf is not None}

    def sim_time(self) -> float:
        with self._lock:
            return self._sim_t

    def velocities(self) -> dict:
        """CV-state velocity for positioned tracks; {} for bearing-only (§6.4)."""
        with self._lock:
            return {name: (float(tr.ekf.x[2]), float(tr.ekf.x[3]))
                    for name, tr in self._tracks.items() if tr.ekf is not None}

    # ---- extended reads ----

    def ranges(self) -> dict:
        """{name: (range_m, src, sigma)} for positioned tracks holding a fused
        range (ICD §5.1 extended read; M3b). src ∈ geom|tof; sigma is the
        current source's measurement sigma. Bearing-only tracks are absent."""
        with self._lock:
            return {name: (tr.range_m, tr.src, self._sigma_for(tr.src))
                    for name, tr in self._tracks.items()
                    if tr.ekf is not None and tr.range_m is not None
                    and tr.src in _RANGED}

    def health(self, name: str) -> str:
        with self._lock:
            tr = self._tracks.get(name)
            return self._health(tr, self._sim_t) if tr is not None else LOST

    def observation(self, name: str):
        with self._lock:
            tr = self._tracks.get(name)
            return self._view(tr, self._sim_t) if tr is not None else None

    def all_views(self) -> list:
        with self._lock:
            return [self._view(tr, self._sim_t) for tr in self._tracks.values()]

    # ---- TargetDesignator ----

    def attach_detector(self, detector) -> None:
        """The designation seam: the pipeline/pilot binds the Detector so
        designate() can drive its request_lock/clear hooks (ICD §6.4/§6.8).
        With nothing bound, designation is a recorded no-op and never crashes."""
        self._detector = detector

    def designate(self, name: str, *, support_z=None, context=None) -> None:
        with self._lock:
            self._designated = name
            self._designated_support_z = support_z
            self._designated_context = context
            # acquisition SM (re)starts at DESIGNATED for the new target (§3.10)
            self._sm_state = DESIGNATED
            self._lock_hits = 0
            self._slip_count = 0
            self._beam_last = None
            tr = self._tracks.get(name)
            if tr is not None and support_z is not None:
                tr.support_z = float(support_z)
            det = self._detector
            seed_xy = tr.foot_px if tr is not None else None
            seed_index = tr.det_index if tr is not None else None
        if det is not None:
            req = getattr(det, "request_lock", None)
            if callable(req):
                req(seed_xy=seed_xy, seed_index=seed_index)

    def clear_designation(self) -> None:
        with self._lock:
            self._designated = None
            self._designated_support_z = None
            self._designated_context = None
            self._sm_state = None
            self._lock_hits = 0
            self._slip_count = 0
            self._beam_last = None
            det = self._detector
        if det is not None:
            clear = getattr(det, "clear_lock", None)
            if callable(clear):
                clear()

    # ---- lifecycle ----

    def reset(self) -> None:
        """Evals per-cell clean slate: all tracks, candidates, names gone."""
        with self._lock:
            self._tracks.clear()
            self._candidates.clear()
            self._graveyard.clear()
            self._counters.clear()
            self._sim_t = 0.0
            self._designated = None
            self._designated_support_z = None
            self._designated_context = None
            self._sm_state = None
            self._lock_hits = 0
            self._slip_count = 0
            self._beam_last = None
            self._beam_mode = None
            self._beam_speed = 0.0


__all__ = ["TrackerConfig", "CvEkf", "VisionContacts"]
