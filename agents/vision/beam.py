"""vision/beam.py — beam-footprint association for the forward ToF (ICD §6.6).

The camera answers "what and which way"; the single-point rangefinder answers
"exactly how far" for the ONE designated target (design §3.10). BeamAssociator
decides, per detection cycle, what the beam actually hit:

  ASSOCIATED       footprint disc ⊂ exactly ONE region (mask, or 22%-eroded
                   box) — the reserved designated one — and the range passes
                   the 3σ consistency gate against the filter's prediction
  AMBIGUOUS        footprint ⊂ two or more regions (attribution unsafe)
  EDGE             footprint straddles a region boundary (incl. the erosion
                   margin: inside the raw box but not the eroded one)
  OFF_TARGET       footprint on background, on a NON-designated region (the
                   designated detection is reserved — §3.10's deterministic
                   consumption), or range rejected by the consistency gate
  OUT_OF_ENVELOPE  caller's envelope flag is False — fusion declined a priori
                   (opportunistic-only territory, §3.10)
  NO_SAMPLE        no usable sample: None, range None, non-VALID status
                   (incl. STALE / EDGE_MIX), or stale vs the frame stamp

Gate order: NO_SAMPLE → OUT_OF_ENVELOPE → footprint gates → consistency.
Absence of a valid range is NEVER read as free space (§3.10).

Frames: camera = x-right/y-down/z-forward (ICD §0.3). The rangefinder is
rigidly co-mounted with the camera (sim SDF: range_link and the camera share
the same base_link pose, sim/models/x500_depth/model.sdf), so the beam's
image projection is ATTITUDE-INVARIANT — both sensors rotate together and the
footprint stays glued to the principal point (+parallax of the rigid offset).
`attitude` is retained in the signature per ICD §6.6 for independently-
gimballed mounts; in this build, pitch/roll authority over fusion lives in
the caller's in_envelope flag, not in the projection.
"""
import math
from dataclasses import dataclass

from agents.core.contact import Frame  # noqa: F401
from agents.core.rangefinder import VALID, RangeSample  # noqa: F401
from agents.perception.projection import (HFOV_DEG, erode_box,
                                          footprint_in_region, vfov_deg)
from agents.vision.types import Detection, rle_decode

ASSOCIATED, AMBIGUOUS, EDGE, OFF_TARGET, OUT_OF_ENVELOPE, NO_SAMPLE = (
    "ASSOCIATED", "AMBIGUOUS", "EDGE", "OFF_TARGET", "OUT_OF_ENVELOPE",
    "NO_SAMPLE")
_REGION_INSIDE, _REGION_TOUCH, _REGION_OUTSIDE = 2, 1, 0


@dataclass(frozen=True)
class BeamAssociation:
    """One association verdict (ICD §1). footprint_px is the disc RADIUS."""
    status: str                  # ASSOCIATED|AMBIGUOUS|EDGE|OFF_TARGET|
                                 # OUT_OF_ENVELOPE|NO_SAMPLE
    detection_index: int | None
    residual_m: float | None     # sample minus predicted range
    footprint_px: float
    reason: str


def in_fusion_envelope(mode: str | None, own_speed: float, dz: float,
                       off_boresight_deg: float, *, speed_max_mps: float = 3.0,
                       orbit_speed_max_mps: float = 6.0,
                       dz_max_m: float = 3.0,
                       half_width_deg: float = 0.25) -> bool:
    """The RELIABLE fusion envelope (design §3.10): shadow mode, own speed
    ≤3 m/s, near co-altitude (|Δz| ≤ 3 m), target within the beam half-width
    of boresight (0.25° for the 0.5° beam). Outside it fusion is
    opportunistic-only — the caller may still pass in_envelope=True, this
    module never does on its own.

    W3a: orbit is admitted DELIBERATELY, under its own speed clause. The
    shadow gate's ≤3 m/s exists to bound the footprint's smear across the
    box while translating; an orbit at radius R and rate ω flies
    tangentially at R·ω (15 m · 15°/s ≈ 3.9 m/s — above 3) while the yaw
    servo holds the nose, hence the beam, ON the target, so the smear
    argument doesn't transfer. orbit_speed_max_mps = 6.0 covers the demo
    envelope (≈23 m at 15°/s, or 15 m at ~21°/s) and stays inside the
    pursuit speed band where the detector holds the box steady. The shadow
    clause is NOT loosened."""
    if mode == "orbit":
        return (own_speed <= orbit_speed_max_mps
                and abs(dz) <= dz_max_m
                and abs(off_boresight_deg) <= half_width_deg)
    return (mode == "shadow" and own_speed <= speed_max_mps
            and abs(dz) <= dz_max_m
            and abs(off_boresight_deg) <= half_width_deg)


class BeamAssociator:
    """ICD §6.6. Knobs are documented defaults, not TrackerConfig vectors."""

    def __init__(self, *, hfov_deg: float = HFOV_DEG,
                 cam_to_beam_offset_m: tuple = (0.0, 0.0, 0.0),
                 beam_half_angle_deg: float = 0.25,
                 erosion_frac: float = 0.22,
                 consistency_sigma: float = 3.0,
                 stale_s: float = 0.2) -> None:
        self._hfov_deg = float(hfov_deg)
        self._offset = tuple(float(v) for v in cam_to_beam_offset_m)
        self._half = math.radians(beam_half_angle_deg)
        self._erosion = float(erosion_frac)
        self._k_sigma = float(consistency_sigma)
        self._stale_s = float(stale_s)

    # ---- footprint geometry ----

    def footprint(self, frame: Frame, range_m: float):
        """Beam spot -> (center (u,v), disc radius px). Pinhole projection of
        (cam_to_beam_offset + range along camera z); zero offset => principal
        point. radius = f·tan(half-angle) at the frame's focal lengths."""
        fx = (frame.width / 2.0) / math.tan(math.radians(self._hfov_deg) / 2.0)
        vf = vfov_deg(frame.width, frame.height, self._hfov_deg)
        fy = (frame.height / 2.0) / math.tan(math.radians(vf) / 2.0)
        ox, oy, oz = self._offset
        z = oz + float(range_m)
        cu = frame.width / 2.0 + fx * ox / z
        cv = frame.height / 2.0 + fy * oy / z
        return (cu, cv), 0.5 * (fx + fy) * math.tan(self._half)

    # ---- region relationships ----

    def _mask_relation(self, mask: bytes, frame: Frame, c, r_px: float) -> int:
        rows = rle_decode(mask, frame.width, frame.height)
        cu, cv = c
        r2 = r_px * r_px
        any_in, all_in = False, True
        bound = max(1, math.ceil(r_px))
        for dy in range(-bound, bound + 1):
            for dx in range(-bound, bound + 1):
                if dx * dx + dy * dy > r2 + 1e-9:
                    continue
                x, y = int(round(cu + dx)), int(round(cv + dy))
                inside = (0 <= y < len(rows) and 0 <= x < len(rows[y])
                          and rows[y][x])
                any_in = any_in or inside
                all_in = all_in and inside
        if all_in:
            return _REGION_INSIDE
        return _REGION_TOUCH if any_in else _REGION_OUTSIDE

    def _box_relation(self, xyxy, c, r_px: float) -> int:
        if footprint_in_region(c, r_px, erode_box(xyxy, self._erosion)):
            return _REGION_INSIDE
        x1, y1, x2, y2 = xyxy
        cu, cv = c
        # disc/box intersection: closest point of the raw box to the center
        nx = min(max(cu, x1), x2)
        ny = min(max(cv, y1), y2)
        touch = (nx - cu) ** 2 + (ny - cv) ** 2 <= r_px * r_px + 1e-9
        return _REGION_TOUCH if touch else _REGION_OUTSIDE

    def _relation(self, d: Detection, frame: Frame, c, r_px: float) -> int:
        # The documented region is "mask, OR 22%-eroded box" (module
        # docstring). The blob's mask erodes and HOLES on the shadowed face
        # (the thresholds reject dark pixels) — a disc on a mask hole is on
        # the OBJECT, but a mask-exclusive relation calls it background
        # (v14: every read at 15-30 m with the box centred). Inside when
        # fully inside EITHER the pixel mask or the eroded box; the box side
        # alone when no mask exists (the original fallback).
        box_rel = self._box_relation(d.xyxy, c, r_px)
        if d.mask is None:
            return box_rel
        mask_rel = self._mask_relation(d.mask, frame, c, r_px)
        if box_rel == _REGION_INSIDE or mask_rel == _REGION_INSIDE:
            return _REGION_INSIDE
        if box_rel != _REGION_OUTSIDE or mask_rel != _REGION_OUTSIDE:
            return _REGION_TOUCH
        return _REGION_OUTSIDE

    # ---- the one entry point (ICD §6.6) ----

    def associate(self, frame: Frame, detections: list,
                  sample: "RangeSample | None",
                  attitude: "tuple | None",
                  designated_index: "int | None",
                  predicted_range: "float | None",
                  predicted_sigma: "float | None",
                  *, in_envelope: bool = True) -> BeamAssociation:
        """attitude: (roll, pitch, yaw) rad — accepted per ICD §6.6, unused in
        the projection (co-mounted => attitude-invariant, module docstring)."""
        del attitude                          # see module docstring
        # 1. NO_SAMPLE — no usable range at all (never "free space", §3.10)
        if sample is None:
            return BeamAssociation(NO_SAMPLE, None, None, 0.0, "no sample")
        residual = (sample.range_m - predicted_range
                    if sample.range_m is not None
                    and predicted_range is not None else None)
        if sample.range_m is None:
            return BeamAssociation(NO_SAMPLE, None, residual, 0.0,
                                   "no return")
        if sample.status != VALID:
            return BeamAssociation(NO_SAMPLE, None, residual, 0.0,
                                   f"status {sample.status}")
        if abs(sample.sample_time - frame.sim_stamp) > self._stale_s:
            return BeamAssociation(NO_SAMPLE, None, residual, 0.0,
                                   "stale vs frame")
        # 2. OUT_OF_ENVELOPE — the caller's a-priori decline flag
        if not in_envelope:
            return BeamAssociation(OUT_OF_ENVELOPE, None, residual, 0.0,
                                   "caller envelope gate")
        c, r_px = self.footprint(frame, sample.range_m)
        # 3. footprint gates against every region (the designated one is
        #    RESERVED: inside exactly one NON-designated region is not a
        #    fallback — it is OFF_TARGET, §3.10's deterministic consumption)
        inside, touch = [], []
        for k, d in enumerate(detections):
            rel = self._relation(d, frame, c, r_px)
            if rel == _REGION_INSIDE:
                inside.append(k)
            elif rel == _REGION_TOUCH:
                touch.append(k)
        if len(inside) >= 2:
            return BeamAssociation(AMBIGUOUS, None, residual, r_px,
                                   f"footprint inside {len(inside)} regions")
        if len(inside) == 1:
            k = inside[0]
            if designated_index is not None and k != designated_index:
                return BeamAssociation(
                    OFF_TARGET, k, residual, r_px,
                    f"on non-designated det {k}; designated reserved")
            # 4. range-consistency gate (design §3.10 step 4): one valid-
            #    looking edge/multipath return must not jump the aircraft
            if (predicted_range is not None and predicted_sigma is not None
                    and predicted_sigma > 0.0
                    and abs(residual) > self._k_sigma * predicted_sigma):
                return BeamAssociation(
                    OFF_TARGET, k, residual, r_px,
                    f"consistency reject |{residual:.2f}| > "
                    f"{self._k_sigma:g}·{predicted_sigma:.2f}")
            return BeamAssociation(ASSOCIATED, k, residual, r_px,
                                   "footprint inside reserved region")
        if touch:
            return BeamAssociation(EDGE, touch[0], residual, r_px,
                                   f"footprint straddles det {touch[0]} edge")
        return BeamAssociation(OFF_TARGET, None, residual, r_px,
                               "footprint on background")


__all__ = ["BeamAssociator", "BeamAssociation", "in_fusion_envelope",
           "ASSOCIATED", "AMBIGUOUS", "EDGE", "OFF_TARGET", "OUT_OF_ENVELOPE",
           "NO_SAMPLE"]
