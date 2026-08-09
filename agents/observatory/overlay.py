"""Pure overlay match/staleness + degraded-banner logic (M4, design §3.7).

The cockpit draws the authoritative fusion state (from ``/pilot/detections``)
on the live video frame ONLY when the snapshot's ``sim_stamp`` matches the
frame's stamp within ``OVERLAY_MAX_AGE_S`` — a stale overlay is worse than
none. This module is the tested authority for that rule; the browser UI
(static/index.html) mirrors the same three functions in JS.

Everything here is pure/dumb (no ROS, no gz, no asyncio) so the M4 contract
is unit-testable off-sim: snapshots are plain parsed-JSON dicts.

Also the click hit-test (design v0.3 W0.3): SERVER-side, keyed on the
authoritative ``contacts[].bbox_xyxy``, gated on the same 0.5 s freshness —
an ambiguous (overlapped-crowd) or stale click is rejected, never guessed.

M3 (deep-perception plan §5/§6): the slowlane annotation view, the pinpoint
mask passthrough, and the fp_suspect advisory flags for /state — all joined
by the same 0.5 s frame-age rule, all advisory (no contact mutation).
"""

OVERLAY_MAX_AGE_S = 0.5

SENSING_DEGRADED = "SENSING DEGRADED — manual flight OK"
RANGE_UNAVAILABLE = "RANGE UNAVAILABLE"

# Track states in which the operator expects a live range on the target
# (ICD §1 snapshot vocabulary; IDLE/LOST carry no such expectation).
_TRACK_ACTIVE = ("DESIGNATED", "ACQUIRING", "RANGE_LOCKED", "WORLD_TRACKED",
                 "COASTING")


def overlay_age_s(frame_stamp: float, snap_stamp: float | None) -> float | None:
    """|Δt| (sim seconds) between a video frame and a snapshot.

    None when either side has no valid stamp (0.0/None = pre-first-frame) —
    an unstamped pair can never match.
    """
    if not frame_stamp or snap_stamp is None or snap_stamp <= 0.0:
        return None
    return abs(frame_stamp - snap_stamp)


def overlay_fresh(frame_stamp: float, snap: dict | None,
                  max_age_s: float = OVERLAY_MAX_AGE_S) -> bool:
    """True iff `snap` (parsed PerceptionSnapshot v1, or None) may be drawn on
    the video frame stamped `frame_stamp` (sim seconds)."""
    if not snap:
        return False
    age = overlay_age_s(frame_stamp, snap.get("sim_stamp"))
    return age is not None and age <= max_age_s


def hit_test(snap: dict | None, x: float, y: float,
             frame_stamp: float | None = None,
             max_age_s: float = OVERLAY_MAX_AGE_S) -> dict:
    """Click hit-test against `contacts[].bbox_xyxy` (authoritative, W0.3).

    Returns {"contact": name, "bbox_xyxy": box} when EXACTLY ONE contact box
    contains the frame-pixel point (x, y); otherwise {"contact": None,
    "reason": ...} — "stale" when the snapshot fails the overlayFresh gate
    against the server's newest frame stamp (an unstamped pair never hits),
    "ambiguous" on an overlapped crowd, "miss" when no box contains it.
    """
    if not overlay_fresh(frame_stamp, snap, max_age_s):
        return {"contact": None, "reason": "stale"}
    hits = [c for c in snap.get("contacts") or []
            if _box_contains(c.get("bbox_xyxy"), x, y)]
    if len(hits) > 1:
        return {"contact": None, "reason": "ambiguous"}
    if not hits:
        return {"contact": None, "reason": "miss"}
    c = hits[0]
    return {"contact": c.get("name"), "bbox_xyxy": c.get("bbox_xyxy")}


def _box_contains(box, x: float, y: float) -> bool:
    return (box is not None and len(box) == 4
            and box[0] <= x <= box[2] and box[1] <= y <= box[3])


def hud_banner(snap: dict | None) -> str | None:
    """The first-class degraded states (design §3.7/§3.10).

    Detector down beats everything (SENSING DEGRADED — manual flight still
    OK); an active track whose target has no valid range reads RANGE
    UNAVAILABLE — never "free space". No snapshot yet => no verdict => no
    banner (absence of data is not a failure).
    """
    if not snap:
        return None
    if (snap.get("detector") or {}).get("healthy") is False:
        return SENSING_DEGRADED
    track = snap.get("track") or {}
    if track.get("state") in _TRACK_ACTIVE:
        target = track.get("target")
        tgt = next((c for c in snap.get("contacts") or []
                    if c.get("name") == target), None)
        if tgt is not None and tgt.get("range_m") is None:
            return RANGE_UNAVAILABLE
    return None


# ---- deep-perception M3 (plan §5/§6): slowlane annotations, the fp_suspect
# advisory, and the pinpoint-mask passthrough. Everything joins by the SAME
# 0.5 s frame-age rule as the overlay (codex F3: a stale advisory is worse
# than none); the pilot-side slowlane computes the overlap against the exact
# InferenceResult — here we only render/flag, never recompute geometry.

def annotations_for(payload: dict | None, frame_stamp: float | None,
                    max_age_s: float = OVERLAY_MAX_AGE_S) -> list:
    """The /state annotation view of a /pilot/slowlane payload: the published
    dets (cls, conf, xyxy) each stamped with frame_seq/sim_stamp/age_ms while
    the payload survives the 0.5 s frame-age gate, else [] (expired)."""
    if not overlay_fresh(frame_stamp, payload, max_age_s):
        return []
    age_ms = round(overlay_age_s(frame_stamp, payload["sim_stamp"]) * 1000)
    return [{"cls": d.get("cls"), "conf": d.get("conf"), "xyxy": d.get("xyxy"),
             "frame_seq": payload.get("frame_seq"),
             "sim_stamp": payload.get("sim_stamp"), "age_ms": age_ms}
            for d in payload.get("dets") or []]


def pinpoint_mask_for(payload: dict | None, frame_stamp: float | None,
                      max_age_s: float = OVERLAY_MAX_AGE_S) -> dict | None:
    """The /state passthrough of the latest /pilot/deep pinpoint mask while
    frame-fresh (the UI draws it box-local, frame_seq-joined), else None."""
    if not isinstance(payload, dict) \
            or payload.get("type") != "pinpoint_mask" \
            or payload.get("mask") is None:
        return None
    if not overlay_fresh(frame_stamp, payload, max_age_s):
        return None
    out = {k: payload.get(k) for k in
           ("frame_seq", "sim_stamp", "frame_w", "frame_h", "xyxy", "mask",
            "centroid", "area_px", "score", "cls", "color_rgb")}
    out["age_ms"] = round(
        overlay_age_s(frame_stamp, payload["sim_stamp"]) * 1000)
    return out


def _iou(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0.0:
        return 0.0
    area = lambda r: max(0.0, r[2] - r[0]) * max(0.0, r[3] - r[1])
    union = area(a) + area(b) - inter
    return inter / union if union > 0.0 else 0.0


def mark_fp_suspects(contacts: list | None, payload: dict | None,
                     snap_stamp: float | None, frame_stamp: float | None,
                     max_age_s: float = OVERLAY_MAX_AGE_S,
                     iou_min: float = 0.5) -> list | None:
    """fp_suspect flags on the /state contact views (advisory only).

    A contact is flagged when a fp_suspects entry of the slowlane payload
    matches it (same cls, box IoU ≥ iou_min — the entry carries the fast det
    box from the exact annotated frame; a fresh contact box of the same
    object is near-identical). Gated twice: the payload must be fresh vs the
    served camera frame AND within max_age_s of the snapshot the contacts
    came from. Returns a NEW list (input never mutated); contacts without a
    match carry fp_suspect False so the flag can CLEAR as advisories expire.
    """
    if contacts is None:
        return None
    suspects = []
    if overlay_fresh(frame_stamp, payload, max_age_s) and snap_stamp \
            and abs(payload["sim_stamp"] - snap_stamp) <= max_age_s:
        suspects = payload.get("fp_suspects") or []
    out = []
    for c in contacts:
        c = dict(c)
        box = c.get("bbox_xyxy")
        c["fp_suspect"] = bool(
            box is not None and any(
                s.get("cls") == c.get("cls") and s.get("xyxy") is not None
                and _iou(s["xyxy"], box) >= iou_min
                for s in suspects))
        out.append(c)
    return out
