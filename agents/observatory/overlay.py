"""Pure overlay match/staleness + degraded-banner logic (M4, design §3.7).

The cockpit draws the authoritative fusion state (from ``/pilot/detections``)
on the live video frame ONLY when the snapshot's ``sim_stamp`` matches the
frame's stamp within ``OVERLAY_MAX_AGE_S`` — a stale overlay is worse than
none. This module is the tested authority for that rule; the browser UI
(static/index.html) mirrors the same three functions in JS.

Everything here is pure/dumb (no ROS, no gz, no asyncio) so the M4 contract
is unit-testable off-sim: snapshots are plain parsed-JSON dicts.
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
