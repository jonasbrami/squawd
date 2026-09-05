"""detect tool text composition (ICD §5.5 grammar), pilot layer.

Formats the authoritative PerceptionSnapshot into the detect result text:
header + pipe-joined entries `id cls conf bearing [range src (world)]`.
At M2 (pre-VisionContacts): ids are ephemeral per-frame `vis_{cls}_{k}`;
range is geometric (support-plane, attitude-corrected) when computable,
else "bearing only".
"""
from agents.perception.projection import (contact_world, pixel_to_angles,
                                          ray_support_range)
from agents.perception.perception import rel_bearing
from agents.vision.pipeline import PerceptionSnapshot  # noqa: F401

import math
import time


def _bearing_word(ax: float):
    """Reuse perception.rel_bearing's labeling on a direct pixel angle."""
    return rel_bearing(math.sin(ax), math.cos(ax), 0.0)


def make_detect_text(world, bridge, pipeline, i: int = 0):
    """-> detect_text(classes) for make_pilot_options (ICD §5.5)."""

    def detect_text(classes: str | None) -> str:
        snap = pipeline.latest() if pipeline is not None else None
        if snap is None:
            return "NOT_READY: nothing detected yet (no frames)"
        if not snap.detector.get("healthy", True):
            return "NOT_READY: sensing degraded (detector down)"

        want = ({c.strip().lower() for c in classes.split(",") if c.strip()}
                if classes else None)
        dets = [d for d in snap.dets
                if want is None or d.cls.lower() in want]
        age = time.monotonic() - snap.completed_monotonic
        header = f"{len(dets)} detections (frame #{snap.frame_seq}, {age:.1f}s old)"
        if not dets:
            return header + ": nothing detected" + (
                f" (filter: {classes})" if classes else "")

        st = world.drone_state(bridge, i)
        att = world.attitude_at(snap.sim_stamp)
        entries = []
        for k, d in enumerate(dets):
            uid = f"vis_{d.cls}_{k}"
            ax, ay = pixel_to_angles(d.cx, d.cy, snap.frame_w, snap.frame_h)
            word, inview, _rel = _bearing_word(ax)
            tag = "" if inview else " (edge)"
            rng_txt = "(bearing only)"
            if st is not None and att is not None:
                rng = ray_support_range(ax, ay, roll=att[0], pitch=att[1],
                                        alt=st[2])
                if rng is not None:
                    we, wn = contact_world(st[0], st[1], st[3], ax, rng)
                    rng_txt = (f"~{rng:.0f}m geom (at E{we:.0f} N{wn:.0f})")
            entries.append(f"{uid} {d.cls} conf {d.conf:.2f} {word}{tag} {rng_txt}")
        return header + ": " + " | ".join(entries)

    return detect_text
