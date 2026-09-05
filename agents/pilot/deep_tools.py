"""Deep-perception LLM tools (deep-perception plan §4, milestone M2).

`look` (open-vocab YOLO-World detect) and `pinpoint` (one-shot SAM segment)
against the host-GPU sidecar, cloned on detect_text's shape: SYNC text
producers — agents/flight/tools.py binds them with `await
asyncio.to_thread(...)`, so the pilot loop (and the estop) never stalls on a
sidecar call (codex B2).

ADVISORY ONLY (M1b finding, docs/benchmarks/deep-perception-m1.md): on the
flat sim renders YOLO-World confidences compress to 0.05-0.25 and it can
mislabel (a red car read as "person" at 0.23). Deep outputs are hints for the
LLM to reason over — NEVER flight targets; the fast COCO `detect` tool stays
the mover authority. The tool descriptions in tools.py carry this
verbatim-ish, and every result text is tagged "advisory".

One in-flight call max across the pair (a threading busy flag — the sync
bodies run in to_thread workers): a second concurrent call gets BUSY instead
of queueing behind a GPU inference (plan §4 hung-sidecar safety). The tools
never raise into the pilot loop: typed sidecar failures map to legible
UNAVAILABLE/BUSY/ERROR text, and the mask-publisher hook is best-effort.

`mask_publisher` is the M3 cockpit seam: a dependency-injected callable that
receives one pinpoint-mask payload dict (box-local rle + dims + frame_seq +
class/color hints). run.py wires it to the pilot's /pilot/deep channel
(detections-adjacent); elsewhere it stays None (no-op).
"""
import math
import threading
import time

from agents.perception import deep_client as dc
from agents.perception.perception import rel_bearing
from agents.perception.projection import (contact_world, pixel_to_angles,
                                          ray_support_range)

DEFAULT_CONF = 0.05   # M1b: confidences compress to 0.05-0.25 on sim renders
MAX_PROMPTS = 16        # mirror agents/vision/deep/registry.py's wire caps,
MAX_PROMPT_CHARS = 32   # enforced CLIENT-side so a bad call never hits the wire
EDGE_MARGIN_PX = 2      # a box bottom this close to the frame floor is not visible


def _bearing_word(ax: float):
    """Reuse perception.rel_bearing's labeling on a direct pixel angle."""
    return rel_bearing(math.sin(ax), math.cos(ax), 0.0)


def make_deep_tools(world, bridge, pipeline, frame_source, client, i: int = 0,
                    *, mask_publisher=None):
    """-> (look, pinpoint) for make_pilot_options (deep-perception plan §4).

    frame_source: zero-arg callable returning the CURRENT raw Frame
    (run.py injects `lambda: cameras.snapshot(0)`; codex B1 — frames never
    come from the PerceptionSnapshot). client: a DeepClient-shaped object
    (typed DeepResult statuses, never raises for operational failures).
    """
    busy = threading.Lock()
    last_look = {"dets": []}     # closure cache for pinpoint's label resolution

    def _feed_age() -> float | None:
        """Age of the perception feed (the pipeline tracks the same camera at
        10 Hz, so its completion age bounds the grabbed frame's age)."""
        if pipeline is None:
            return None
        snap = pipeline.latest()
        if snap is None:
            return None
        return time.monotonic() - snap.completed_monotonic

    def _fail_text(res) -> str:
        if res.status == dc.UNAVAILABLE:
            return (f"UNAVAILABLE: deep sidecar unreachable ({res.detail}) — "
                    "look/pinpoint are down; the fast `detect` tool still works")
        if res.status == dc.BUSY:
            return ("BUSY: deep sidecar is serving another request — "
                    "retry in a moment")
        return f"ERROR: deep sidecar call failed ({res.detail})"

    def _resolve_label(label: str):
        want = label.strip().lower()
        hits = [h for h in last_look["dets"]
                if h["cls"].lower() == want or h["uid"].lower() == want]
        return None if not hits else max(hits, key=lambda h: h["conf"])

    def _publish_mask(frame, d, cls_hint) -> None:
        """Best-effort cockpit hook (M3 joins it frame_seq-keyed): never
        fails the tool."""
        if mask_publisher is None:
            return
        try:
            cx, cy = d["centroid"]
            px, py = int(cx), int(cy)
            color = None
            if 0 <= px < frame.width and 0 <= py < frame.height:
                off = (py * frame.width + px) * 3
                color = list(frame.rgb[off:off + 3])
            mask_publisher({
                "type": "pinpoint_mask",
                "frame_seq": frame.seq, "sim_stamp": frame.sim_stamp,
                "frame_w": frame.width, "frame_h": frame.height,
                "xyxy": d["xyxy"], "mask": d["mask"],
                "centroid": d["centroid"], "area_px": d["area_px"],
                "score": d["score"],
                "cls": cls_hint,        # class hint: None unless seeded by look()
                "color_rgb": color,     # pixel sample at the centroid
            })
        except Exception:
            pass

    def look(what: str, conf: float = DEFAULT_CONF) -> str:
        prompts = [p.strip() for p in str(what).split(",") if p.strip()]
        if not prompts:
            return ("INVALID_PARAM: `what` must name at least one thing to "
                    "look for (comma-separated)")
        if len(prompts) > MAX_PROMPTS:
            return (f"INVALID_PARAM: too many prompts "
                    f"({len(prompts)} > {MAX_PROMPTS})")
        if any(len(p) > MAX_PROMPT_CHARS for p in prompts):
            return f"INVALID_PARAM: a prompt exceeds {MAX_PROMPT_CHARS} chars"
        if isinstance(conf, bool):
            return f"INVALID_PARAM: conf must be a number in (0, 1], got {conf!r}"
        try:
            conf = float(conf)
        except (TypeError, ValueError):
            return (f"INVALID_PARAM: conf must be a number in (0, 1], "
                    f"got {conf!r}")
        if not 0.0 < conf <= 1.0:
            return f"INVALID_PARAM: conf must be in (0, 1], got {conf}"
        frame = frame_source() if frame_source is not None else None
        if frame is None:
            return "NOT_READY: no camera frame yet"
        if not busy.acquire(blocking=False):
            return ("BUSY: a deep-perception call is already in flight — "
                    "retry in a moment")
        try:
            try:
                res = client.detect(frame, prompts, conf=conf)
            except dc.DeepError as e:
                return f"ERROR: sidecar protocol violation: {e}"
        finally:
            busy.release()
        if not res.ok:
            return _fail_text(res)

        dets = res.data["dets"]
        age = _feed_age()
        header = (f"{len(dets)} advisory deep hit(s) for '{what}' "
                  f"(frame #{frame.seq}")
        if age is not None:
            header += f", feed {age:.1f}s old"
        header += f", {res.data['latency_ms']:.0f} ms)"
        if not dets:
            return (header + ": nothing found — deep output is low-confidence "
                    "ADVISORY; the fast `detect` tool stays the mover authority")

        st = world.drone_state(bridge, i) if world is not None else None
        att = (world.attitude_at(frame.sim_stamp)
               if world is not None else None)
        entries = []
        cached = []
        for k, d in enumerate(dets):
            x1, y1, x2, y2 = (float(v) for v in d["xyxy"])
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            ax, _ay = pixel_to_angles(cx, cy, frame.width, frame.height)
            word, inview, _rel = _bearing_word(ax)
            tag = "" if inview else " (edge)"
            uid = f"deep_{'_'.join(d['cls'].split())}_{k}"
            entry = f"{uid} {d['cls']} conf {d['conf']:.2f} {word}{tag}"
            # ground_intersection ONLY from a VISIBLE bottom-center — never a
            # facade-centroid "range" (codex F9), and labeled as such.
            if y2 < frame.height - EDGE_MARGIN_PX \
                    and st is not None and att is not None:
                bax, bay = pixel_to_angles(cx, y2, frame.width, frame.height)
                rng = ray_support_range(bax, bay, roll=att[0], pitch=att[1],
                                        alt=st[2])
                if rng is not None:
                    we, wn = contact_world(st[0], st[1], st[3], bax, rng)
                    entry += (f" ground_intersection ~{rng:.0f}m at "
                              f"E{we:.0f} N{wn:.0f} (from visible box bottom)")
            entries.append(entry)
            cached.append({"uid": uid, "cls": d["cls"],
                           "conf": float(d["conf"]), "cx": cx, "cy": cy,
                           "frame_seq": frame.seq})
        last_look["dets"] = cached
        return header + ": " + " | ".join(entries)

    def pinpoint(x: int | None = None, y: int | None = None,
                 label: str | None = None) -> str:
        cls_hint = None
        if x is not None or y is not None:
            if x is None or y is None:
                return ("INVALID_PARAM: pinpoint needs BOTH x and y, or a "
                        "`label` from a previous look()")
            if isinstance(x, bool) or isinstance(y, bool) \
                    or not isinstance(x, (int, float)) \
                    or not isinstance(y, (int, float)):
                return (f"INVALID_PARAM: x/y must be pixel numbers, "
                        f"got {x!r},{y!r}")
            px, py = int(round(x)), int(round(y))
            src = "explicit pixel"
        elif label:
            hit = _resolve_label(str(label))
            if hit is None:
                return (f"INVALID_PARAM: no previous look() hit named "
                        f"{label!r} — call look() first (same camera view) "
                        "or pass explicit x,y")
            px, py = int(round(hit["cx"])), int(round(hit["cy"]))
            cls_hint = hit["cls"]
            src = (f"look() hit {hit['uid']} on frame #{hit['frame_seq']} "
                   "(it may have moved since)")
        else:
            return ("INVALID_PARAM: pinpoint needs pixel x,y or a `label` "
                    "from a previous look()")
        frame = frame_source() if frame_source is not None else None
        if frame is None:
            return "NOT_READY: no camera frame yet"
        if not (0 <= px < frame.width and 0 <= py < frame.height):
            return (f"INVALID_PARAM: pixel ({px},{py}) outside the "
                    f"{frame.width}x{frame.height} frame")
        if not busy.acquire(blocking=False):
            return ("BUSY: a deep-perception call is already in flight — "
                    "retry in a moment")
        try:
            try:
                res = client.segment(frame, points=[[px, py]])
            except dc.DeepError as e:
                return f"ERROR: sidecar protocol violation: {e}"
        finally:
            busy.release()
        if not res.ok:
            return _fail_text(res)

        d = res.data
        if d["xyxy"] is None:
            return (f"no mask at ({px},{py}) — SAM found nothing coherent "
                    f"there (frame #{frame.seq})")
        x1, y1, x2, y2 = d["xyxy"]
        cx, cy = d["centroid"]
        ax, _ay = pixel_to_angles(cx, cy, frame.width, frame.height)
        word, inview, _rel = _bearing_word(ax)
        tag = "" if inview else " (edge)"
        label_txt = (f"labeled '{cls_hint}' from look()" if cls_hint
                     else "UNLABELED (SAM does not identify — pair with "
                     "look() to name it)")
        _publish_mask(frame, d, cls_hint)
        return (f"mask at ({px},{py}) [{src}]: centroid {word}{tag}, "
                f"area {d['area_px']}px, tight box "
                f"[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}], "
                f"score {d['score']:.2f} (frame #{frame.seq}, "
                f"{d['latency_ms']:.0f} ms) — mask {label_txt}; ADVISORY "
                "only, never a flight target")

    return look, pinpoint
