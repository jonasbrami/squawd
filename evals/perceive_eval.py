"""perceive_eval — OFFLINE detector-accuracy harness + the TargetLockEvent path
(ICD §10, design §3.8).

accuracy_report: fixture-based, runs the BACKEND synchronously over recorded
frames TIMESTAMP-JOINED to truth by sim_stamp (nearest within 50 ms — not
"same tick", review Codex-B2). Reports per-class precision/recall (IoU>=0.5 vs
projected truth boxes), box-center error p50/p95 overall and by truth-range
bucket, and ID-switch rate + track fragmentation (review Codex-Mj4) when the
truth rows carry object ids and the backend emits track ids. Pure + offline.

note_target_lock: the identified_target data path (review Codex-B5) — called
SYNCHRONOUSLY at Trace.observe time (≤ one mover-tick association error,
stated): the first track/goto call whose target is a vis_* id becomes a
TargetLockEvent and its contact measurement is associated to oracle truth AT
THAT sim moment; the truth id lands in run_meta['target_lock'] for the oracle.
Report text is never graded (§4.3 stands).

Truth row schema (one per truth sample):
    {"stamp": float,                  # sim seconds
     "boxes":  {cls: [xyxy, ...]},    # projected truth boxes
     "ids":    {cls: [name, ...]},    # optional, aligned with boxes[cls]
     "ranges": {cls: [meters, ...]}}  # optional, aligned — truth slant range

The LIVE contact-position harness is evals/perceive_accuracy.py (the M2 gate
instrument); range_report below serves the M3b ToF gate.
"""
import math
import statistics

from agents.core.contact import TargetLockEvent  # noqa: F401  (the event DTO)

JOIN_TOLERANCE_S = 0.05   # sim_stamp join tolerance (ICD §10)
RANGE_BUCKETS = ((30.0, "<=30m"), (60.0, "30-60m"), (math.inf, ">60m"))
MAX_ASSOC_M = 25.0        # contact→truth association gate: the shadow
                          # controller's dynamic lag runs ~10-13 m at chase
                          # speeds (track_shadow_gate), so 25 m is generous
                          # yet still decoy-proof at >=40 m separation


def iou(a: tuple, b: tuple) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if not inter:
        return 0.0
    area = lambda r: (r[2] - r[0]) * (r[3] - r[1])
    return inter / (area(a) + area(b) - inter)


def _center(box):
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def _bucket(range_m: float) -> str:
    for hi, label in RANGE_BUCKETS:
        if range_m <= hi:
            return label
    return RANGE_BUCKETS[-1][1]


def _p50(xs):
    return statistics.median(xs) if xs else None


def _p95(xs):
    xs = sorted(xs)
    return xs[int(0.95 * (len(xs) - 1))] if len(xs) >= 2 else None


def join_frames_to_truth(frames: list, truths: list,
                         tol_s: float = JOIN_TOLERANCE_S) -> tuple[list, list]:
    """Nearest-within-tolerance join on sim_stamp (the Codex-B2 contract:
    'same tick' matching silently misgrades a 10 Hz detector against a 2 Hz
    truth). Returns (joined [(frame, truth)], unjoined [frame])."""
    joined, unjoined = [], []
    for f in frames:
        best, best_dt = None, tol_s
        for t in truths:
            dt = abs(float(t["stamp"]) - f.sim_stamp)
            if dt <= best_dt:
                best, best_dt = t, dt
        (joined.append((f, best)) if best is not None
         else unjoined.append(f))
    return joined, unjoined


def _match_frame(dets: list, truth: dict, iou_min: float):
    """Greedy IoU match of one frame's detections against one truth row.
    Returns (per_class stats delta, matches [(cls, truth_idx, det)],
    unmatched_dets [(cls, det)])."""
    by_cls: dict[str, list] = {}
    for d in dets:
        by_cls.setdefault(d.cls, []).append(d)
    per_class: dict[str, dict] = {}
    matches, unmatched = [], []
    tboxes_all = truth.get("boxes", {})
    for cls, tboxes in tboxes_all.items():
        st = per_class.setdefault(cls, {"tp": 0, "fp": 0, "fn": 0})
        used = set()
        for k, tb in enumerate(tboxes):
            best, best_j = 0.0, None
            for j, d in enumerate(by_cls.get(cls, [])):
                if j in used:
                    continue
                v = iou(tb, d.xyxy)
                if v > best:
                    best, best_j = v, j
            if best >= iou_min:
                st["tp"] += 1
                used.add(best_j)
                matches.append((cls, k, by_cls[cls][best_j]))
            else:
                st["fn"] += 1
        st["fp"] += len(by_cls.get(cls, [])) - len(used)
        for j, d in enumerate(by_cls.get(cls, [])):
            if j not in used:
                unmatched.append((cls, d))
    for cls, ds in by_cls.items():
        if cls not in tboxes_all:
            per_class.setdefault(cls, {"tp": 0, "fp": 0, "fn": 0})["fp"] += len(ds)
            unmatched.extend((cls, d) for d in ds)
    return per_class, matches, unmatched


def accuracy_report(frames: list, truths: list, backend, conf: float = 0.45,
                    iou_min: float = 0.5,
                    join_tolerance_s: float = JOIN_TOLERANCE_S) -> dict:
    """The M5 detector-accuracy report (ICD §10). Pure + offline.

    Per-class precision/recall at IoU>=iou_min; box-center error p50/p95 (px)
    overall and bucketed by TRUTH range (never the estimate — fable-R3);
    ID-switch rate + track fragmentation over the joined sequence, counted
    only where truth rows carry `ids` and matched detections carry a `tid`
    (single-shot backends without track ids report None there)."""
    joined, unjoined = join_frames_to_truth(frames, truths, join_tolerance_s)
    totals: dict[str, dict] = {}
    center_errors: list[float] = []
    by_range: dict[str, list] = {}
    matched_pairs = 0
    id_switches = 0
    fragmentations = 0
    last_tid: dict[str, int] = {}       # truth id -> last matched backend tid
    was_matched: dict[str, bool] = {}   # truth id -> matched on the previous joined frame
    saw_ids = False

    for frame, truth in joined:
        dets = backend.infer(frame, conf)
        per_class, matches, _unmatched = _match_frame(dets, truth, iou_min)
        for cls, st in per_class.items():
            tot = totals.setdefault(cls, {"tp": 0, "fp": 0, "fn": 0})
            for key in ("tp", "fp", "fn"):
                tot[key] += st[key]
        tids = truth.get("ids", {})
        tranges = truth.get("ranges", {})
        frame_matched_ids = set()
        for cls, k, det in matches:
            matched_pairs += 1
            (cx_t, cy_t), (cx_d, cy_d) = _center(truth["boxes"][cls][k]), _center(det.xyxy)
            err = math.hypot(cx_t - cx_d, cy_t - cy_d)
            center_errors.append(err)
            rng = (tranges.get(cls) or [None] * len(truth["boxes"][cls]))[k]
            if rng is not None:
                by_range.setdefault(_bucket(rng), []).append(err)
            tid = (tids.get(cls) or [None] * len(truth["boxes"][cls]))[k]
            if tid is not None and det.tid is not None:
                saw_ids = True
                frame_matched_ids.add(tid)
                if tid in last_tid and last_tid[tid] != det.tid:
                    id_switches += 1        # same truth object, new track id
                last_tid[tid] = det.tid
        # fragmentation: a truth object that HAD a track, went unmatched for
        # >=1 joined frame, and is matched again = one broken track segment
        for tid in last_tid.keys() | frame_matched_ids:
            now = tid in frame_matched_ids
            if now and was_matched.get(tid) is False:
                fragmentations += 1
            was_matched[tid] = now

    out: dict = {"n_frames": len(frames), "n_joined": len(joined),
                 "n_unjoined": len(unjoined)}
    per_class_out = {}
    for cls, st in totals.items():
        tp, fp, fn = st["tp"], st["fp"], st["fn"]
        per_class_out[cls] = {
            "precision": tp / (tp + fp) if tp + fp else 1.0,
            "recall": tp / (tp + fn) if tp + fn else 1.0,
            "tp": tp, "fp": fp, "fn": fn,
        }
    out["per_class"] = per_class_out
    out["center_err_p50"] = _p50(center_errors)
    out["center_err_p95"] = _p95(center_errors)
    out["center_err_by_range"] = {
        b: {"n": len(es), "p50": _p50(es), "p95": _p95(es)}
        for b, es in sorted(by_range.items())}
    out["id_switches"] = id_switches if saw_ids else None
    out["id_switch_rate"] = (id_switches / matched_pairs
                             if saw_ids and matched_pairs else None)
    out["fragmentations"] = fragmentations if saw_ids else None
    out["fragmentation_rate"] = (fragmentations / matched_pairs
                                 if saw_ids and matched_pairs else None)
    return out


# ---- live truth projection (evals/perceive_report.py uses this) ----

def project_truth_box(drone_e: float, drone_n: float, alt: float,
                      heading: float, e: float, n: float, z_center: float,
                      size_w: float, size_h: float,
                      frame_w: int, frame_h: int):
    """World→pixel inverse of the level-attitude projection path: the truth
    mover's projected box (xyxy) + slant range, or (None, slant) when its
    center falls outside the FOV. Level-hover approximation — the live driver
    gates on |roll|,|pitch| < 2° (stated in its docstring). size_w is the
    mover's horizontal extent (max of w/d), size_h its height."""
    from agents.perception.projection import HFOV_DEG, vfov_deg
    rel_e, rel_n = e - drone_e, n - drone_n
    hd = math.hypot(rel_e, rel_n)
    dz = alt - z_center
    slant = math.sqrt(hd * hd + dz * dz)
    if hd < 1e-6:
        return None, slant
    az = (math.atan2(rel_e, rel_n) - heading + math.pi) % (2 * math.pi) - math.pi
    if abs(az) > math.radians(HFOV_DEG) / 2:
        return None, slant
    el = math.atan2(dz, hd)               # depression angle (down = positive)
    fx = (frame_w / 2) / math.tan(math.radians(HFOV_DEG) / 2)
    fy = (frame_h / 2) / math.tan(math.radians(vfov_deg(frame_w, frame_h)) / 2)
    u = frame_w / 2 + fx * math.tan(az)
    v = frame_h / 2 + fy * math.tan(el)
    if not (0 <= v <= frame_h):
        return None, slant
    half_w = fx * (size_w / 2) / slant
    half_h = fy * (size_h / 2) / slant
    return (u - half_w, v - half_h, u + half_w, v + half_h), slant


# ---- identified_target data path (design §3.8, review Codex-B5) ----

def _contact_xy(contacts, name: str):
    """The contact's current measured (e, n) — ContactView.observation first
    (full read model), poses() fallback. None when bearing-only/unknown."""
    obs_fn = getattr(contacts, "observation", None)
    if callable(obs_fn):
        try:
            v = obs_fn(name)
        except (KeyError, ValueError):
            v = None
        if v is not None and v.e is not None and v.n is not None:
            return (v.e, v.n)
    pos = contacts.poses().get(name)
    if pos is not None:
        return (pos[0], pos[1])
    return None


def associate_to_truth(xy: tuple, truth, gate_m: float = MAX_ASSOC_M):
    """Nearest truth-mover id within gate_m of the measured contact point.
    Returns (truth_id, err_m), or (None, None) when nothing is inside the
    gate — a lock with no plausible truth association must NOT invent one."""
    best, best_d = None, None
    for name, pos in (truth.poses() or {}).items():
        d = math.hypot(xy[0] - pos[0], xy[1] - pos[1])
        if d <= gate_m and (best_d is None or d < best_d):
            best, best_d = name, d
    return best, best_d


def note_target_lock(trace, contacts, truth) -> None:
    """First track/goto tool call whose target is a vis_* id -> TargetLockEvent
    + truth association AT THAT sim moment -> trace.meta['target_lock'] (the
    runner forwards it into run_meta for the oracle's identified_target check).
    Idempotent: only the FIRST lock is recorded (identity is decided at lock
    time; later rebinds are the ID-switch metric's business)."""
    if "target_lock" in trace.meta:
        return
    for ev in trace.events:
        if ev.get("type") != "tool_call":
            continue
        tool = ev.get("name", "").rsplit("__", 1)[-1]
        if tool not in ("track", "goto"):
            continue
        target = (ev.get("args") or {}).get("target") or ""
        if not str(target).startswith("vis_"):
            continue
        sim_stamp = float(contacts.sim_time())
        event = TargetLockEvent(contact_id=str(target), sim_stamp=sim_stamp,
                                tool=tool)
        xy = _contact_xy(contacts, event.contact_id)
        truth_id, assoc_err = (None, None)
        if xy is not None:
            truth_id, assoc_err = associate_to_truth(xy, truth)
            assoc_err = round(assoc_err, 2) if truth_id is not None else None
        trace.meta["target_lock"] = {
            "contact_id": event.contact_id, "sim_stamp": event.sim_stamp,
            "tool": event.tool, "measured_xy": xy, "truth_id": truth_id,
            "assoc_err_m": assoc_err,
        }
        return


# ---- M3b ToF range metrics (design §7 M3b) ----

def range_report(rows: list, associator) -> dict:
    """Beam-association metrics over recorded in-envelope samples.

    rows: [(frame, detections, sample, attitude, designated_index, truth_slant)]
    — truth_slant is the GzPoses distance for scoring only. Pure + offline.

    Returns the M3b gate numbers: slant error p50/p95 (of ASSOCIATED samples),
    availability (fraction of VALID samples that associate), and
    false-association rate (samples assigned to a det whose projected contact
    sits >3 m from the beam point — must be 0)."""
    errs, avail, false_assoc, total = [], 0, 0, 0
    for frame, dets, sample, att, didx, truth_slant in rows:
        a = associator.associate(frame, dets, sample, att, didx, None, None)
        if a.status == "NO_SAMPLE":
            continue
        total += 1
        if a.status == "ASSOCIATED":
            avail += 1
            if sample.range_m is not None and truth_slant is not None:
                errs.append(abs(sample.range_m - truth_slant))
        elif a.status in ("AMBIGUOUS", "OFF_TARGET") and \
                getattr(a, "assigned_m", None) is not None:
            # a WRONG assignment made anyway (the thing the gates exist to
            # prevent): projected contact >3 m from the measured beam point
            if abs(a.assigned_m - sample.range_m) > 3.0:
                false_assoc += 1
    errs = sorted(errs)
    return {
        "n": total,
        "availability": avail / total if total else 0.0,
        "slant_err_p50": statistics.median(errs) if errs else None,
        "slant_err_p95": (errs[int(0.95 * (len(errs) - 1))]
                          if len(errs) >= 2 else None),
        "false_assoc_rate": false_assoc / total if total else 0.0,
    }
