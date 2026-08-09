"""vision/slowlane.py — the gated slow-lane annotator (deep-perception plan
§5, milestone M3).

A daemon THREAD (never the pilot's asyncio loop) sampling the latest camera
frame at ~0.3 Hz (DEEP_SLOWLANE_HZ overrides) and running open-vocab detect
on the host-GPU sidecar with a static vocabulary (DEEP_SLOWLANE_VOCAB,
default "building,house,tree,pole,tower"; DEEP_SLOWLANE_CONF, default 0.05 —
the M1b operating point). Each sample cycle is published through the injected
publisher (run.py → /pilot/slowlane, String JSON on STATE_QOS) keyed by
frame_seq + sim_stamp; the cockpit joins annotations to the video exactly
like the detection overlay and expires them at ≤0.5 s of frame age
(codex F3).

SKIP-IF-BUSY, zero queue (codex F4): the loop runs one call at a time — a
call that overruns the period eats the missed ticks (no catch-up burst) —
and a sidecar BUSY/UNAVAILABLE/ERROR answer drops the tick. On-demand
look/pinpoint calls keep priority via the sidecar's own one-inference lock
(429 → BUSY here).

FP advisory (codex F3): overlap = intersection / fast_box_area of each fast
det against each building/house annotation, computed against the fast dets
of the EXACT submitted InferenceResult — never the current frame. ≥0.6 lists
the fast det in the payload's fp_suspects; the cockpit marks the matching
/state contact view with fp_suspect: true. Pure advisory: this module never
touches VisionContacts or the tracker, and annotations never birth contacts.

GATE (plan §5): off by default under RENDER_BACKEND=nvidia (gz shares the
GPU — stays gated); the armed gate was lifted for intel after the M3 A/B
coexistence gate passed (docs/benchmarks/deep-perception-m3.md), so the
shipped default is ON unless nvidia. DEEP_SLOWLANE=on forces, =off
disables. See gate_decision().
"""
import os
import threading
import time

from agents.perception import deep_client as dc

DEFAULT_HZ = 0.3
DEFAULT_VOCAB = "building,house,tree,pole,tower"
DEFAULT_CONF = 0.05           # M1b: confidences compress to 0.05–0.25 on sim
MIN_HZ, MAX_HZ = 0.05, 2.0    # env clamp: never a second fast lane
MAX_PROMPTS = 16              # mirror agents/vision/deep/registry.py wire caps
MAX_PROMPT_CHARS = 32
FP_OVERLAP_MIN = 0.6                    # intersection / fast_box_area (codex F3)
FP_ANNOTATION_CLASSES = ("building", "house")
EXACT_FRAME_WAIT_S = 1.5      # poll cap for the exact-seq InferenceResult

# Render backends exempt from the armed gate. The M3 A/B coexistence gate
# PASSED for intel (docs/benchmarks/deep-perception-m3.md: RTF, PX4 time-sync,
# fast-lane latency/cadence, sidecar p50/p95, VRAM all flat over ≥5 min arms
# with the drone armed in HOLD) → intel is exempt. nvidia stays gated
# regardless — gz renders on the same GPU (codex F4).
ARMED_GATE_EXEMPT: tuple = ("intel",)


def gate_decision(force, render_backend, armed, *,
                  armed_exempt=ARMED_GATE_EXEMPT) -> tuple[bool, str]:
    """The M3 gate -> (enabled, legible reason).

    force          -> DEEP_SLOWLANE value ("on"/"off"/None); always wins
    render_backend -> RENDER_BACKEND env ("intel"/"nvidia"/"cpu")
    armed          -> True while the drone is armed (PX4 arming_state ARMED)
    armed_exempt   -> backends the A/B gate cleared for armed operation
                      (shipped: intel — docs/benchmarks/deep-perception-m3.md)
    """
    f = (force or "").strip().lower()
    if f == "off":
        return False, "DEEP_SLOWLANE=off"
    if f == "on":
        return True, "DEEP_SLOWLANE=on"
    rb = (render_backend or "").strip().lower()
    if rb == "nvidia":
        return False, "RENDER_BACKEND=nvidia (gz shares the GPU)"
    if armed and rb not in armed_exempt:
        return False, f"drone armed ({rb or 'unknown'} not A/B-exempt)"
    return True, "default (non-nvidia)"


def box_area(b) -> float:
    return max(0.0, float(b[2]) - float(b[0])) * max(0.0, float(b[3]) - float(b[1]))


def intersection_over_fast(fast_xyxy, ann_xyxy) -> float:
    """intersection / fast_box_area in [0, 1] — the codex F3 overlap."""
    fa = box_area(fast_xyxy)
    if fa <= 0.0:
        return 0.0
    x1 = max(fast_xyxy[0], ann_xyxy[0])
    y1 = max(fast_xyxy[1], ann_xyxy[1])
    x2 = min(fast_xyxy[2], ann_xyxy[2])
    y2 = min(fast_xyxy[3], ann_xyxy[3])
    return max(0.0, x2 - x1) * max(0.0, y2 - y1) / fa


def fp_suspects(fast_dets, annotations, *, overlap_min=FP_OVERLAP_MIN,
                fp_classes=FP_ANNOTATION_CLASSES) -> list:
    """Advisory flags (plan §5): a fast det whose box is ≥ overlap_min covered
    by a building/house annotation is fp_suspect. Both sides are duck-typed
    {cls, conf, xyxy} — the fast side MUST come from the exact submitted
    InferenceResult (the caller's contract; a wrong frame makes the geometry
    meaningless, codex F3)."""
    anns = [a for a in annotations if a["cls"] in fp_classes]
    out = []
    for d in fast_dets:
        best = None
        for a in anns:
            r = intersection_over_fast(d["xyxy"], a["xyxy"])
            if r >= overlap_min and (best is None or r > best["overlap"]):
                best = {"cls": d["cls"], "conf": d["conf"],
                        "xyxy": [round(float(v), 1) for v in d["xyxy"]],
                        "ann_cls": a["cls"],
                        "ann_xyxy": [round(float(v), 1) for v in a["xyxy"]],
                        "overlap": round(r, 3)}
        if best is not None:
            out.append(best)
    return out


def _hz(value) -> float:
    if value is None:
        value = os.environ.get("DEEP_SLOWLANE_HZ")
    try:
        hz = float(value) if value is not None else DEFAULT_HZ
    except (TypeError, ValueError):
        return DEFAULT_HZ
    return min(MAX_HZ, max(MIN_HZ, hz))


def _vocab(value) -> list:
    if value is None:
        value = os.environ.get("DEEP_SLOWLANE_VOCAB")
    parts = [p.strip() for p in str(value or DEFAULT_VOCAB).split(",")
             if p.strip()]
    return [p[:MAX_PROMPT_CHARS] for p in parts[:MAX_PROMPTS]] or \
        DEFAULT_VOCAB.split(",")


def _conf(value) -> float:
    if value is None:
        value = os.environ.get("DEEP_SLOWLANE_CONF")
    try:
        c = float(value) if value is not None else DEFAULT_CONF
    except (TypeError, ValueError):
        return DEFAULT_CONF
    return c if 0.0 < c <= 1.0 else DEFAULT_CONF


class SlowLane:
    """One gated annotator thread. All dependencies injected (run.py):
    frame_source -> zero-arg callable returning the latest Frame (codex B1 —
    frames never come from the PerceptionSnapshot); client -> DeepClient-shaped
    (typed statuses, never raises for operational failures); detector -> the
    fast lane's Detector (exact-frame dets for the FP advisory; None disables
    the advisory, never the annotations); publisher -> one-dict callable;
    gate -> zero-arg callable returning gate_decision's (enabled, reason).
    """

    def __init__(self, frame_source, client, *, detector=None, publisher=None,
                 gate=None, hz=None, vocab=None, conf=None,
                 monotonic=time.monotonic, sleep=time.sleep) -> None:
        self._frame_source = frame_source
        self._client = client
        self._detector = detector
        self._publisher = publisher
        self._gate = gate or (lambda: (True, "no gate"))
        self._hz = _hz(hz)
        self._vocab = _vocab(vocab)
        self._conf = _conf(conf)
        self._now = monotonic
        self._sleep = sleep
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()        # guards _counters/_last
        self._counters = {"ticks": 0, "calls": 0, "ok": 0,
                          "dropped_busy": 0, "dropped_unavailable": 0,
                          "dropped_error": 0, "skipped_gate": 0,
                          "skipped_no_frame": 0, "fp_checked": 0}
        self._last: dict = {"frame_seq": None, "sim_stamp": None,
                            "frame_w": None, "frame_h": None,
                            "captured_mono": None, "dets": [],
                            "fp_suspects": [], "fp_checked": False,
                            "fast_dets": [], "latency_ms": None}
        self._last_error: str | None = None

    # ---- lifecycle ----

    def start(self) -> None:
        self._thread = threading.Thread(target=self.run, daemon=True,
                                        name="slowlane")
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def run(self) -> None:
        """Fixed-rate loop. A tick that overruns the period drops the missed
        beats (next_t resets to now) — never a catch-up burst (zero queue)."""
        period = 1.0 / self._hz
        next_t = self._now()
        while not self._stop.is_set():
            next_t += period
            self.tick_once()
            delay = next_t - self._now()
            if delay < 0.0:
                next_t = self._now()
                delay = period
            self._stop.wait(delay)

    # ---- one sample cycle ----

    def tick_once(self) -> dict:
        """Gate → frame → detect → exact-frame overlap → publish. Returns the
        published payload (also when gated/skipped: a health-only beat)."""
        enabled, reason = self._gate()
        with self._lock:
            self._counters["ticks"] += 1
        if not enabled:
            with self._lock:
                self._counters["skipped_gate"] += 1
            return self._publish(self._payload(False, reason))
        frame = self._frame_source() if self._frame_source is not None else None
        if frame is None:
            with self._lock:
                self._counters["skipped_no_frame"] += 1
            return self._publish(self._payload(False, "no camera frame yet"))
        captured = self._now()
        with self._lock:
            self._counters["calls"] += 1
        try:
            res = self._client.detect(frame, self._vocab, conf=self._conf)
        except dc.DeepError as e:
            return self._drop("dropped_error", f"protocol: {e}")
        if not res.ok:
            key = {"BUSY": "dropped_busy",
                   "UNAVAILABLE": "dropped_unavailable"}.get(
                       res.status, "dropped_error")
            return self._drop(key, f"{res.status}: {res.detail}")
        anns = [{"cls": d["cls"], "conf": d["conf"], "xyxy": d["xyxy"]}
                for d in res.data["dets"]]
        fps, fast = self._fp_advisory(frame, anns)
        with self._lock:
            self._counters["ok"] += 1
            self._last_error = None
            self._last = {
                "frame_seq": frame.seq, "sim_stamp": frame.sim_stamp,
                "frame_w": frame.width, "frame_h": frame.height,
                "captured_mono": captured,
                "dets": anns, "fp_suspects": fps or [],
                "fp_checked": fps is not None, "fast_dets": fast,
                "latency_ms": res.data["latency_ms"]}
        return self._publish(self._payload(True, reason))

    def _fp_advisory(self, frame, anns):
        """-> (fp_suspects | None, fast_dets). None = NOT checked: the exact
        InferenceResult for frame.seq was unavailable (no fast lane, or it
        scrolled past) — the advisory is then absent, never computed against
        a different frame (codex F3)."""
        if self._detector is None:
            return None, []
        deadline = self._now() + EXACT_FRAME_WAIT_S
        while True:
            res = self._detector.detections()
            if res is not None and res.frame.seq == frame.seq:
                fast = [{"cls": d.cls, "conf": d.conf,
                         "xyxy": [float(v) for v in d.xyxy]}
                        for d in res.detections]
                with self._lock:
                    self._counters["fp_checked"] += 1
                return fp_suspects(fast, anns), fast
            if res is not None and res.frame.seq > frame.seq:
                return None, []          # the exact result scrolled past
            if self._now() >= deadline:
                return None, []
            self._sleep(0.02)

    # ---- state / publish ----

    def _drop(self, counter: str, detail: str) -> dict:
        with self._lock:
            self._counters[counter] += 1
            self._last_error = detail
        return self._publish(self._payload(True, detail))

    def _payload(self, active: bool, note: str) -> dict:
        with self._lock:
            last = dict(self._last)
            health = {"active": active, "note": note, "hz": self._hz,
                      "vocab": list(self._vocab), "conf": self._conf,
                      "last_error": self._last_error, **self._counters}
        return {"type": "slowlane",
                "frame_seq": last["frame_seq"],
                "sim_stamp": last["sim_stamp"],
                "frame_w": last["frame_w"], "frame_h": last["frame_h"],
                # pilot-process monotonic — audit only, NOT comparable across
                # processes (the cockpit expires by sim_stamp frame age).
                "captured_mono": last["captured_mono"],
                "dets": last["dets"], "fp_suspects": last["fp_suspects"],
                "fp_checked": last["fp_checked"],
                "fast_dets": last["fast_dets"],
                "latency_ms": last["latency_ms"],
                "health": health}

    def _publish(self, payload: dict) -> dict:
        if self._publisher is not None:
            try:
                self._publisher(payload)
            except Exception:
                pass                     # best-effort, like the mask publisher
        return payload

    def state(self) -> dict:
        """The small health dict (the payload's `health` section)."""
        with self._lock:
            return {"hz": self._hz, "vocab": list(self._vocab),
                    "conf": self._conf, "last_error": self._last_error,
                    "last_frame_seq": self._last["frame_seq"],
                    **self._counters}
