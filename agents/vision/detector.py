"""vision/detector.py — the inference thread (ICD §6.3).

One daemon thread per Detector instance; the gz callback thread never does
inference. Lifecycle: INIT -> RUNNING -> DEGRADED (3 consecutive frame failures
or stale camera >2s) -> STOPPED. Designation matching also runs on this thread.
"""
import threading
import time

from agents.core.contact import Frame  # noqa: F401 (type clarity)
from agents.vision.types import AssociationHit, BackendError, InferenceResult

INIT, RUNNING, DEGRADED, STOPPED = "INIT", "RUNNING", "DEGRADED", "STOPPED"
_STALE_S = 2.0


class Detector:
    def __init__(self, cameras, backend, *, i: int = 0, hz: float = 5.0,
                 conf: float = 0.45) -> None:
        self._cameras, self._backend, self._i = cameras, backend, i
        self._hz, self._conf = hz, conf
        self._state = INIT
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest: InferenceResult | None = None
        self._thread: threading.Thread | None = None
        self._fails = 0
        self._latency_ms = 0.0
        self._last_frame_mono = 0.0
        self._lock_request: tuple | None = None   # (seed_xy, seed_index)

    # ---- lifecycle ----
    def start(self) -> None:
        """Raises BackendError (the pilot catches it into a degraded boot)."""
        load = getattr(self._backend, "load", None)
        if callable(load):
            try:
                load()
            except Exception as e:
                raise BackendError(str(e))
        with self._lock:
            self._state = RUNNING
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"detector-{self._i}")
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        with self._lock:
            self._state = STOPPED
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def healthy(self) -> bool:
        with self._lock:
            return (self._state == RUNNING
                    and time.monotonic() - self._last_frame_mono < _STALE_S + 1.0)

    def state(self) -> str:
        with self._lock:
            return self._state

    def latency_ms(self) -> float:
        with self._lock:
            return self._latency_ms

    # ---- consumption ----
    def detections(self) -> InferenceResult | None:
        with self._lock:
            return self._latest

    def wait_next(self, after_seq: int, timeout: float) -> InferenceResult | None:
        """Wait for an inference NEWER than after_seq (post-face freshness)."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                if self._latest is not None and self._latest.frame.seq > after_seq:
                    return self._latest
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(timeout=min(remaining, 0.5))

    def request_lock(self, seed_xy: tuple | None = None,
                     seed_index: int | None = None) -> None:
        """Match one designation against the next inference."""
        with self._lock:
            self._lock_request = (seed_xy, seed_index)

    def clear_lock(self) -> None:
        with self._lock:
            self._lock_request = None

    # ---- thread body ----
    def _run(self) -> None:
        period = 1.0 / self._hz
        last_seq = 0
        while True:
            with self._lock:
                if self._state == STOPPED:
                    return
            t0 = time.monotonic()
            frame = self._cameras.snapshot(self._i)
            if frame is None or frame.seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = frame.seq
            self._last_frame_mono = t0
            try:
                t_inf = time.monotonic()
                dets = self._backend.infer(frame, self._conf)
                hit = self._consume_lock_request(frame, dets)
                res = InferenceResult(frame, dets, time.monotonic(), hit)
                self._latency_ms = (0.9 * self._latency_ms
                                    + 0.1 * (time.monotonic() - t_inf) * 1000.0)
                self._fails = 0
            except Exception:
                self._fails += 1
                if self._fails >= 3:
                    with self._lock:
                        self._state = DEGRADED
                continue
            with self._cond:
                self._latest = res
                self._cond.notify_all()
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

    def _consume_lock_request(self, frame: Frame, dets: list):
        with self._lock:
            req, self._lock_request = self._lock_request, None
        if req is None:
            return None
        seed_xy, seed_index = req
        if seed_index is not None and 0 <= seed_index < len(dets):
            k = seed_index
        elif seed_xy is not None and dets:
            x, y = seed_xy
            inside = [k for k, d in enumerate(dets)
                      if d.xyxy[0] <= x <= d.xyxy[2]
                      and d.xyxy[1] <= y <= d.xyxy[3]]
            k = min(inside or range(len(dets)),
                    key=lambda j: (dets[j].cx - x) ** 2 + (dets[j].cy - y) ** 2)
        else:
            return None
        d = dets[k]
        return AssociationHit(k, d.xyxy, d.footpoint, d.conf, None)
