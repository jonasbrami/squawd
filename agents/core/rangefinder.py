"""Rangefinder: reader + reducer for the forward single-point ToF (ICD §2.5).

gz-direct (same ownership pattern as GzCameras/GzPoses): subscribes the 3x3
ray-bundle lidar topic, reduces each bundle to one canonical RangeSample
(min-reduce, intra-bundle spread -> EDGE_MIX, impairment -> quality), and keeps
a short sample buffer for timestamp-addressable robust reads.

numpy is NOT used here (confined to agents/vision) — statistics are stdlib.
"""
import math
import random
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Protocol

RANGE_TOPIC = ("/world/{world}/model/x500_depth_0/link/range_link"
               "/sensor/lidar/scan")  # gz derives the topic from the sensor
                                       # NAME ("lidar"), not its type gpu_lidar

VALID, LOW_SIGNAL, SATURATED, OUT_OF_RANGE, STALE, EDGE_MIX = (
    "VALID", "LOW_SIGNAL", "SATURATED", "OUT_OF_RANGE", "STALE", "EDGE_MIX")


@dataclass(frozen=True)
class RangeSample:
    sample_time: float           # sim clock of the measurement
    receive_time: float          # monotonic receive time
    range_m: float | None        # None = no valid return (NOT free space)
    min_m: float
    max_m: float
    fov_rad: float
    quality: float               # 0..1
    status: str                  # VALID|LOW_SIGNAL|SATURATED|OUT_OF_RANGE|STALE|EDGE_MIX
    seq: int


class RangeProvider(Protocol):
    def latest(self) -> RangeSample | None: ...
    def robust_at(self, sim_stamp: float, *, window_s: float = 0.12,
                  sync_tolerance_s: float = 0.05) -> RangeSample | None: ...


class ImpairmentModel(Protocol):
    def apply(self, hits_m: list, ideal_range_m: float | None,
              rng: random.Random) -> tuple[float | None, float, str]:
        """(bundle hits, RAW IDEAL sensor range — never GzPoses oracle truth)
        -> (range_m|None, quality, status). Injects distance-scaled noise,
        reflectivity max, dropouts, latency. STALE is stamped by
        latest()/robust_at() at READ time, not here."""


class SimImpairment:
    """Documented sim impairment (design §3.10): distance-scaled gaussian
    noise, reflectivity-scaled effective max range, dropouts, latency."""

    def __init__(self, *, sigma_frac: float = 0.005, eff_max_m: float = 60.0,
                 dropout_p: float = 0.05, latency_s: float = 0.02,
                 seed: int = 7) -> None:
        self.sigma_frac, self.eff_max_m = sigma_frac, eff_max_m
        self.dropout_p, self.latency_s = dropout_p, latency_s
        self._rng = random.Random(seed)

    def apply(self, hits_m, ideal_range_m, rng=None):
        r = self._rng if rng is None else rng
        if ideal_range_m is None:
            return None, 0.0, OUT_OF_RANGE
        if r.random() < self.dropout_p:
            return None, 0.2, LOW_SIGNAL
        if ideal_range_m > self.eff_max_m:
            return None, 0.3, LOW_SIGNAL
        noisy = ideal_range_m + r.gauss(0.0, max(0.01,
                                                 self.sigma_frac * ideal_range_m)
                                        ) if ideal_range_m else ideal_range_m
        noisy = max(0.0, noisy)
        quality = max(0.0, 1.0 - ideal_range_m / self.eff_max_m)
        return noisy, quality, VALID


class GzRangeProvider:
    """Reads the 3x3 ray-bundle gz lidar topic; reduces to canonical samples."""

    def __init__(self, topic: str, *, bundle: int = 9,
                 impair: ImpairmentModel | None = None,
                 edge_spread_m: float = 0.3, buf_s: float = 2.0) -> None:
        self.topic = topic
        self.bundle = bundle
        self.impair = impair
        self.edge_spread_m = edge_spread_m
        self.buf_s = buf_s
        self._lock = threading.Lock()
        self._buf: deque = deque()
        self._seq = 0
        self._node = None
        self._connected = False

    def connect(self) -> None:
        from gz.transport13 import Node as GzNode          # lazy: gz at runtime
        from gz.msgs10.laserscan_pb2 import LaserScan
        self._node = GzNode()
        self._node.subscribe(LaserScan, self.topic, self._on_scan)
        self._connected = True

    # ---- reduction (bounded, pure, side-effect-free — §0.2) ----
    def reduce(self, stamp: float, receive_t: float, hits: list,
               min_m: float, max_m: float, fov_rad: float) -> RangeSample:
        valid = [h for h in hits
                 if h is not None and not math.isinf(h) and not math.isnan(h)
                 and min_m <= h <= max_m]
        ideal = min(valid) if valid else None
        spread = (max(valid) - min(valid)) if len(valid) >= 2 else 0.0
        status, quality, rng = VALID, 1.0, ideal
        if ideal is None:
            status, quality = OUT_OF_RANGE, 0.0
        elif spread > self.edge_spread_m:
            status, quality = EDGE_MIX, 0.4
        if self.impair is not None and status == VALID:
            rng, quality, status = self.impair.apply(hits, ideal, None)
        with self._lock:
            self._seq += 1
            seq = self._seq
        return RangeSample(stamp, receive_t, rng, min_m, max_m, fov_rad,
                           quality, status, seq)

    def _on_scan(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        hits = list(msg.ranges)
        min_m = getattr(msg, "range_min", 0.2) or 0.2
        max_m = getattr(msg, "range_max", 100.0) or 100.0
        fov = abs(getattr(msg, "angle_max", 0.0) - getattr(msg, "angle_min", 0.0)) \
            or 0.0087
        s = self.reduce(stamp, time.monotonic(), hits, min_m, max_m, fov)
        with self._lock:
            self._buf.append(s)
            while self._buf and self._buf[0].sample_time < stamp - self.buf_s:
                self._buf.popleft()

    # ---- test seam: feed a synthetic scan without gz ----
    def feed(self, stamp: float, hits: list, min_m: float = 0.2,
             max_m: float = 100.0, fov_rad: float = 0.0087,
             receive_t: float | None = None) -> RangeSample:
        s = self.reduce(stamp, receive_t if receive_t is not None
                        else time.monotonic(), hits, min_m, max_m, fov_rad)
        with self._lock:
            self._buf.append(s)
        return s

    def latest(self) -> RangeSample | None:
        with self._lock:
            if not self._buf:
                return None
            s = self._buf[-1]
        return self._with_stale(s)

    def robust_at(self, sim_stamp: float, *, window_s: float = 0.12,
                  sync_tolerance_s: float = 0.05) -> RangeSample | None:
        """Hampel/median-of-residuals about a linear fit over samples with
        sample_time <= sim_stamp, window_s deep. Returns the selected sample
        with its ORIGINAL sample_time; None when nothing is within
        sync_tolerance_s (staleness honesty)."""
        with self._lock:
            cands = [s for s in self._buf
                     if sim_stamp - window_s <= s.sample_time <= sim_stamp
                     and s.range_m is not None]
        if not cands:
            return None
        newest = cands[-1]
        if sim_stamp - newest.sample_time > sync_tolerance_s:
            return None                    # staleness honesty: nothing to join
        rs = [s.range_m for s in cands]
        if len(rs) >= 3:                       # Hampel: median + 3*MAD reject
            med = statistics.median(rs)
            mad = statistics.median([abs(r - med) for r in rs]) or 1e-6
            rs = [r for r in rs if abs(r - med) <= 3 * 1.4826 * mad] or [med]
        if len(rs) >= 2:                       # residual-of-linear-fit median
            n = len(rs)
            xs = list(range(n))
            sx, sy = sum(xs), sum(rs)
            sxx = sum(x * x for x in xs)
            sxy = sum(x * r for x, r in zip(xs, rs))
            denom = n * sxx - sx * sx
            slope = (n * sxy - sx * sy) / denom if denom else 0.0
            inter = (sy - slope * sx) / n
            resid = [r - (inter + slope * x) for x, r in zip(xs, rs)]
            keep = sorted(zip((abs(v) for v in resid), rs))
            best = keep[len(keep) // 2][1]
        else:
            best = rs[0]
        out = [s for s in cands if abs(s.range_m - best)
               <= max(0.05, 3 * self.edge_spread_m)]
        base = out[-1] if out else newest
        return RangeSample(base.sample_time, base.receive_time, best,
                           base.min_m, base.max_m, base.fov_rad,
                           base.quality, VALID, base.seq)

    def _with_stale(self, s: RangeSample, force: bool = False) -> RangeSample:
        stale = force or (time.monotonic() - s.receive_time) > 1.0
        if not stale:
            return s
        return RangeSample(s.sample_time, s.receive_time, s.range_m,
                           s.min_m, s.max_m, s.fov_rad, s.quality,
                           STALE, s.seq)
