"""Shared data-transfer objects (ICD §1/§2.6). Pure stdlib — importable anywhere,
testable on any host. ROS/gz-free by law (ICD §0.1).

Frame: one atomic camera frame (C1). LatestFrame: the lock-guarded holder
GzCameras uses internally — snapshot() is always one consistent generation.
"""
import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    seq: int                     # strictly increasing per drone, ≥1
    sim_stamp: float             # gz Image header stamp, seconds; 0.0 pre-first
    width: int
    height: int
    rgb: bytes                   # RGB888, len == width*height*3


class LatestFrame:
    """One writer thread (gz callback), many readers. get() builds an atomic
    Frame under the same lock that set() writes — a reader can never observe
    fields from two different generations (the C1 race)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._f: Frame | None = None

    def set(self, sim_stamp: float, w: int, h: int, rgb: bytes) -> int:
        with self._lock:
            seq = (self._f.seq + 1) if self._f else 1
            self._f = Frame(seq, sim_stamp, w, h, rgb)
            return seq

    def get(self) -> Frame | None:
        with self._lock:
            return self._f

    def seq(self) -> int:
        with self._lock:
            return self._f.seq if self._f else 0


# ---- contact read-model (ContactProvider extended reads, ICD §1) ----

ContactHealth = str   # "MEASURED" | "COASTING" | "ACQUIRING" | "LOST"
RangeSource = str     # "tof" | "geom" | "bearing"
PositionSource = str  # "measured" | "predicted" | "none"


@dataclass(frozen=True)
class ContactView:
    name: str                    # "vis_{cls}_{k}"
    cls: str
    conf: float
    e: float | None              # None while bearing-only-newborn
    n: float | None
    z: float | None
    position_src: PositionSource
    ve: float
    vn: float
    bearing_deg: float | None
    elevation_deg: float | None
    range_m: float | None
    range_src: RangeSource
    range_conf: float
    health: ContactHealth
    age_s: float                 # sim-time seconds since last measurement
    foot_px: tuple | None = None  # last accepted det footpoint (u, v) — the
                                  # acquisition SM's image-servo aim (§3.10)
    bbox_xyxy: tuple | None = None  # last accepted det bbox — the vertical-
                                  # centre reference (erosion-robust, §3.10)
