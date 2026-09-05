"""vision/types.py — detector-internal DTOs (ICD §1/§6.1). numpy enters the
package here (confined to agents/vision per ICD §0.1); no flight/pilot/ROS/gz
imports anywhere in this package.

RLE mask codec: row-major bool array as run-length varints, starting with the
count of leading ZEROS, alternating. rle_encode/rle_decode are the contract
(tested in tests/test_vision_detector.py)."""
from dataclasses import dataclass
from agents.core.contact import Frame  # noqa: F401  (re-exported for convenience)


@dataclass(frozen=True)
class Detection:
    cls: str                     # backend label; ColorBlobBackend pins "target"
    conf: float
    xyxy: tuple[float, float, float, float]
    mask: bytes | None = None    # RLE (codec below) when the backend has one

    @property
    def cx(self) -> float:
        return (self.xyxy[0] + self.xyxy[2]) / 2.0

    @property
    def cy(self) -> float:
        return (self.xyxy[1] + self.xyxy[3]) / 2.0

    @property
    def footpoint(self) -> tuple[float, float]:
        """Bottom-center of the box (ground-contact projection point)."""
        return ((self.xyxy[0] + self.xyxy[2]) / 2.0, float(self.xyxy[3]))


@dataclass(frozen=True)
class InferenceResult:
    frame: Frame
    detections: list
    completed_monotonic: float
    designated_hit: "AssociationHit | None" = None


@dataclass(frozen=True)
class AssociationHit:
    detection_index: int | None
    xyxy: tuple[float, float, float, float] | None
    aim_px: tuple[float, float]   # footpoint when derivable, else patch centroid
    conf: float                   # tracker-side confidence (display only)


class BackendError(Exception):
    """Model/backend load or inference setup failure (pilot → degraded boot)."""


def rle_encode(mask_rows) -> bytes:
    """bool 2-D array-like -> RLE bytes (varint counts, leading ZEROS first)."""
    flat = [bool(v) for row in mask_rows for v in row]
    out = bytearray()
    cur, n = False, 0
    for v in flat:
        if v == cur:
            n += 1
        else:
            while True:
                b = n & 0x7F
                n >>= 7
                out.append(b | (0x80 if n else 0))
                if not n:
                    break
            cur, n = v, 1
    while True:                          # flush the final run
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            break
    return bytes(out)


def rle_decode(data: bytes, width: int, height: int) -> list[list[bool]]:
    """RLE bytes -> bool rows (width x height)."""
    counts = []
    shift = 0
    n = 0
    for b in data:
        n |= (b & 0x7F) << shift
        if not b & 0x80:
            counts.append(n)
            n, shift = 0, 0
        else:
            shift += 7
    rows: list[list[bool]] = []
    row: list[bool] = []
    cur = False
    for c in counts:
        for _ in range(c):
            if len(row) == width:
                rows.append(row)
                row = []
            row.append(cur)
        cur = not cur
    if row:
        rows.append(row)
    return rows
