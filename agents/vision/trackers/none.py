"""vision/trackers/none.py — the no-op pursuit tracker (ICD §6.8).

`none` is the DEFAULT registry entry: no image-space tracking at all.
lock() resolves the seed against the current detections (seed_index first,
then the box containing/nearest seed_xy) and stores it, so the designation
frame's hit force-associates the seeded detection to the designated track;
update() then always returns None — after the seed frame the designated
contact rides VisionContacts' built-in world-space NN/NIS gate like every
other contact.
"""
from agents.core.contact import Frame  # noqa: F401
from agents.vision.types import AssociationHit, Detection  # noqa: F401


class NoOpTracker:
    """TargetTracker protocol (trackers/base.py) as a pure seed resolver."""

    needs_track_ids = False
    tracker_yaml = None

    def __init__(self, name: str = "none", device: str = "cpu") -> None:
        self.name = name
        self._seed = None                   # (seed_xy, seed_index) of lock()

    def lock(self, frame: Frame, dets: list, *,
             seed_xy: tuple | None = None,
             seed_index: int | None = None) -> AssociationHit | None:
        self._seed = (seed_xy, seed_index)
        k = self._resolve(dets, seed_xy, seed_index)
        if k is None:
            return None
        d = dets[k]
        return AssociationHit(k, d.xyxy, d.footpoint, d.conf, None)

    def update(self, frame: Frame, dets: list) -> AssociationHit | None:
        return None                         # world-space gate takes over

    def mask(self) -> bytes | None:
        return None

    def reset(self) -> None:
        self._seed = None

    @staticmethod
    def _resolve(dets: list, seed_xy: tuple | None,
                 seed_index: int | None) -> int | None:
        if seed_index is not None and 0 <= seed_index < len(dets):
            return seed_index
        if seed_xy is None or not dets:
            return None
        x, y = seed_xy
        inside = [k for k, d in enumerate(dets)
                  if d.xyxy[0] <= x <= d.xyxy[2] and d.xyxy[1] <= y <= d.xyxy[3]]
        cands = inside or list(range(len(dets)))
        return min(cands,
                   key=lambda k: (dets[k].cx - x) ** 2 + (dets[k].cy - y) ** 2)
