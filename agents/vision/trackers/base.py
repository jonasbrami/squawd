"""vision/trackers/base.py — the designated-target pursuit protocol (ICD §6.8).

Trackers serve the ONE designated contact, on the Detector thread. They never
rename contacts and their conf is display-only (never fed to the EKF).
AssociationHit / TrackingMode live in vision/types.py (no cycles: trackers
import types, never the reverse).
"""
from typing import Protocol

from agents.core.contact import Frame  # noqa: F401
from agents.vision.types import AssociationHit, Detection  # noqa: F401


class TargetTracker(Protocol):
    """A pluggable pursuit algorithm for the DESIGNATED contact."""

    name: str
    needs_track_ids: bool         # True ⇒ Detector must run track mode
    tracker_yaml: str | None      # ultralytics tracker config, DNN family only

    def lock(self, frame: Frame, dets: list, *,
             seed_xy: tuple | None = None,
             seed_index: int | None = None) -> AssociationHit | None:
        """DNN impls MUST return None when the seed detection's tid is None
        (stale/pre-switch frame — the lock-time race, Fable-B2)."""
        ...

    def update(self, frame: Frame, dets: list) -> AssociationHit | None: ...

    def mask(self) -> bytes | None: ...

    def reset(self) -> None: ...
