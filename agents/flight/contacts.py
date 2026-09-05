"""flight/contacts.py — the decoupling seam (ICD §5.1). Protocols ONLY.

GzPoses (ground truth, evals) and VisionContacts (vision-fed, production)
both satisfy ContactProvider structurally; VisionContacts alone implements
TargetDesignator. Extended reads (ranges/health/observation/all_views) are
OPTIONAL and consumed via getattr with defaults ({}, 'MEASURED', None).
"""
from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@runtime_checkable
class ContactProvider(Protocol):
    """Minimum read contract (ICD §5.1)."""
    def poses(self) -> dict[str, tuple[float, float, float]]: ...
    def sim_time(self) -> float: ...
    def velocities(self) -> dict: ...


class TargetDesignator(Protocol):
    """Implemented by VisionContacts only — the SOLE owner of range sampling,
    beam association and fusion (§6.4). GzPoses does not implement it; the
    getattr guard makes designation a no-op with ground truth."""
    def designate(self, name: str, *, support_z: float | None = None,
                  context: "TrackingContext | None" = None) -> None: ...
    def clear_designation(self) -> None: ...


@dataclass(frozen=True)
class TrackingContext:
    mode: str                    # "shadow" | "intercept"
    commanded_speed: float
    own_alt: float
    task_ceiling_m: float | None
