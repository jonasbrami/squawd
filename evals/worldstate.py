"""WorldTrack: the ROS-free ground-truth record the oracle grades.

A run's sampler (evals.sampler) appends a Snapshot per poll; the oracle
(evals.oracle) reads the finished track. Pure dataclasses + math so it imports
and unit-tests without rclpy/mavsdk (same discipline as agents.core.store)."""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class DronePose:
    e: float
    n: float
    alt: float
    heading: float


@dataclass(frozen=True)
class Snapshot:
    t: float
    poses: dict[int, "DronePose"]
    # scripted-mover positions captured in the SAME tick as the drone poses,
    # so dynamic checks are within-snapshot geometry (no time-base drift)
    movers: dict[str, tuple[float, float, float]] = field(default_factory=dict)


@dataclass
class WorldTrack:
    snapshots: list[Snapshot]
    objects: dict[str, tuple[float, float]]
    geofence_m: float
    buildings: list[dict] = field(default_factory=list)

    def min_dist_to(self, xy: tuple[float, float]) -> float:
        best = math.inf
        for s in self.snapshots:
            for p in s.poses.values():
                best = min(best, math.hypot(p.e - xy[0], p.n - xy[1]))
        return best

    def max_dist_from_origin(self) -> float:
        best = 0.0
        for s in self.snapshots:
            for p in s.poses.values():
                best = max(best, math.hypot(p.e, p.n))
        return best

    def positions(self) -> list[tuple[float, float]]:
        return [(p.e, p.n) for s in self.snapshots for p in s.poses.values()]
