"""WorldTrack: the ROS-free ground-truth record the oracle grades.

A run's sampler (evals.sampler) appends a Snapshot per poll; the oracle
(evals.oracle) reads the finished track. Pure dataclasses + math so it imports
and unit-tests without rclpy/mavsdk (same discipline as agents.core.store)."""
import math
from dataclasses import dataclass


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


@dataclass
class WorldTrack:
    snapshots: list[Snapshot]
    objects: dict[str, tuple[float, float]]
    n_drones: int
    geofence_m: float

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
