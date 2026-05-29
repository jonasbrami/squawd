# src/dronebot/control/safety.py
"""Non-bypassable safety supervisor. Pure, LLM-agnostic, unit-tested.
The prompt is NEVER a safety boundary; these invariants are.
"""
from __future__ import annotations

from dataclasses import dataclass

from .geo import GeoPoint, horizontal_distance_m

# Flight modes where the autopilot is in control and new goto commands
# must be refused.
_AUTOPILOT_CONTROLLED = {"RETURN_TO_LAUNCH", "LAND"}


@dataclass(frozen=True)
class SafetyLimits:
    max_altitude_m: float          # max altitude above home
    geofence_radius_m: float       # max horizontal distance from home
    max_goto_distance_m: float     # max distance for a single goto
    min_takeoff_altitude_m: float
    max_takeoff_altitude_m: float


@dataclass(frozen=True)
class DroneSnapshot:
    is_connected: bool
    is_armed: bool
    in_air: bool
    has_position: bool
    flight_mode: str
    home: GeoPoint | None
    position: GeoPoint | None


class SafetyError(Exception):
    """Raised when a command would violate a safety invariant."""


class SafetyGuard:
    def __init__(self, limits: SafetyLimits) -> None:
        self._limits = limits

    def check_arm(self, snap: DroneSnapshot) -> None:
        if not snap.is_connected:
            raise SafetyError("cannot arm: not connected to the vehicle")

    def check_takeoff(self, altitude_m: float, snap: DroneSnapshot) -> None:
        if not snap.is_connected:
            raise SafetyError("cannot take off: not connected")
        if not snap.is_armed:
            raise SafetyError("cannot take off: not armed")
        if snap.in_air:
            raise SafetyError("cannot take off: already in the air")
        if not snap.has_position:
            raise SafetyError("cannot take off: no position fix")
        if altitude_m < self._limits.min_takeoff_altitude_m:
            raise SafetyError(
                f"takeoff altitude {altitude_m}m below minimum "
                f"{self._limits.min_takeoff_altitude_m}m"
            )
        if altitude_m > self._limits.max_takeoff_altitude_m:
            raise SafetyError(
                f"takeoff altitude {altitude_m}m above maximum "
                f"{self._limits.max_takeoff_altitude_m}m"
            )
        if altitude_m > self._limits.max_altitude_m:
            raise SafetyError(
                f"takeoff altitude {altitude_m}m above altitude cap "
                f"{self._limits.max_altitude_m}m"
            )

    def check_goto(self, target: GeoPoint, snap: DroneSnapshot) -> None:
        if not snap.is_connected:
            raise SafetyError("cannot move: not connected")
        if not snap.in_air:
            raise SafetyError("cannot move: not in the air")
        if not snap.has_position or snap.position is None or snap.home is None:
            raise SafetyError("cannot move: no position fix")
        if snap.flight_mode in _AUTOPILOT_CONTROLLED:
            raise SafetyError(
                f"cannot move: autopilot is in control ({snap.flight_mode})"
            )

        dist_from_home = horizontal_distance_m(snap.home, target)
        if dist_from_home > self._limits.geofence_radius_m:
            raise SafetyError(
                f"target {dist_from_home:.0f}m from home exceeds geofence "
                f"radius {self._limits.geofence_radius_m}m"
            )

        dist_from_here = horizontal_distance_m(snap.position, target)
        if dist_from_here > self._limits.max_goto_distance_m:
            raise SafetyError(
                f"move of {dist_from_here:.0f}m exceeds per-command limit "
                f"{self._limits.max_goto_distance_m}m"
            )

        alt_above_home = target.absolute_altitude_m - snap.home.absolute_altitude_m
        if alt_above_home > self._limits.max_altitude_m:
            raise SafetyError(
                f"target altitude {alt_above_home:.0f}m above home exceeds cap "
                f"{self._limits.max_altitude_m}m"
            )
