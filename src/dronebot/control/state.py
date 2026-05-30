# src/dronebot/control/state.py
"""Single authoritative drone state, fed by the telemetry task. The LLM is
never the source of truth; every safety check and status report reads here.
"""
from __future__ import annotations

from dronebot.control.geo import GeoPoint
from dronebot.control.safety import DroneSnapshot


class StateStore:
    def __init__(self) -> None:
        self._connected = False
        self._armed = False
        self._in_air = False
        self._flight_mode = "UNKNOWN"
        self._home: GeoPoint | None = None
        self._position: GeoPoint | None = None
        self.battery_remaining: float | None = None
        self.last_telemetry_ts: float | None = None

    def set_connection(self, value: bool) -> None:
        self._connected = value

    def set_armed(self, value: bool) -> None:
        self._armed = value

    def set_in_air(self, value: bool) -> None:
        self._in_air = value

    def set_flight_mode(self, mode: str) -> None:
        self._flight_mode = mode

    def set_home(self, point: GeoPoint) -> None:
        self._home = point

    def set_position(self, point: GeoPoint) -> None:
        self._position = point
        # Capture home from the first known fix (the software sim can take
        # minutes to converge, so we don't block startup waiting for it).
        if self._home is None:
            self._home = point

    def set_battery(self, remaining: float) -> None:
        self.battery_remaining = remaining

    def mark_telemetry_seen(self, timestamp: float) -> None:
        self.last_telemetry_ts = timestamp

    @property
    def flight_mode(self) -> str:
        return self._flight_mode

    @property
    def position(self) -> GeoPoint | None:
        return self._position

    @property
    def home(self) -> GeoPoint | None:
        return self._home

    def snapshot(self) -> DroneSnapshot:
        return DroneSnapshot(
            is_connected=self._connected,
            is_armed=self._armed,
            in_air=self._in_air,
            has_position=self._position is not None,
            flight_mode=self._flight_mode,
            home=self._home,
            position=self._position,
        )
