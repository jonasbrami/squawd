# src/dronebot/control/executor.py
"""Portable command boundary. SDK-agnostic. Combines controller + safety +
state into typed commands returning structured results. Any agent (Claude,
other LLM, scripted) targets this interface.
"""
from __future__ import annotations

from dataclasses import dataclass

from dronebot.control.controller import ControllerError
from dronebot.control.geo import GeoPoint, horizontal_distance_m, offset_point
from dronebot.control.safety import DroneSnapshot, SafetyError, SafetyGuard
from dronebot.control.state import StateStore

# How close counts as "arrived" when clearing the positioning interlock.
_REACHED_HORIZONTAL_M = 2.0
_REACHED_TAKEOFF_FRACTION = 0.9


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    message: str


class CommandExecutor:
    def __init__(self, controller, state: StateStore, guard: SafetyGuard) -> None:
        self._c = controller
        self._state = state
        self._guard = guard
        # Positioning interlock (droneserver lesson, spec D-log): at most one
        # positioning maneuver in flight at a time. (kind, target) where target
        # is the takeoff altitude (float), the goto GeoPoint, or None (RTL).
        self._active: tuple[str, object] | None = None

    async def _run(self, check, action, success_msg: str) -> CommandResult:
        try:
            if check is not None:
                check()
            await action()
            return CommandResult(True, success_msg)
        except SafetyError as exc:
            return CommandResult(False, f"refused: {exc}")
        except ControllerError as exc:
            return CommandResult(False, f"command failed: {exc}")

    def _reached(self, snap: DroneSnapshot) -> bool:
        """Has the active positioning maneuver completed (per telemetry)?"""
        if self._active is None:
            return True
        if snap.position is None or snap.home is None:
            return False  # can't confirm -> stay busy (fail safe)
        kind, target = self._active
        if kind == "takeoff":
            rel_alt = snap.position.absolute_altitude_m - snap.home.absolute_altitude_m
            return snap.in_air and rel_alt >= _REACHED_TAKEOFF_FRACTION * float(target)
        if kind == "goto":
            assert isinstance(target, GeoPoint)
            return horizontal_distance_m(snap.position, target) <= _REACHED_HORIZONTAL_M
        if kind == "rtl":
            return not snap.in_air  # RTL ends by landing
        return True

    def _interlock(self, snap: DroneSnapshot) -> CommandResult | None:
        """Refuse a new positioning command while one is still in flight.
        Re-checks completion each call, so the flag never gets stuck."""
        if self._active is None:
            return None
        if self._reached(snap):
            self._active = None
            return None
        return CommandResult(
            False,
            f"refused: still executing {self._active[0]}; say 'stop' to override",
        )

    async def arm(self) -> CommandResult:
        snap = self._state.snapshot()
        return await self._run(
            lambda: self._guard.check_arm(snap), self._c.arm, "armed"
        )

    async def takeoff(self, altitude_m: float) -> CommandResult:
        snap = self._state.snapshot()
        blocked = self._interlock(snap)
        if blocked is not None:
            return blocked
        result = await self._run(
            lambda: self._guard.check_takeoff(altitude_m, snap),
            lambda: self._c.takeoff(altitude_m),
            f"taking off to {altitude_m:.0f}m (climbing)",
        )
        if result.ok:
            self._active = ("takeoff", altitude_m)
        return result

    async def land(self) -> CommandResult:
        self._active = None  # terminator: clears the interlock
        return await self._run(None, self._c.land, "landing")

    async def return_to_launch(self) -> CommandResult:
        snap = self._state.snapshot()
        blocked = self._interlock(snap)
        if blocked is not None:
            return blocked
        result = await self._run(
            None, self._c.return_to_launch, "returning to launch"
        )
        if result.ok:
            self._active = ("rtl", None)
        return result

    async def hold(self) -> CommandResult:
        self._active = None  # terminator/override: clears the interlock
        return await self._run(None, self._c.hold, "holding position")

    async def goto_relative(self, north_m: float, east_m: float, up_m: float) -> CommandResult:
        snap = self._state.snapshot()
        blocked = self._interlock(snap)
        if blocked is not None:
            return blocked
        if snap.position is None:
            return CommandResult(False, "refused: no position fix")
        target = offset_point(snap.position, north_m, east_m, up_m)
        result = await self._run(
            lambda: self._guard.check_goto(target, snap),
            lambda: self._c.goto(target),
            f"moving N{north_m:.0f} E{east_m:.0f} U{up_m:.0f} (in progress)",
        )
        if result.ok:
            self._active = ("goto", target)
        return result

    def status(self) -> CommandResult:
        snap = self._state.snapshot()
        bat = self._state.battery_remaining
        bat_s = f"{bat * 100:.0f}%" if bat is not None else "unknown"
        return CommandResult(
            True,
            f"connected={snap.is_connected} armed={snap.is_armed} "
            f"in_air={snap.in_air} mode={snap.flight_mode} battery={bat_s}",
        )
