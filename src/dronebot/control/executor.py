# src/dronebot/control/executor.py
"""Portable command boundary. SDK-agnostic. Combines controller + safety +
state into typed commands returning structured results. Any agent (Claude,
other LLM, scripted) targets this interface.
"""
from __future__ import annotations

from dataclasses import dataclass

from dronebot.control.controller import ControllerError
from dronebot.control.geo import GeoPoint, offset_point
from dronebot.control.safety import SafetyError, SafetyGuard
from dronebot.control.state import StateStore


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    message: str


class CommandExecutor:
    def __init__(self, controller, state: StateStore, guard: SafetyGuard) -> None:
        self._c = controller
        self._state = state
        self._guard = guard

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

    async def arm(self) -> CommandResult:
        snap = self._state.snapshot()
        return await self._run(
            lambda: self._guard.check_arm(snap), self._c.arm, "armed"
        )

    async def takeoff(self, altitude_m: float) -> CommandResult:
        snap = self._state.snapshot()
        return await self._run(
            lambda: self._guard.check_takeoff(altitude_m, snap),
            lambda: self._c.takeoff(altitude_m),
            f"taking off to {altitude_m:.0f}m (climbing)",
        )

    async def land(self) -> CommandResult:
        return await self._run(None, self._c.land, "landing")

    async def return_to_launch(self) -> CommandResult:
        return await self._run(None, self._c.return_to_launch, "returning to launch")

    async def hold(self) -> CommandResult:
        return await self._run(None, self._c.hold, "holding position")

    async def goto_relative(self, north_m: float, east_m: float, up_m: float) -> CommandResult:
        snap = self._state.snapshot()
        if snap.position is None:
            return CommandResult(False, "refused: no position fix")
        target = offset_point(snap.position, north_m, east_m, up_m)
        return await self._run(
            lambda: self._guard.check_goto(target, snap),
            lambda: self._c.goto(target),
            f"moving N{north_m:.0f} E{east_m:.0f} U{up_m:.0f} (in progress)",
        )

    def status(self) -> CommandResult:
        snap = self._state.snapshot()
        bat = self._state.battery_remaining
        bat_s = f"{bat * 100:.0f}%" if bat is not None else "unknown"
        return CommandResult(
            True,
            f"connected={snap.is_connected} armed={snap.is_armed} "
            f"in_air={snap.in_air} mode={snap.flight_mode} battery={bat_s}",
        )
