# src/dronebot/control/controller.py
"""Thin async wrapper over MAVSDK actions. Fire-and-monitor: each method
issues the command with a timeout and returns; it does NOT block until the
maneuver completes. This is the code that transfers to real PX4 hardware.
"""
from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from mavsdk import System

from dronebot.control.geo import GeoPoint

_ACTION_TIMEOUT_S = 10.0


class ControllerError(Exception):
    """A MAVSDK action failed or timed out."""


class DroneController:
    def __init__(self, drone: "System") -> None:
        self._drone = drone

    async def connect(self, system_address: str, timeout_s: float = 90.0) -> None:
        # 90s default: a cold container boots PX4 SITL + Gazebo before the
        # vehicle heartbeats; 30s was too tight for first-run startup.
        await self._drone.connect(system_address=system_address)

        async def _await_connected() -> None:
            async for state in self._drone.core.connection_state():
                if state.is_connected:
                    return

        try:
            await asyncio.wait_for(_await_connected(), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise ControllerError(
                f"connection to {system_address} timed out after {timeout_s}s"
            ) from exc

    async def _with_timeout(self, coro, what: str):
        try:
            return await asyncio.wait_for(coro, timeout=_ACTION_TIMEOUT_S)
        except asyncio.TimeoutError as exc:
            raise ControllerError(f"{what} timed out after {_ACTION_TIMEOUT_S}s") from exc
        except Exception as exc:  # MAVSDK ActionError etc.
            raise ControllerError(f"{what} failed: {exc}") from exc

    async def arm(self) -> None:
        await self._with_timeout(self._drone.action.arm(), "arm")

    async def disarm(self) -> None:
        await self._with_timeout(self._drone.action.disarm(), "disarm")

    async def takeoff(self, altitude_m: float) -> None:
        await self._with_timeout(
            self._drone.action.set_takeoff_altitude(altitude_m), "set takeoff altitude"
        )
        await self._with_timeout(self._drone.action.takeoff(), "takeoff")

    async def land(self) -> None:
        await self._with_timeout(self._drone.action.land(), "land")

    async def return_to_launch(self) -> None:
        await self._with_timeout(self._drone.action.return_to_launch(), "return to launch")

    async def hold(self) -> None:
        await self._with_timeout(self._drone.action.hold(), "hold")

    async def goto(self, target: GeoPoint, yaw_deg: float = float("nan")) -> None:
        await self._with_timeout(
            self._drone.action.goto_location(
                target.latitude_deg,
                target.longitude_deg,
                target.absolute_altitude_m,
                yaw_deg,
            ),
            "goto",
        )
