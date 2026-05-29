# src/dronebot/control/telemetry.py
"""Background tasks draining MAVSDK telemetry streams into the StateStore.
Each stream is its own coroutine; run them with asyncio.gather in app.py.
No blocking work here — these only update the store.
"""
from __future__ import annotations

import asyncio
import time
from typing import AsyncIterator

from dronebot.control.geo import GeoPoint
from dronebot.control.state import StateStore


async def update_position_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for position in stream:
        store.set_position(
            GeoPoint(
                latitude_deg=position.latitude_deg,
                longitude_deg=position.longitude_deg,
                absolute_altitude_m=position.absolute_altitude_m,
            )
        )
        store.mark_telemetry_seen(time.monotonic())


async def update_flight_mode_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for mode in stream:
        store.set_flight_mode(str(mode))


async def update_armed_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for armed in stream:
        store.set_armed(bool(armed))


async def update_in_air_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for in_air in stream:
        store.set_in_air(bool(in_air))


async def update_battery_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for battery in stream:
        store.set_battery(float(battery.remaining_percent))


def start_telemetry(drone, store: StateStore) -> list[asyncio.Task]:
    """Spawn one task per telemetry stream. Returns the tasks so the caller
    can cancel them on shutdown."""
    return [
        asyncio.create_task(update_position_stream(drone.telemetry.position(), store)),
        asyncio.create_task(update_flight_mode_stream(drone.telemetry.flight_mode(), store)),
        asyncio.create_task(update_armed_stream(drone.telemetry.armed(), store)),
        asyncio.create_task(update_in_air_stream(drone.telemetry.in_air(), store)),
        asyncio.create_task(update_battery_stream(drone.telemetry.battery(), store)),
    ]
