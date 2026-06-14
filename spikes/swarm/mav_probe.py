"""Bounded MAVSDK connection probe — diagnose whether mavsdk_server discovers PX4.

EVERY await is wrapped in asyncio.wait_for; all prints flush. Cannot hang.
"""
import asyncio

import mavsdk
from mavsdk import System


def log(m: str) -> None:
    print(m, flush=True)


async def main() -> None:
    log(f"mavsdk-python version: {getattr(mavsdk, '__version__', '?')}")
    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    try:
        await asyncio.wait_for(drone.connect(), timeout=10)
        log("connect() OK (gRPC to mavsdk_server@50051)")
    except asyncio.TimeoutError:
        log("RESULT: connect() to mavsdk_server@50051 TIMED OUT")
        return

    async def wait_conn():
        async for s in drone.core.connection_state():
            if s.is_connected:
                return True

    try:
        await asyncio.wait_for(wait_conn(), timeout=20)
        log("RESULT: VEHICLE DISCOVERED")
    except asyncio.TimeoutError:
        log("RESULT: NO VEHICLE in 20s (mavsdk_server got no MAVLink from PX4 on 14540)")
        return

    pos = await asyncio.wait_for(anext(drone.telemetry.position()), timeout=10)
    log(f"position: rel_alt={pos.relative_altitude_m:.2f}m lat={pos.latitude_deg:.5f}")


if __name__ == "__main__":
    asyncio.run(main())
