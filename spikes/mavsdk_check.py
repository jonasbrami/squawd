# spikes/mavsdk_check.py
"""Cheap check: connect to PX4 SITL and read one telemetry value.
Requires the sim running (sim/launch/one_drone.sh). No API key needed.
"""
import asyncio
from mavsdk import System


async def main() -> None:
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected")
            break

    async for position in drone.telemetry.position():
        print(
            f"lat={position.latitude_deg:.6f} "
            f"lon={position.longitude_deg:.6f} "
            f"rel_alt={position.relative_altitude_m:.2f}m"
        )
        break

    print("SPIKE A PASS: MAVSDK <-> SITL works on this loop")


if __name__ == "__main__":
    asyncio.run(main())
