"""M1b face helper (in-container): PURE yaw-in-place — mavsdk goto_location
at the CURRENT position/altitude with a new heading (the FlightOps.face
pattern, minus ROS so it runs on /opt/venv python). Never translates.

  PYTHONPATH=/workspace /opt/venv/bin/python evals/out/deep_m1b/face.py 165
"""
import asyncio
import sys

from mavsdk import System

COMPASS = {"north": 0.0, "n": 0.0, "northeast": 45.0, "ne": 45.0, "east": 90.0,
           "e": 90.0, "southeast": 135.0, "se": 135.0, "south": 180.0,
           "s": 180.0, "southwest": 225.0, "sw": 225.0, "west": 270.0,
           "w": 270.0, "northwest": 315.0, "nw": 315.0}


async def main() -> None:
    word = sys.argv[1].strip().lower()
    yaw = COMPASS.get(word) if word in COMPASS else float(word)
    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    pos = await anext(drone.telemetry.position())
    await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg,
                                     pos.absolute_altitude_m, yaw)
    stream = drone.telemetry.heading()
    cur = None
    for _ in range(100):                     # settle ≤6 deg, 10 s cap (O5)
        await asyncio.sleep(0.1)
        try:
            cur = await asyncio.wait_for(anext(stream), 1.0)
            if abs((yaw - cur.heading_deg + 180.0) % 360.0 - 180.0) <= 6.0:
                print(f"facing {yaw:.0f} (heading {cur.heading_deg:.0f})",
                      flush=True)
                return
        except Exception:
            break
    print(f"still turning (heading {cur.heading_deg if cur else '?'}); "
          f"target {yaw:.0f}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
