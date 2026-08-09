"""M3 staging helper (in-container): take off to ALT if on the ground, then
PURE yaw-in-place to HEADING (the deep_m1b face.py pattern — mavsdk
goto_location at the CURRENT position; never translates).

  PYTHONPATH=/workspace /opt/venv/bin/python evals/out/deep_m3/stage.py 16 180
"""
import asyncio
import sys

from mavsdk import System

COMPASS = {"north": 0.0, "n": 0.0, "northeast": 45.0, "ne": 45.0,
           "east": 90.0, "e": 90.0, "southeast": 135.0, "se": 135.0,
           "south": 180.0, "s": 180.0, "southwest": 225.0, "sw": 225.0,
           "west": 270.0, "w": 270.0, "northwest": 315.0, "nw": 315.0}


async def main() -> None:
    alt = float(sys.argv[1]) if len(sys.argv) > 1 else 16.0
    word = sys.argv[2].strip().lower() if len(sys.argv) > 2 else "south"
    yaw = COMPASS.get(word) if word in COMPASS else float(word)
    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    async for air in drone.telemetry.in_air():
        in_air = air
        break
    if not in_air:
        print(f"taking off to {alt:.0f} m…", flush=True)
        await drone.action.set_takeoff_altitude(alt)
        await drone.action.takeoff()
        for _ in range(300):                 # settle near target alt, ≤60 s
            await asyncio.sleep(0.2)
            pos = await anext(drone.telemetry.position())
            if abs(pos.relative_altitude_m - alt) < 0.8:
                break
        print(f"airborne at {pos.relative_altitude_m:.1f} m", flush=True)
    pos = await anext(drone.telemetry.position())
    await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg,
                                     pos.absolute_altitude_m, yaw)
    stream = drone.telemetry.heading()
    cur = None
    for _ in range(100):                     # settle ≤6 deg, 10 s cap
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
