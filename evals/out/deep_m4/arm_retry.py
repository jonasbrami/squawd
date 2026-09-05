"""M4 arming retry loop (in-container): the fresh stack's EKF yaw check
oscillates (Preflight Fail: Yaw estimate error -> pass windows). Attempt
takeoff to 9 m every 20 s, up to 15 tries; stop at the first real climb
(alt > 6 m within 40 s of the command). Prints one line per attempt.

  PYTHONPATH=/workspace /opt/venv/bin/python evals/out/deep_m4/arm_retry.py
"""
import asyncio
import time

from mavsdk import System


async def main() -> None:
    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    for attempt in range(1, 16):
        try:
            await drone.action.set_takeoff_altitude(9.0)
            await drone.action.takeoff()
        except Exception as e:
            print(f"[attempt {attempt}] takeoff rejected: {type(e).__name__} "
                  f"{e}", flush=True)
            await asyncio.sleep(20.0)
            continue
        t0 = time.monotonic()
        climbed = False
        while time.monotonic() - t0 < 40.0:
            await asyncio.sleep(1.0)
            pos = await anext(drone.telemetry.position())
            if pos.relative_altitude_m > 6.0:
                climbed = True
                break
        print(f"[attempt {attempt}] commanded; alt="
              f"{pos.relative_altitude_m:.1f} climbed={climbed}", flush=True)
        if climbed:
            print("AIRBORNE", flush=True)
            return
        await asyncio.sleep(20.0)
    print("FAILED all attempts", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
