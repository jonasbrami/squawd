"""W3-integration positioning (LLM-free): takeoff + stand-off hover.

Uses the repo's own flight machinery (agents.flight.ops.FlightOps over a
second mavsdk System connection at 127.0.0.1:50051 — the same server the
pilot agent rides; the pilot stays idle, no /pilot/user_input is ever sent)
to take the drone to ALT and park it south of the car_1 demo loop facing
north, so the mover transits the camera's FOV at ~25-45 m slant.

Run INSIDE the container:
  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/w3_integration/w3_position.py [alt] [east] [north]
"""
import asyncio
import math
import os
import sys

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.gzposes import GzPoses
from agents.flight.ops import FlightOps
from agents.world import World

MOVERS = ["car_1", "car_2", "car_3", "walker_1", "walker_2"]


async def main() -> None:
    alt = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
    t_e = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    t_n = float(sys.argv[3]) if len(sys.argv) > 3 else -60.0

    bridge = RosBridge(node_name="w3_position")
    # world.world_xy / _await_arrival read the px4 fix off the bridge — it
    # only flows if we subscribe FIRST (the pilot does this in PilotAgent).
    from px4_msgs.msg import VehicleLocalPosition
    bridge.subscribe("/px4_0/fmu/out/vehicle_local_position",
                     VehicleLocalPosition)
    world = World()
    gz = GzPoses(os.environ.get("GZ_WORLD", "demo"), MOVERS)
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break
    ops = FlightOps(system, world, bridge, 0, 1, contacts=gz)
    bridge.start()
    await asyncio.sleep(2.0)          # let px4 + gz pose feeds land

    print(f"[w3] takeoff to {alt:.0f} m …", flush=True)
    print(f"[w3] {await ops.take_off(alt)}", flush=True)
    heading = sys.argv[4] if len(sys.argv) > 4 else "north"
    if heading == "face":
        # drift-safe: hold the telemetry position, yaw only (a goto whose
        # absolute-altitude target derives from a drifted z fix can fly the
        # drone into the ground — observed live 2026-08-01).
        print(f"[w3] {await ops.face('north')}", flush=True)
    elif len(sys.argv) > 2:
        print(f"[w3] positioning E{t_e:.0f} N{t_n:.0f} facing {heading} …",
              flush=True)
        print(f"[w3] {await ops.goto(east=t_e, north=t_n, up=alt, heading=heading)}",
              flush=True)
    else:                            # no xy: takeoff in place, yaw only
        print(f"[w3] {await ops.face(heading)}", flush=True)

    me = world.world_xy(bridge, 0)
    print(f"[w3] drone at {me}", flush=True)
    for name, p in sorted(gz.poses().items()):
        d = math.hypot(p[0] - me[0], p[1] - me[1]) if me else -1
        print(f"[w3] mover {name}: E{p[0]:.1f} N{p[1]:.1f} — {d:.1f} m away",
              flush=True)
    print("[w3] positioning complete — holding.", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
