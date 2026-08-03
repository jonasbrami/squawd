"""W3-run3 reposition (LLM-free): goto-only — the drone is ALREADY airborne
(the w3_position.py takeoff would reject); relative fly via the repo's
FlightOps over the second mavsdk System, pilot stays idle.

Run INSIDE the container:
  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w3_run3/w3_reposition.py <alt> <east> <north> <heading>
"""
import asyncio
import os
import sys

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.gzposes import GzPoses
from agents.flight.ops import FlightOps
from agents.world import World

MOVERS = ["car_1", "car_2", "car_3", "walker_1", "walker_2"]


async def main() -> None:
    alt = float(sys.argv[1])
    t_e, t_n = float(sys.argv[2]), float(sys.argv[3])
    heading = sys.argv[4] if len(sys.argv) > 4 else "south"

    bridge = RosBridge(node_name="w3_reposition")
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
    await asyncio.sleep(2.0)

    print(f"[w3] reposition -> E{t_e} N{t_n} alt {alt} facing {heading}",
          flush=True)
    print(f"[w3] {await ops.goto(east=t_e, north=t_n, up=alt, heading=heading)}",
          flush=True)
    me = world.world_xy(bridge, 0)
    print(f"[w3] drone at {me}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
