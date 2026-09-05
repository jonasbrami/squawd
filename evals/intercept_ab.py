"""intercept_ab — the M3a velocity-direct (O3) A/B: intercept convergence
   with the provider's own velocity channel (feed_direct) vs the EMA
   finite-difference fallback. Truth-fed (GzPoses) so the ONLY difference is
   the velocity path. Measures wall-clock time to close within 15 m.

  docker exec pilot-sim bash -lc 'uv run --no-project python evals/intercept_ab.py'
"""
import asyncio
import math
import os
import sys
import time

sys.path.insert(0, "/workspace")

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.telemetry import Px4StateRecorder
from agents.world import World
from agents.flight.ops import FlightOps

TARGET = "mov_1"


class OracleVel:
    """GzPoses + a velocities() channel (finite-difference oracle velocity —
    stands in for VisionContacts' EKF velocity: the O3 feed_direct path)."""

    def __init__(self, gz):
        self._gz = gz
        self._last = None
        self._vels = {}
        self.config = None

    def poses(self):
        p = self._gz.poses()
        t = self._gz.sim_time()
        for name, pos in p.items():
            if self._last and name in self._last[1]:
                dt = t - self._last[0]
                if dt > 1e-3:
                    lp = self._last[1][name]
                    self._vels[name] = ((pos[0] - lp[0]) / dt,
                                        (pos[1] - lp[1]) / dt)
        self._last = (t, p)
        return p

    def sim_time(self):
        return self._gz.sim_time()

    def velocities(self):
        return dict(self._vels)


async def main() -> int:
    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    gz = GzPoses(os.environ.get("GZ_WORLD", "dynamic"), [TARGET])
    rec = Px4StateRecorder(bridge, world, i=0, sim_time_ref=gz.sim_time)
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    ops = FlightOps(system, world, bridge, 0, 1, contacts=gz)
    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break
    bridge.start()
    rec.start()
    for _ in range(150):
        if gz.poses().get(TARGET):
            break
        await asyncio.sleep(0.2)

    async def one_run(with_vel: bool) -> float:
        ops.contacts = OracleVel(gz) if with_vel else gz
        if with_vel:
            for _ in range(20):                # let the oracle velocity settle
                ops.contacts.poses()
                await asyncio.sleep(0.1)
        print(await ops.take_off(6.0), flush=True)
        t0 = time.monotonic()
        result = await ops.track(TARGET, mode="intercept", alt=6.0,
                                 duration_s=60.0, within_m=15.0, speed=8.0)
        dt = time.monotonic() - t0
        print(("VEL" if with_vel else "EMA"), "->", result, flush=True)
        print(await ops.land(), flush=True)
        await asyncio.sleep(4.0)
        sys.stdout.flush()
        return dt if "INTERCEPTED" in result else float("inf")

    for label, vel in (("EMA", False), ("VEL", True)):
        dt = await one_run(vel)
        print(f"{label}: convergence "
              f"{'%.1fs' % dt if dt != float('inf') else 'NONE in 60s'}",
              flush=True)
    os._exit(0)


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
