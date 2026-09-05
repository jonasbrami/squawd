"""DroneAgent: one autonomous drone-agent as a single object.

Bundles everything a drone owns: its MAVSDK link, its command inbox
(/swarm/cmd/drone_<i>), its report outbox (/swarm/report/drone_<i>), its onboard
Claude client (flight tools via flight.make_pilot_options over its own
FlightOps), and its react loop. It talks to the Commander ONLY over ROS topics,
so a drone can run on separate hardware unchanged. Commander-driven: it acts
when tasked, holds the last command between tasks, and reports back when done.
"""
import asyncio

from std_msgs.msg import String
from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition

from agents.core.bus import CHAT_QOS, publish_str
from agents.core.store import TopicLog
from agents.flight import FlightOps, make_pilot_options
from agents.flight.backend import BackendClient


class DroneAgent:
    """drone_<i>: its MAVSDK System + cmd channel + onboard Claude client + loop."""

    def __init__(self, i: int, world, bridge, n: int, cameras, env=None, model=None) -> None:
        self.i = i
        self.name = f"drone_{i}"
        self._bridge = bridge
        # Own the MAVSDK link (created, not yet connected). Keep the server address:
        # it controls how the SDK reaches its mavsdk_server.
        self._system = System(mavsdk_server_address="127.0.0.1", port=50051 + i)
        bridge.subscribe(f"/px4_{i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
        # Inbox: the Commander dispatches directed tasks here (topic tokens can't
        # start with a digit, so drone_<i>, not <i>).
        self._cmd = TopicLog(bridge, f"/swarm/cmd/drone_{i}", String, CHAT_QOS)
        ops = FlightOps(self._system, world, bridge, i, n)
        self.client = BackendClient(make_pilot_options(
            ops, report=self.report, env=env, model=model))
        self._seen = 0

    def report(self, message: str) -> None:
        """Send a result back to the Commander (+ mirror onto /swarm/chat for the UI)."""
        publish_str(self._bridge, f"/swarm/report/drone_{self.i}", message)
        publish_str(self._bridge, "/swarm/chat", f"drone_{self.i}: {message}")

    async def connect(self) -> None:
        """Connect the MAVSDK link and arm PX4's own geofence as the hard safety layer.

        Warning action (GF_ACTION=1) so it never disrupts the demo; raise it later to
        actually contain drones."""
        await self._system.connect()
        async for s in self._system.core.connection_state():
            if s.is_connected:
                break
        try:
            await self._system.param.set_param_float("GF_MAX_HOR_DIST", 300.0)
            await self._system.param.set_param_float("GF_MAX_VER_DIST", 80.0)
            await self._system.param.set_param_int("GF_ACTION", 1)
        except Exception as e:
            print(f"geofence setup skipped for drone_{self.i}: {e}", flush=True)
        print(f"drone_{self.i} connected", flush=True)

    async def run(self) -> None:
        """Act only when the Commander tasks THIS drone on /swarm/cmd/drone_<i>."""
        async with self.client:
            while True:
                await asyncio.sleep(1.0)
                new, self._seen = self._cmd.since(self._seen)
                for task in new:
                    async for _ev in self.client.query(
                            f"Task from commander: {task}\n\nCarry it out with "
                            "your tools, then call report(...) with a short "
                            "result (what you did and what you saw)."):
                        pass
