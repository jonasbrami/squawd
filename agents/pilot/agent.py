"""PilotAgent: the one drone agent (design §3.6, ICD §7.1).

Owns: the MAVSDK link (injected, shared), THE one FlightOps (injected — the
same instance the tools and the estop arbiter use), its user inbox
(/pilot/user_input), its report outbox (/pilot/chat), its Claude client
(make_pilot_options), the react loop, and the estop supervisor task. W3a:
also owns THE CommandArbiter — UI ops from /pilot/cmd take the operator
lease through it, and every LLM tool call passes its guard_llm first.
"""
import asyncio

from std_msgs.msg import String
from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition

from agents.core.bus import CMD_QOS, publish_str
from agents.core.store import TopicLog
from agents.flight.backend import make_backend_client
from agents.pilot.arbiter import CommandArbiter
from agents.pilot.cmd import cmd_supervisor, make_run_op
from agents.pilot.estop import ActiveToolRegistry, estop_supervisor


class PilotAgent:
    """drone_0: shared System + shared FlightOps + inbox + Claude client + loop."""

    def __init__(self, system: System, ops, bridge, env=None, model=None,
                 cli_path=None, detect_text=None, deep_tools=None,
                 backend=None, codex_effort=None) -> None:
        self._system = system
        self._ops = ops
        self._bridge = bridge
        self.registry = ActiveToolRegistry()
        self.arbiter = CommandArbiter(self.registry, ops, make_run_op(ops))
        bridge.subscribe("/px4_0/fmu/out/vehicle_local_position",
                         VehicleLocalPosition)
        self._inbox = TopicLog(bridge, "/pilot/user_input", String, CMD_QOS)
        self.client = make_backend_client(
            ops, backend=backend, detect_text=detect_text, deep_tools=deep_tools,
            report=self.report,
            registry=self.registry, guard=self.arbiter.guard_llm,
            env=env, model=model, cli_path=cli_path,
            codex_effort=codex_effort)
        self._seen = 0

    def report(self, message: str) -> None:
        publish_str(self._bridge, "/pilot/chat", f"pilot: {message}")

    async def connect(self) -> None:
        """Connect MAVSDK and arm PX4's geofence FROM the Envelope instance
        (single source of truth; param-set failure is degraded, not fatal)."""
        await self._system.connect()
        async for s in self._system.core.connection_state():
            if s.is_connected:
                break
        env = self._ops.envelope
        try:
            if env is not None:
                await self._system.param.set_param_float(
                    "GF_MAX_HOR_DIST", float(env.geofence_radius_m))
                await self._system.param.set_param_float(
                    "GF_MAX_VER_DIST", float(env.geofence_alt_m))
            await self._system.param.set_param_int("GF_ACTION", 1)
        except Exception as e:
            print(f"geofence setup skipped for drone_0: {e}", flush=True)
        print("drone_0 connected", flush=True)

    async def run(self) -> None:
        self._seen = len(self._inbox.all())      # skip any latched backlog
        async with self.client:
            await asyncio.gather(
                self._loop(),
                estop_supervisor(self._bridge, self.registry, self._ops),
                cmd_supervisor(self._bridge, self.arbiter))

    async def _loop(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            new, self._seen = self._inbox.since(self._seen)
            for cmd in new:
                async for _ev in self.client.query(
                        f"Command from the operator: {cmd}\n\nCarry it out with "
                        "your tools, then call report(...) with a short result "
                        "(what you did and what you saw)."):
                    pass
