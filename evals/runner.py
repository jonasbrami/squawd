"""Run ONE eval cell (task x model x repeat) against a live sim, single-drone layer.

Builds a DroneAgent with the chosen model, soft-resets the world, starts the sampler,
injects the task prompt at the drone's own Claude client, and bounds the run by the
spec's wall-clock + step budget. Captures latency + tool-call trace, then grades the
sampled WorldTrack. Infra failures (no fix, RTL/connection errors) are flagged, not
scored as task failures."""
import asyncio
import time
from dataclasses import dataclass, field

from claude_agent_sdk import AssistantMessage, ToolUseBlock

from evals.oracle import grade
from evals.reset import soft_reset
from evals.sampler import Sampler

TIERS = {
    "opus": "claude-opus-4-8",
    "sonnet": "claude-sonnet-5",
    "haiku": "claude-haiku-4-5-20251001",
}


def model_for(assignment: dict, role: str) -> str | None:
    tier = assignment.get(role)
    return TIERS[tier] if tier else None


def assignment_label(assignment: dict) -> str:
    return ",".join(f"{k}={v}" for k, v in sorted(assignment.items())) or "default"


class Trace:
    def __init__(self) -> None:
        self.steps = 0
        self.first_action_t: float | None = None

    def observe(self, msg, now: float) -> None:
        if not isinstance(msg, AssistantMessage):
            return
        for blk in msg.content:
            if isinstance(blk, ToolUseBlock):
                self.steps += 1
                if self.first_action_t is None:
                    self.first_action_t = now


@dataclass
class CellResult:
    task_id: str
    assignment_label: str
    repeat: int
    passed: bool
    checks: list = field(default_factory=list)
    latency_s: float | None = None
    steps: int = 0
    infra_fail: bool = False
    failure_reason: str = ""

    def to_row(self) -> dict:
        return {
            "task_id": self.task_id,
            "assignment": self.assignment_label,
            "repeat": self.repeat,
            "passed": self.passed,
            "latency_s": round(self.latency_s, 2) if self.latency_s is not None else None,
            "steps": self.steps,
            "infra_fail": self.infra_fail,
            "failure_reason": self.failure_reason,
            "checks": [{"name": c.name, "passed": c.passed, "detail": c.detail}
                       for c in self.checks],
        }


@dataclass
class Deps:
    """Live handles shared across cells in one sweep (built once by run_evals)."""
    world: object
    bridge: object
    cameras: object


class DroneHarness:
    """Owns the persistent MAVSDK link + telemetry subscription for drone 0, built
    ONCE and reused across every cell — this is the fix for the per-cell subscription
    /System leak (building a DroneAgent per cell re-subscribed the non-idempotent
    RosBridge and spun a new System each time, growing linearly over a sweep).

    It hands out a FRESH Claude client per cell (bound to the shared System with the
    cell's model), because the SDK model is fixed at client construction AND each
    cell/repeat must run an independent agent session — a reused client would bleed
    one cell's conversation into the next and poison the accuracy numbers. So: cache
    the model-independent flight link, rebuild only the cheap per-cell client.

    The `agent_factory`/`client_builder` hooks exist so the caching lifecycle is
    unit-testable without ROS; production paths default to the real deferred imports."""

    def __init__(self, deps: Deps, agent_factory=None, client_builder=None) -> None:
        self._deps = deps
        self._agent_factory = agent_factory
        self._client_builder = client_builder
        self._agent = None  # cached DroneAgent; we use only its connected _system

    def _make_agent(self):
        if self._agent_factory is not None:
            return self._agent_factory()
        from agents.swarm.drone import DroneAgent  # deferred: rclpy/mavsdk at runtime only
        return DroneAgent(0, self._deps.world, self._deps.bridge, 1,
                          self._deps.cameras, model=None)

    async def _ensure(self):
        if self._agent is None:
            agent = self._make_agent()
            await agent.connect()  # connects the MAVSDK link + arms PX4 geofence, once
            self._agent = agent
        return self._agent

    async def system(self):
        """The shared, connected MAVSDK System for drone 0 (built + connected once)."""
        return (await self._ensure())._system

    def client_for(self, model):
        """A fresh ClaudeSDKClient bound to the shared System, with `model`. Caller
        drives it under `async with` so each cell gets an independent session."""
        if self._client_builder is not None:
            return self._client_builder(model)
        from claude_agent_sdk import ClaudeSDKClient
        from agents.flight import make_drone_options
        opts = make_drone_options(0, self._agent._system, self._deps.world,
                                  self._deps.bridge, 1, self._deps.cameras,
                                  report=lambda _m: None, env=None, model=model)
        return ClaudeSDKClient(options=opts)


async def _drive(client, prompt: str, deadline_s: float, max_steps: int) -> tuple[Trace, bool, str]:
    """Inject the prompt, drain the response, enforce deadline + step budget.
    Returns (trace, crashed, reason). crashed stays False here; the oracle's `alive`
    check derives crash from geofence breach + the reason string."""
    trace = Trace()
    t0 = time.monotonic()
    reason = ""

    async def _run():
        nonlocal reason
        await client.query(prompt)
        async for msg in client.receive_response():
            trace.observe(msg, time.monotonic() - t0)
            if trace.steps > max_steps:
                reason = "step budget exceeded"
                return
    try:
        await asyncio.wait_for(_run(), timeout=deadline_s)
    except asyncio.TimeoutError:
        reason = "wall-clock deadline"
    return trace, False, reason


def require_single_drone(spec) -> None:
    """This runner only flies drone 0. Reject multi-drone specs loudly rather than
    silently producing plausible-but-wrong results."""
    if spec.setup.n_drones != 1:
        raise ValueError(
            f"single_drone runner requires n_drones==1, got {spec.setup.n_drones} "
            f"(task {spec.id})")


async def run_cell(spec, assignment: dict, repeat: int, deps: Deps,
                   harness: "DroneHarness") -> CellResult:
    require_single_drone(spec)
    label = assignment_label(assignment)
    base = CellResult(spec.id, label, repeat, passed=False)
    n = spec.setup.n_drones  # 1 for single_drone tasks

    try:
        system = await harness.system()  # built + connected once, reused across cells
    except Exception as e:
        base.infra_fail = True
        base.failure_reason = f"connect failed: {e}"
        return base

    rr = await soft_reset([system], deps.world, deps.bridge, n)
    if not rr.ok:
        base.infra_fail = True
        base.failure_reason = f"reset unclean: {rr.reason}"
        return base

    sampler = Sampler(deps.world, deps.bridge, n, spec.objects_map(),
                      geofence_m=300.0)
    samp_task = asyncio.create_task(sampler.run())
    client = harness.client_for(model_for(assignment, "drones"))
    try:
        async with client:  # fresh session per cell — no context bleed between cells
            trace, crashed, reason = await _drive(
                client, spec.prompt, spec.budget.wall_clock_s, spec.budget.max_steps)
    except Exception as e:
        base.infra_fail = True
        base.failure_reason = f"agent run errored: {e}"
        return base
    finally:
        sampler.stop()
        await samp_task

    track = sampler.track()
    # Crash is inferred by the oracle's `alive` check (geofence breach); the deadline/
    # step-budget note lives in `reason`. So run_meta only carries steps + a False crash flag.
    run_meta = {"steps": trace.steps, "crashed": False}
    g = grade(track, spec.oracle, run_meta)

    base.passed = g.passed
    base.checks = g.checks
    base.steps = trace.steps
    base.latency_s = trace.first_action_t
    base.failure_reason = reason
    return base
