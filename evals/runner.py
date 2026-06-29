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
    "sonnet": "claude-sonnet-4-6",
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
    latency_s: float = 0.0
    steps: int = 0
    infra_fail: bool = False
    failure_reason: str = ""

    def to_row(self) -> dict:
        return {
            "task_id": self.task_id,
            "assignment": self.assignment_label,
            "repeat": self.repeat,
            "passed": self.passed,
            "latency_s": round(self.latency_s, 2),
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


async def run_cell(spec, assignment: dict, repeat: int, deps: Deps) -> CellResult:
    from agents.swarm.drone import DroneAgent  # deferred: pulls in rclpy/mavsdk at runtime only

    label = assignment_label(assignment)
    base = CellResult(spec.id, label, repeat, passed=False)
    n = spec.setup.n_drones  # 1 for single_drone tasks

    drone = DroneAgent(0, deps.world, deps.bridge, n, deps.cameras,
                       model=model_for(assignment, "drones"))
    try:
        await drone.connect()
    except Exception as e:
        base.infra_fail = True
        base.failure_reason = f"connect failed: {e}"
        return base

    rr = await soft_reset([drone._system], deps.world, deps.bridge, n)
    if not rr.ok:
        base.infra_fail = True
        base.failure_reason = f"reset unclean: {rr.reason}"
        return base

    sampler = Sampler(deps.world, deps.bridge, n, spec.objects_map(),
                      geofence_m=300.0)
    samp_task = asyncio.create_task(sampler.run())
    try:
        async with drone.client:
            trace, crashed, reason = await _drive(
                drone.client, spec.prompt, spec.budget.wall_clock_s, spec.budget.max_steps)
    except Exception as e:
        sampler.stop()
        await samp_task
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
    base.latency_s = trace.first_action_t or 0.0
    base.failure_reason = reason
    return base
