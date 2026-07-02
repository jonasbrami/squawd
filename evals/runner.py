"""Run ONE eval cell (task x model x repeat) against a live sim, single-drone layer.

Builds a DroneAgent with the chosen model, soft-resets the world, starts the sampler,
injects the task prompt at the drone's own Claude client, and bounds the run by the
spec's wall-clock + step budget. Captures latency + tool-call trace, then grades the
sampled WorldTrack. Infra failures (no fix, RTL/connection errors) are flagged, not
scored as task failures."""
import asyncio
import math
import time
from dataclasses import dataclass, field

from claude_agent_sdk import (AssistantMessage, ResultMessage, TextBlock,
                              ToolResultBlock, ToolUseBlock, UserMessage)

from evals.oracle import grade
from evals.reset import soft_reset
from evals.sampler import Sampler

TIERS = {
    "opus": "claude-opus-4-8",
    "sonnet": "claude-sonnet-5",
    "haiku": "claude-haiku-4-5-20251001",
}

SETTLE_S = 45.0   # post-turn settle allowance, independent of the turn budget


def model_for(assignment: dict, role: str) -> str | None:
    tier = assignment.get(role)
    return TIERS[tier] if tier else None


def assignment_label(assignment: dict) -> str:
    return ",".join(f"{k}={v}" for k, v in sorted(assignment.items())) or "default"


def _summarize_result(content, cap: int = 500) -> str:
    """Flatten a ToolResultBlock's content to a short string; never store image b64
    (a single `look` frame is ~100KB of base64 — it would balloon the transcript)."""
    if content is None:
        return ""
    if isinstance(content, str):
        return content[:cap]
    parts = []
    for b in content:
        if isinstance(b, dict) and b.get("type") == "image":
            parts.append(f"<image {len(b.get('data') or '')} b64 chars>")
        else:
            parts.append(str(b.get("text", b) if isinstance(b, dict) else b)[:cap])
    return " | ".join(parts)[:cap]


class Trace:
    """Counts steps AND records the full per-cell transcript: every tool call with
    args/result/duration, agent text between calls, and end-of-run usage/cost. All
    of it comes off the SDK stream `_drive` already iterates — tool choice per tier
    is thereby observed, not inferred from step counts."""

    def __init__(self) -> None:
        self.steps = 0
        self.first_action_t: float | None = None
        self.events: list[dict] = []        # tool calls + agent text, in order
        self._open: dict[str, dict] = {}    # tool_use_id -> event awaiting its result
        self.model: str | None = None
        self.usage: dict | None = None      # from ResultMessage; None if deadline cut
        self.cost_usd: float | None = None
        self.num_turns: int | None = None
        self.api_ms: int | None = None

    def observe(self, msg, now: float) -> None:
        if isinstance(msg, AssistantMessage):
            self.model = self.model or msg.model
            for blk in msg.content:
                if isinstance(blk, ToolUseBlock):
                    self.steps += 1
                    if self.first_action_t is None:
                        self.first_action_t = now
                    ev = {"type": "tool_call", "t": round(now, 2), "name": blk.name,
                          "args": blk.input, "result": None, "is_error": None,
                          "dur_s": None}
                    self.events.append(ev)
                    self._open[blk.id] = ev
                elif isinstance(blk, TextBlock) and blk.text.strip():
                    self.events.append({"type": "text", "t": round(now, 2),
                                        "text": blk.text})
        elif isinstance(msg, UserMessage) and isinstance(msg.content, list):
            for blk in msg.content:
                if isinstance(blk, ToolResultBlock) and blk.tool_use_id in self._open:
                    ev = self._open.pop(blk.tool_use_id)
                    ev["result"] = _summarize_result(blk.content)
                    ev["is_error"] = bool(blk.is_error)
                    ev["dur_s"] = round(now - ev["t"], 2)
        elif isinstance(msg, ResultMessage):
            self.usage = msg.usage
            self.cost_usd = msg.total_cost_usd
            self.num_turns = msg.num_turns
            self.api_ms = msg.duration_api_ms

    def transcript(self, t0_epoch: float) -> dict:
        return {"model": self.model, "t0_epoch": round(t0_epoch, 2),
                "usage": self.usage, "cost_usd": self.cost_usd,
                "num_turns": self.num_turns, "api_ms": self.api_ms,
                "events": self.events}


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
    difficulty: dict = field(default_factory=dict)
    suite: str | None = None
    transcript: dict = field(default_factory=dict)

    def to_transcript_row(self) -> dict:
        """One transcripts.jsonl line, keyed by the same triple as to_row so a cell
        present in results.jsonl always has exactly one aligned transcript line."""
        return {"task_id": self.task_id, "assignment": self.assignment_label,
                "repeat": self.repeat, **self.transcript}

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
            "difficulty": self.difficulty,
            "suite": self.suite,
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


async def _settle(world, bridge, n: int, deadline: float,
                  still_speed: float = 0.8, poll: float = 1.0) -> None:
    """Wait for in-flight, fire-and-forget moves to finish after the agent's turn.

    The flight tools (`goto`/`fly`) issue a movement and return BEFORE the drone
    arrives, so grading the instant the agent stops talking scores the drone mid-flight.
    Poll every `poll` s until every drone's horizontal speed is below `still_speed` m/s
    (arrived/holding) or `deadline` (monotonic clock) passes. The Sampler keeps running,
    so the final WorldTrack reflects where the drone actually ends up — not where it was
    when the agent finished. A drone with no fix counts as still-moving (don't settle on
    a blind sample)."""
    prev = None
    while time.monotonic() < deadline:
        cur = {i: world.world_xy(bridge, i) for i in range(n)}
        if prev is not None:
            moving = False
            for i in range(n):
                a, b = prev.get(i), cur.get(i)
                if a is None or b is None:
                    moving = True
                elif math.hypot(b[0] - a[0], b[1] - a[1]) / poll > still_speed:
                    moving = True
            if not moving:
                return
        prev = cur
        await asyncio.sleep(poll)


async def run_cell(spec, assignment: dict, repeat: int, deps: Deps,
                   harness: "DroneHarness") -> CellResult:
    require_single_drone(spec)
    label = assignment_label(assignment)
    base = CellResult(spec.id, label, repeat, passed=False)
    base.difficulty = dict(spec.difficulty)
    base.suite = spec.suite
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
    t_start = time.monotonic()
    t0_epoch = time.time()
    client = harness.client_for(model_for(assignment, "drones"))
    try:
        async with client:  # fresh session per cell — no context bleed between cells
            trace, crashed, reason = await _drive(
                client, spec.prompt, spec.budget.wall_clock_s, spec.budget.max_steps)
        # Settle gets its OWN allowance, not the tail of the turn budget: sharing one
        # deadline gave slower-thinking tiers less real-time flight before grading —
        # a structural bias against exactly the tiers being compared. With blocking
        # goto/fly this is normally near-instant (a safety net for wait=false moves).
        await _settle(deps.world, deps.bridge, n,
                      deadline=time.monotonic() + SETTLE_S)
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
    base.transcript = trace.transcript(t0_epoch)
    return base
