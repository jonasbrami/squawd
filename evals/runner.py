"""Run ONE eval cell (task x model x repeat) against a live sim: single_drone,
operator, or commander layer (per spec.target_layer, or a --layer override carried
in assignment["_layer"]).

Builds the fleet of DroneAgents with the chosen model, soft-resets the world, starts
the sampler, injects the task prompt at the (single- or fleet-scoped) Claude client,
and bounds the run by the spec's wall-clock + step budget. Captures latency +
tool-call trace, then grades the sampled WorldTrack. Infra failures (no fix, RTL/
connection errors) are flagged, not scored as task failures."""
import asyncio
import math
import time
from dataclasses import dataclass, field

from agents.flight.backend import (KIMI_MODELS, Result, Text, ToolCall,
                                   ToolResult)

from evals.oracle import grade
from evals.reset import soft_reset
from evals.sampler import Sampler

TIERS = {
    "opus": "claude-opus-4-8",
    "sonnet": "claude-sonnet-5",
    "haiku": "claude-haiku-4-5-20251001",
    # Kimi subscription tiers (design §5.2) — wired at M6: client_for routes
    # them through the backend seam with the §5.2 env recipe; make_pilot_options
    # enforces cli_path=which("claude") on the tier (R5).
    "kimi": "kimi-for-coding",
    "kimi3": "k3",
    "codex": "gpt-5.6-terra",
    "pilot": None,        # scripted reference pilot (no LLM) — see evals/pilot.py
    "pilot_null": None,   # must-FAIL baseline (e.g. naive chaser) — same machinery
}

SETTLE_S = 45.0   # post-turn settle allowance, independent of the turn budget


def model_for(assignment: dict, role: str) -> str | None:
    tier = assignment.get(role)
    return TIERS[tier] if tier else None


def assignment_label(assignment: dict) -> str:
    return ",".join(f"{k}={v}" for k, v in sorted(assignment.items())) or "default"


def _summarize_result(content, cap: int = 500) -> str:
    """Flatten a tool result's content to a short string; never store image b64
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
    args/result/duration, agent text between calls, and end-of-run usage/cost —
    plus the §5.5 quota metrics the backend seam stamps on the Result event
    (exact inference requests, classified quota errors, ttfa/gap_p50/wall
    timings). All of it comes off the backend seam's typed event stream (design
    §6.5) that `_drive` already iterates — tool choice per tier is thereby
    observed, not inferred from step counts, and no SDK message type reaches
    this file."""

    def __init__(self) -> None:
        self.steps = 0
        self.first_action_t: float | None = None
        self.events: list[dict] = []        # tool calls + agent text, in order
        self._open: dict[str, dict] = {}    # tool_use_id -> event awaiting its result
        self.model: str | None = None
        self.usage: dict | None = None      # from the Result event; None if deadline cut
        self.cost_usd: float | None = None
        self.num_turns: int | None = None
        self.api_ms: int | None = None
        # §5.5 quota metrics (stamped on the Result event by BackendClient)
        self.inference_requests: int | None = None
        self.quota_errors: int = 0
        self.ttfa_s: float | None = None
        self.gap_p50_s: float | None = None
        self.wall_ms: int | None = None
        self.meta: dict = {}   # first-class side-channel; e.g. commander's drone_steps

    def observe(self, ev, now: float) -> None:
        model = getattr(ev, "model", None)
        if model:
            self.model = self.model or model
        if isinstance(ev, ToolCall):
            self.steps += 1
            if self.first_action_t is None:
                self.first_action_t = now
            call = {"type": "tool_call", "t": round(now, 2), "name": ev.name,
                    "args": ev.input, "result": None, "is_error": None,
                    "dur_s": None}
            self.events.append(call)
            self._open[ev.id] = call
        elif isinstance(ev, Text):
            self.events.append({"type": "text", "t": round(now, 2),
                                "text": ev.text})
        elif isinstance(ev, ToolResult):
            call = self._open.pop(ev.tool_use_id, None)
            if call is not None:
                call["result"] = _summarize_result(ev.content)
                call["is_error"] = bool(ev.is_error)
                call["dur_s"] = round(now - call["t"], 2)
        elif isinstance(ev, Result):
            self.usage = ev.usage
            self.cost_usd = ev.cost_usd
            self.num_turns = ev.num_turns
            self.api_ms = ev.api_ms
            self.inference_requests = ev.inference_requests
            self.quota_errors = ev.quota_errors
            self.ttfa_s = ev.ttfa_s
            self.gap_p50_s = ev.gap_p50_s
            self.wall_ms = ev.wall_ms

    def transcript(self, t0_epoch: float) -> dict:
        return {"model": self.model, "t0_epoch": round(t0_epoch, 2),
                "usage": self.usage, "cost_usd": self.cost_usd,
                "num_turns": self.num_turns, "api_ms": self.api_ms,
                "inference_requests": self.inference_requests,
                "quota_errors": self.quota_errors,
                "ttfa_s": self.ttfa_s, "gap_p50_s": self.gap_p50_s,
                "wall_ms": self.wall_ms,
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
    layer: str = ""            # effective layer this cell ran under (spec/CLI override)
    drone_steps: int = 0       # commander layer only: total tool calls across dispatched drones

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
            "layer": self.layer,
            "drone_steps": self.drone_steps,
        }


@dataclass
class Deps:
    """Live handles shared across cells in one sweep (built once by run_evals).

    Deps split (design §3.8, review Codex-Mj11): `oracle_truth` feeds the
    SAMPLER + ORACLE ONLY — it is ground truth and must never reach the flight
    path. `flight_contacts` is the ContactProvider the flight tools read
    (VisionContacts camera-fed, or GzPoses for the explicit truth-fed control);
    `detector` is the live Detector when cells run camera-fed. `pipeline` is
    the live VisionPipeline on the vision lane (None on the truth lane) — it
    feeds the eval client's `detect` tool exactly like the production pilot."""
    world: object
    bridge: object
    cameras: object
    oracle_truth: object = None    # GzPoses — sampler + oracle ONLY
    flight_contacts: object = None # ContactProvider fed to FlightOps
    detector: object = None
    pipeline: object = None        # VisionPipeline — detect tool feed (M6)

    @property
    def gzposes(self):
        """Back-compat read alias (pre-M5 name) — the oracle truth feed."""
        return self.oracle_truth


class FleetHarness:
    """Owns the persistent MAVSDK links for drones 0..n-1, built ONCE and reused
    across every cell — this is the fix for the per-cell subscription/System leak
    (building a DroneAgent per cell re-subscribed the non-idempotent RosBridge and
    spun a new System each time, growing linearly over a sweep), generalized from
    one drone to a fleet of n.

    It hands out a FRESH Claude client per cell (bound to the shared Systems with the
    cell's model), because the SDK model is fixed at client construction AND each
    cell/repeat must run an independent agent session — a reused client would bleed
    one cell's conversation into the next and poison the accuracy numbers. So: cache
    the model-independent flight links, rebuild only the cheap per-cell client.
    single_drone specs (n_drones==1) get the classic one-drone options; operator
    specs get make_operator_options (one client, all n drones).

    The `agent_factory`/`client_builder` hooks exist so the caching lifecycle is
    unit-testable without ROS; production paths default to the real deferred imports."""

    def __init__(self, deps: Deps, n: int = 1, agent_factory=None,
                client_builder=None) -> None:
        self._deps = deps
        self._n = n
        self._agent_factory = agent_factory
        self._client_builder = client_builder
        self._agents: list | None = None  # cached DroneAgents; we use only their connected _system
        # Strategy-snippet A/B (design §13 item 6): run_cell sets this per cell
        # from the assignment's `strategy` key; appended to the system prompt.
        self.prompt_append: str | None = None

    def _make_agent(self, i: int):
        if self._agent_factory is not None:
            return self._agent_factory(i)
        from agents.swarm.drone import DroneAgent  # deferred: rclpy/mavsdk at runtime only
        return DroneAgent(i, self._deps.world, self._deps.bridge, self._n,
                          self._deps.cameras, model=None)

    async def _ensure(self) -> list:
        if self._agents is None:
            agents = [self._make_agent(i) for i in range(self._n)]
            for a in agents:
                await a.connect()  # connects the MAVSDK link + arms PX4 geofence, once
            self._agents = agents
        return self._agents

    async def systems_list(self) -> list:
        """The shared, connected MAVSDK Systems for drones 0..n-1 (built + connected
        once)."""
        return [a._system for a in await self._ensure()]

    async def system(self):
        """Back-compat accessor: the shared System for drone 0."""
        return (await self.systems_list())[0]

    def _make_ops(self):
        """A fresh FlightOps for one cell, wired to `deps.flight_contacts` —
        NEVER to `deps.oracle_truth` (the Deps split, design §3.8: truth is for
        the sampler/oracle only; the truth-fed control is an explicit
        `flight_contacts = gzposes` choice made by run_evals, not by this
        harness) — and carrying the SAME Envelope the production pilot builds
        (agents/pilot/run.py), so eval cells fly with envelope parity (M6).
        Deferred imports: mavsdk-free unit tests can call this with
        fake agents in place."""
        from agents.flight.envelope import Envelope
        from agents.flight.ops import FlightOps
        return FlightOps(self._agents[0]._system, self._deps.world,
                         self._deps.bridge, 0, 1,
                         contacts=self._deps.flight_contacts,
                         envelope=Envelope())

    def client_for(self, model, n_drones: int = 1):
        """A fresh BackendClient (the §6.5 seam) bound to the shared System(s),
        with `model`. Caller drives it under `async with` so each cell gets an
        independent session. n_drones<=1 gets the classic single-drone options;
        n_drones>1 gets the operator layer (one client, all n drones).
        Kimi tiers (KIMI_MODELS) get the §5.2 env recipe here; make_pilot_options
        enforces cli_path=which("claude") on that tier (R5) — a missing CLI is a
        hard error, never a silent fall-back to the bundled one."""
        if self._client_builder is not None:
            return self._client_builder(model)
        if n_drones > 1:
            raise ValueError("operator/commander eval layers were dropped "
                             "(single-drone rebuild); use n_drones=1")
        from agents.flight.backend import kimi_recipe, make_backend_client
        from agents.pilot.detect_text import make_detect_text
        # detect wired exactly like the production pilot (agents/pilot/run.py):
        # the live VisionPipeline feeds make_detect_text; no pipeline (the
        # truth-fed lane / perception down) -> detect_text None -> the detect
        # tool is simply not registered (make_pilot_options guard).
        detect = (make_detect_text(self._deps.world, self._deps.bridge,
                                   self._deps.pipeline)
                  if self._deps.pipeline is not None else None)
        backend = ("codex" if model == "gpt-5.6-terra" else
                   "kimi" if model in KIMI_MODELS else "claude")
        return make_backend_client(
            self._make_ops(), detect_text=detect, report=lambda _m: None,
            backend=backend,
            env=(kimi_recipe() if backend == "kimi" else None), model=model,
            extra_prompt=self.prompt_append)


DroneHarness = FleetHarness  # back-compat alias for older imports/tests


async def _drive(client, prompt: str, deadline_s: float, max_steps: int,
                 on_msg=None) -> tuple[Trace, bool, str]:
    """Inject the prompt, drain the typed event stream, enforce deadline + step
    budget. Returns (trace, crashed, reason). crashed stays False here; the oracle's
    `alive` check derives crash from geofence breach + the reason string.
    `on_msg(trace, ev, now)` fires synchronously after every observe — the
    TargetLockEvent path (evals/perceive_eval.note_target_lock) rides it so the
    contact→truth association happens AT the event's sim moment."""
    trace = Trace()
    t0 = time.monotonic()
    reason = ""

    async def _run():
        nonlocal reason
        async for ev in client.query(prompt):
            now = time.monotonic() - t0
            trace.observe(ev, now)
            if on_msg is not None:
                on_msg(trace, ev, now)
            if trace.steps > max_steps:
                reason = "step budget exceeded"
                return
    try:
        await asyncio.wait_for(_run(), timeout=deadline_s)
    except asyncio.TimeoutError:
        reason = "wall-clock deadline"
    return trace, False, reason


def client_failed(trace: Trace) -> bool:
    """True when the SDK client never actually ran the model — stale credentials
    surface as a '<synthetic>' assistant message ('Failed to authenticate...'),
    and a dead client yields an empty stream. Such cells are INFRA failures;
    scoring them as task FAILs poisoned a 24-cell run (0 steps everywhere)."""
    return trace.model is None or trace.model == "<synthetic>"


def require_layer_supported(spec, layer: str | None = None) -> None:
    """single_drone cells must be n==1; operator/commander layers were dropped
    in the single-drone rebuild (design §3.9) and are rejected loudly, never
    silently. `layer` (a --layer CLI override) takes precedence over
    spec.target_layer when given."""
    effective = layer or getattr(spec, "target_layer", "single_drone")
    if effective != "single_drone":
        raise ValueError(f"target_layer {effective!r} was dropped in the "
                         f"single-drone rebuild (task {spec.id})")
    if spec.setup.n_drones != 1:
        raise ValueError(
            f"single_drone runner requires n_drones==1, got {spec.setup.n_drones} "
            f"(task {spec.id})")


require_single_drone = require_layer_supported  # back-compat alias for older imports/tests


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


def _completed_land(trace: Trace) -> bool:
    """True only when the final tool call is a successful, completed land.

    A started/cancelled land must still take the emergency-hold path below.
    Provider prefixes differ, so compare the final MCP component only.
    """
    call = next((event for event in reversed(trace.events)
                 if event.get("type") == "tool_call"), None)
    return bool(call and call["name"].split("__")[-1] == "land"
                and call.get("result") is not None
                and call.get("is_error") is False)


async def _wait_disarmed(system, timeout_s: float = 15.0) -> bool:
    """Wait for PX4 auto-disarm after a completed landing command."""
    async def wait():
        async for armed in system.telemetry.armed():
            if not armed:
                return True
        return False

    try:
        return bool(await asyncio.wait_for(wait(), timeout=timeout_s))
    except Exception:
        return False


def reset_per_cell(deps: Deps) -> None:
    """Per-cell clean slate (design §3.8): `VisionContacts.reset()` on the flight
    contacts so no EKF filter state or `vis_*` ID leaks across anchored repeats.
    GzPoses (the truth-fed control) has no reset — the getattr guard makes that
    lane a no-op. The oracle truth feed itself is never reset here; it is only
    re-ANCHORED (phase 0) by run_cell below."""
    reset = getattr(deps.flight_contacts, "reset", None)
    if callable(reset):
        reset()


def _detector_tag(deps: Deps) -> str | None:
    """Backend class name for the transcript's detector dim (primitive stats,
    §13 item 7) — None when cells ran without a live detector."""
    if deps.detector is None:
        return None
    return type(getattr(deps.detector, "_backend", None)).__name__


async def run_cell(spec, assignment: dict, repeat: int, deps: Deps,
                   harness: "FleetHarness") -> CellResult:
    effective_layer = assignment.get("_layer") or spec.target_layer
    require_layer_supported(spec, effective_layer)
    label = assignment_label(assignment)
    base = CellResult(spec.id, label, repeat, passed=False)
    base.difficulty = dict(spec.difficulty)
    base.suite = spec.suite
    base.layer = effective_layer
    n = spec.setup.n_drones  # 1 for single_drone tasks; >1 for operator tasks

    try:
        systems = await harness.systems_list()  # built + connected once, reused across cells
    except Exception as e:
        base.infra_fail = True
        base.failure_reason = f"connect failed: {e}"
        return base

    rr = await soft_reset(systems, deps.world, deps.bridge, n)
    if not rr.ok:
        base.infra_fail = True
        base.failure_reason = f"reset unclean: {rr.reason}"
        return base
    reset_per_cell(deps)   # no filter/ID leak across anchored repeats (§3.8)
    if deps.oracle_truth is not None:
        # every repeat starts at trajectory phase 0 — unanchored repeats sample
        # random mover phases and confound pass rates across K
        deps.oracle_truth.anchor()

    sampler = Sampler(deps.world, deps.bridge, n, spec.objects_map(),
                      geofence_m=300.0, gzposes=deps.oracle_truth)
    samp_task = asyncio.create_task(sampler.run())
    t_start = time.monotonic()
    t0_epoch = time.time()
    try:
        from evals.task_prompt import render_task_prompt
        # Strategy-snippet A/B (§13 item 6): the assignment's `strategy` key names
        # a snippet appended to the system prompt for THIS cell only.
        strategy = assignment.get("strategy")
        if strategy:
            from evals.strategy_ab import load_snippet
            harness.prompt_append = load_snippet(strategy)
        else:
            harness.prompt_append = None
        # identified_target data path (Codex-B5): associate the first vis_*
        # lock to oracle truth AT its sim moment, synchronously at observe time.
        lock_hook = None
        if deps.flight_contacts is not None and deps.oracle_truth is not None:
            from evals.perceive_eval import note_target_lock

            def lock_hook(trace, _msg, _now):
                note_target_lock(trace, deps.flight_contacts, deps.oracle_truth)

        client = harness.client_for(model_for(assignment, "drones"), n_drones=n)
        async with client:  # fresh session per cell — no context bleed between cells
            trace, crashed, reason = await _drive(
                client, render_task_prompt(spec),
                spec.budget.wall_clock_s, spec.budget.max_steps,
                on_msg=lock_hook)
        # HALT before settling when flight is still active: if the deadline
        # cancelled a blocking goto, its setpoint otherwise keeps flying.  A
        # successfully completed final `land` is the one exception: HOLD would
        # interrupt PX4's touchdown/auto-disarm sequence and leave the vehicle
        # armed in LOITER (caught by the backend-switch smoke, 2026-08-08).
        landed = False
        if _completed_land(trace):
            landed = all([await _wait_disarmed(s) for s in systems])
        else:
            # Per-system isolation: one dead link must not skip the others.
            for s in systems:
                try:
                    await asyncio.wait_for(s.action.hold(), timeout=5)
                except Exception:
                    pass
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

    if client_failed(trace):
        first = next((e["text"] for e in trace.events if e.get("type") == "text"), "")
        base.infra_fail = True
        base.failure_reason = f"client never ran the model: {first[:120]}"
        base.transcript = trace.transcript(t0_epoch)
        base.drone_steps = trace.meta.get("drone_steps", 0)
        if _detector_tag(deps):
            base.transcript["detector"] = _detector_tag(deps)
        return base

    track = sampler.track()
    # Crash is inferred by the oracle's `alive` check (geofence breach); the deadline/
    # step-budget note lives in `reason`. So run_meta only carries steps + a False crash flag.
    run_meta = {"steps": trace.steps, "crashed": False, "landed": landed}
    if "target_lock" in trace.meta:   # TargetLockEvent → identified_target (§3.8)
        run_meta["target_lock"] = trace.meta["target_lock"]
    g = grade(track, spec.oracle, run_meta)

    base.passed = g.passed
    base.checks = g.checks
    base.steps = trace.steps
    base.latency_s = trace.first_action_t
    base.failure_reason = reason
    base.transcript = trace.transcript(t0_epoch)
    base.drone_steps = trace.meta.get("drone_steps", 0)
    if _detector_tag(deps):
        base.transcript["detector"] = _detector_tag(deps)
    return base
