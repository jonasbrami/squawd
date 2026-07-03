"""Scripted reference pilot: fly a task's ideal tool sequence with NO LLM.

The single biggest validity threat found in the first sweeps was tooling, not
models (the fire-and-forget goto trap zeroed plan_depth for every tier and
inverted c1). The pilot is the gate: each task YAML may declare `pilot:` — the
ideal tool calls — and `run_evals --pilot` drives them through the SAME
run_cell / sampler / oracle path as a real agent. A task whose pilot can't pass
is quarantined as a harness bug before any LLM cells are spent on it; run it at
K=3 and the reach-distance spread is the sim-noise floor for free.

ScriptedClient mimics the ClaudeSDKClient surface run_cell uses (async context
manager + query + receive_response) and yields REAL SDK message objects, so
Trace, step budgets, and transcripts.jsonl work unchanged.

Dynamic tasks add two twists:
- a script step may be {behavior: <name>, args: {...}} — behaviors compute
  their tool calls at runtime from live mover positions (ops.gzposes) and
  yield every underlying ops call, so step budgets and transcripts stay honest;
- the DUAL-baseline gate: `pilot:` is the must-PASS reference (e.g. a lead
  predictor), an optional `null_pilot:` is the must-FAIL baseline (e.g. the
  naive chaser). A dynamic task where the naive chaser passes is grading tool
  semantics, not prediction — both gates run under `run_evals --pilot`."""
import asyncio
import math

from claude_agent_sdk import AssistantMessage, ToolResultBlock, ToolUseBlock, UserMessage


def _mover_xy(ops, name: str) -> tuple[float, float]:
    p = ops.gzposes.poses().get(name)
    if p is None:
        raise ValueError(f"behavior: mover {name!r} not visible on gz poses")
    return (p[0], p[1])


async def naive_chaser(ops, args):
    """goto(the mover's CURRENT position), `rounds` times — the tail-chase every
    dynamic rung above L1 must make FAIL, else the task doesn't probe prediction."""
    alt = float(args.get("alt", 12.0))
    for _ in range(int(args.get("rounds", 6))):
        e, n = _mover_xy(ops, args["mover"])
        res = await ops.goto(east=e, north=n, up=alt)
        yield ("goto", {"east": e, "north": n, "up": alt}, res)


async def lead_chaser(ops, args):
    """Chase with a lead: goto(current + velocity x lead), velocity finite-
    differenced from consecutive rounds using gz sim time. The lead ADAPTS to
    the measured round cadence — a blocking goto costs ~4s of fixed overhead
    (arrival detection, yaw, accel) on top of flight time, so a hardcoded lead
    tuned for fast legs perpetually trails the mover (observed live: a 5s lead
    at 7.5s cycles orbited 15m behind a 4 m/s rover forever)."""
    alt = float(args.get("alt", 12.0))
    lead_s = float(args.get("lead_s", 8.0))
    prev, prev_t = None, None
    for _ in range(int(args.get("rounds", 8))):
        e, n = _mover_xy(ops, args["mover"])
        t = ops.gzposes.sim_time()
        ve = vn = 0.0
        lead = lead_s
        if prev is not None and t > prev_t:
            dt = t - prev_t
            ve, vn = (e - prev[0]) / dt, (n - prev[1]) / dt
            lead = max(lead_s, dt * 1.15)
        prev, prev_t = (e, n), t
        te, tn = e + ve * lead, n + vn * lead
        res = await ops.goto(east=te, north=tn, up=alt)
        yield ("goto", {"east": te, "north": tn, "up": alt}, res)


async def lead_intercept(ops, args):
    """Two observations obs_s apart -> velocity -> closed-form intercept point at
    our cruise speed -> one goto. The reference for estimate-and-intercept rungs."""
    alt = float(args.get("alt", 12.0))
    obs_s = float(args.get("obs_s", 5.0))
    speed = float(args.get("speed_mps", ops._speed))
    if speed != ops._speed:
        res = await ops.set_speed(speed=speed)   # solve AND fly at this speed
        yield ("set_speed", {"speed": speed}, res)
    e1, n1 = _mover_xy(ops, args["mover"])
    t1 = ops.gzposes.sim_time()
    await asyncio.sleep(obs_s)
    e2, n2 = _mover_xy(ops, args["mover"])
    t2 = ops.gzposes.sim_time()
    dt = max(t2 - t1, 1e-6)
    ve, vn = (e2 - e1) / dt, (n2 - n1) / dt
    st = ops.world.drone_state(ops.bridge, ops.i)
    de, dn = e2 - st[0], n2 - st[1]
    # solve |target + v*t - drone| = speed*t for the earliest positive t
    a = ve * ve + vn * vn - speed * speed
    b = 2.0 * (de * ve + dn * vn)
    c = de * de + dn * dn
    t_hit = None
    if abs(a) < 1e-9:
        t_hit = -c / b if b < 0 else None
    else:
        disc = b * b - 4 * a * c
        if disc >= 0.0:
            roots = sorted(((-b - math.sqrt(disc)) / (2 * a),
                            (-b + math.sqrt(disc)) / (2 * a)))
            t_hit = next((r for r in roots if r > 0), None)
    t_hit = t_hit if t_hit is not None else c ** 0.5 / max(speed, 1e-6)
    # Aim a little PAST the meeting point along the mover's course: the drone
    # doesn't fly at cruise instantly (accel lag cost a clean solve a 14.6m
    # miss live), and arriving early to let the target come to you is robust.
    margin = float(args.get("lead_margin_s", 2.5))
    te, tn = e2 + ve * (t_hit + margin), n2 + vn * (t_hit + margin)
    res = await ops.goto(east=te, north=tn, up=alt)
    yield ("goto", {"east": te, "north": tn, "up": alt}, res)


async def await_gap(ops, args):
    """Hover until the mover's coordinate ('e'|'n') passes min_value while
    RECEDING from it — the timing-gate primitive. Bounded by timeout_s."""
    idx = 0 if args.get("coord", "n") == "e" else 1
    min_value = float(args["min_value"])
    poll_s = float(args.get("poll_s", 2.0))
    deadline = float(args.get("timeout_s", 60.0))
    waited, last = 0.0, None
    while waited < deadline:
        p = ops.gzposes.poses()[args["mover"]]
        v = p[idx]
        if v >= min_value and last is not None and v > last:
            yield ("hover", {"seconds": 0}, f"gap open: mover at {v:.1f}, receding")
            return
        last = v
        res = await ops.hover(seconds=poll_s)
        waited += poll_s
        yield ("hover", {"seconds": poll_s}, res)
    yield ("hover", {"seconds": 0}, f"await_gap TIMED OUT after {deadline}s")


BEHAVIORS = {
    "naive_chaser": naive_chaser,
    "lead_chaser": lead_chaser,
    "lead_intercept": lead_intercept,
    "await_gap": await_gap,
}


class ScriptedClient:
    """Executes `script` (list of {tool, args}) against a FlightOps built lazily by
    `ops_provider` (the shared MAVSDK System isn't connected until the harness is)."""

    def __init__(self, ops_provider, script: list[dict]) -> None:
        self._ops_provider = ops_provider
        self._script = list(script or [])

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt: str) -> None:  # the pilot ignores the prose
        pass

    async def receive_response(self):
        ops = await self._ops_provider()
        seq = 0
        for i, step in enumerate(self._script):
            if "behavior" in step:
                fn = BEHAVIORS.get(step["behavior"])
                if fn is None:
                    raise ValueError(f"pilot step {i}: unknown behavior "
                                     f"'{step['behavior']}' ({sorted(BEHAVIORS)})")
                async for tool, targs, out in fn(ops, step.get("args", {})):
                    yield AssistantMessage(
                        content=[ToolUseBlock(id=f"pilot{seq}", name=f"pilot__{tool}",
                                              input=targs)], model="pilot")
                    yield UserMessage(content=[ToolResultBlock(
                        tool_use_id=f"pilot{seq}", content=str(out), is_error=False)])
                    seq += 1
                continue
            tool = step["tool"]
            args = step.get("args", {})
            yield AssistantMessage(
                content=[ToolUseBlock(id=f"pilot{seq}", name=f"pilot__{tool}", input=args)],
                model="pilot")
            fn = getattr(ops, tool, None)
            if fn is None or tool.startswith("_"):
                raise ValueError(f"pilot step {i}: unknown tool '{tool}'")
            out = fn(**args)
            if hasattr(out, "__await__"):
                out = await out
            yield UserMessage(content=[ToolResultBlock(
                tool_use_id=f"pilot{seq}", content=str(out), is_error=False)])
            seq += 1


def pilot_client_builder(harness, deps):
    """A DroneHarness client_builder for pilot mode. The script varies per cell, so
    run_evals sets `harness.pilot_script` before each run_cell; the builder reads it."""

    async def ops_provider():
        from agents.flight.ops import FlightOps
        system = await harness.system()
        return FlightOps(system, deps.world, deps.bridge, 0, 1,
                         gzposes=deps.gzposes)

    def build(model):
        return ScriptedClient(ops_provider, getattr(harness, "pilot_script", []))

    return build
