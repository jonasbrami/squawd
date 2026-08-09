"""Scripted reference pilot: fly a task's ideal tool sequence with NO LLM.

The single biggest validity threat found in the first sweeps was tooling, not
models (the fire-and-forget goto trap zeroed plan_depth for every tier and
inverted c1). The pilot is the gate: each task YAML may declare `pilot:` — the
ideal tool calls — and `run_evals --pilot` drives them through the SAME
run_cell / sampler / oracle path as a real agent. A task whose pilot can't pass
is quarantined as a harness bug before any LLM cells are spent on it; run it at
K=3 and the reach-distance spread is the sim-noise floor for free.

ScriptedClient mimics the BackendClient surface run_cell uses (async context
manager + query(prompt) -> typed event stream) and emits the seam's typed
events (design §6.5) directly, so Trace, step budgets, and transcripts.jsonl
work unchanged.

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

from agents.flight.backend import ToolCall, ToolResult


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


def _circumcircle(p1, p2, p3):
    """Center of the circle through 3 points, or None if near-collinear."""
    (ax, ay), (bx, by), (cx, cy) = p1, p2, p3
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-6:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    return ((a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d,
            (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d)


async def shadow_loop(ops, args):
    """The reference for loop-shadowing: measured goto physics (~4s fixed
    overhead/leg) proved NO discrete-hop strategy holds a tight dwell on a
    moving loop — the winning move is matching the mover's trajectory. Three
    observations -> circumcircle fit -> join the circle ahead of the mover ->
    author a matched-speed run_mission lap. This is exactly the strategy an
    agent must discover; the behavior exists so the task stays pilot-gated."""
    mover = args["mover"]
    alt = float(args.get("alt", 12.0))
    obs_s = float(args.get("obs_s", 4.0))
    join_m = float(args.get("join_m", 12.0))
    obs = []
    for k in range(3):
        e, n = _mover_xy(ops, mover)
        obs.append((ops.gzposes.sim_time(), e, n))
        if k < 2:
            res = await ops.hover(seconds=obs_s)
            yield ("hover", {"seconds": obs_s}, res)
    (t1, *p1), (t2, *p2), (t3, *p3) = obs
    center = _circumcircle(p1, p2, p3)
    if center is None:
        raise ValueError("shadow_loop: mover track is not a loop (collinear obs)")
    ce, cn = center
    radius = math.hypot(p3[0] - ce, p3[1] - cn)
    speed = math.hypot(p3[0] - p2[0], p3[1] - p2[1]) / max(t3 - t2, 1e-6)
    cross = ((p2[0] - p1[0]) * (p3[1] - p2[1]) - (p2[1] - p1[1]) * (p3[0] - p2[0]))
    sign = 1.0 if cross > 0 else -1.0
    ang = math.atan2(p3[1] - cn, p3[0] - ce)
    # join the circle ~40m of arc ahead of the mover, then let it close
    join_ang = ang + sign * (40.0 / radius)
    je, jn = ce + radius * math.cos(join_ang), cn + radius * math.sin(join_ang)
    res = await ops.goto(east=je, north=jn, up=alt)
    yield ("goto", {"east": je, "north": jn, "up": alt}, res)
    prev_m = None
    for _ in range(40):
        me, mn = _mover_xy(ops, mover)
        st = ops.world.drone_state(ops.bridge, ops.i)
        sep = math.hypot(me - st[0], mn - st[1])
        # start the lap only with the mover BEHIND and closing: joining with it
        # alongside/ahead makes any pacing error open the gap immediately
        # (observed: 4.5s dwell on an otherwise perfect lap)
        behind = True
        if prev_m is not None:
            mve, mvn = me - prev_m[0], mn - prev_m[1]
            behind = (st[0] - me) * mve + (st[1] - mn) * mvn > 0
        prev_m = (me, mn)
        if 4.0 <= sep <= join_m and behind:
            break
        res = await ops.hover(seconds=2)
        yield ("hover", {"seconds": 2}, res)
    # author a matched-speed lap-and-a-half from the drone's own angle
    st = ops.world.drone_state(ops.bridge, ops.i)
    dang = math.atan2(st[1] - cn, st[0] - ce)
    step = sign * (2.0 * math.pi / 10.0)
    pts = [(round(ce + radius * math.cos(dang + i * step), 2),
            round(cn + radius * math.sin(dang + i * step), 2)) for i in range(1, 16)]
    code = (
        "from mavsdk.mission import MissionPlan\n"
        "items = []\n"
        f"for (e, n) in {pts!r}:\n"
        f"    g = await world_to_geo(e, n, {alt})\n"
        "    items.append(mission_item(latitude_deg=g.latitude_deg,\n"
        "        longitude_deg=g.longitude_deg,\n"
        f"        relative_altitude_m={alt}, speed_m_s={speed:.2f},\n"
        "        acceptance_radius_m=3.0))\n"
        "await drone.mission.upload_mission(MissionPlan(items))\n"
        "await arm_and_start()\n"
        "async for p in drone.mission.mission_progress():\n"
        "    if p.current == p.total:\n"
        "        break\n"
        "return 'loop shadow complete'\n")
    is_err, text = await ops.run_mission(code=code, timeout=180)
    yield ("run_mission", {"code": f"<matched-speed lap, {len(pts)} wps>"},
           f"error={is_err}: {text[:120]}")


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


async def track_vis(ops, args):
    """The scripted PERCEPTION path (design §3.8): read the contact provider —
    the same source `scan`/`detect` render for the LLM — wait for a contact of
    class `cls` (a `vis_<cls>_<k>` id), then designate+track it. This is the
    detect→lock→track path the LLM walks, minus the language: no ground-truth
    name ever reaches the flight tools.

    `max_range_m` (optional): only lock a contact whose own range estimate is
    within it — the support-plane projection is hyper-sensitive to attitude
    noise at shallow depression (far range + low alt), so a first sighting
    beyond the proven envelope (>=6 deg depression, M2/M3a) seeds the EKF
    tens of meters off and the TargetLockEvent association gate rightly
    rejects it. Lock when CLOSE, not when barely-visible."""
    cls = args.get("cls", "target")
    wait_s = float(args.get("wait_s", 30.0))
    poll_s = float(args.get("poll_s", 1.0))
    max_rng = args.get("max_range_m")
    max_rng = float(max_rng) if max_rng is not None else None
    waited, name = 0.0, None
    while waited < wait_s:
        poses = ops.contacts.poses()
        cands = sorted(n for n in poses if n.startswith(f"vis_{cls}_"))
        if max_rng is not None and cands:
            views_fn = getattr(ops.contacts, "all_views", None)
            views = {v.name: v for v in
                     (views_fn() if callable(views_fn) else [])}
            cands = [n for n in cands
                     if views.get(n) is not None
                     and views[n].range_m is not None
                     and views[n].range_m <= max_rng]
        if cands:
            name = cands[0]
            break
        res = await ops.hover(seconds=poll_s)
        waited += poll_s
        yield ("hover", {"seconds": poll_s}, res)
    if name is None:
        yield ("hover", {"seconds": 0},
               f"track_vis: no vis_{cls}_* contact after {wait_s:.0f}s")
        return
    targs = {"target": name, "mode": args.get("mode", "shadow"),
             "alt": float(args.get("alt", 12.0)),
             "duration_s": float(args.get("duration_s", 60.0)),
             "within_m": float(args.get("within_m", 15.0))}
    # the USE block must reach the Trace BEFORE the call runs: the
    # TargetLockEvent path (identified_target) timestamps the lock AT CALL
    # TIME — emitting post-hoc (after a 60-75 s track) would associate a
    # long-dropped contact and measure nothing (observed live: every lock
    # 'measured_xy=None', 2026-07-22).
    yield ("_pending", "track", targs)
    res = await ops.track(targs["target"], mode=targs["mode"], alt=targs["alt"],
                          duration_s=targs["duration_s"],
                          within_m=targs["within_m"])
    yield ("track", targs, res)


BEHAVIORS = {
    "naive_chaser": naive_chaser,
    "lead_chaser": lead_chaser,
    "lead_intercept": lead_intercept,
    "shadow_loop": shadow_loop,
    "await_gap": await_gap,
    "track_vis": track_vis,
}


class ScriptedClient:
    """Executes `script` (list of {tool, args}) against a FlightOps built lazily by
    `ops_provider` (the shared MAVSDK System isn't connected until the harness is).
    `query(prompt)` drives the whole script and yields the seam's typed events
    (ToolCall/ToolResult pairs), exactly like BackendClient does for a live LLM."""

    def __init__(self, ops_provider, script: list[dict]) -> None:
        self._ops_provider = ops_provider
        self._script = list(script or [])

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt: str):   # the pilot ignores the prose
        ops = await self._ops_provider()
        seq = 0
        # fleet routing: on a FLEET ops the `drone:` key picks a body via the
        # drone(i) METHOD; single-drone FlightOps has a `.drone` ATTRIBUTE (the
        # MAVSDK System) — callable-guard the routing or behavior steps crash
        # with "'System' object is not callable" (latent until M5: the
        # dynamic-ladder behavior gates ran under the old fleet ops).
        drone_fn = getattr(ops, "drone", None)
        route = drone_fn if callable(drone_fn) else None
        for i, step in enumerate(self._script):
            if "behavior" in step:
                fn = BEHAVIORS.get(step["behavior"])
                if fn is None:
                    raise ValueError(f"pilot step {i}: unknown behavior "
                                     f"'{step['behavior']}' ({sorted(BEHAVIORS)})")
                body = route(step.get("drone", 0)) if route else ops
                pending = None   # (id, tool) of a ToolCall emitted pre-call
                async for item in fn(body, step.get("args", {})):
                    # "_pending": emit the ToolCall BEFORE the tool runs so
                    # observe-time hooks (TargetLockEvent) see true call time —
                    # behavior triples are otherwise post-hoc by construction.
                    if item and item[0] == "_pending":
                        _, tool, targs = item
                        yield ToolCall(id=f"pilot{seq}", name=f"pilot__{tool}",
                                       input=targs, model="pilot")
                        pending = (f"pilot{seq}", tool)
                        seq += 1
                        continue
                    tool, targs, out = item
                    if pending is not None and pending[1] == tool:
                        yield ToolResult(tool_use_id=pending[0],
                                         content=str(out), is_error=False)
                        pending = None
                        continue
                    yield ToolCall(id=f"pilot{seq}", name=f"pilot__{tool}",
                                   input=targs, model="pilot")
                    yield ToolResult(tool_use_id=f"pilot{seq}",
                                     content=str(out), is_error=False)
                    seq += 1
                continue
            tool = step["tool"]
            args = step.get("args", {})
            yield ToolCall(id=f"pilot{seq}", name=f"pilot__{tool}", input=args,
                           model="pilot")
            if hasattr(ops, tool):
                target = ops
            elif route is not None:
                target = route(step.get("drone", 0))
            else:
                target = ops
            fn = getattr(target, tool, None)
            # ``report`` belongs to the neutral provider tool catalog rather
            # than FlightOps.  The eval harness installs a no-op report sink,
            # so mirror that exact observable result for no-LLM pilot gates.
            if tool == "report" and fn is None:
                out = "reported"
                yield ToolResult(tool_use_id=f"pilot{seq}", content=out,
                                 is_error=False)
                seq += 1
                continue
            if fn is None or tool.startswith("_"):
                raise ValueError(f"pilot step {i}: unknown tool '{tool}'")
            out = fn(**args)
            if hasattr(out, "__await__"):
                out = await out
            yield ToolResult(tool_use_id=f"pilot{seq}", content=str(out),
                             is_error=False)
            seq += 1


def pilot_client_builder(harness, deps):
    """A DroneHarness client_builder for pilot mode. The script varies per cell, so
    run_evals sets `harness.pilot_script` before each run_cell; the builder reads it."""

    async def ops_provider():
        from agents.flight.ops import FlightOps
        systems = await harness.systems_list()
        if len(systems) != 1:
            raise ValueError("single-drone rebuild: pilot harness expects n==1, "
                             f"got {len(systems)}")
        # Deps split (§3.8): the scripted pilot reads the SAME contact provider
        # the LLM's flight tools get (flight_contacts) — never the oracle truth
        # unless run_evals explicitly chose the truth-fed control.
        return FlightOps(systems[0], deps.world, deps.bridge, 0, 1,
                         contacts=deps.flight_contacts)

    def build(model):
        return ScriptedClient(ops_provider, getattr(harness, "pilot_script", []))

    return build
