"""Provider-neutral pilot tool catalog plus backend adapters (ICD §5.5).

`make_pilot_options` takes THE one FlightOps (built by the assembler — estop and
tools share the instance, Fable-B1) and binds the 12 M1 tools (13 once
`detect_text` is supplied at M2, 15 once a `deep_tools` (look, pinpoint) pair
is supplied). The wrappers are deliberately thin: parse args
-> envelope check -> call FlightOps -> wrap text/typed errors (ICD §9).

Deep-perception tools (deep-perception plan §4 / codex B2): `look`/`pinpoint`
are SYNC text producers from agents/pilot/deep_tools.py; every binding wraps
the call in `await asyncio.to_thread(...)` so a slow/hung sidecar never stalls
the pilot loop the estop shares — a cancelled await still returns ESTOPPED via
the _handler mapping (it does not kill the worker thread, which the client
timeouts bound).

`extra_prompt` appends a strategy snippet to the system prompt for ONE options
instance — the evals strategy-snippet A/B path (design §13 item 6); production
pilots leave it None until a snippet activates on measured lift.

The pre-M5 `make_drone_options` compat shim is gone (M5): callers assemble
their own FlightOps (wired to flight contacts, never oracle truth) and call
make_pilot_options directly.
"""
import asyncio
import shutil
import traceback
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any

from claude_agent_sdk import (ClaudeAgentOptions, create_sdk_mcp_server,
                              tool as claude_tool)

from agents.flight import envelope as envmod
from agents.flight.backend import is_kimi_tier
from agents.flight.errors import ToolFailure
from agents.flight.ops import FlightOps


def _ok(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}]}


def _err(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}], "is_error": True}


def _handler(name, registry, fn, guard=None):
    """Registration + the ICD §9 error mapping, IN ORDER (CancelledError is
    BaseException — caught first, never by `except Exception`). The W3a
    `guard` (arbiter.guard_llm) runs BEFORE register(): while the operator
    lease or the estop holds, the tool is rejected OPERATOR_ACTIVE and never
    touches the slot. The clear is generation-guarded so a preempted tool's
    finally can never steal a newer owner's slot (the W0.4 race)."""
    async def h(args):
        gen = None
        try:
            if guard is not None:
                guard(name)
            if registry is not None:
                gen = registry.register(asyncio.current_task())
            return await fn(args)
        except asyncio.CancelledError:
            return _err(f"ESTOPPED: operator halted {name}")
        except ToolFailure as e:
            return _err(f"{e.code}: {e.text}")
        except asyncio.TimeoutError as e:
            return _err(f"TIMEOUT: {e}")
        except Exception as e:
            traceback.print_exc()
            return _err(f"INTERNAL: {type(e).__name__}: {e}")
        finally:
            if gen is not None:                 # only the slot's owner clears
                registry.clear(gen)
    return h


def _schema(properties: dict, required: list[str] | None = None) -> dict:
    return {"type": "object", "properties": properties,
            "required": sorted(required or []), "additionalProperties": False}


_N = {"type": "number"}
_S = {"type": "string"}
_B = {"type": "boolean"}
_I = {"type": "integer"}


@dataclass(frozen=True, slots=True)
class ToolSpec:
    """One provider-neutral MCP tool definition and its async implementation."""

    name: str
    description: str
    input_schema: dict[str, Any]
    handler: Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]


def tool(name: str, description: str, input_schema: dict[str, Any]):
    """Small catalog decorator matching the old Claude SDK declaration shape."""
    def bind(handler):
        return ToolSpec(name, description, input_schema, handler)
    return bind

PILOT_SYSTEM_PROMPT = (
    "You are drone_0, an autonomous drone with your own onboard thinking. The "
    "OPERATOR sends you commands; you carry them out with your tools, then call "
    "report(...) with a short result. Be terse.\n"
    "MOVE: `goto` (an absolute world point east/north/up OR a named target like "
    "'bldg_7') — it returns once you ARRIVE, so for an ordered route just call it "
    "once per leg, in order; `orbit` (circle a target keeping your camera on it — "
    "ONE call, no need to compute waypoints); `fly` (relative north/east/up, also "
    "returns on arrival); `face` (turn in place to aim your camera; waits until "
    "you face the target); `hover` (hold; seconds=N holds N seconds — use it for "
    "dwell tasks); `set_speed`; `take_off`; `land`. Pass wait=false to goto/fly "
    "if you need to scan/report while moving. Prefer `goto`/`orbit` with named "
    "targets and the world coords from `scan` over hand-computing paths.\n"
    "SENSE: `scan` lists nearby buildings (from the known map) and moving "
    "contacts with distance and bearing RELATIVE to where you face — items "
    "marked [IN VIEW] are in your camera. Camera is fixed forward (~69deg): to "
    "see something not [IN VIEW], `face` or `orbit` it. Use `scan` before "
    "moving near obstacles.\n"
    "TRACK: to follow a MOVING contact (a mov_* from scan), `track(target, mode, "
    "alt, duration_s, within_m)` runs an onboard real-time pursuit controller — "
    "mode='shadow' to stay on it (dwell tasks), mode='intercept' to close on it "
    "fast (returns early on contact). One call beats any sequence of gotos. "
    "Verify the returned gap/dwell numbers against your task before reporting "
    "success.\n"
    "PLAN: when a task carries constraints (no-fly zones, altitude ceilings, "
    "distance or action budgets), write out your full waypoint plan FIRST and "
    "check every leg against every constraint before your first move — a leg "
    "that clips a no-fly zone or busts the budget fails the mission even if you "
    "reach the goal.\n"
    "MISSION: for a smooth or geometry-heavy trajectory (arcs, figure-8s, "
    "per-leg speed/camera control), `run_mission(code, timeout)` runs your OWN "
    "async MAVSDK. Pre-bound (no import): `drone`, `mission_item(**fields)`, "
    "`await world_to_geo(east,north,up)`, `await arm_and_start()`, `log(msg)`; "
    "import MAVSDK classes (e.g. MissionPlan) yourself. Coords: lat/lon from "
    "`world_to_geo`, set `relative_altitude_m` to the world `up` (world `up` is "
    "height above launch; NOT its absolute altitude). run_mission is NOT covered "
    "by the safety envelope's static checks — PX4's geofence is the only hard "
    "bound there; write conservative code. Set `timeout` to the seconds the "
    "path needs.\n"
    "SAFETY: the operator can kill any action instantly (their estop cancels "
    "your tools and holds the drone — you receive ESTOPPED). If a tool call is "
    "cancelled mid-flight, re-assess state with `scan` before acting again. "
    "Tool results carry stable codes: INVALID_PARAM (re-plan), NOT_READY (wait "
    "or reposition), BLOCKED (retry or choose another route), LOST (re-acquire), "
    "TIMEOUT, ESTOPPED, INTERNAL (report it)."
)


def make_pilot_tools(ops: FlightOps, *, detect_text=None, deep_tools=None,
                     report, registry=None, guard=None) -> tuple[ToolSpec, ...]:
    """Bind the provider-neutral pilot catalog to one shared ``FlightOps``.

    Every backend receives these exact names, descriptions, JSON schemas, and
    handlers. ``guard`` is the W3a arbiter's guard_llm — see :func:`_handler`.
    """
    name = "drone_0"
    T = lambda n, fn: _handler(n, registry, fn, guard)

    async def _take_off(args):
        alt = float(args.get("altitude", 10.0))
        if ops.envelope:
            envmod.check_takeoff(ops.envelope, alt)
        return _ok(await ops.take_off(alt))

    async def _fly(args):
        return _ok(await ops.fly(args.get("north", 0), args.get("east", 0),
                                 args.get("up", 0), args.get("wait", True)))

    async def _goto(args):
        return _ok(await ops.goto(args.get("target", ""), args.get("east"),
                                  args.get("north"), args.get("up"),
                                  args.get("heading", "travel"),
                                  args.get("wait", True)))

    async def _orbit(args):
        return _ok(await ops.orbit(args.get("target", ""), args.get("east"),
                                   args.get("north"), args.get("radius", 12.0),
                                   args.get("speed", 3.0),
                                   args.get("direction", "cw"), args.get("alt")))

    async def _hover(args):
        return _ok(await ops.hover(args.get("seconds", 0)))

    async def _set_speed(args):
        v = float(args.get("speed", 5.0))
        if ops.envelope:
            envmod.check_speed(ops.envelope, v)
        return _ok(await ops.set_speed(v))

    async def _face(args):
        return _ok(await ops.face(args.get("target", "")))

    async def _land(args):
        return _ok(await ops.land())

    async def _report(args):
        report(args.get("message", ""))
        return _ok("reported")

    async def _scan(args):
        return _ok(ops.scan())

    async def _run_mission(args):
        err, text = await ops.run_mission(args.get("code", ""),
                                          args.get("timeout"))
        return _err(text) if err else _ok(text)

    async def _track(args):
        alt = args.get("alt")
        if alt is not None and ops.envelope:
            # M6 commitment safeguard: an EXPLICIT pursuit altitude is
            # envelope-checked at the boundary (same soft pre-check as
            # take_off/set_speed); an omitted alt passes None through —
            # FlightOps.track then holds the CURRENT altitude.
            envmod.check_track(ops.envelope, float(alt))
        return _ok(await ops.track(
            args.get("target", ""), args.get("mode", "shadow"),
            alt, args.get("duration_s", 60.0),
            args.get("within_m", 15.0), args.get("speed", 12.0),
            args.get("standoff_east", 0.0), args.get("standoff_north", 0.0)))

    tools = [
        tool("take_off", "Arm and take off (default 10m). Returns once airborne "
             "at altitude.", _schema({"altitude": _N}))(T("take_off", _take_off)),
        tool("fly", "Fly a relative offset from the current position (metres). "
             "Turns to face the travel direction so your camera looks where "
             "you're going. Returns when you ARRIVE; set wait=false to return "
             "immediately and act mid-flight.",
             _schema({"north": _N, "east": _N, "up": _N, "wait": _B}))(T("fly", _fly)),
        tool("goto", "Fly to an ABSOLUTE world point (east, north, up=altitude "
             "metres) OR a named target (a building like 'bldg_7'). Returns when "
             "you ARRIVE — so fly an ordered route as one goto per leg, in "
             "order. Optional heading: a compass word ('north'..) or 'travel' "
             "(default, face the way you go). Set wait=false to return "
             "immediately and act mid-flight.",
             _schema({"target": _S, "east": _N, "north": _N, "up": _N,
                      "heading": _S, "wait": _B}))(T("goto", _goto)),
        tool("orbit", "Circle a target (a building like 'bldg_7', or an "
             "east/north point) at a radius, keeping your camera pointed at "
             "the center. One call = the whole orbit.",
             _schema({"target": _S, "east": _N, "north": _N, "radius": _N,
                      "speed": _N, "direction": _S, "alt": _N}))(T("orbit", _orbit)),
        tool("hover", "Hold current position (loiter in place). Pass seconds=N "
             "to keep holding for N seconds before returning — use this for "
             "'hold/dwell for N seconds' tasks.",
             _schema({"seconds": _N}))(T("hover", _hover)),
        tool("set_speed", "Set cruise speed (m/s) for subsequent moves.",
             _schema({"speed": _N}, ["speed"]))(T("set_speed", _set_speed)),
        tool("face", "Turn in place to aim your camera at a target: a building "
             "('bldg_7') or a compass direction ('north'/'east'/'south'/'west'). "
             "Returns once you actually face it.",
             _schema({"target": _S}, ["target"]))(T("face", _face)),
        tool("land", "Land in place. Returns once on the ground.",
             _schema({}))(T("land", _land)),
        tool("report", "Report back to the operator: a short summary of what "
             "you did and what you saw. Call this when you finish a task.",
             _schema({"message": _S}, ["message"]))(T("report", _report)),
        tool("scan", "Sense nearby buildings (known map) and moving contacts: "
             "distance + bearing relative to where you face.",
             _schema({}))(T("scan", _scan)),
        tool("run_mission",
             "Author and run your OWN async MAVSDK mission code for a multi-leg "
             "or smooth trajectory. Pre-bound (no import): `drone` (the "
             "connected System), `mission_item(**fields)` (a MissionItem with "
             "every field defaulted, overridable by its real name), `await "
             "world_to_geo(east, north, up)` (-> GeoPoint; use its lat/lon ONLY, "
             "set relative_altitude_m to the world `up`), `await arm_and_start()` "
             "(arm + start the uploaded mission, retrying the transient PX4 "
             "DENIED — use it instead of arm()+start_mission()), `log(msg)`. "
             "Import MAVSDK classes you use (e.g. MissionPlan). Set `timeout` to "
             "the seconds you expect; you are uninterruptible until the mission "
             "finishes or the timeout fires (which halts you).",
             _schema({"code": _S, "timeout": _N}, ["code"]))(T("run_mission", _run_mission)),
        tool("track",
             "REAL-TIME PURSUIT of a moving contact (a mov_* from scan): an "
             "onboard 10 Hz controller flies the chase for you — far better "
             "than chasing with repeated gotos. mode='shadow' holds station on "
             "the moving target (optional standoff_east/standoff_north offset, "
             "metres); mode='intercept' flies a lead-collision course and "
             "returns EARLY the moment the horizontal gap closes within "
             "within_m. The controller MEASURES the target's velocity and "
             "computes the lead itself, so you do NOT need to scan the target "
             "repeatedly first — for a fleeing or deadline target, take off and "
             "track IMMEDIATELY. Blocks up to duration_s (max 120s) and reports "
             "min/mean gap, best contiguous dwell within within_m, and the "
             "target's measured velocity. You must be airborne first (take_off). "
             "alt defaults to your CURRENT altitude — pass it only for a "
             "deliberate altitude change (the safety ceiling still applies).",
             _schema({"target": _S, "mode": _S, "alt": _N, "duration_s": _N,
                      "within_m": _N, "speed": _N, "standoff_east": _N,
                      "standoff_north": _N}, ["target"]))(T("track", _track)),
    ]
    if detect_text is not None:
        async def _detect(args):
            return _ok(detect_text(args.get("classes")))
        tools.append(
            tool("detect", "Onboard vision (YOLO): list objects currently "
                 "visible in your camera, as TEXT — contact id, class, "
                 "confidence, relative bearing, estimated distance and world "
                 "position when computable. Optional `classes` filter "
                 "(comma-separated).",
                 _schema({"classes": _S}))(T("detect", _detect)))
    if deep_tools is not None:
        look, pinpoint = deep_tools

        async def _look(args):
            # to_thread: the sidecar HTTP call must never stall the loop the
            # estop shares (codex B2); cancellation still maps to ESTOPPED.
            return _ok(await asyncio.to_thread(
                look, args.get("what", ""), args.get("conf", 0.05)))

        async def _pinpoint(args):
            return _ok(await asyncio.to_thread(
                pinpoint, args.get("x"), args.get("y"), args.get("label")))

        tools.append(
            tool("look",
                 "ADVISORY deep scan (host-GPU open-vocabulary detector) of "
                 "the CURRENT camera frame for whatever you name — `what` is "
                 "comma-separated concepts, e.g. 'building,house' or 'truck'. "
                 "Use it to identify things the fast `detect` tool cannot "
                 "name (buildings, trees, poles, unusual objects). LOW "
                 "CONFIDENCE on this sim's flat renders: scores run "
                 "0.05-0.25 and labels can be plain wrong (a red car once "
                 "read as 'person'), so treat every result as a hint to "
                 "reason over — NEVER a flight target; the fast COCO "
                 "`detect` tool remains the authority for movers/vehicles. "
                 "Returns one line per hit: id, class, confidence, relative "
                 "bearing, and a ground_intersection estimate ONLY when the "
                 "object's bottom edge is visible in frame. `conf` defaults "
                 "to 0.05 (deliberately low — raise it only to cut noise).",
                 _schema({"what": _S, "conf": _N}, ["what"]))(T("look", _look)))
        tools.append(
            tool("pinpoint",
                 "ADVISORY deep mask (host-GPU SAM segmentation) of ONE "
                 "point in the CURRENT camera frame: pass pixel `x`,`y` "
                 "(integers, origin top-left) OR `label` to re-use the "
                 "centroid of a hit from a previous look() call. Returns the "
                 "mask's centroid bearing, pixel area and tight box. The "
                 "mask is UNLABELED — SAM segments but does not identify — "
                 "unless it was seeded from a look() label. Advisory only, "
                 "never a flight target; the fast `detect` tool remains the "
                 "mover authority.",
                 _schema({"x": _I, "y": _I, "label": _S}))(T("pinpoint", _pinpoint)))

    return tuple(tools)


def _claude_pilot_server(specs: tuple[ToolSpec, ...]):
    """Adapt the neutral catalog to Claude SDK's in-process MCP server."""
    tools = [claude_tool(s.name, s.description, s.input_schema)(s.handler)
             for s in specs]
    server = create_sdk_mcp_server(name="pilot", tools=tools)
    return server, [f"mcp__pilot__{s.name}" for s in specs]


def make_pilot_options(ops: FlightOps, *, detect_text=None, deep_tools=None,
                       report, registry=None,
                       env=None, model=None, cli_path=None,
                       extra_prompt=None, guard=None) -> ClaudeAgentOptions:
    """ICD §5.5: bind THE ONE FlightOps (shared with the estop arbiter).
    deep_tools: the (look, pinpoint) pair from agents.pilot.deep_tools
    (deep-perception M2) — bound ONLY when supplied, like detect_text.
    extra_prompt: a validated strategy snippet appended to the system prompt
    (evals A/B cells only — activation requires measured lift, §13 item 6).
    guard: the W3a CommandArbiter's guard_llm — every tool call is rejected
    OPERATOR_ACTIVE while the operator lease/estop holds.

    Kimi tier (design §5.2, R5): the SDK's bundled CLI ignores
    ANTHROPIC_BASE_URL (#677), so `cli_path=shutil.which("claude")` is
    REQUIRED — auto-resolved here, and a hard error if the external CLI is
    absent (never a silent fallback to the bundled one)."""
    if is_kimi_tier(model, env):
        cli_path = cli_path or shutil.which("claude")
        if cli_path is None:
            raise RuntimeError(
                "Kimi tier requires the external `claude` CLI on PATH: the "
                "SDK's bundled CLI ignores ANTHROPIC_BASE_URL "
                "(anthropics/claude-agent-sdk-python#677, design R5), so the "
                "agent would silently hit Anthropic instead of "
                "api.kimi.com/coding. Install the CLI (npm i -g "
                "@anthropic-ai/claude-code) or pass cli_path explicitly.")
    specs = make_pilot_tools(
        ops, detect_text=detect_text, deep_tools=deep_tools, report=report,
        registry=registry, guard=guard)
    server, allowed = _claude_pilot_server(specs)
    prompt = PILOT_SYSTEM_PROMPT + (f"\n\n{extra_prompt}" if extra_prompt else "")
    return ClaudeAgentOptions(
        mcp_servers={"pilot": server},
        allowed_tools=allowed,
        tools=[],
        setting_sources=[],
        env=env or {},
        model=model,
        cli_path=cli_path,
        system_prompt=prompt,
    )
