"""Bind a pilot's FlightOps to Claude-Agent-SDK MCP tools (ICD §5.5).

`make_pilot_options` takes THE one FlightOps (built by the assembler — estop and
tools share the instance, Fable-B1) and binds the 12 M1 tools (13 once
`detect_text` is supplied at M2). The wrappers are deliberately thin: parse args
-> envelope check -> call FlightOps -> wrap text/typed errors (ICD §9).

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

from claude_agent_sdk import tool, create_sdk_mcp_server, ClaudeAgentOptions

from agents.flight import envelope as envmod
from agents.flight.backend import is_kimi_tier
from agents.flight.errors import ToolFailure
from agents.flight.ops import FlightOps


def _ok(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}]}


def _err(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}], "is_error": True}


def _handler(name, registry, fn):
    """Registration + the ICD §9 error mapping, IN ORDER (CancelledError is
    BaseException — caught first, never by `except Exception`)."""
    async def h(args):
        if registry is not None:
            registry.register(asyncio.current_task())
        try:
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
            if registry is not None:
                registry.clear()
    return h


def _schema(properties: dict, required: list[str] | None = None) -> dict:
    return {"type": "object", "properties": properties,
            "required": sorted(required or []), "additionalProperties": False}


_N = {"type": "number"}
_S = {"type": "string"}
_B = {"type": "boolean"}

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


def _pilot_server(ops: FlightOps, detect_text, report, registry):
    """The 12 M1 tools (13 with `detect`) bound to one FlightOps."""
    name = "drone_0"
    T = lambda n, fn: _handler(n, registry, fn)

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
        return _ok(await ops.track(
            args.get("target", ""), args.get("mode", "shadow"),
            args.get("alt", 12.0), args.get("duration_s", 60.0),
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
             "target's measured velocity. You must be airborne first (take_off).",
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

    server = create_sdk_mcp_server(name="pilot", tools=tools)
    allowed = [f"mcp__pilot__{t.name}" for t in tools]
    return server, allowed


def make_pilot_options(ops: FlightOps, *, detect_text=None, report, registry=None,
                       env=None, model=None, cli_path=None,
                       extra_prompt=None) -> ClaudeAgentOptions:
    """ICD §5.5: bind THE ONE FlightOps (shared with the estop arbiter).
    extra_prompt: a validated strategy snippet appended to the system prompt
    (evals A/B cells only — activation requires measured lift, §13 item 6).

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
    server, allowed = _pilot_server(ops, detect_text, report, registry)
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
