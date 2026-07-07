"""Bind a drone's FlightOps to Claude-Agent-SDK MCP tools.

`make_drone_options` builds the per-drone MCP server (flight + look/scan/report)
and returns the ClaudeAgentOptions the swarm hands to a ClaudeSDKClient. The
wrappers are deliberately thin: parse args -> call FlightOps -> wrap text/errors.
The camera image (`look`) comes from core.GzCameras; the drone's result goes back
to the Commander via the injected `report` callable (publishes /swarm/report/<i>).
"""
from claude_agent_sdk import tool, create_sdk_mcp_server, ClaudeAgentOptions

from agents.flight.ops import FlightOps


def _ok(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}]}


def _err(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}], "is_error": True}


def _drone_server(i, ops, cameras, report):
    """The 12 per-drone tools bound to one FlightOps; returns (server, allowed)."""
    name = f"drone_{i}"

    @tool("take_off", "Arm and take off (default 10m). Returns once airborne at altitude.",
          {"altitude": {"type": "number"}})
    async def take_off(args):
        try:
            return _ok(await ops.take_off(args.get("altitude", 10.0)))
        except Exception as e:
            return _err(f"{name} takeoff failed: {e}")

    @tool("fly", "Fly a relative offset from the current position (metres). Turns to "
          "face the travel direction so your camera looks where you're going. Returns "
          "when you ARRIVE; set wait=false to return immediately and act mid-flight.",
          {"north": {"type": "number"}, "east": {"type": "number"}, "up": {"type": "number"},
           "wait": {"type": "boolean"}})
    async def fly(args):
        try:
            return _ok(await ops.fly(args.get("north", 0), args.get("east", 0), args.get("up", 0),
                                     args.get("wait", True)))
        except Exception as e:
            return _err(f"{name} fly failed: {e}")

    @tool("goto", "Fly to an ABSOLUTE world point (east, north, up=altitude metres) OR a named "
          "target (a drone like 'drone_1', a building like 'bldg_7'). Returns when you ARRIVE — "
          "so fly an ordered route as one goto per leg, in order. Optional heading: a compass "
          "word ('north'..) or 'travel' (default, face the way you go). Set wait=false to "
          "return immediately and act mid-flight.",
          {"target": {"type": "string"}, "east": {"type": "number"}, "north": {"type": "number"},
           "up": {"type": "number"}, "heading": {"type": "string"}, "wait": {"type": "boolean"}})
    async def goto(args):
        try:
            return _ok(await ops.goto(args.get("target", ""), args.get("east"), args.get("north"),
                                      args.get("up"), args.get("heading", "travel"),
                                      args.get("wait", True)))
        except Exception as e:
            return _err(f"{name} goto failed: {e}")

    @tool("orbit", "Circle a target (a drone, a building like 'bldg_7', or an east/north point) at "
          "a radius, keeping your camera pointed at the center. One call = the whole orbit.",
          {"target": {"type": "string"}, "east": {"type": "number"}, "north": {"type": "number"},
           "radius": {"type": "number"}, "speed": {"type": "number"},
           "direction": {"type": "string"}, "alt": {"type": "number"}})
    async def orbit(args):
        try:
            return _ok(await ops.orbit(args.get("target", ""), args.get("east"), args.get("north"),
                                       args.get("radius", 12.0), args.get("speed", 3.0),
                                       args.get("direction", "cw"), args.get("alt")))
        except Exception as e:
            return _err(f"{name} orbit failed: {e}")

    @tool("hover", "Hold current position (loiter in place). Pass seconds=N to keep "
          "holding for N seconds before returning — use this for 'hold/dwell for N "
          "seconds' tasks.", {"seconds": {"type": "number"}})
    async def hover(args):
        try:
            return _ok(await ops.hover(args.get("seconds", 0)))
        except Exception as e:
            return _err(f"{name} hover failed: {e}")

    @tool("set_speed", "Set cruise speed (m/s) for subsequent moves.", {"speed": {"type": "number"}})
    async def set_speed(args):
        try:
            return _ok(await ops.set_speed(args.get("speed", 5.0)))
        except Exception as e:
            return _err(f"{name} set_speed failed: {e}")

    @tool("face", "Turn in place to aim your camera at a target: a drone ('drone_1'), a "
          "building ('bldg_7'), or a compass direction ('north'/'east'/'south'/'west').",
          {"target": {"type": "string"}})
    async def face(args):
        try:
            return _ok(await ops.face(args.get("target", "")))
        except Exception as e:
            return _err(f"{name} face failed: {e}")

    @tool("land", "Land in place. Returns once on the ground.", {})
    async def land(args):
        try:
            return _ok(await ops.land())
        except Exception as e:
            return _err(f"{name} land failed: {e}")

    @tool("report", "Report back to the commander: a short summary of what you did and "
          "what you saw. Call this when you finish a task.", {"message": {"type": "string"}})
    async def report_tool(args):
        report(args.get("message", ""))
        return _ok("reported")

    @tool("look", "See through your onboard camera (returns the current image).", {})
    async def look(args):
        b64 = cameras.jpeg_b64(i) if cameras is not None else None
        if b64 is None:
            return _err(f"{name}: no camera frame yet")
        return {"content": [{"type": "image", "data": b64, "mimeType": "image/jpeg"}]}

    @tool("scan", "Sense nearby buildings and drones (distance + world-frame bearing).", {})
    async def scan(args):
        return _ok(ops.scan())

    @tool("run_mission",
          "Author and run your OWN async MAVSDK mission code for a multi-leg or "
          "smooth trajectory. Pre-bound (no import): `drone` (the connected System), "
          "`mission_item(**fields)` (a MissionItem with every field defaulted, "
          "overridable by its real name), `await world_to_geo(east, north, up)` "
          "(-> GeoPoint; use its lat/lon ONLY, set relative_altitude_m to the world "
          "`up`), `await arm_and_start()` (arm + start the uploaded mission, retrying "
          "the transient PX4 DENIED — use it instead of arm()+start_mission()), "
          "`log(msg)`. Import MAVSDK classes you use (e.g. MissionPlan). "
          "Set `timeout` to the seconds you expect; you are uninterruptible until "
          "the mission finishes or the timeout fires (which halts you).",
          {"code": {"type": "string"}, "timeout": {"type": "number"}})
    async def run_mission(args):
        try:
            err, text = await ops.run_mission(args.get("code", ""), args.get("timeout"))
            return _err(text) if err else _ok(text)
        except Exception as e:
            return _err(f"{name} run_mission failed: {e}")

    @tool("track",
          "REAL-TIME PURSUIT of a moving contact (a mov_* from scan): an onboard "
          "10 Hz controller flies the chase for you — far better than chasing "
          "with repeated gotos. mode='shadow' holds station on the moving target "
          "(optional standoff_east/standoff_north offset, metres); "
          "mode='intercept' flies a lead-collision course and returns EARLY the "
          "moment the horizontal gap closes within within_m. The controller "
          "MEASURES the target's velocity and computes the lead itself, so you "
          "do NOT need to scan the target repeatedly first — for a fleeing or "
          "deadline target, take off and track IMMEDIATELY (dithering lets a "
          "fast mover escape). Blocks up to "
          "duration_s (max 120s) and reports min/mean gap, best contiguous dwell "
          "within within_m, and the target's measured velocity. You must be "
          "airborne first (take_off).",
          {"target": {"type": "string"}, "mode": {"type": "string"},
           "alt": {"type": "number"}, "duration_s": {"type": "number"},
           "within_m": {"type": "number"}, "speed": {"type": "number"},
           "standoff_east": {"type": "number"}, "standoff_north": {"type": "number"}})
    async def track(args):
        try:
            return _ok(await ops.track(
                args.get("target", ""), args.get("mode", "shadow"),
                args.get("alt", 12.0), args.get("duration_s", 60.0),
                args.get("within_m", 15.0), args.get("speed", 12.0),
                args.get("standoff_east", 0.0), args.get("standoff_north", 0.0)))
        except Exception as e:
            return _err(f"{name} track failed: {e}")

    server = create_sdk_mcp_server(
        name=f"d{i}", tools=[take_off, fly, goto, orbit, hover, set_speed, face, land,
                             report_tool, look, scan, run_mission, track])
    allowed = [f"mcp__d{i}__take_off", f"mcp__d{i}__fly", f"mcp__d{i}__goto",
               f"mcp__d{i}__orbit", f"mcp__d{i}__hover", f"mcp__d{i}__set_speed",
               f"mcp__d{i}__face", f"mcp__d{i}__land", f"mcp__d{i}__report",
               f"mcp__d{i}__look", f"mcp__d{i}__scan", f"mcp__d{i}__run_mission",
               f"mcp__d{i}__track"]
    return server, allowed


def make_drone_options(i, drone, world, bridge, n, cameras, report, env=None, model=None,
                       gzposes=None):
    name = f"drone_{i}"
    ops = FlightOps(drone, world, bridge, i, n, gzposes=gzposes)
    server, allowed = _drone_server(i, ops, cameras, report)
    return ClaudeAgentOptions(
        mcp_servers={f"d{i}": server},
        allowed_tools=allowed,
        setting_sources=[],
        env=env or {},
        model=model,
        system_prompt=(
            f"You are {name}, an autonomous drone in a swarm of {n}, with your own onboard "
            "thinking. The COMMANDER sends you tasks; you do not hear the other drones. Carry "
            "out each task with your tools, then call report(...) with a short result. Be "
            "terse.\n"
            "MOVE: `goto` (an absolute world point east/north/up OR a named target like 'bldg_7' "
            "or 'drone_1') — it returns once you ARRIVE, so for an ordered route just call it "
            "once per leg, in order; `orbit` (circle a target keeping your camera on it — ONE "
            "call, no need to compute waypoints); `fly` (relative north/east/up, also returns on "
            "arrival); `face` (turn in place to aim your camera); `hover` (hold; seconds=N "
            "holds N seconds — use it for dwell tasks); `set_speed`; "
            "`take_off`; `land`. Pass wait=false to goto/fly if you need to scan/look/report "
            "while moving. Prefer `goto`/`orbit` with named targets and the world coords from "
            "`scan` over hand-computing paths.\n"
            "TRACK: for a MOVING contact (mov_* in scan), `track(target, mode, alt, "
            "duration_s, within_m)` runs an onboard real-time pursuit controller — "
            "mode='shadow' to stay on it (dwell tasks), mode='intercept' to close on it "
            "fast (returns early on contact). One call beats any sequence of gotos at "
            "following a mover; verify its returned gap/dwell numbers against your "
            "task before reporting success.\n"
            "PLAN: when a task carries constraints (no-fly zones, altitude ceilings, distance "
            "or action budgets), write out your full waypoint plan FIRST and check every leg "
            "against every constraint before your first move — a leg that clips a no-fly zone "
            "or busts the budget fails the mission even if you reach the goal.\n"
            "SENSE: `scan` lists nearby buildings + drones with distance and bearing RELATIVE to "
            "where you face — items marked [IN VIEW] are in your camera. `look` returns your live "
            "camera image. Camera is fixed forward (~69deg): to see something not [IN VIEW], `face` "
            "or `orbit` it, then `look`. Use `scan` before moving near obstacles.\n"
            "MISSION: for a smooth or geometry-heavy trajectory (arcs, figure-8s, per-leg "
            "speed/camera control), `run_mission(code, timeout)` "
            "runs your OWN async MAVSDK. Pre-bound (no import): `drone`, "
            "`mission_item(**fields)`, `await world_to_geo(east,north,up)`, "
            "`await arm_and_start()`, `log(msg)`; "
            "import MAVSDK classes (e.g. MissionPlan) yourself. Coords: lat/lon from "
            "`world_to_geo`, set `relative_altitude_m` to the world `up` (world `up` "
            "is height above launch; NOT its absolute altitude). Example:\n"
            "  from mavsdk.mission import MissionPlan\n"
            "  pts = [(0,0,15), (40,0,15), (40,40,15)]\n"
            "  items = []\n"
            "  for e,n_,u in pts:\n"
            "      g = await world_to_geo(east=e, north=n_, up=u)\n"
            "      items.append(mission_item(latitude_deg=g.latitude_deg, "
            "longitude_deg=g.longitude_deg, relative_altitude_m=u, speed_m_s=5, "
            "is_fly_through=True))\n"
            "  await drone.mission.upload_mission(MissionPlan(items))\n"
            "  await arm_and_start()\n"
            "  async for p in drone.mission.mission_progress():\n"
            "      log(f'{p.current}/{p.total}')\n"
            "      if p.current == p.total: break\n"
            "  return 'mission complete'\n"
            "Set `timeout` to the seconds the path needs."),
    )


def make_operator_options(systems, world, bridge, n, cameras, gzposes=None,
                          env=None, model=None):
    """ONE client flying ALL n drones: per-drone namespaces d0..d{n-1} plus a
    fleet server whose goto_all moves drones CONCURRENTLY (sequential blocking
    gotos would serialize the fleet). Returns (options, fleet_ops)."""
    from agents.flight.fleet import FleetOps

    ops_list = [FlightOps(systems[i], world, bridge, i, n, gzposes=gzposes)
                for i in range(n)]
    fleet = FleetOps(ops_list)
    servers, allowed = {}, []
    for i in range(n):
        server, names = _drone_server(i, ops_list[i], cameras,
                                      report=lambda _m: None)
        servers[f"d{i}"] = server
        allowed += names

    @tool("goto_all",
          "Move SEVERAL drones at once: moves=[{drone, east, north, up}, ...]. "
          "Issues every move concurrently and returns when ALL arrive, with a "
          "per-drone result line. This is the primitive for coordinated legs — "
          "one-at-a-time goto calls make the other drones WAIT.",
          {"moves": {"type": "array", "items": {"type": "object", "properties": {
              "drone": {"type": "number"}, "east": {"type": "number"},
              "north": {"type": "number"}, "up": {"type": "number"}}}}})
    async def goto_all(args):
        try:
            return _ok(await fleet.goto_all(args.get("moves", [])))
        except Exception as e:
            return _err(f"goto_all failed: {e}")

    @tool("track_all",
          "Run REAL-TIME PURSUIT on SEVERAL drones at once: tracks=[{drone, "
          "target, mode, alt, duration_s, within_m, speed}, ...] — same "
          "semantics as each drone's `track`, but concurrent (sequential track "
          "calls would leave the other drones parked and blow timing windows). "
          "Returns one summary line per drone.",
          {"tracks": {"type": "array", "items": {"type": "object", "properties": {
              "drone": {"type": "number"}, "target": {"type": "string"},
              "mode": {"type": "string"}, "alt": {"type": "number"},
              "duration_s": {"type": "number"}, "within_m": {"type": "number"},
              "speed": {"type": "number"}}}}})
    async def track_all(args):
        try:
            return _ok(await fleet.track_all(args.get("tracks", [])))
        except Exception as e:
            return _err(f"track_all failed: {e}")

    servers["fleet"] = create_sdk_mcp_server(name="fleet", tools=[goto_all, track_all])
    allowed.append("mcp__fleet__goto_all")
    allowed.append("mcp__fleet__track_all")
    drone_words = ", ".join(f"d{i}" for i in range(n))
    return ClaudeAgentOptions(
        mcp_servers=servers, allowed_tools=allowed, setting_sources=[],
        env=env or {}, model=model,
        system_prompt=(
            f"You are the OPERATOR of a fleet of {n} drones. You fly ALL of "
            f"them yourself: each drone has its own tool namespace ({drone_words} "
            f"— e.g. d1's goto is mcp__d1__goto), and mcp__fleet__goto_all moves "
            "several drones AT ONCE (per-drone goto/fly BLOCK until arrival, so "
            "moving drones one at a time leaves the rest parked — use goto_all "
            "for coordinated legs).\n"
            "mcp__fleet__track_all runs real-time pursuit (shadow/intercept) on several "
            "drones AT ONCE — for simultaneous moving-target work, one track_all call "
            "beats interleaving anything by hand.\n"
            "PLAN: before your first move, assign each drone to its goals "
            "EXPLICITLY (which drone takes which target, at which altitude) and "
            "check the assignment against every constraint — separation minimums, "
            "fleet path budgets, timing windows. Keep your drones apart unless "
            "the task says otherwise; give crossing routes different altitudes.\n"
            "SENSE: each drone's scan/look reports from ITS position (other "
            "drones appear as 'drone_j' contacts). MOVE/MISSION semantics per "
            "drone are identical to a single drone's tools."),
    ), fleet
