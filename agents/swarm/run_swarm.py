"""Interactive swarm: a Commander agent + N drone agents, all on one event loop.

- You type a command in the observatory -> it lands on /swarm/user_input.
- The Commander agent reads it, sees the drones' positions, and BROADCASTS
  per-drone instructions on /swarm/chat ("commander: drone_0 take off and go north, ...").
- Each drone agent runs a continuous react loop: when a NEW relevant message
  appears (from the commander, or addressed to it, or to all), it acts via its
  tools (take_off / fly / say / land).

Scales to N drones (SWARM_N): one persistent Claude client per agent; drones only
spend tokens when a relevant message arrives.
"""
import asyncio
import math
import os
import threading

from std_msgs.msg import String
from mavsdk import System
from mavsdk.action import OrbitYawBehavior
from px4_msgs.msg import VehicleLocalPosition
from claude_agent_sdk import (
    tool, create_sdk_mcp_server, ClaudeAgentOptions, ClaudeSDKClient,
)

from agents.common.bus import RosBridge, CHAT_QOS
from agents.common.geo import GeoPoint, offset_point
from agents.swarm import perception

N = int(os.environ.get("SWARM_N", "3"))

COMPASS = {"north": 0.0, "n": 0.0, "northeast": 45.0, "ne": 45.0, "east": 90.0, "e": 90.0,
           "southeast": 135.0, "se": 135.0, "south": 180.0, "s": 180.0, "southwest": 225.0,
           "sw": 225.0, "west": 270.0, "w": 270.0, "northwest": 315.0, "nw": 315.0}

_lock = threading.Lock()
_chat: list[str] = []
_user: list[str] = []

bridge = RosBridge(node_name="swarm_agents")
look = None  # perception.GzLook, created in main() once N cameras exist


def _on_chat(m):
    with _lock:
        _chat.append(m.data)


def _on_user(m):
    with _lock:
        _user.append(m.data)


def publish_chat(text: str) -> None:
    msg = String()
    msg.data = text
    bridge.publish("/swarm/chat", String, msg, CHAT_QOS)


def positions_text(drones) -> str:
    lines = []
    for i, d in enumerate(drones):
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        if p is None:
            lines.append(f"drone_{i}: (no telemetry)")
        else:
            lines.append(f"drone_{i}: north={p.x:.0f}m east={p.y:.0f}m alt={-p.z:.0f}m")
    return "\n".join(lines)


# ---------- Commander ----------
def make_commander():
    @tool("broadcast", "Broadcast an instruction to the whole swarm over the chat. "
          "Address drones by name, e.g. 'drone_0 take off and go 50m north; drone_1 hold'.",
          {"message": {"type": "string"}})
    async def broadcast(args):
        publish_chat(f"commander: {args.get('message', '')}")
        return {"content": [{"type": "text", "text": "broadcast sent"}]}

    server = create_sdk_mcp_server(name="cmd", tools=[broadcast])
    options = ClaudeAgentOptions(
        mcp_servers={"cmd": server},
        allowed_tools=["mcp__cmd__broadcast"],
        setting_sources=[],
        system_prompt=(
            f"You are the COMMANDER of a swarm of {N} drones (drone_0..drone_{N-1}). "
            "The user gives you high-level commands. Translate each into concrete per-drone "
            "instructions and send them with the broadcast tool, addressing drones by name. "
            "Drones can: take_off; goto an absolute world point (east/north/up m from the "
            "situation map) OR a named target ('bldg_7', 'drone_1'); orbit a target at a radius "
            "keeping camera on it (ONE instruction — don't list waypoints); fly a relative "
            "offset; face/aim camera at a target or compass dir; hover; set_speed; look; scan; "
            "land. Prefer goto/orbit with named targets. To make two drones see each other, tell "
            "each to face or orbit the other then look. Keep instructions short; use the "
            "situation map (positions, facing, obstacles)."),
    )
    return ClaudeSDKClient(options=options)


# ---------- Drone ----------
def make_drone_options(i: int, drone: System):
    name = f"drone_{i}"

    def _alt():
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        return None if p is None else -p.z

    @tool("take_off", "Arm and take off (default 10m). Returns once airborne at altitude.",
          {"altitude": {"type": "number"}})
    async def take_off(args):
        try:
            target = float(args.get("altitude", 10.0))
            await drone.action.arm()
            await drone.action.set_takeoff_altitude(target)
            await drone.action.takeoff()
            for _ in range(20):                      # gate on reaching altitude (safety)
                await asyncio.sleep(1)
                a = _alt()
                if a is not None and a >= target * 0.9:
                    break
            a = _alt()
            return {"content": [{"type": "text", "text": f"{name} airborne at {a:.0f}m" if a else f"{name} airborne"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} takeoff failed: {e}"}], "is_error": True}

    def _keep_yaw():
        st = perception.drone_state(bridge, i)
        return math.degrees(st[3]) if st else 0.0

    @tool("fly", "Fly a relative offset from the current position (metres). Turns to "
          "face the travel direction so your camera looks where you're going.",
          {"north": {"type": "number"}, "east": {"type": "number"}, "up": {"type": "number"}})
    async def fly(args):
        try:
            north, east, up = (float(args.get("north", 0)), float(args.get("east", 0)),
                               float(args.get("up", 0)))
            pos = await anext(drone.telemetry.position())
            origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
            tgt = offset_point(origin, north, east, up)
            yaw = math.degrees(math.atan2(east, north)) if (north or east) else _keep_yaw()
            await drone.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                             tgt.absolute_altitude_m, yaw)
            return {"content": [{"type": "text", "text": f"{name} moving N{north:+.0f} E{east:+.0f} U{up:+.0f}"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} fly failed: {e}"}], "is_error": True}

    async def _world_to_geo(t_e, t_n, t_u):
        """Convert a world ENU point to a GeoPoint, relative to the drone's live GPS fix."""
        me = perception.drone_world_xy(bridge, i)
        pos = await anext(drone.telemetry.position())
        origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
        if me is None:
            return origin
        return offset_point(origin, t_n - me[1], t_e - me[0], t_u - me[2])

    def _resolve_xy(target, args):
        """(east, north) for a symbolic target name, or explicit east/north args, or None."""
        if target:
            return perception.resolve_xy(target, bridge, N)
        if args.get("east") is not None or args.get("north") is not None:
            me = perception.drone_world_xy(bridge, i)
            return (float(args.get("east", me[0] if me else 0.0)),
                    float(args.get("north", me[1] if me else 0.0)))
        return None

    @tool("goto", "Fly to an ABSOLUTE world point (east, north, up=altitude metres) OR a named "
          "target (a drone like 'drone_1', a building like 'bldg_7'). Optional heading: a compass "
          "word ('north'..) or 'travel' (default, face the way you go).",
          {"target": {"type": "string"}, "east": {"type": "number"}, "north": {"type": "number"},
           "up": {"type": "number"}, "heading": {"type": "string"}})
    async def goto(args):
        try:
            me = perception.drone_world_xy(bridge, i)
            target = str(args.get("target", "")).strip().lower()
            xy = _resolve_xy(target, args)
            if xy is None:
                return {"content": [{"type": "text", "text": f"{name}: need a target or east/north"}], "is_error": True}
            t_e, t_n = xy
            t_u = float(args["up"]) if args.get("up") is not None else (me[2] if me else 10.0)
            tgt = await _world_to_geo(t_e, t_n, t_u)
            hh = str(args.get("heading", "travel")).strip().lower()
            if hh in COMPASS:
                yaw = COMPASS[hh]
            elif hh not in ("", "travel") and hh.lstrip("-").replace(".", "", 1).isdigit():
                yaw = float(hh)
            elif me and (abs(t_e - me[0]) > 0.5 or abs(t_n - me[1]) > 0.5):
                yaw = math.degrees(math.atan2(t_e - me[0], t_n - me[1]))   # face travel
            else:
                yaw = _keep_yaw()
            await drone.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                             tgt.absolute_altitude_m, yaw)
            return {"content": [{"type": "text", "text": f"{name} -> E{t_e:.0f} N{t_n:.0f} alt {t_u:.0f}"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} goto failed: {e}"}], "is_error": True}

    @tool("orbit", "Circle a target (a drone, a building like 'bldg_7', or an east/north point) at "
          "a radius, keeping your camera pointed at the center. One call = the whole orbit.",
          {"target": {"type": "string"}, "east": {"type": "number"}, "north": {"type": "number"},
           "radius": {"type": "number"}, "speed": {"type": "number"},
           "direction": {"type": "string"}, "alt": {"type": "number"}})
    async def orbit(args):
        try:
            target = str(args.get("target", "")).strip().lower()
            xy = _resolve_xy(target, args)
            if xy is None:
                return {"content": [{"type": "text", "text": f"{name}: can't resolve orbit center"}], "is_error": True}
            me = perception.drone_world_xy(bridge, i)
            radius = abs(float(args.get("radius", 12.0)))
            speed = abs(float(args.get("speed", 3.0)))
            alt = float(args["alt"]) if args.get("alt") is not None else (me[2] if me else 12.0)
            direction = str(args.get("direction", "cw")).strip().lower()
            # MAVLink DO_ORBIT: direction is the SIGN of radius (+ = clockwise, - = ccw).
            signed = -radius if direction in ("ccw", "counterclockwise", "anticlockwise") else radius
            center = await _world_to_geo(xy[0], xy[1], alt)
            await drone.action.do_orbit(signed, speed, OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
                                        center.latitude_deg, center.longitude_deg,
                                        center.absolute_altitude_m)
            return {"content": [{"type": "text", "text": f"{name} orbiting {target or 'point'} "
                                 f"r={radius:.0f}m {direction} at {alt:.0f}m, camera on center"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} orbit failed: {e}"}], "is_error": True}

    @tool("hover", "Hold current position (loiter in place).", {})
    async def hover(args):
        try:
            await drone.action.hold()
            return {"content": [{"type": "text", "text": f"{name} holding"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} hover failed: {e}"}], "is_error": True}

    @tool("set_speed", "Set cruise speed (m/s) for subsequent moves.", {"speed": {"type": "number"}})
    async def set_speed(args):
        try:
            v = abs(float(args.get("speed", 5.0)))
            await drone.action.set_current_speed(v)
            return {"content": [{"type": "text", "text": f"{name} speed {v:.1f} m/s"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} set_speed failed: {e}"}], "is_error": True}

    @tool("face", "Turn in place to aim your camera at a target: a drone ('drone_1'), a "
          "building ('bldg_7'), or a compass direction ('north'/'east'/'south'/'west').",
          {"target": {"type": "string"}})
    async def face(args):
        try:
            tgt = str(args.get("target", "")).strip().lower()
            compass = {"north": 0.0, "n": 0.0, "northeast": 45.0, "ne": 45.0, "east": 90.0,
                       "e": 90.0, "southeast": 135.0, "se": 135.0, "south": 180.0, "s": 180.0,
                       "southwest": 225.0, "sw": 225.0, "west": 270.0, "w": 270.0,
                       "northwest": 315.0, "nw": 315.0}
            if tgt in compass:
                yaw = compass[tgt]
            else:
                me = perception.drone_world_xy(bridge, i)
                txy = perception.resolve_xy(tgt, bridge, N)
                if me is None or txy is None:
                    return {"content": [{"type": "text", "text": f"{name}: can't resolve target '{tgt}'"}], "is_error": True}
                yaw = perception.yaw_deg_to(me[0], me[1], txy[0], txy[1])
            pos = await anext(drone.telemetry.position())
            await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg,
                                             pos.absolute_altitude_m, yaw)
            return {"content": [{"type": "text", "text": f"{name} turning to face {tgt} (heading {yaw:.0f}deg)"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} face failed: {e}"}], "is_error": True}

    @tool("land", "Land in place. Returns once on the ground.", {})
    async def land(args):
        try:
            await drone.action.land()
            for _ in range(30):                      # confirm touchdown (safety)
                await asyncio.sleep(1)
                a = _alt()
                if a is not None and a < 0.5:
                    break
            return {"content": [{"type": "text", "text": f"{name} landed"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} land failed: {e}"}], "is_error": True}

    @tool("say", "Say something on the swarm chat.", {"message": {"type": "string"}})
    async def say(args):
        publish_chat(f"{name}: {args.get('message', '')}")
        return {"content": [{"type": "text", "text": "sent"}]}

    @tool("look", "See through your onboard camera (returns the current image).", {})
    async def look_tool(args):
        b64 = look.latest_jpeg(i) if look is not None else None
        if b64 is None:
            return {"content": [{"type": "text", "text": f"{name}: no camera frame yet"}],
                    "is_error": True}
        return {"content": [{"type": "image", "data": b64, "mimeType": "image/jpeg"}]}

    @tool("scan", "Sense nearby buildings and drones (distance + world-frame bearing).", {})
    async def scan_tool(args):
        return {"content": [{"type": "text", "text": perception.scan_text(bridge, i, N)}]}

    server = create_sdk_mcp_server(
        name=f"d{i}", tools=[take_off, fly, goto, orbit, hover, set_speed, face, land,
                             say, look_tool, scan_tool])
    options = ClaudeAgentOptions(
        mcp_servers={f"d{i}": server},
        allowed_tools=[f"mcp__d{i}__take_off", f"mcp__d{i}__fly", f"mcp__d{i}__goto",
                       f"mcp__d{i}__orbit", f"mcp__d{i}__hover", f"mcp__d{i}__set_speed",
                       f"mcp__d{i}__face", f"mcp__d{i}__land", f"mcp__d{i}__say",
                       f"mcp__d{i}__look", f"mcp__d{i}__scan"],
        setting_sources=[],
        system_prompt=(
            f"You are {name}, an autonomous drone in a swarm of {N}. You receive swarm-chat "
            "messages. When a message is an instruction for YOU (mentions your name) or for "
            "ALL drones (everyone/all/swarm), carry it out with your tools. If a message is "
            "not for you, do nothing. Be terse; only say() if you have something useful to add.\n"
            "MOVE: `goto` (an absolute world point east/north/up OR a named target like 'bldg_7' "
            "or 'drone_1'); `orbit` (circle a target keeping your camera on it — ONE call, no need "
            "to compute waypoints); `fly` (relative north/east/up); `face` (turn in place to aim "
            "your camera); `hover` (hold); `set_speed`; `take_off`; `land`. Prefer `goto`/`orbit` "
            "with named targets and the world coords from `scan` over hand-computing paths.\n"
            "SENSE: `scan` lists nearby buildings + drones with distance and bearing RELATIVE to "
            "where you face — items marked [IN VIEW] are in your camera. `look` returns your live "
            "camera image. Camera is fixed forward (~69deg): to see something not [IN VIEW], `face` "
            "or `orbit` it, then `look`. Use `scan` before moving near obstacles."),
    )
    return options


def relevant_to(msg: str, i: int) -> bool:
    low = msg.lower()
    if low.startswith(f"drone_{i}:"):  # own message
        return False
    return (low.startswith("commander:") or f"drone_{i}" in low
            or "everyone" in low or " all " in f" {low} " or "swarm" in low)


async def drone_loop(i: int, client: ClaudeSDKClient):
    seen = 0
    async with client:
        while True:
            await asyncio.sleep(1.5)
            with _lock:
                new = _chat[seen:]
                seen = len(_chat)
            rel = [m for m in new if relevant_to(m, i)]
            if not rel:
                continue
            await client.query("New swarm chat:\n" + "\n".join(rel) +
                               "\nAct on anything addressed to you or to all drones; else do nothing.")
            async for _ in client.receive_response():
                pass


async def commander_loop(client: ClaudeSDKClient, drones):
    seen = 0
    async with client:
        while True:
            await asyncio.sleep(1.0)
            with _lock:
                new = _user[seen:]
                seen = len(_user)
            for cmd in new:
                await client.query(
                    f"User command: {cmd}\n\nSwarm situation (positions + nearest buildings, "
                    f"world frame ENU):\n{perception.situation_text(bridge, N)}\n\n"
                    "Broadcast concrete per-drone instructions now. Route drones around "
                    "buildings when relevant.")
                async for _ in client.receive_response():
                    pass


async def main():
    bridge.subscribe("/swarm/chat", String, CHAT_QOS, _on_chat)
    bridge.subscribe("/swarm/user_input", String, CHAT_QOS, _on_user)
    for i in range(N):
        bridge.subscribe(f"/px4_{i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
    bridge.start()

    drones = []
    for i in range(N):
        d = System(mavsdk_server_address="127.0.0.1", port=50051 + i)
        await d.connect()
        async for s in d.core.connection_state():
            if s.is_connected:
                break
        drones.append(d)
        print(f"drone_{i} connected", flush=True)

    # Reuse PX4's own geofence as the hard safety layer (autopilot-enforced even on
    # link loss) rather than custom Python bounds-checks. Warning action so it never
    # disrupts the demo; raise GF_ACTION later to actually contain drones.
    for idx, d in enumerate(drones):
        try:
            await d.param.set_param_float("GF_MAX_HOR_DIST", 300.0)
            await d.param.set_param_float("GF_MAX_VER_DIST", 80.0)
            await d.param.set_param_int("GF_ACTION", 1)
        except Exception as e:
            print(f"geofence setup skipped for drone_{idx}: {e}", flush=True)

    global look
    look = perception.GzLook(N)            # start reading each drone's camera off gz
    print(f"perception: {len(perception.load_boxes().get('buildings', []))} buildings loaded; "
          f"cameras subscribed for {N} drones.", flush=True)

    commander = make_commander()
    drone_clients = [ClaudeSDKClient(options=make_drone_options(i, drones[i])) for i in range(N)]
    print(f"swarm online: commander + {N} drones. Waiting for commands on /swarm/user_input.", flush=True)
    await asyncio.gather(
        commander_loop(commander, drones),
        *[drone_loop(i, drone_clients[i]) for i in range(N)],
    )


if __name__ == "__main__":
    asyncio.run(main())
