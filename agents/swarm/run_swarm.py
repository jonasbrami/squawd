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
import os
import threading

from std_msgs.msg import String
from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition
from claude_agent_sdk import (
    tool, create_sdk_mcp_server, ClaudeAgentOptions, ClaudeSDKClient,
)

from agents.common.bus import RosBridge, CHAT_QOS
from agents.common.geo import GeoPoint, offset_point

N = int(os.environ.get("SWARM_N", "3"))

_lock = threading.Lock()
_chat: list[str] = []
_user: list[str] = []

bridge = RosBridge(node_name="swarm_agents")


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
            "Drones can: take off, fly a relative offset (north/east/up metres), hold, land. "
            "Keep instructions short and unambiguous. Use the drones' reported positions to "
            "decide. One broadcast per user command is usually enough."),
    )
    return ClaudeSDKClient(options=options)


# ---------- Drone ----------
def make_drone_options(i: int, drone: System):
    name = f"drone_{i}"

    @tool("take_off", "Arm and take off to 10m.", {})
    async def take_off(args):
        try:
            await drone.action.arm()
            await drone.action.set_takeoff_altitude(10.0)
            await drone.action.takeoff()
            await asyncio.sleep(8)
            return {"content": [{"type": "text", "text": f"{name} airborne"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} takeoff failed: {e}"}], "is_error": True}

    @tool("fly", "Fly a relative offset from the current position (metres).",
          {"north": {"type": "number"}, "east": {"type": "number"}, "up": {"type": "number"}})
    async def fly(args):
        try:
            pos = await anext(drone.telemetry.position())
            origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
            tgt = offset_point(origin, float(args.get("north", 0)), float(args.get("east", 0)),
                               float(args.get("up", 0)))
            await drone.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                             tgt.absolute_altitude_m, 0.0)
            return {"content": [{"type": "text", "text": f"{name} moving"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} fly failed: {e}"}], "is_error": True}

    @tool("land", "Land in place.", {})
    async def land(args):
        try:
            await drone.action.land()
            return {"content": [{"type": "text", "text": f"{name} landing"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} land failed: {e}"}], "is_error": True}

    @tool("say", "Say something on the swarm chat.", {"message": {"type": "string"}})
    async def say(args):
        publish_chat(f"{name}: {args.get('message', '')}")
        return {"content": [{"type": "text", "text": "sent"}]}

    server = create_sdk_mcp_server(name=f"d{i}", tools=[take_off, fly, land, say])
    options = ClaudeAgentOptions(
        mcp_servers={f"d{i}": server},
        allowed_tools=[f"mcp__d{i}__take_off", f"mcp__d{i}__fly",
                       f"mcp__d{i}__land", f"mcp__d{i}__say"],
        setting_sources=[],
        system_prompt=(
            f"You are {name}, an autonomous drone in a swarm of {N}. You receive swarm-chat "
            "messages. When a message is an instruction for YOU (mentions your name) or for "
            "ALL drones (everyone/all/swarm), carry it out with your tools. If a message is "
            "not for you, do nothing. Be terse; only say() if you have something useful to add."),
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
                    f"User command: {cmd}\n\nDrone positions:\n{positions_text(drones)}\n\n"
                    "Broadcast concrete per-drone instructions now.")
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

    commander = make_commander()
    drone_clients = [ClaudeSDKClient(options=make_drone_options(i, drones[i])) for i in range(N)]
    print(f"swarm online: commander + {N} drones. Waiting for commands on /swarm/user_input.", flush=True)
    await asyncio.gather(
        commander_loop(commander, drones),
        *[drone_loop(i, drone_clients[i]) for i in range(N)],
    )


if __name__ == "__main__":
    asyncio.run(main())
