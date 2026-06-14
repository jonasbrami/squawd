"""Swarm demo: N drones, each a Claude agent, coordinating over /swarm/chat to
cover distinct sectors. One process, one asyncio loop, one shared RosBridge.

Each agent has tools take_off / say / fly_to. They are started staggered so each
reads the prior claims in the chat and picks a still-free sector.
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
CHAT_TOPIC = "/swarm/chat"
SECTORS = {"north": (60.0, 0.0), "south": (-60.0, 0.0), "east": (0.0, 60.0)}

_chat_lock = threading.Lock()
_chat_log: list[str] = []


def _on_chat(msg) -> None:
    with _chat_lock:
        _chat_log.append(msg.data)


def chat_text() -> str:
    with _chat_lock:
        return "\n".join(_chat_log) if _chat_log else "(no messages yet)"


bridge = RosBridge()


def make_agent(i: int, drone: System):
    name = f"drone_{i}"

    @tool("take_off", "Arm and take off to 10m.", {})
    async def take_off(args):
        try:
            await drone.action.arm()
            await drone.action.set_takeoff_altitude(10.0)
            await drone.action.takeoff()
            await asyncio.sleep(8)
            return {"content": [{"type": "text", "text": f"{name} airborne (~10m)"}]}
        except Exception as e:  # SITL EKF may reject arming early; report, don't crash
            return {"content": [{"type": "text", "text": f"{name} takeoff failed: {e}"}],
                    "is_error": True}

    @tool("say", "Broadcast a short message to the swarm chat (every drone sees it).",
          {"message": {"type": "string"}})
    async def say(args):
        msg = String()
        msg.data = f"{name}: {args.get('message', '')}"
        bridge.publish(CHAT_TOPIC, String, msg, CHAT_QOS)
        return {"content": [{"type": "text", "text": "sent"}]}

    @tool("fly_to", "Fly to a sector: one of north, south, east.",
          {"sector": {"type": "string"}})
    async def fly_to(args):
        sector = args.get("sector", "").lower().strip()
        if sector not in SECTORS:
            return {"content": [{"type": "text", "text": f"unknown sector '{sector}'"}],
                    "is_error": True}
        try:
            north_m, east_m = SECTORS[sector]
            pos = await anext(drone.telemetry.position())
            origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
            tgt = offset_point(origin, north_m, east_m, 0.0)
            await drone.action.goto_location(
                tgt.latitude_deg, tgt.longitude_deg, tgt.absolute_altitude_m, 0.0)
            return {"content": [{"type": "text", "text": f"{name} heading {sector}"}]}
        except Exception as e:
            return {"content": [{"type": "text", "text": f"{name} fly_to failed: {e}"}],
                    "is_error": True}

    server = create_sdk_mcp_server(name=f"flight_{i}", tools=[take_off, say, fly_to])
    options = ClaudeAgentOptions(
        mcp_servers={f"flight_{i}": server},
        allowed_tools=[f"mcp__flight_{i}__take_off",
                       f"mcp__flight_{i}__say",
                       f"mcp__flight_{i}__fly_to"],
        setting_sources=[],
        system_prompt=(
            f"You are {name}, one of {N} autonomous drones in a swarm. The sectors to "
            f"divide among the drones are: north, south, east. Coordinate over the shared "
            f"chat so that each drone covers a DIFFERENT sector. Never claim a sector a "
            f"peer already claimed in the chat."),
    )
    return name, options


async def run_agent(i: int, drone: System, stagger: float) -> None:
    name, options = make_agent(i, drone)
    await asyncio.sleep(stagger)
    async with ClaudeSDKClient(options=options) as client:
        await client.query(
            f"Take off. Here is the swarm chat so far:\n{chat_text()}\n\n"
            f"Claim a sector that no peer has claimed yet, announce your claim with say(), "
            f"then fly_to that sector.")
        async for _ in client.receive_response():
            pass
    print(f"{name}: mission turn complete", flush=True)


async def main() -> None:
    bridge.subscribe(CHAT_TOPIC, String, CHAT_QOS, _on_chat)
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

    await asyncio.gather(*[run_agent(i, drones[i], i * 12.0) for i in range(N)])
    print("=== swarm chat transcript ===", flush=True)
    print(chat_text(), flush=True)
    await asyncio.sleep(15)  # let goto_location settle
    bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
