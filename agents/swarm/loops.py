"""Agent definitions + react loops for the swarm.

- make_commander: the Commander agent (one `broadcast` tool).
- commander_loop: poll /swarm/user_input, inject the situation map, let the
  Commander translate each user command into per-drone broadcasts.
- drone_loop: poll /swarm/chat; when a NEW relevant message appears, the drone
  agent acts via its tools.

The wiring (bridge, world, cameras, clients) is assembled in run.py.
"""
import asyncio

from claude_agent_sdk import (
    tool, create_sdk_mcp_server, ClaudeAgentOptions, ClaudeSDKClient,
)

from agents import perception


def make_commander(n: int, publish_chat, env=None) -> ClaudeSDKClient:
    @tool("broadcast", "Broadcast an instruction to the whole swarm over the chat. "
          "Address drones by name, e.g. 'drone_0 take off and go north, drone_1 hold'.",
          {"message": {"type": "string"}})
    async def broadcast(args):
        publish_chat(f"commander: {args.get('message', '')}")
        return {"content": [{"type": "text", "text": "broadcast sent"}]}

    server = create_sdk_mcp_server(name="cmd", tools=[broadcast])
    options = ClaudeAgentOptions(
        mcp_servers={"cmd": server},
        allowed_tools=["mcp__cmd__broadcast"],
        setting_sources=[],
        env=env or {},
        system_prompt=(
            f"You are the COMMANDER of a swarm of {n} drones (drone_0..drone_{n-1}). "
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


def relevant_to(msg: str, i: int) -> bool:
    low = msg.lower()
    if low.startswith(f"drone_{i}:"):  # own message
        return False
    return (low.startswith("commander:") or f"drone_{i}" in low
            or "everyone" in low or " all " in f" {low} " or "swarm" in low)


async def drone_loop(i: int, client: ClaudeSDKClient, chat) -> None:
    seen = 0
    async with client:
        while True:
            await asyncio.sleep(1.5)
            new, seen = chat.since(seen)
            rel = [m for m in new if relevant_to(m, i)]
            if not rel:
                continue
            await client.query("New swarm chat:\n" + "\n".join(rel) +
                               "\nAct on anything addressed to you or to all drones; else do nothing.")
            async for _ in client.receive_response():
                pass


async def commander_loop(client: ClaudeSDKClient, user, world, bridge, n: int) -> None:
    seen = 0
    async with client:
        while True:
            await asyncio.sleep(1.0)
            new, seen = user.since(seen)
            for cmd in new:
                await client.query(
                    f"User command: {cmd}\n\nSwarm situation (positions + nearest buildings, "
                    f"world frame ENU):\n{perception.situation_text(world, bridge, n)}\n\n"
                    "Broadcast concrete per-drone instructions now. Route drones around "
                    "buildings when relevant.")
                async for _ in client.receive_response():
                    pass
