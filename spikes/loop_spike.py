# spikes/loop_spike.py
"""GO/NO-GO: prove a MAVSDK coroutine can be awaited from inside a
Claude Agent SDK in-process @tool, on a single shared event loop.

Requires: sim running + ANTHROPIC_API_KEY in environment.
"""
import asyncio

from mavsdk import System
from claude_agent_sdk import (
    tool,
    create_sdk_mcp_server,
    ClaudeAgentOptions,
    ClaudeSDKClient,
)

# Connected once, on the loop that ClaudeSDKClient will drive.
_drone = System()


@tool("get_altitude", "Get the drone's current relative altitude in meters", {})
async def get_altitude(args):
    async for position in _drone.telemetry.position():
        alt = position.relative_altitude_m
        return {"content": [{"type": "text", "text": f"relative altitude: {alt:.2f} m"}]}
    return {"content": [{"type": "text", "text": "no telemetry"}], "is_error": True}


async def main() -> None:
    await _drone.connect(system_address="udp://:14540")
    async for state in _drone.core.connection_state():
        if state.is_connected:
            break

    server = create_sdk_mcp_server(name="flight", version="0.0.1", tools=[get_altitude])
    options = ClaudeAgentOptions(
        mcp_servers={"flight": server},
        allowed_tools=["mcp__flight__get_altitude"],
        system_prompt="You control a drone. When asked the altitude, call the tool.",
    )

    async with ClaudeSDKClient(options=options) as client:
        await client.query("What is the drone's current altitude?")
        async for message in client.receive_response():
            print(message)

    print("SPIKE B PASS: MAVSDK coroutine ran inside a Claude SDK tool on one loop")


if __name__ == "__main__":
    asyncio.run(main())
