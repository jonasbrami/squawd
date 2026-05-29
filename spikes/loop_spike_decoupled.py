# spikes/loop_spike_decoupled.py
"""Decoupled GO/NO-GO: validate the in-process @tool + single-shared-loop
mechanic WITHOUT a sim, using the Claude Code OAuth session (no API key).

This substitutes a fake async "drone" for MAVSDK. It proves the highest-risk
unknown: that a Claude Agent SDK in-process @tool handler can `await` a
coroutine from a separate async object on the same loop ClaudeSDKClient drives,
and that the SDK authenticates via the local `claude` CLI OAuth.

The remaining piece (real MAVSDK System() against PX4 SITL) is verified by
spikes/loop_spike.py once the simulator is up.
"""
import asyncio

from claude_agent_sdk import (
    tool,
    create_sdk_mcp_server,
    ClaudeAgentOptions,
    ClaudeSDKClient,
)


class FakeDrone:
    """Stand-in for a MAVSDK System: an async object whose coroutines must run
    on the same event loop the SDK drives."""

    async def get_altitude(self) -> float:
        await asyncio.sleep(0.05)  # mimic async I/O (gRPC/MAVLink would go here)
        return 12.34


_drone = FakeDrone()
_tool_ran = {"ok": False, "value": None}


@tool("get_altitude", "Get the drone's current relative altitude in meters", {})
async def get_altitude(args):
    # The critical line: await another async object's coroutine inside the
    # SDK's in-process tool handler, on the SDK's loop.
    alt = await _drone.get_altitude()
    _tool_ran["ok"] = True
    _tool_ran["value"] = alt
    return {"content": [{"type": "text", "text": f"relative altitude: {alt:.2f} m"}]}


async def main() -> None:
    server = create_sdk_mcp_server(name="flight", version="0.0.1", tools=[get_altitude])
    options = ClaudeAgentOptions(
        mcp_servers={"flight": server},
        allowed_tools=["mcp__flight__get_altitude"],
        system_prompt=(
            "You control a drone. When asked the altitude, call the "
            "get_altitude tool, then report the value to the user."
        ),
    )

    async with ClaudeSDKClient(options=options) as client:
        await client.query("What is the drone's current altitude? Use the tool.")
        async for message in client.receive_response():
            print(f"[{type(message).__name__}] {message}")

    if _tool_ran["ok"]:
        print(f"\nDECOUPLED SPIKE PASS: in-process tool awaited a coroutine on "
              f"the SDK loop (got {_tool_ran['value']} m) via OAuth")
    else:
        print("\nDECOUPLED SPIKE INCONCLUSIVE: the tool handler never ran")


if __name__ == "__main__":
    asyncio.run(main())
