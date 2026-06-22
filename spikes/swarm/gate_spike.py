"""GO/NO-GO GATE: Claude Agent SDK in-process tool awaits MAVSDK AND reads rclpy,
all on one event loop in one process, against PX4 SITL.

Bounded so it cannot hang. Uses the standalone mavsdk_server (survives the SDK
forking the claude CLI). OAuth via mounted /root/.claude; setting_sources=[]
keeps the SDK from inheriting host Claude Code settings/hooks.
"""
import asyncio

from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition
from claude_agent_sdk import (
    tool, create_sdk_mcp_server, ClaudeAgentOptions, ClaudeSDKClient,
)

from agents.core.bus import RosBridge

TOPIC = "/fmu/out/vehicle_local_position"

bridge = RosBridge()
drone = System(mavsdk_server_address="127.0.0.1", port=50051)


@tool("takeoff_and_report", "Arm, take off to 5m, and report altitude from ROS2 telemetry.", {})
async def takeoff_and_report(args):
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    await asyncio.sleep(8)  # let it climb
    ros_msg = bridge.latest(TOPIC)
    alt = None if ros_msg is None else -ros_msg.z
    text = (f"Took off. ROS2-reported altitude: {alt:.2f} m" if alt is not None
            else "Took off, but no ROS2 telemetry received.")
    return {"content": [{"type": "text", "text": text}]}


async def run_agent() -> None:
    server = create_sdk_mcp_server(name="flight", tools=[takeoff_and_report])
    options = ClaudeAgentOptions(
        mcp_servers={"flight": server},
        allowed_tools=["mcp__flight__takeoff_and_report"],
        setting_sources=[],
    )
    async with ClaudeSDKClient(options=options) as client:
        await client.query("Take off and tell me the drone's altitude.")
        async for msg in client.receive_response():
            print(msg, flush=True)


async def main() -> None:
    bridge.subscribe(TOPIC, VehicleLocalPosition)
    bridge.start()
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("MAVSDK connected to PX4", flush=True)
            break
    try:
        await asyncio.wait_for(run_agent(), timeout=150)
        print("GATE: agent run completed", flush=True)
    except asyncio.TimeoutError:
        print("GATE: TIMEOUT (agent did not finish in 150s)", flush=True)
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
