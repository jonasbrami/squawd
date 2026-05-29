# src/dronebot/agent/tools.py
"""Claude Agent SDK in-process tool adapters over CommandExecutor and
PerceptionStore. Thin: translate, format, and surface errors as is_error.
"""
from __future__ import annotations

import base64

from dronebot.control.executor import CommandExecutor, CommandResult
from dronebot.perception.store import PerceptionStore


def _text(msg: str, is_error: bool = False) -> dict:
    out = {"content": [{"type": "text", "text": msg}]}
    if is_error:
        out["is_error"] = True
    return out


def _from_result(result: CommandResult) -> dict:
    return _text(result.message, is_error=not result.ok)


# Factory functions return bare async handlers (testable without the SDK).
def make_takeoff_tool(executor: CommandExecutor):
    async def handler(args):
        return _from_result(await executor.takeoff(float(args["altitude_m"])))
    return handler


def make_status_tool(executor: CommandExecutor):
    async def handler(args):
        return _from_result(executor.status())
    return handler


def build_flight_server(executor: CommandExecutor, perception: PerceptionStore):
    """Register all tools with the SDK and return the in-process MCP server."""
    from claude_agent_sdk import tool, create_sdk_mcp_server

    @tool("arm", "Arm the drone motors", {})
    async def arm(args):
        return _from_result(await executor.arm())

    @tool("takeoff", "Take off and climb to the given altitude in meters", {"altitude_m": float})
    async def takeoff(args):
        return _from_result(await executor.takeoff(float(args["altitude_m"])))

    @tool("land", "Land the drone at the current position", {})
    async def land(args):
        return _from_result(await executor.land())

    @tool("return_to_launch", "Fly back to the launch point and land", {})
    async def rtl(args):
        return _from_result(await executor.return_to_launch())

    @tool("hold", "Stop and hover in place", {})
    async def hold(args):
        return _from_result(await executor.hold())

    @tool(
        "goto_relative",
        "Move relative to the drone in meters: north/east/up (negatives for south/west/down)",
        {"north_m": float, "east_m": float, "up_m": float},
    )
    async def goto_relative(args):
        return _from_result(await executor.goto_relative(
            float(args["north_m"]), float(args["east_m"]), float(args["up_m"]),
        ))

    @tool("get_status", "Report the drone's current state", {})
    async def get_status(args):
        return _from_result(executor.status())

    @tool("scan_surroundings", "Report nearby obstacles from the depth sensor", {})
    async def scan_surroundings(args):
        return _text(perception.surroundings_summary())

    @tool("look", "Look through the drone camera and return the current image", {})
    async def look(args):
        snap = perception.latest()
        if snap is None or snap.jpeg_frame is None:
            return _text("no camera image available", is_error=True)
        b64 = base64.b64encode(snap.jpeg_frame).decode("ascii")
        return {
            "content": [
                {"type": "text", "text": perception.surroundings_summary()},
                {"type": "image",
                 "source": {"type": "base64", "media_type": "image/jpeg", "data": b64}},
            ]
        }

    return create_sdk_mcp_server(
        name="flight",
        version="1.0.0",
        tools=[arm, takeoff, land, rtl, hold, goto_relative, get_status,
               scan_surroundings, look],
    )


ALLOWED_TOOLS = [
    "mcp__flight__arm", "mcp__flight__takeoff", "mcp__flight__land",
    "mcp__flight__return_to_launch", "mcp__flight__hold",
    "mcp__flight__goto_relative", "mcp__flight__get_status",
    "mcp__flight__scan_surroundings", "mcp__flight__look",
]
