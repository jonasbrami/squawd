"""Agent definitions + react loops for the swarm (commander-driven, distributed hub).

- make_commander: the Commander agent. One `dispatch(drone_id, task)` tool that
  sends a directed task to a single drone over /swarm/cmd/<i>.
- commander_loop: poll /swarm/user_input AND every /swarm/report/<i>. A user
  command -> the Commander decomposes it into per-drone dispatches (with the live
  situation map). A drone's report -> the Commander decides any follow-up.
- drone_loop: poll this drone's own /swarm/cmd/<i>; on a new task, act via tools
  and call report(...) to send the result back to the Commander.

Each drone is its own agent (its own ClaudeSDKClient + tools) and talks to the
Commander only over ROS topics, so a drone can run on separate hardware unchanged.
The wiring (bridge, world, cameras, clients, publishers) is assembled in run.py.
"""
import asyncio

from claude_agent_sdk import (
    tool, create_sdk_mcp_server, ClaudeAgentOptions, ClaudeSDKClient,
)

from agents import perception


def make_commander(n: int, dispatch, env=None) -> ClaudeSDKClient:
    @tool("dispatch", "Send a task to ONE drone by index. The drone carries it out "
          "with its own onboard agent and reports back. Call once per drone you want "
          "to task (e.g. four calls to move four drones).",
          {"drone_id": {"type": "number"}, "task": {"type": "string"}})
    async def dispatch_tool(args):
        try:
            i = int(args.get("drone_id"))
        except (TypeError, ValueError):
            return {"content": [{"type": "text", "text": "drone_id must be a number"}],
                    "is_error": True}
        if not 0 <= i < n:
            return {"content": [{"type": "text", "text": f"no drone_{i} (have drone_0..drone_{n-1})"}],
                    "is_error": True}
        dispatch(i, args.get("task", ""))
        return {"content": [{"type": "text", "text": f"dispatched to drone_{i}"}]}

    server = create_sdk_mcp_server(name="cmd", tools=[dispatch_tool])
    options = ClaudeAgentOptions(
        mcp_servers={"cmd": server},
        allowed_tools=["mcp__cmd__dispatch"],
        setting_sources=[],
        env=env or {},
        system_prompt=(
            f"You are the COMMANDER of a swarm of {n} drones (drone_0..drone_{n-1}). "
            "You are the only one who talks to the human and the only one who tasks "
            "drones; the drones do not hear each other. Translate each human command "
            "into concrete per-drone tasks and send each with the dispatch tool "
            "(drone_id + a short natural-language task). Each drone can: take_off; goto "
            "an absolute world point (east/north/up m from the situation map) OR a named "
            "target ('bldg_7', 'drone_1'); orbit a target at a radius keeping its camera "
            "on it (ONE task — don't list waypoints); fly a relative offset; face/aim at a "
            "target or compass dir; hover; set_speed; look; scan; land; then it reports "
            "back. Prefer goto/orbit with named targets; use the situation map (positions, "
            "facing, obstacles). To make two drones see each other, task each to face or "
            "orbit the other then look. When a drone REPORTS, read it and only dispatch a "
            "follow-up if the goal is not yet met; otherwise just summarize for the human "
            "and wait — do not re-task drones that are already done. Keep tasks short."),
    )
    return ClaudeSDKClient(options=options)


async def drone_loop(i: int, client: ClaudeSDKClient, cmd) -> None:
    """Act only when the Commander tasks THIS drone on /swarm/cmd/<i>."""
    seen = 0
    async with client:
        while True:
            await asyncio.sleep(1.0)
            new, seen = cmd.since(seen)
            for task in new:
                await client.query(
                    f"Task from commander: {task}\n\nCarry it out with your tools, then "
                    "call report(...) with a short result (what you did and what you saw).")
                async for _ in client.receive_response():
                    pass


async def commander_loop(client: ClaudeSDKClient, user, reports, world, bridge, n: int) -> None:
    """Service human commands and drone reports on one Commander client.

    `reports` is a list of N TopicLogs, one per /swarm/report/<i>. A new user
    command triggers fresh dispatches; a new report lets the Commander react.
    """
    user_seen = 0
    report_seen = [0] * n
    async with client:
        while True:
            await asyncio.sleep(1.0)
            new_user, user_seen = user.since(user_seen)
            for cmd in new_user:
                await client.query(
                    f"User command: {cmd}\n\nSwarm situation (positions + nearest buildings, "
                    f"world frame ENU):\n{perception.situation_text(world, bridge, n)}\n\n"
                    "Dispatch a concrete task to each drone that should act now. Route "
                    "drones around buildings when relevant.")
                async for _ in client.receive_response():
                    pass
            for i in range(n):
                new_rep, report_seen[i] = reports[i].since(report_seen[i])
                for rep in new_rep:
                    await client.query(
                        f"drone_{i} reports: {rep}\n\nDecide if any follow-up is needed. "
                        "Dispatch more tasks only if the goal isn't met yet; otherwise just "
                        "note it for the human and take no action.")
                    async for _ in client.receive_response():
                        pass
