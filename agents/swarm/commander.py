"""CommanderAgent: the one agent that talks to the human and tasks the drones.

The Commander is the hub: it reads human commands on /swarm/user_input, dispatches
directed tasks to individual drones on /swarm/cmd/drone_<i>, and reacts to each
drone's report on /swarm/report/drone_<i>. The drones do not hear each other; only
the Commander sees the whole picture (the situation map) and decides follow-ups.

`make_commander` builds the Commander's Claude client (its sole tool is `dispatch`).
`CommanderAgent` owns the user/report channels, the dispatch publisher, and the loop.
"""
import asyncio

from std_msgs.msg import String
from claude_agent_sdk import (tool, create_sdk_mcp_server, ClaudeAgentOptions,
                              ClaudeSDKClient, AssistantMessage, TextBlock, ThinkingBlock)

from agents import perception
from agents.core.bus import CHAT_QOS, publish_str
from agents.core.store import TopicLog


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


class CommanderAgent:
    """The swarm hub: services human commands and drone reports on one Claude client."""

    def __init__(self, n: int, bridge, world, env=None) -> None:
        self.n = n
        self._bridge = bridge
        self._world = world
        self._user = TopicLog(bridge, "/swarm/user_input", String, CHAT_QOS)
        self._reports = [TopicLog(bridge, f"/swarm/report/drone_{i}", String, CHAT_QOS)
                         for i in range(n)]
        self.client = make_commander(n, self.dispatch, env)
        self._user_seen = 0
        self._report_seen = [0] * n

    def dispatch(self, i: int, task: str) -> None:
        """Send a directed task to drone_i (+ mirror onto /swarm/chat for the UI)."""
        publish_str(self._bridge, f"/swarm/cmd/drone_{i}", task)
        publish_str(self._bridge, "/swarm/chat", f"commander→drone_{i}: {task}")

    async def _drain(self) -> None:
        """Consume one Commander turn, mirroring its thinking/replies onto /swarm/chat.

        The Commander's only tool is dispatch (already mirrored), but its
        natural-language reasoning + answers to the human would otherwise be
        discarded. Surface every text/thinking block as `commander: ...` so the
        human sees the Commander's thinking and its summaries/answers in the UI."""
        async for msg in self.client.receive_response():
            if not isinstance(msg, AssistantMessage):
                continue
            for blk in msg.content:
                text = blk.text if isinstance(blk, TextBlock) else (
                    blk.thinking if isinstance(blk, ThinkingBlock) else None)
                if text and text.strip():
                    publish_str(self._bridge, "/swarm/chat", f"commander: {text.strip()}")

    def _skip_replay_backlog(self) -> None:
        """Advance all cursors past whatever is already buffered.

        /swarm/user_input and /swarm/report/* use TRANSIENT_LOCAL QoS, so on
        subscribe ROS REPLAYS the retained history (up to depth=100) to this fresh
        Commander. Without this, a (re)started Commander re-executes every past
        command and report — re-dispatching the whole session. Call this once, after
        the retained burst has been delivered, so the Commander acts only on
        commands/reports that arrive AFTER it comes online."""
        _, self._user_seen = self._user.since(self._user_seen)
        for i in range(self.n):
            _, self._report_seen[i] = self._reports[i].since(self._report_seen[i])

    async def run(self) -> None:
        """Poll /swarm/user_input AND every /swarm/report/drone_<i>.

        A new user command triggers fresh dispatches (with the live situation map);
        a new report lets the Commander decide any follow-up."""
        async with self.client:
            await asyncio.sleep(1.0)        # let the TRANSIENT_LOCAL backlog arrive,
            self._skip_replay_backlog()     # then ignore it — only act on new traffic
            while True:
                await asyncio.sleep(1.0)
                new_user, self._user_seen = self._user.since(self._user_seen)
                for cmd in new_user:
                    await self.client.query(
                        f"User command: {cmd}\n\nSwarm situation (positions + nearest "
                        f"buildings, world frame ENU):\n"
                        f"{perception.situation_text(self._world, self._bridge, self.n)}\n\n"
                        "Dispatch a concrete task to each drone that should act now. Route "
                        "drones around buildings when relevant.")
                    await self._drain()
                for i in range(self.n):
                    new_rep, self._report_seen[i] = self._reports[i].since(self._report_seen[i])
                    for rep in new_rep:
                        await self.client.query(
                            f"drone_{i} reports: {rep}\n\nDecide if any follow-up is needed. "
                            "Dispatch more tasks only if the goal isn't met yet; otherwise just "
                            "note it for the human and take no action.")
                        await self._drain()
