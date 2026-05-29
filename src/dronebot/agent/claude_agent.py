# src/dronebot/agent/claude_agent.py
"""Claude Agent SDK wiring. Owns the ClaudeSDKClient lifecycle and exposes a
minimal interface to the REPL: ask(text) streams a reply; interrupt() aborts
the current turn (the hard safety abort is separate and bypasses this).
"""
from __future__ import annotations

import os
from typing import AsyncIterator

from claude_agent_sdk import ClaudeAgentOptions, ClaudeSDKClient

from dronebot.agent.prompts import SYSTEM_PROMPT
from dronebot.agent.tools import build_flight_server, ALLOWED_TOOLS
from dronebot.control.executor import CommandExecutor
from dronebot.perception.store import PerceptionStore


class DroneAgent:
    def __init__(self, executor: CommandExecutor, perception: PerceptionStore, model: str) -> None:
        server = build_flight_server(executor, perception)
        os.makedirs("/tmp/dronebot-agent", exist_ok=True)
        self._options = ClaudeAgentOptions(
            model=model,
            system_prompt=SYSTEM_PROMPT,
            mcp_servers={"flight": server},
            allowed_tools=ALLOWED_TOOLS,
            setting_sources=[],          # do NOT load user/project/local settings
            cwd="/tmp/dronebot-agent",   # clean cwd: no project CLAUDE.md / hooks
        )
        self._client: ClaudeSDKClient | None = None

    async def __aenter__(self) -> "DroneAgent":
        self._client = ClaudeSDKClient(options=self._options)
        await self._client.__aenter__()
        return self

    async def __aexit__(self, *exc) -> None:
        assert self._client is not None
        await self._client.__aexit__(*exc)

    async def ask(self, text: str) -> AsyncIterator:
        assert self._client is not None
        await self._client.query(text)
        async for message in self._client.receive_response():
            yield message

    async def interrupt(self) -> None:
        if self._client is not None:
            await self._client.interrupt()
