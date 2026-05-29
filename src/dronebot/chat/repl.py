# src/dronebot/chat/repl.py
"""Terminal chat loop. Renders agent replies and status; owns the direct,
non-LLM abort path.
"""
from __future__ import annotations

import asyncio

from dronebot.agent.claude_agent import DroneAgent
from dronebot.control.executor import CommandExecutor
from dronebot.flight_log import FlightLog

_ABORT_WORDS = {"stop", "abort", "emergency", "land now"}


async def _read_line(prompt: str) -> str:
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, input, prompt)


def _text_of(message) -> str:
    # Extract printable text from an SDK message; tolerate non-text blocks.
    content = getattr(message, "content", None)
    if not content:
        return ""
    parts = []
    for block in content:
        text = getattr(block, "text", None)
        if text:
            parts.append(text)
    return " ".join(parts)


async def run_repl(agent: DroneAgent, executor: CommandExecutor, log: FlightLog) -> None:
    print("Drone chatbot ready. Type a command, 'status', or 'stop' to abort. Ctrl-D to quit.")
    while True:
        try:
            user = (await _read_line("\nyou> ")).strip()
        except EOFError:
            print("\nshutting down.")
            return

        if not user:
            continue

        if user.lower() in _ABORT_WORDS:
            # DIRECT abort — never routed through the LLM.
            log.record("abort", {"trigger": user})
            await agent.interrupt()
            result = await executor.hold()
            print(f"[ABORT] {result.message}")
            continue

        log.record("utterance", {"text": user})
        print("drone> ", end="", flush=True)
        async for message in agent.ask(user):
            text = _text_of(message)
            if text:
                print(text, end="", flush=True)
        print()
