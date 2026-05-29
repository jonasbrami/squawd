# src/dronebot/app.py
"""Terminal entrypoint. Owns the single asyncio loop; reuses the shared stack."""
from __future__ import annotations

import asyncio

from dotenv import load_dotenv

from dronebot.chat.repl import run_repl
from dronebot.config import load_config
from dronebot.stack import build_stack, start_stack, stop_stack


async def main() -> None:
    load_dotenv()
    stack = build_stack(load_config())
    print(f"connecting to {stack.config.connection_url} ...")
    await start_stack(stack)
    print("connected.")
    try:
        async with stack.agent:
            await run_repl(stack.agent, stack.executor, stack.log)
    finally:
        await stop_stack(stack)


if __name__ == "__main__":
    asyncio.run(main())
