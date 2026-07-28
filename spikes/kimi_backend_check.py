"""M0 spike — Kimi subscription backend through claude-agent-sdk (sim-free).

S0 checks (design spec §5.6 + M0 gate):
  1. auth + real destination — query completes on the Kimi endpoint (path probed
     separately: /coding/ + /v1/messages = 200; key is pay-per-sub, NOT moonshot.ai)
  2. multi-turn MCP tool calling — dummy in-process server, two sequential tools
  3. tier-var sanity — no silent background-feature failures with the full recipe
Plus: record what ResultMessage.usage / total_cost_usd / modelUsage contain on
the Kimi endpoint (§5.5 metrics depend on it), and confirm cli_path works (R5).

Run: python3 spikes/kimi_backend_check.py   (reads KIMI_API_KEY from .env)
"""
import asyncio
import json
import os
import shutil
import sys

from dotenv import load_dotenv

load_dotenv()
KEY = os.environ.get("KIMI_API_KEY")
if not KEY:
    sys.exit("KIMI_API_KEY not set (put it in .env)")

from claude_agent_sdk import (ClaudeAgentOptions, ClaudeSDKClient, AssistantMessage,
                              ResultMessage, TextBlock, ToolUseBlock, tool,
                              create_sdk_mcp_server)


@tool("add_numbers", "Add two numbers and return the sum.",
      {"a": {"type": "number"}, "b": {"type": "number"}})
async def add_numbers(args):
    return {"content": [{"type": "text", "text": str(args["a"] + args["b"])}]}


@tool("get_time", "Return the current sim-free fake time.", {})
async def get_time(args):
    return {"content": [{"type": "text", "text": "12:00:00"}]}


def kimi_env() -> dict:
    cfg = "/tmp/m0-kimi-config"
    os.makedirs(cfg, exist_ok=True)
    return {
        "CLAUDE_CONFIG_DIR": cfg,
        "ANTHROPIC_BASE_URL": "https://api.kimi.com/coding/",
        "ANTHROPIC_API_KEY": KEY,
        "ANTHROPIC_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_OPUS_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_SONNET_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_HAIKU_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_FABLE_MODEL": "kimi-for-coding",
        "CLAUDE_CODE_SUBAGENT_MODEL": "kimi-for-coding",
        "ENABLE_TOOL_SEARCH": "false",
        "CLAUDE_CODE_AUTO_COMPACT_WINDOW": "262144",
    }


async def main() -> int:
    server = create_sdk_mcp_server(name="m0", tools=[add_numbers, get_time])
    allowed = ["mcp__m0__add_numbers", "mcp__m0__get_time"]
    opts = ClaudeAgentOptions(
        mcp_servers={"m0": server}, allowed_tools=allowed, tools=[],
        setting_sources=[], env=kimi_env(), model="kimi-for-coding",
        cli_path=shutil.which("claude"),
        system_prompt="You are a tool-use checker. Be terse.",
    )
    tool_calls, texts, result, errors = [], [], None, []
    try:
        async with ClaudeSDKClient(options=opts) as client:
            await client.query(
                "Call add_numbers with a=17 and b=25, then call get_time, "
                "then reply with both results in one line.")
            async for msg in client.receive_response():
                if isinstance(msg, AssistantMessage):
                    for b in msg.content:
                        if isinstance(b, ToolUseBlock):
                            tool_calls.append(b.name)
                        elif isinstance(b, TextBlock):
                            texts.append(b.text)
                elif isinstance(msg, ResultMessage):
                    result = msg
    except Exception as e:
        errors.append(f"{type(e).__name__}: {e}")

    ok_tools = ({"mcp__m0__add_numbers", "mcp__m0__get_time"}
                <= set(tool_calls))
    out = {
        "check1_auth_destination": "PASS" if (result or texts) and not errors else "FAIL",
        "check2_multi_turn_mcp": "PASS" if ok_tools else f"FAIL (calls: {tool_calls})",
        "check3_tier_vars": "PASS" if not errors else f"FAIL ({errors[:1]})",
        "tool_calls": tool_calls,
        "reply": (texts[-1][:200] if texts else None),
        "result_fields": (None if result is None else {
            "num_turns": result.num_turns,
            "total_cost_usd": result.total_cost_usd,
            "usage": result.usage,
            "duration_api_ms": result.duration_api_ms,
        }),
        "errors": errors,
    }
    print(json.dumps(out, indent=1, default=str))
    return 0 if all(str(v).startswith("PASS") for k, v in out.items()
                    if k.startswith("check")) else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
