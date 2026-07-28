"""M0 spike — fallback harness smoke: kimi-agent-sdk (evidence for spec §6.5).

Creates a Session with a one-tool agent file and drives one tool-requiring
prompt, verifying: session creation on the subscription key, tool-call wire
events, and TokenUsage presence (what our eval Trace would consume if we ever
pivot to this harness).

Run: /tmp/kimi-sdk-venv/bin/python spikes/kimi_agent_sdk_check.py
"""
import asyncio
import json
import os
import sys
from pathlib import Path

from dotenv import load_dotenv

load_dotenv()
if not os.environ.get("KIMI_API_KEY"):
    sys.exit("KIMI_API_KEY not set")

from kaos.path import KaosPath
from kimi_agent_sdk import Session, StatusUpdate, TokenUsage, ToolCallPart, ToolResult  # noqa
from kimi_cli.config import Config, LLMModel, LLMProvider
from pydantic import SecretStr

SMOKE_DIR = Path(__file__).parent / "kimi_sdk_smoke"
KEY = os.environ["KIMI_API_KEY"]


def kimi_config() -> Config:
    """Subscription lane proven by probe: Anthropic-compatible /coding/ + key."""
    return Config(
        providers={"kimi-sub": LLMProvider(
            type="anthropic",
            base_url="https://api.kimi.com/coding/",
            api_key=SecretStr(KEY),
        )},
        models={"kimi-for-coding": LLMModel(
            provider="kimi-sub", model="kimi-for-coding",
            max_context_size=262_144)},
        default_model="kimi-for-coding",
    )


async def main() -> int:
    sys.path.insert(0, str(SMOKE_DIR))
    events, texts, usages, errors = [], [], [], []
    session = None
    try:
        session = await Session.create(
            work_dir=KaosPath(str(SMOKE_DIR)),
            agent_file=SMOKE_DIR / "agent.yaml",
            config=kimi_config(),
            yolo=True,
        )
        async for msg in session.prompt("Use plus to add 17 and 25, then tell me the sum."):
            t = type(msg).__name__
            events.append(t)
            if isinstance(msg, TokenUsage):
                usages.append(str(msg))
            elif isinstance(msg, StatusUpdate) and getattr(msg, "token_usage", None):
                usages.append(f"StatusUpdate.token_usage={msg.token_usage} "
                              f"context_usage={msg.context_usage}")
            elif isinstance(msg, (ToolCallPart, ToolResult)):
                pass
            text = getattr(msg, "text", None) or getattr(msg, "content", None)
            if isinstance(text, str) and text.strip():
                texts.append(text.strip())
    except Exception as e:
        errors.append(f"{type(e).__name__}: {e}")
    finally:
        if session is not None:
            try:
                await session.close()
            except Exception:
                pass

    used_tool = any("Tool" in e for e in events)
    out = {
        "session_create": "PASS" if session and not errors else f"FAIL ({errors[:1]})",
        "tool_wire_events": "PASS" if used_tool else f"FAIL (events: {sorted(set(events))})",
        "token_usage_present": "PASS" if usages else "FAIL (no TokenUsage events)",
        "event_types": sorted(set(events)),
        "usages": usages[-1:],
        "reply": (texts[-1][:160] if texts else None),
        "errors": errors,
    }
    print(json.dumps(out, indent=1))
    return 0 if all(str(v).startswith("PASS") for k, v in out.items()
                    if k in ("session_create", "tool_wire_events",
                             "token_usage_present")) else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
