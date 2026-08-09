"""Authenticated, explicitly-budgeted Codex subscription/MCP spike.

Run only when deliberately spending two Codex turns:

    SQUAWD_RUN_CODEX_SPIKE=1 uv run pytest \
      tests/integration/test_codex_sdk.py -q
"""
import os
from pathlib import Path

import pytest

from agents.flight.backend import Result, ToolCall, ToolResult
from agents.flight.codex_backend import CodexBackendClient
from agents.flight.tools import ToolSpec

pytestmark = pytest.mark.skipif(
    os.environ.get("SQUAWD_RUN_CODEX_SPIKE") != "1",
    reason="authenticated Codex spike requires an explicit two-turn budget")


async def _echo(arguments):
    text = arguments["text"]
    return {"content": [{"type": "text", "text": f"echo:{text}"}]}


ECHO = ToolSpec(
    name="echo",
    description="Echo the supplied text exactly once.",
    input_schema={
        "type": "object",
        "properties": {"text": {"type": "string"}},
        "required": ["text"],
        "additionalProperties": False,
    },
    handler=_echo,
)


async def test_authenticated_echo_two_turns_usage_and_same_thread(tmp_path):
    auth = Path(os.environ.get("CODEX_HOME", Path.home() / ".codex")) / "auth.json"
    if not auth.is_file():
        pytest.skip("run `codex login` first")
    client = CodexBackendClient(
        [ECHO],
        system_prompt=(
            "You are an MCP echo verifier. For every user message, call the "
            "echo tool exactly once with the entire user message, then answer "
            "with the single word echoed."),
        codex_home=str(auth.parent),
        workdir=str(tmp_path),
    )
    async with client:
        thread_id = client._thread.id
        first = [event async for event in client.query("first-turn")]
        second = [event async for event in client.query("second-turn")]
        assert client._thread.id == thread_id

    for expected, events in (("first-turn", first), ("second-turn", second)):
        calls = [event for event in events if isinstance(event, ToolCall)]
        results = [event for event in events if isinstance(event, ToolResult)]
        final = next(event for event in events if isinstance(event, Result))
        assert len(calls) == 1 and calls[0].name == "mcp__pilot__echo"
        assert calls[0].input == {"text": expected}
        assert len(results) == 1 and f"echo:{expected}" in str(results[0].content)
        assert final.usage and final.usage["input_tokens"] > 0
        assert final.usage["output_tokens"] > 0
        assert not final.is_error


async def test_required_mcp_fails_closed_before_a_turn(tmp_path):
    class UnavailableMCP:
        def __init__(self, _tools):
            self.url = "http://127.0.0.1:9/mcp"
            self.token = "unused"

        async def __aenter__(self):
            return self

        async def __aexit__(self, *_exc):
            return False

    auth = Path(os.environ.get("CODEX_HOME", Path.home() / ".codex")) / "auth.json"
    if not auth.is_file():
        pytest.skip("run `codex login` first")
    client = CodexBackendClient(
        [ECHO], system_prompt="echo only", codex_home=str(auth.parent),
        workdir=str(tmp_path), mcp_server_factory=UnavailableMCP)
    with pytest.raises(Exception, match="(?i)mcp|initialize|connection"):
        await client.__aenter__()
