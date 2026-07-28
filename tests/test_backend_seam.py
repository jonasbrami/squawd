"""ICD §11 M6: the backend seam (design §6.5, agents/flight/backend.py).

① the seam emits the normalized typed events (Text / ToolCall / ToolResult /
  Result with usage) — here driven by a FAKE SDK client, no live backend;
② `cli_path` is honored — and REQUIRED — on the Kimi tier (R5: the bundled
  CLI ignores ANTHROPIC_BASE_URL, so a missing external CLI is a hard error,
  never a silent fall-back);
plus the §5.2/§5.3 env recipes as the single source for agents AND evals.
"""
import os
import shutil

import pytest

from agents.flight import FlightOps, make_pilot_options
from agents.flight.backend import (BackendClient, Result, Text, ToolCall,
                                   ToolResult, agent_env, is_kimi_tier,
                                   kimi_recipe, normalize)


# ---------- ① seam emits typed events ----------

def _sdk_messages():
    """Canned SDK messages: text -> tool call -> tool result -> end-of-run."""
    from claude_agent_sdk import (AssistantMessage, ResultMessage, TextBlock,
                                  ToolResultBlock, ToolUseBlock, UserMessage)
    return [
        AssistantMessage(content=[TextBlock(text="Taking off now.")],
                         model="kimi-for-coding"),
        AssistantMessage(content=[ToolUseBlock(id="t1",
                                               name="mcp__pilot__take_off",
                                               input={"altitude": 12})],
                         model="kimi-for-coding"),
        UserMessage(content=[ToolResultBlock(tool_use_id="t1",
                                             content="airborne at 12m",
                                             is_error=False)]),
        ResultMessage(subtype="success", duration_ms=1000, duration_api_ms=800,
                      is_error=False, num_turns=2, session_id="s",
                      total_cost_usd=None,
                      usage={"input_tokens": 1200, "output_tokens": 300}),
    ]


def test_normalize_maps_sdk_messages_to_typed_events():
    evs = [ev for m in _sdk_messages() for ev in normalize(m)]
    assert [type(e) for e in evs] == [Text, ToolCall, ToolResult, Result]
    assert evs[0].text == "Taking off now." and evs[0].model == "kimi-for-coding"
    assert evs[1].name == "mcp__pilot__take_off"
    assert evs[1].input == {"altitude": 12}
    assert evs[2].tool_use_id == "t1" and evs[2].content == "airborne at 12m"
    assert evs[2].is_error is False
    assert evs[3].usage == {"input_tokens": 1200, "output_tokens": 300}
    assert evs[3].num_turns == 2 and evs[3].api_ms == 800
    assert evs[3].cost_usd is None        # subscription tier: no meaningful cost


def test_normalize_skips_blank_text_and_unknown_messages():
    from claude_agent_sdk import AssistantMessage, TextBlock
    assert normalize(AssistantMessage(content=[TextBlock(text="   ")],
                                      model="m")) == []
    assert normalize(object()) == []


class _FakeSDKClient:
    """Duck-type of ClaudeSDKClient: records the prompt, replays canned messages."""

    def __init__(self, msgs):
        self._msgs = msgs
        self.prompt = None

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def query(self, prompt):
        self.prompt = prompt

    async def receive_response(self):
        for m in self._msgs:
            yield m


async def test_backend_client_streams_typed_events_from_fake_sdk():
    fake = _FakeSDKClient(_sdk_messages())
    async with BackendClient(sdk_client=fake) as client:
        evs = [ev async for ev in client.query("take off to 12m")]
    assert fake.prompt == "take off to 12m"
    assert [type(e) for e in evs] == [Text, ToolCall, ToolResult, Result]
    assert evs[-1].usage["output_tokens"] == 300


# ---------- ② cli_path honored / REQUIRED on the Kimi tier ----------

def _make(**kw):
    return make_pilot_options(FlightOps(None, None, None, 0, 1),
                              report=lambda _m: None, **kw)


def test_kimi_tier_detection():
    assert is_kimi_tier("kimi-for-coding", None)
    assert is_kimi_tier("k3", None)
    assert is_kimi_tier(None, kimi_recipe())          # via the recipe's base URL
    assert not is_kimi_tier("claude-opus-4-8", None)
    assert not is_kimi_tier(None, {})
    assert not is_kimi_tier(None, None)


def test_cli_path_honored_on_kimi_tier():
    opts = _make(env=kimi_recipe(), model="kimi-for-coding",
                 cli_path="/usr/local/bin/claude")
    assert opts.cli_path == "/usr/local/bin/claude"


def test_cli_path_auto_resolved_on_kimi_tier(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _name: "/resolved/claude")
    opts = _make(env=kimi_recipe(), model="kimi-for-coding")
    assert opts.cli_path == "/resolved/claude"


def test_cli_path_missing_on_kimi_tier_is_a_legible_hard_error(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _name: None)
    with pytest.raises(RuntimeError, match="external `claude` CLI"):
        _make(env=kimi_recipe(), model="kimi-for-coding")


def test_kimi_detected_from_env_recipe_alone(monkeypatch):
    """A None model does not escape the requirement when the env IS the Kimi
    recipe — the tier is what the options carry, not the ambient shell."""
    monkeypatch.setattr(shutil, "which", lambda _name: None)
    with pytest.raises(RuntimeError):
        _make(env=kimi_recipe(), model=None)


def test_claude_tier_needs_no_cli_path():
    assert _make(env={}, model="claude-opus-4-8").cli_path is None
    assert _make().cli_path is None       # legacy default path unchanged


# ---------- §5.2/§5.3 env recipes ----------

def test_agent_env_kimi_recipe_and_config_dir_isolation(monkeypatch, tmp_path):
    monkeypatch.setenv("SQUAWD_BACKEND", "kimi")
    monkeypatch.setenv("KIMI_API_KEY", "sk-kimi-test")
    monkeypatch.setenv("CLAUDE_CONFIG_DIR", str(tmp_path / ".claude"))
    env = agent_env("pilot")
    assert env["CLAUDE_CONFIG_DIR"] == str(tmp_path / ".claude-pilot")
    assert os.path.isdir(env["CLAUDE_CONFIG_DIR"])      # created, per-agent
    assert env["ANTHROPIC_BASE_URL"] == "https://api.kimi.com/coding/"
    assert env["ANTHROPIC_API_KEY"] == "sk-kimi-test"
    for var in ("ANTHROPIC_MODEL", "ANTHROPIC_DEFAULT_OPUS_MODEL",
                "ANTHROPIC_DEFAULT_SONNET_MODEL", "ANTHROPIC_DEFAULT_HAIKU_MODEL",
                "ANTHROPIC_DEFAULT_FABLE_MODEL", "CLAUDE_CODE_SUBAGENT_MODEL"):
        assert env[var] == "kimi-for-coding"   # every tier var set (silent-fail guard)
    assert env["ENABLE_TOOL_SEARCH"] == "false"
    assert env["CLAUDE_CODE_AUTO_COMPACT_WINDOW"] == "262144"


def test_agent_env_claude_tier_has_no_kimi_vars(monkeypatch, tmp_path):
    monkeypatch.setenv("SQUAWD_BACKEND", "claude")
    monkeypatch.setenv("CLAUDE_CONFIG_DIR", str(tmp_path / ".claude"))
    env = agent_env("pilot")
    assert "ANTHROPIC_BASE_URL" not in env
    assert env["CLAUDE_CONFIG_DIR"].endswith(".claude-pilot")
