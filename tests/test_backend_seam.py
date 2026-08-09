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
                                   is_quota_error, kimi_recipe, normalize,
                                   resolve_backend, resolve_model)


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


# ---------- §5.5 quota instrumentation ----------

def test_quota_error_classification():
    assert is_quota_error('API Error: 429 {"error": {"type": "rate_limit_error"}}')
    assert is_quota_error("error_during_execution: quota exhausted")
    assert is_quota_error("429 Too Many Requests")
    assert not is_quota_error("Failed to authenticate. API Error: 401")
    assert not is_quota_error("airborne at 12m")
    assert not is_quota_error(None) and not is_quota_error("")


async def test_result_event_carries_exact_requests_timings_and_usage():
    """§5.5: the seam stamps the run's Result with the exact inference count
    (assistant turns), ttfa/gap_p50/wall measured at the seam, and passes the
    Kimi-shaped usage dict (M0: input/output + cache fields) through whole."""
    usage = {"input_tokens": 602, "output_tokens": 211,
             "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0,
             "service_tier": "standard"}
    from claude_agent_sdk import ResultMessage
    msgs = _sdk_messages()[:-1] + [
        ResultMessage(subtype="success", duration_ms=1000, duration_api_ms=800,
                      is_error=False, num_turns=2, session_id="s",
                      total_cost_usd=None, usage=usage)]
    async with BackendClient(sdk_client=_FakeSDKClient(msgs)) as client:
        evs = [ev async for ev in client.query("q")]
    res = evs[-1]
    assert res.usage == usage               # input/output/cache verbatim
    assert res.num_turns == 2               # fallback proxy preserved
    assert res.inference_requests == 2      # exact: two assistant turns
    assert res.is_error is False and res.quota_errors == 0
    assert res.ttfa_s is not None and res.ttfa_s >= 0
    assert res.gap_p50_s is not None and res.gap_p50_s >= 0
    assert res.wall_ms is not None and res.wall_ms >= 0
    assert client.queries == 1
    assert client.inference_requests == 2 and client.quota_errors == 0


async def test_usage_absent_records_null_explicitly():
    from claude_agent_sdk import ResultMessage
    msgs = [ResultMessage(subtype="success", duration_ms=1, duration_api_ms=1,
                          is_error=False, num_turns=1, session_id="s",
                          total_cost_usd=None, usage=None)]
    async with BackendClient(sdk_client=_FakeSDKClient(msgs)) as client:
        evs = [ev async for ev in client.query("q")]
    assert evs[-1].usage is None            # recorded as null, not fabricated
    assert evs[-1].inference_requests == 0  # exact count independent of usage


async def test_quota_error_stream_is_classified_and_counted():
    """A 429/quota rejection surfaced as assistant text + an error Result is
    classified and counted on BOTH the Result event and the client."""
    from claude_agent_sdk import (AssistantMessage, ResultMessage, TextBlock)
    msgs = [
        AssistantMessage(content=[TextBlock(
            text='API Error: 429 {"error": {"type": "rate_limit_error", '
                 '"message": "quota exhausted"}}')], model="<synthetic>"),
        ResultMessage(subtype="error_during_execution", duration_ms=100,
                      duration_api_ms=50, is_error=True, num_turns=1,
                      session_id="s", total_cost_usd=None, usage=None),
    ]
    async with BackendClient(sdk_client=_FakeSDKClient(msgs)) as client:
        evs = [ev async for ev in client.query("q")]
    res = evs[-1]
    assert isinstance(res, Result)
    assert res.is_error is True
    assert res.quota_errors == 1            # the 429 assistant turn, once
    assert res.inference_requests == 1
    assert client.quota_errors == 1


class _RaisingSDKClient(_FakeSDKClient):
    async def receive_response(self):
        raise RuntimeError("HTTP 429: quota exhausted")
        yield                                     # pragma: no cover


async def test_quota_exception_mid_stream_is_counted_and_reraised():
    """A quota failure that KILLS the stream (no Result event at all) is still
    classified on the client's counter before propagating."""
    async with BackendClient(sdk_client=_RaisingSDKClient([])) as client:
        with pytest.raises(RuntimeError, match="429"):
            _ = [ev async for ev in client.query("q")]
    assert client.quota_errors == 1


async def test_non_quota_exception_is_not_counted():
    class Boom(_FakeSDKClient):
        async def receive_response(self):
            raise ConnectionError("socket closed")
            yield                                 # pragma: no cover

    async with BackendClient(sdk_client=Boom([])) as client:
        with pytest.raises(ConnectionError):
            _ = [ev async for ev in client.query("q")]
    assert client.quota_errors == 0


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


def test_backend_selection_and_model_defaults(monkeypatch):
    monkeypatch.delenv("SQUAWD_MODEL", raising=False)
    assert resolve_backend("codex") == "codex"
    assert resolve_backend("KIMI") == "kimi"
    assert resolve_model("codex") == "gpt-5.6-terra"
    assert resolve_model("kimi") == "kimi-for-coding"
    assert resolve_model("claude") is None
    assert resolve_model("codex", "gpt-custom") == "gpt-custom"
    with pytest.raises(ValueError, match="invalid SQUAWD_BACKEND"):
        resolve_backend("responses")
