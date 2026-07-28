"""Backend seam (design §6.5) — THE one module coupled to claude-agent-sdk's
client + message types. Everything downstream (the pilot loop, the eval
runner's Trace.observe) consumes the NORMALIZED typed event stream defined
here — `Text / ToolCall / ToolResult / Result(usage)` — never SDK imports. A
future backend swap to the designated fallback harness (kimi-agent-sdk,
Apache-2.0, https://github.com/MoonshotAI/kimi-agent-sdk — pivot triggers in
§6.5) touches THIS file only; the MCP tool binding in
`agents/flight/tools.py` is the other half of the seam (`make_pilot_options`).

ToS confirmation (owner, documented at M6, 2026-07-22): the Kimi Code
subscription is gated by terms AND a UA whitelist limited to Kimi CLI /
Claude Code / Roo Code
(https://www.kimi.com/help/kimi-code/third-party-agents); hand-rolled
third-party clients get `403 access_terminated_error`
(https://github.com/kodustech/kodus-ai/issues/1257). This project drives the
OFFICIAL Claude Code CLI through claude-agent-sdk — the tolerated
integration path under those terms — and the owner has confirmed that
non-coding drone-control usage of the subscription is accepted for this
project (design §5.2 terms check, §6.5). Hand-rolling an OpenAI/kosong loop
against the subscription is NOT an option (pay-as-you-go api.moonshot.ai is
a different product, unaffected).

Risk R5 (design §8): the SDK's BUNDLED CLI ignores ANTHROPIC_BASE_URL
(anthropics/claude-agent-sdk-python#677, confirmed live in locked 0.2.107),
so on the Kimi tier `cli_path=shutil.which("claude")` is REQUIRED — enforced
in `make_pilot_options` (agents/flight/tools.py), not left to callers.
"""
import os
import shutil
from collections.abc import AsyncIterator
from dataclasses import dataclass
from typing import Any

from claude_agent_sdk import (AssistantMessage, ClaudeSDKClient, ResultMessage,
                              TextBlock, ToolResultBlock, ToolUseBlock,
                              UserMessage)

KIMI_BASE_URL = "https://api.kimi.com/coding/"
KIMI_MODELS = frozenset({"kimi-for-coding", "k3"})


# ---- normalized typed event stream (design §6.5) ----

@dataclass
class Text:
    """Agent prose between tool calls (blank text is dropped by normalize)."""
    text: str
    model: str | None = None


@dataclass
class ToolCall:
    """One tool invocation (SDK ToolUseBlock on an assistant turn)."""
    id: str
    name: str
    input: dict
    model: str | None = None


@dataclass
class ToolResult:
    """The result paired to a ToolCall by `tool_use_id`."""
    tool_use_id: str
    content: Any
    is_error: bool = False


@dataclass
class Result:
    """End-of-run usage/cost. On a non-Anthropic backend `cost_usd` is
    meaningless (null/zero/a Claude-price estimate) — evals report quota
    metrics (request count, tokens, latency) per §5.5 instead."""
    usage: dict | None = None
    cost_usd: float | None = None
    num_turns: int | None = None
    api_ms: int | None = None


Event = Text | ToolCall | ToolResult | Result


def normalize(msg) -> list[Event]:
    """One SDK message -> zero or more typed events. Unknown message/block
    kinds (e.g. thinking blocks) yield nothing — the stream stays exactly the
    four contract types."""
    if isinstance(msg, AssistantMessage):
        out: list[Event] = []
        for blk in msg.content:
            if isinstance(blk, ToolUseBlock):
                out.append(ToolCall(id=blk.id, name=blk.name, input=blk.input,
                                    model=msg.model))
            elif isinstance(blk, TextBlock) and blk.text.strip():
                out.append(Text(text=blk.text, model=msg.model))
        return out
    if isinstance(msg, UserMessage) and isinstance(msg.content, list):
        return [ToolResult(tool_use_id=b.tool_use_id, content=b.content,
                           is_error=bool(b.is_error))
                for b in msg.content if isinstance(b, ToolResultBlock)]
    if isinstance(msg, ResultMessage):
        return [Result(usage=msg.usage, cost_usd=msg.total_cost_usd,
                       num_turns=msg.num_turns, api_ms=msg.duration_api_ms)]
    return []


class BackendClient:
    """The one session wrapper: async-context manager + `query(prompt)` ->
    async stream of typed Events. Duck-compatible with the eval harness's
    ScriptedClient (evals/pilot.py), which emits the same Event types.

    `sdk_client` injects a duck-typed SDK client (tests / a future harness
    swap); production passes options and gets a ClaudeSDKClient."""

    def __init__(self, options=None, *, sdk_client=None) -> None:
        self.options = options     # exposed for introspection (tests, transcripts)
        self._client = sdk_client if sdk_client is not None \
            else ClaudeSDKClient(options=options)

    async def __aenter__(self):
        await self._client.__aenter__()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return await self._client.__aexit__(exc_type, exc, tb)

    async def query(self, prompt: str) -> AsyncIterator[Event]:
        await self._client.query(prompt)
        async for msg in self._client.receive_response():
            for ev in normalize(msg):
                yield ev


# ---- env recipes (design §5.2/§5.3) — the single source for agents AND evals

def kimi_recipe() -> dict:
    """The §5.2 Kimi Code subscription recipe. Every tier var must be set or
    background CLI features fail SILENTLY (S0-verified, M0). Pure: no
    filesystem side effects, safe for the eval harness."""
    return {
        "ANTHROPIC_BASE_URL": KIMI_BASE_URL,
        "ANTHROPIC_API_KEY": os.environ.get("KIMI_API_KEY", ""),
        "ANTHROPIC_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_OPUS_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_SONNET_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_HAIKU_MODEL": "kimi-for-coding",
        "ANTHROPIC_DEFAULT_FABLE_MODEL": "kimi-for-coding",
        "CLAUDE_CODE_SUBAGENT_MODEL": "kimi-for-coding",
        "ENABLE_TOOL_SEARCH": "false",          # endpoint lacks tool search
        "CLAUDE_CODE_AUTO_COMPACT_WINDOW": "262144",
    }


def is_kimi_tier(model: str | None, env: dict | None) -> bool:
    """Tier detection for the cli_path requirement: a Kimi model name OR the
    §5.2 recipe's base URL in the per-agent env. Ambient SQUAWD_BACKEND is
    deliberately NOT read here — the tier is what the options carry."""
    if model in KIMI_MODELS:
        return True
    return "kimi" in (env or {}).get("ANTHROPIC_BASE_URL", "").lower()


def agent_env(tag: str, backend: str | None = None) -> dict:
    """Per-agent CLAUDE_CONFIG_DIR isolation (a sibling of the base config
    dir, with its OAuth credentials copied over — from swarm/run.py:30-44) +
    the Kimi recipe when SQUAWD_BACKEND=kimi (or `backend` says so)."""
    backend = backend or os.environ.get("SQUAWD_BACKEND", "claude")
    base = os.environ.get("CLAUDE_CONFIG_DIR", "/root/.claude")
    d = os.path.join(os.path.dirname(base.rstrip("/")) or "/", f".claude-{tag}")
    os.makedirs(d, exist_ok=True)
    try:
        shutil.copy(os.path.join(base, ".credentials.json"),
                    os.path.join(d, ".credentials.json"))
    except Exception:
        pass
    env = {"CLAUDE_CONFIG_DIR": d}
    if backend == "kimi":
        env.update(kimi_recipe())
    return env
