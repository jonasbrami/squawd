"""Provider seam for Claude, Kimi, and Codex backends.

Everything downstream (the pilot loop and eval Trace.observe) consumes the
normalized event stream defined here — ``Text / ToolCall / ToolResult /
Result`` — and never imports a provider SDK.  Claude and the Anthropic-
compatible Kimi route use ``BackendClient`` below.  The factory at the end of
this module selects the Codex adapter in ``codex_backend.py`` when requested.
The provider-neutral tool catalog lives in ``agents/flight/tools.py``.

§5.5 quota instrumentation (M6): `BackendClient.query` measures what the
subscription backend actually costs — top-level query count, exact inference
requests (assistant turns; `num_turns` stays the fallback proxy), 429/quota
error classification (`is_quota_error`), and ttfa/gap_p50/wall timings —
stamped per run on the Result event, accumulated on the client.

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
import re
import shutil
import statistics
import time
from collections.abc import AsyncIterator
from dataclasses import dataclass
from typing import Any

from claude_agent_sdk import (AssistantMessage, ClaudeSDKClient, ResultMessage,
                              TextBlock, ToolResultBlock, ToolUseBlock,
                              UserMessage)

KIMI_BASE_URL = "https://api.kimi.com/coding/"
# All model IDs the subscription endpoint serves (GET /v1/models, verified
# 2026-08-02): K2.7 Code standard/highspeed + K3 1M/256k.
KIMI_MODELS = frozenset({"kimi-for-coding", "kimi-for-coding-highspeed",
                         "k3", "k3-256k"})
VALID_BACKENDS = frozenset({"claude", "kimi", "codex"})
BACKEND_MODEL_DEFAULTS = {
    # Preserve the pre-switch Claude/Kimi effective defaults. Claude's None
    # delegates to the installed Claude Code default; Kimi's recipe pins the
    # compatible subscription route. Codex is explicit and fail-closed.
    "claude": None,
    "kimi": "kimi-for-coding",
    "codex": "gpt-5.6-terra",
}


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
    """End-of-run usage/cost + the §5.5 quota instrumentation. On a
    non-Anthropic backend `cost_usd` is meaningless (null/zero/a Claude-price
    estimate) — evals report quota metrics (request count, tokens, latency)
    per §5.5 instead. Fields:
      usage              — input/output/cache token counts verbatim from the
                           SDK (Kimi populates them, M0); explicitly None when
                           the backend reports nothing (recorded as null)
      num_turns          — the SDK-reported turn count; the FALLBACK proxy for
                           inference requests
      inference_requests — the EXACT inference-request count (assistant turns
                           observed on the stream; stamped by BackendClient)
      is_error           — the SDK's end-of-run error status
      quota_errors       — API/quota rejections (429 / quota-exhausted /
                           rate-limit) classified during this run
      ttfa_s/gap_p50_s/wall_ms — stream timings measured at the seam (time to
                           first activity, median inter-message gap, total
                           wall time); api_ms stays the SDK's API latency"""
    usage: dict | None = None
    cost_usd: float | None = None
    num_turns: int | None = None
    api_ms: int | None = None
    is_error: bool = False
    inference_requests: int | None = None
    quota_errors: int = 0
    ttfa_s: float | None = None
    gap_p50_s: float | None = None
    wall_ms: int | None = None


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
        return [Result(usage=msg.usage if msg.usage else None,
                       cost_usd=msg.total_cost_usd,
                       num_turns=msg.num_turns, api_ms=msg.duration_api_ms,
                       is_error=bool(msg.is_error))]
    return []


# ---- §5.5 quota-error classification ----

_QUOTA_RE = re.compile(
    r"\b429\b|quota|rate.?limit|too many requests|insufficient.{0,16}credit",
    re.IGNORECASE)


def is_quota_error(text: str | None) -> bool:
    """429 / quota-exhausted / rate-limit phrasing in SDK-surfaced error text
    (assistant error messages, error subtypes, raised exceptions) — the
    subscription backend's real failure mode, counted per §5.5."""
    return bool(text) and bool(_QUOTA_RE.search(text))


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
        # §5.5 cumulative quota counters across all queries on this client
        # (the per-run values ride each Result event).
        self.queries = 0
        self.inference_requests = 0
        self.quota_errors = 0

    async def __aenter__(self):
        await self._client.__aenter__()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return await self._client.__aexit__(exc_type, exc, tb)

    async def query(self, prompt: str) -> AsyncIterator[Event]:
        """One top-level query. Measures the §5.5 run metrics at the seam —
        exact inference-request count (assistant turns), quota-error
        classification, ttfa/gap_p50/wall timings — and stamps them on the
        run's Result event; the client counters accumulate across queries.
        A raised stream exception is still classified (429/quota) before
        re-raising, so quota failures are counted even without a Result."""
        self.queries += 1
        t0 = time.monotonic()
        arrivals: list[float] = []
        n_infer = 0
        n_quota = 0
        await self._client.query(prompt)
        try:
            async for msg in self._client.receive_response():
                arrivals.append(time.monotonic())
                if isinstance(msg, AssistantMessage):
                    n_infer += 1
                    if any(isinstance(b, TextBlock) and is_quota_error(b.text)
                           for b in msg.content):
                        n_quota += 1
                elif isinstance(msg, ResultMessage) and msg.is_error \
                        and is_quota_error(getattr(msg, "subtype", None)):
                    n_quota += 1
                for ev in normalize(msg):
                    if isinstance(ev, Result):
                        gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
                        ev.inference_requests = n_infer
                        ev.quota_errors = n_quota
                        ev.ttfa_s = round(arrivals[0] - t0, 3)
                        ev.gap_p50_s = round(statistics.median(gaps), 3) \
                            if gaps else 0.0
                        ev.wall_ms = int((arrivals[-1] - t0) * 1000)
                    yield ev
        except Exception as e:
            if is_quota_error(str(e)):
                n_quota += 1
            raise
        finally:
            self.inference_requests += n_infer
            self.quota_errors += n_quota


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
        # K3 minimal thinking for fast pilot reactions (owner directive
        # 2026-08-02). Kimi maps Claude Code effort low->low; do NOT disable
        # thinking outright — the endpoint routes that to K2.6, not K3
        # (kimi.com/code/docs third-party-agents). Override via env for evals.
        "CLAUDE_CODE_EFFORT_LEVEL": os.environ.get("SQUAWD_KIMI_EFFORT", "low"),
    }


def is_kimi_tier(model: str | None, env: dict | None) -> bool:
    """Tier detection for the cli_path requirement: a Kimi model name OR the
    §5.2 recipe's base URL in the per-agent env. Ambient SQUAWD_BACKEND is
    deliberately NOT read here — the tier is what the options carry."""
    if model in KIMI_MODELS:
        return True
    return "kimi" in (env or {}).get("ANTHROPIC_BASE_URL", "").lower()


def agent_env(tag: str, backend: str | None = None) -> dict:
    """Return the selected provider's isolated runtime environment.

    Claude gets a per-agent config directory, Kimi adds its endpoint recipe,
    and Codex receives only the launcher's isolated ``CODEX_HOME``.
    """
    backend = resolve_backend(backend)
    if backend == "codex":
        # Codex credentials/config are isolated by CODEX_HOME and the launcher;
        # never copy Claude state into that directory.
        return {"CODEX_HOME": os.environ.get(
            "CODEX_HOME", os.path.expanduser("~/.codex"))}
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


def resolve_backend(value: str | None = None) -> str:
    """Validate the provider selector without silently falling back."""
    backend = (value or os.environ.get("SQUAWD_BACKEND", "claude")).strip().lower()
    if backend not in VALID_BACKENDS:
        allowed = "|".join(sorted(VALID_BACKENDS))
        raise ValueError(f"invalid SQUAWD_BACKEND={backend!r}; expected {allowed}")
    return backend


def resolve_model(backend: str, value: str | None = None) -> str | None:
    """Return an explicit override or the provider's compatibility default."""
    backend = resolve_backend(backend)
    if value is None:
        value = os.environ.get("SQUAWD_MODEL")
    return value or BACKEND_MODEL_DEFAULTS[backend]


def make_backend_client(
        ops, *, report, registry=None, detect_text=None, deep_tools=None,
        guard=None, backend: str | None = None, env=None, model=None,
        cli_path=None, codex_effort: str | None = None,
        codex_home: str | None = None, codex_workdir: str | None = None):
    """Build the selected provider behind the common async client contract."""
    from agents.flight.tools import (PILOT_SYSTEM_PROMPT, make_pilot_options,
                                     make_pilot_tools)

    selected = resolve_backend(backend)
    selected_model = resolve_model(selected, model)
    if selected == "codex":
        from agents.flight.codex_backend import CodexBackendClient

        specs = make_pilot_tools(
            ops, detect_text=detect_text, deep_tools=deep_tools, report=report,
            registry=registry, guard=guard)
        return CodexBackendClient(
            specs,
            system_prompt=PILOT_SYSTEM_PROMPT,
            model=selected_model or BACKEND_MODEL_DEFAULTS["codex"],
            effort=(codex_effort or os.environ.get(
                "SQUAWD_CODEX_EFFORT", "low")),
            codex_home=(codex_home or (env or {}).get("CODEX_HOME")),
            workdir=codex_workdir,
        )

    options = make_pilot_options(
        ops, detect_text=detect_text, deep_tools=deep_tools, report=report,
        registry=registry, guard=guard, env=env,
        model=selected_model, cli_path=cli_path)
    return BackendClient(options)
