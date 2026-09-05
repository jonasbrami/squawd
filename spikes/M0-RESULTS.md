# M0 — Kimi backend spike (S0) — RESULTS: **PASS** (2026-07-20)

Gate per design spec §5.6/§7: all S0 checks pass, plus endpoint path, usage
fields, cli_path, and the fallback-harness smoke documented. **Verdict: backend
decision confirmed — claude-agent-sdk primary (Option A), kimi-agent-sdk as the
designated, now evidence-based, fallback. Proceed to M1.**

## 1. Endpoint probes (curl, no SDK)

| Probe | Result |
|---|---|
| `POST https://api.kimi.com/coding/v1/messages` (x-api-key) | **HTTP 200** — Anthropic-compatible, key works |
| `POST https://api.kimi.com/coding/v1/chat/completions` (Bearer) | **HTTP 200** — the endpoint is DUAL-PROTOCOL (OpenAI-compatible too) |
| `POST https://api.kimi.com/coding/messages` | HTTP 404 — the `/v1` prefix is required (client libraries append it) |
| `POST https://api.moonshot.ai/anthropic/v1/messages` | HTTP 401 — subscription `sk-kimi-` keys are NOT pay-as-you-go keys (confirms §5.2) |

Conclusion: `ANTHROPIC_BASE_URL=https://api.kimi.com/coding/` is correct; the
client appends `/v1/messages`. Auth via `x-api-key` (the `ANTHROPIC_API_KEY`
lane from the official Kimi Code third-party recipe) works.

## 2. S0 via claude-agent-sdk (`spikes/kimi_backend_check.py`, exit 0)

Env per spec §5.2 (full tier-var recipe, `ENABLE_TOOL_SEARCH=false`,
`CLAUDE_CODE_AUTO_COMPACT_WINDOW=262144`, isolated `CLAUDE_CONFIG_DIR`),
`model="kimi-for-coding"`, **`cli_path=shutil.which("claude")`** (R5 — used as
required; the bundled-CLI path was not exercised).

| Check | Result |
|---|---|
| 1. auth + real destination | **PASS** — query completed on the Kimi endpoint |
| 2. multi-turn MCP tool calling | **PASS** — dummy in-process server, `add_numbers` → `get_time`, 3 turns, both `ToolUseBlock`s observed |
| 3. tier-var sanity | **PASS** — no silent background-feature failures |

`ResultMessage` on the Kimi endpoint (what §5.5 metrics consume):

```json
{ "num_turns": 3, "duration_api_ms": 6331,
  "total_cost_usd": 0.012743,
  "usage": { "input_tokens": 602, "output_tokens": 211,
             "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0,
             "service_tier": "standard" } }
```

`usage` IS populated (input/output tokens; cache fields zero in this run).
`total_cost_usd` is nonzero — endpoint-reported, semantics unverified (treat as
informational only; §5.5 quota metrics use requests/tokens/latency/quota-errors
regardless).

## 3. Fallback harness smoke — kimi-agent-sdk (`spikes/kimi_agent_sdk_check.py`, exit 0)

Env: `kimi-agent-sdk 0.0.5`, `kimi-cli 1.12.0`, `kosong 0.42.0`, `pykaos 0.7.0`
(isolated venv `/tmp/kimi-sdk-venv`). Findings the spec's §6.5 fallback note
now rests on:

- **Session + agent.yaml + `CallableTool2` custom tool: PASS.** Wire events:
  `TurnBegin, StepBegin, TextPart, ToolCall, ToolCallPart, ToolResult,
  StatusUpdate, TurnEnd`.
- **Usage comes via `StatusUpdate.token_usage`** (NOT the `TokenUsage` class):
  `input_other=0 output=7 input_cache_read=131` — cache reads work on Kimi.
- **Default `Config` ships EMPTY providers/models** ("LLMNotSet") — an explicit
  `Config(providers=..., models=..., default_model=...)` is required. Provider
  type `"anthropic"` at `https://api.kimi.com/coding/` works; type
  `"openai_legacy"` at `https://api.kimi.com/coding/v1` also proven (dual
  protocol). Env augmentation exists for the `"kimi"` provider type:
  `KIMI_BASE_URL` / `KIMI_API_KEY` / `KIMI_MODEL_NAME`.
- API quirks recorded: `work_dir` must be `KaosPath`, `agent_file` is a plain
  `Path`; tools are dotted-path `CallableTool2` classes in an agent YAML.

## 4. Model-quality observations (kimi-for-coding, 2 runs)

- One run garbled the final answer text (tool returned 42 for 17+25; the
  model's reply wrote "1725") — the tool path was correct, the prose was not.
  Direct evidence for design principle 4 (tool results must be verifiable; the
  harness grades state, never report text).
- One run's final text was a bare ".". Orchestration text quality is a
  watch item for the M6 ladder (compare against Claude tier on identical tasks).

## 5. Artifacts

- `spikes/kimi_backend_check.py` — S0 harness (reusable as a regression check).
- `spikes/kimi_agent_sdk_check.py` + `spikes/kimi_sdk_smoke/` — fallback smoke.
- This document. Endpoint probe transcripts in shell history only (curl).
