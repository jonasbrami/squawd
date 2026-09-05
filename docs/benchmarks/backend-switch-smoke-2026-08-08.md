# Codex/Kimi backend switch smoke — 2026-08-08

## Scope

Fresh `squawd:dev` containers on the flat `default` world, one `gz_x500`,
CPU rendering, and the same bounded task in
`evals/tasks/smoke/backend_switch.yaml`:

`take_off(3 m) → scan → report → land`

The image was rebuilt from `docker/Dockerfile.swarm` and contained
`openai-codex==0.144.4` plus `openai-codex-cli-bin==0.144.4`. Codex started
from a fresh writable home initially containing only a copy of `auth.json`.
No host Codex config, MCP definitions, plugins, skills, or workspace state was
copied. Temporary authentication copies were removed after the run.

## Results

| lane | result | calls | first action | backend wall | turns / requests | tokens | quota errors | final PX4 state |
|---|---|---:|---:|---:|---:|---:|---:|---|
| scripted pilot | **PASS** | 4 | 0.0 s | n/a | n/a | n/a | 0 | disarmed; 0.4 m from home |
| Codex `gpt-5.6-terra`, effort `low` | **PASS** | 4 | 13.35 s | 36.783 s | 1 / 1 | 16,379 input; 4 output; 16,383 total (16,128 cached input) | 0 | disarmed; 0.4 m from home |
| Kimi `kimi-for-coding` | **BLOCKED — QUOTA** | 0 | n/a | 4.349 s | 1 / 1 attempted | 0 | 1 | disarmed; remained at home |

The successful pilot and Codex transcripts contain the identical ordered tool
names (provider prefixes omitted): `take_off`, `scan`, `report`, `land`. Every
tool result had `is_error=false`; the oracle passed `alive`, `final_pos`,
`landed` (PX4 disarm confirmation), and `within_step_budget`.

Kimi returned a classified HTTP 403 billing-cycle usage-limit response before
the model produced a turn or called a tool. The harness recorded it as
`infra_fail=true`, `quota_errors=1`, and zero steps; it was not retried.

Evidence:

- `evals/out/backend_switch_20260808/pilot_fixed/`
- `evals/out/backend_switch_20260808/codex_fixed/`
- `evals/out/backend_switch_20260808/kimi/`

The superseded `pilot/` and `codex/` diagnostic attempts from before the
landing/disarm gate fix were discarded after their conclusion was captured
below; they are not acceptance evidence.

## Gate fix found during the run

The first Codex attempt completed all four calls, but the eval runner then sent
its unconditional safety `HOLD` while PX4 was still finishing touchdown. That
interrupted auto-disarm and left the vehicle armed in LOITER. The runner now
recognizes only a successful *final* `land`, waits for telemetry to confirm
disarm, and skips `HOLD` in that case. Started, failed, cancelled, or non-final
land calls retain the existing emergency-hold path. The task now carries an
explicit `landed` oracle. The scripted baseline and Codex retry both passed the
stricter gate with clean process exits.

## SDK/MCP preflight

Before simulator quota was spent, the authenticated Codex integration test ran
two turns on one persistent thread and observed one MCP call/result per turn,
token usage, and clean completion. A second test configured the pilot MCP as
required but unavailable and confirmed startup failed closed before a model
turn. Result: **2 passed in 29.52 s**.
