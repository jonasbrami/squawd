# Agent Task-Eval Harness — Design

**Date:** 2026-06-29
**Branch context:** sibling to the existing `bench/` capacity harness
**Status:** approved design, pre-plan

## Goal

Investigate the limits of task complexity our LLM-piloted drone agents can handle,
and measure how Claude model choice trades **latency** against **correctness**.
This is distinct from the existing `bench/` capacity harness (which measures sim/infra
throughput — RTF/FPS/cores, "how many drones fit"). This harness measures
**task quality** — "how hard a task succeeds."

Three questions to answer:
1. **Complexity limit** — where do tasks start to fail along each difficulty axis?
2. **Model trade-off** — latency vs correctness across Claude tiers (the Pareto front).
3. **Tooling/prompting lift** — how much do better tool descriptions / system prompts raise capacity?

## Decisions (locked during brainstorming)

- **Unit under test:** all three layers, layered. The same task spec can target a
  single drone agent, the Commander, or the full swarm. Build single-drone first;
  reuse the grading infra upward.
- **Grading:** sim-state oracle only. Every task must bottom out in a programmatically
  checkable Gazebo-state outcome. No LLM-judge.
- **Model matrix:** Claude tiers only (Opus/Sonnet/Haiku) on the current Claude
  subscription, including per-role mixes (e.g. Opus-Commander + Haiku-drones).
  No cross-provider. "Cost" = rate-limit/token budget, not dollars; concurrency is
  subscription-capped.
- **Complexity axes (all four):** plan depth/steps, coordination/N-drones,
  ambiguity/underspecification, spatial/perceptual difficulty. Declared as metadata
  per task so the report can plot success-rate vs each axis.
- **Repeats:** fixed K per cell (default K=5). Report success-rate + latency distribution.
- **Reset:** soft reset via PX4 **RTL** (return-to-launch → land → disarm) — returns
  drones to fixed home XY *without teleporting* (avoids EKF divergence, the failure mode
  of naive soft-resets) → clear shared store/map → reset task clock. Health check
  (each drone within tolerance of home + EKF healthy). On health-check failure, or every
  M runs, escalate to full Gazebo+PX4 teardown.

## Approach (chosen)

New sibling `evals/` module that reuses sim launch + `agents/world`/`agents/core/store`
for ground truth, with its own task-spec → runner → oracle → report pipeline.
Rejected: extending `bench/` (tangles two different reports); black-box via the
observatory bus (ground-truth and reset hooks too awkward).

## Architecture

```
evals/
  tasks/                 # task specs (YAML), one per scenario, tagged by the 4 axes
  spec.py                # TaskSpec loader/schema: setup, prompt, target_layer, oracle, budget
  areas.py               # named world-frame regions (e.g. ne_quadrant) as polygons
  oracle.py              # sim-state graders: pure fns over WorldState
  reset.py               # soft RTL-reset + health check; escalate to hard teardown
  runner.py              # drives ONE cell: inject prompt, run unit, capture trace+latency, grade
  matrix.py              # expands {model_assignments} × {tasks} × K → cells; schedules; resumable
  report.py              # success-rate + latency dist per cell; markdown + jsonl
  run_evals.py           # CLI entrypoint (mirrors run_bench.py conventions)
```

Reuses: `sim/launch/swarm_sim.sh` (launch), `agents/world` + `agents/core/store`
(ground truth), `agents/swarm` (units under test). A **model assignment** is a config
row — `{commander: opus, drones: haiku}` — so per-role mixes (and provider-agnostic-later)
fall out for free.

## Task spec

Layer-agnostic YAML, one file per scenario. Only `target_layer` and which prompt is
injected changes between layers.

```yaml
id: search_quadrant_4drone
target_layer: swarm            # single_drone | commander | swarm
difficulty:                    # the 4 axes, 1–5 each — what you sweep/plot against
  plan_depth: 3
  coordination: 4
  ambiguity: 4
  spatial: 3
setup:
  world: baylands
  n_drones: 4
  spawn: home                  # drones start landed at home
  seed_objects:                # oracle targets placed at fixed coords (deterministic)
    - {kind: marker, id: tgt_a, east: 120, north: -40}
prompt: "Search the north-east quadrant and report the location of any markers."
budget:
  wall_clock_s: 180            # hard deadline (per time-bound-long-waits practice)
  max_steps: 40                # tool-call budget; exceeding = fail
oracle:                        # ALL must pass = success; each a pure fn over WorldState
  - {check: alive}
  - {check: coverage, area: ne_quadrant, min_pct: 80}
  - {check: reached, target: tgt_a, tol_m: 15}
```

Difficulty is **declared metadata**, not inferred. Targets come from **spec-seeded
objects** (fixed coords, reproducible across repeats), not existing world features.

## Oracle

Each check is a pure function `(WorldState, params) -> CheckResult{passed, detail, value}`.
`WorldState` is sampled from `agents/world` + `agents/core/store` (drone poses, geofence,
object positions) — no new sim plumbing. Starter library:

| check | grades | params |
|---|---|---|
| `reached` | min distance from a drone to target ever ≤ tol | `target, tol_m` |
| `coverage` | % of a named area's cells overflown (position-overflight) | `area, min_pct` |
| `formation` | drones held a spatial relation (spacing/line/ring) for ≥T | `kind, tol_m, hold_s` |
| `ordering` | targets visited in required sequence | `sequence` |
| `alive` | no crash, stayed in geofence, all disarmed cleanly at end | — |
| `within_step_budget` | tool-calls ≤ max_steps (also a trace metric) | — |

**Coverage** is measured by **position-overflight**: a cell counts when any drone's
ground position passed within radius R. (Camera-footprint coverage may be logged as a
trace metric later, but does not drive pass/fail.) Named areas live in `areas.py` as
world-frame polygons so specs stay terse.

**Trace capture (not used for pass/fail):** tool-call count, wall-clock,
latency-to-first-action, per-tool timing → into the cell's jsonl. This is what enables
the latency comparison across tiers and shows *how* a tier failed.

## Runner — one cell `(task × model-assignment × repeat k)`

1. `reset.py` ensures clean world (soft RTL-reset + health check, else hard teardown).
2. `setup` applies: seed markers at fixed coords, confirm `n_drones` landed at home.
3. Apply **model assignment** — Agent SDK `model` param per role.
4. Inject `prompt` at the `target_layer` real entry point: single_drone → the drone's
   SDK client; commander → commander; swarm → `/swarm/user_input` bus.
5. Run under `budget` (hard wall-clock deadline + max_steps). Capture trace.
6. On deadline/step-budget/crash → terminate, mark run, capture partial state.
7. Sample `WorldState`, run all `oracle` checks → `cell_result`.
8. Append to jsonl; release for next cell.

## Matrix & scheduling

`matrix.py` expands `{model_assignments} × {tasks} × K` into cells. Execution is
**sequential by default** (one sim, soft-reset between): the subscription caps
concurrency and parallel sims would confound the latency measurement (a headline metric).
Resumable: skip cells already present in the out jsonl. Mirrors `bench/`'s timestamped
out-dir + per-run failure containment.

## Reporting

Same conventions as `bench/` (`RESULTS.md` + raw jsonl):
- Per cell: `success_rate = passes/K`, latency distribution (p50/p95), mean steps,
  failure breakdown (which check / deadline / crash / infra-fail).
- **Complexity limit** — success-rate vs each difficulty axis, per model → the knee.
- **Model trade-off** — latency vs success-rate scatter across tiers (Pareto front).
- **Role-mix value** — Opus-Commander/Haiku-drones vs uniform on the same tasks.

## Error handling / containment

Every run wrapped. Infra failure (sim crash, EKF reject, failed arm) is tagged
`infra-fail` and retried once (reusing the `bench/` pattern from commit `cecc6c9`),
never scored as a task failure. Distinguishing `infra-fail` from genuine task-fail is
essential — otherwise accuracy numbers are meaningless.

## Build phases (each independently testable)

1. **Spec + oracle + areas** — load a YAML, grade a hand-recorded WorldState. No sim.
2. **Reset** — soft RTL-reset + health check against a live sim; prove drones return
   home and store clears.
3. **Runner, single_drone only** — one task end-to-end, one model, K=1.
4. **Matrix + report** — sweep tiers on the single-drone ladder, K=5, produce RESULTS.md.
5. **Lift to commander, then swarm** — reuse oracle/report; add the 2 missing
   prompt-injection paths.
6. **Tooling/prompting iteration loop** — with the harness trustworthy, vary tool
   descriptions / system prompts as another config dimension and measure the lift
   (closes the "improve tooling & prompting" goal).

## Out of scope

- Cross-provider (non-Claude) models.
- LLM-judge / semantic grading.
- Camera-footprint coverage as a pass/fail metric (trace-only, later).
- Parallel sim execution (would confound latency).
