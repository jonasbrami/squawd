# evals/ — agent task-eval harness

Measures how hard a task the drone agent can accomplish and how Claude model tier
trades latency vs correctness. Distinct from `bench/` (sim/infra throughput).

- **Grading:** sim-state oracle only (`oracle.py`) — every check is a pure function
  over a sampled `WorldTrack`. No LLM-judge.
- **Tasks:** declarative YAML in `tasks/`, tagged on 4 difficulty axes (plan depth,
  coordination, ambiguity, spatial). Targets are spec-declared coordinates.
- **Models:** `{opus, sonnet, haiku}` via per-role assignments (`drones=opus`).
- **Repeats:** K per cell → success-rate + latency distribution.
- **Reset:** RTL soft-reset between cells; health check escalates to a fresh sim.

## Run

```bash
# 1) launch a single-drone sim (in the swarm container)
SWARM_N=1 sim/launch/swarm_sim.sh        # or the project's documented launch

# 2) run the sweep (same container/venv)
python -m evals.run_evals \
    --tasks evals/tasks/reach_marker_single.yaml \
    --assignments "drones=opus;drones=haiku" \
    --k 5
```

Outputs `evals/out/<timestamp>/results.jsonl` + `RESULTS.md`. Re-running the same
command resumes (cells already in `results.jsonl` are skipped).

## Scope

Single-drone layer only. Commander + full-swarm layers and the prompt/tooling
iteration loop are follow-on work (see the design doc).
