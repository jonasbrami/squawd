# evals/ — agent task-eval harness

Measures how hard a task the drone agent can accomplish and how Claude model tier
trades latency vs correctness. Distinct from `bench/` (sim/infra throughput).

- **Grading:** sim-state oracle only (`oracle.py`) — every check is a pure function
  over a sampled `WorldTrack`. No LLM-judge.
- **Tasks:** declarative YAML in `tasks/`, tagged on 4 difficulty axes (plan depth,
  coordination, ambiguity, spatial). Targets are spec-declared coordinates.
- **Models:** `{opus, sonnet, haiku}` via per-role assignments (`drones=opus`).
  Tier→id: `opus`=claude-opus-4-8, `sonnet`=claude-sonnet-5, `haiku`=claude-haiku-4-5.
- **Repeats:** K per cell → success-rate + latency distribution.
- **Reset:** RTL soft-reset between cells; health check escalates to a fresh sim.
- **Flight link:** one MAVSDK `System` + telemetry sub is built once and reused across
  all cells (`runner.DroneHarness`); each cell gets a *fresh* Claude client so repeats
  don't share conversation context.
- **Blocking tools (2026-07-02):** `goto`/`fly` return on ARRIVAL (`wait=false` opts
  out) and `hover(seconds=N)` blocks — fire-and-forget setpoint overrides zeroed
  plan_depth for every tier and inverted c1 (a tooling trap, not a capability gap).
  The post-turn settle is now a 45 s safety net with its own budget (sharing the turn
  deadline gave slower-thinking tiers less flight time before grading).
- **Transcripts:** every cell writes a `transcripts.jsonl` line (tool calls with
  args/results/durations, agent text, tokens/cost) keyed like `results.jsonl` —
  tool choice per tier is observed, not inferred. `TOOLS.md` reports per-tier tool
  mix, goto-burst score, inter-call gap (patience), tokens + cost.
- **Statistics:** every success rate carries a Wilson 95% interval (at K=3, 0/3 vs
  3/3 is Fisher p=0.10 — bare percentages are noise); steps are conditioned on
  success; `gcs` is the mean fraction of oracle checks passed (graded signal near
  the knee). Cell order is shuffled with a logged `--seed` (resume-safe).
- **Reference pilot (`--pilot`):** task YAMLs declare `pilot:` — the ideal tool
  sequence — and `--pilot` flies it with NO LLM through the same runner/oracle
  path. A task the pilot can't pass is a harness bug; quarantine it before
  spending LLM cells. Pilot K=3 also measures the sim-noise floor.

## Use a FLAT world

Run evals on a flat world (`default` or `lawn`), **not `baylands`**. baylands has terrain
elevation that offsets PX4's local-altitude frame by ~570 m; `take_off`'s altitude gate
then trips immediately and the drone never climbs, so navigation tasks fail spuriously.
Confirmed 2026-07-01: `reach_marker_single` PASSES on `default` (drone reached 0.5 m from
the marker), FAILS on baylands for that reason.

## Run (verified 2026-07-01, single-drone smoke)

```bash
# 1) bring up a single-drone FLAT-world sim in the swarm container (host):
docker run -d --name evals-sim \
  -e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500 \
  -v "$PWD:/workspace" \
  -v /tmp/evals-claude:/root/.claude -v /tmp/evals-claude.json:/root/.claude.json \
  -v /tmp/swarm-gz-fuel:/root/.gz/fuel \
  -e SWARM_N=1 -e PX4_GZ_WORLD=default -e GZ_WORLD=default \
  squawd:dev bash -lc 'sim/launch/swarm_sim.sh'
# (first mount /tmp/evals-claude with a copy of ~/.claude/.credentials.json, + '{}' in the .json)
# wait until: ros2 topic list | grep -c vehicle_local_position  == 1

# 2) run the sweep INSIDE the container (ROS must be sourced so rclpy/px4_msgs resolve,
#    and keep $PYTHONPATH — do NOT overwrite it):
docker exec evals-sim bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash;
  cd /workspace && PYTHONPATH=/workspace:$PYTHONPATH SWARM_N=1 GZ_WORLD=default \
  uv run --no-project python -m evals.run_evals \
    --tasks evals/tasks/reach_marker_single.yaml \
    --assignments "drones=opus;drones=haiku" --k 5'
```

cpu render backend is fine for navigation tasks (no camera needed); use an Intel-GPU
container (see `scripts/run_swarm_demo.sh`) only for tasks that need `look`/`scan`.

Outputs `evals/out/<timestamp>/results.jsonl` + `RESULTS.md`. Re-running the same
command resumes (cells already in `results.jsonl` are skipped).

## Obstacle scenarios — HELD (city world unusable)

The obstacle ladder (`evals/tasks/obstacle/*`, `capstone/c2_*`) is **not shipped** yet.
It needs the `city` world (flat ground + buildings), but as of 2026-07-01 the `city` PX4
gz world fails a live validation gate:

- gz ground truth puts the drone at spawn `(0, 0, z≈-0.01)`, but PX4's
  `vehicle_local_position` reports a stable **`(x=-40, y=120, z=-12)`** offset — the EKF
  local frame is grossly displaced from the world frame, so `World.drone_state` (and thus
  the oracle) would grade against bogus coordinates.
- Preflight never passes (`Preflight Fail: vertical velocity unstable` → `Yaw estimate
  error`), so the drone **cannot arm** — every obstacle cell would be an `infra_fail`.

This is a `city`-world sim/EKF problem (same class as the `baylands` altitude offset),
not a harness bug. The `clearance` oracle check and `WorldTrack.buildings` plumbing are
built and unit-tested, ready for obstacle tasks once a usable flat-with-buildings world
exists (fix `city`'s EKF/world-origin config, or add buildings to the `default`/`lawn`
world). Until then, run only the flat-world ladders (plan-depth / spatial / ambiguity /
`c1`).

## Scope

Single-drone layer only. Commander + full-swarm layers and the prompt/tooling
iteration loop are follow-on work (see the design doc).
