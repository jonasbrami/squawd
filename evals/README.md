# evals/ — agent task-eval harness

Measures how hard a task the drone agent can accomplish and how the selected
LLM backend/model trades latency vs correctness.

- **Grading:** sim-state oracle only (`oracle.py`) — every check is a pure function
  over a sampled `WorldTrack`. No LLM-judge.
- **Tasks:** declarative YAML in `tasks/`, tagged on 4 difficulty axes (plan depth,
  coordination, ambiguity, spatial). Targets are spec-declared coordinates.
- **Models:** `{opus, sonnet, haiku, kimi, kimi3, codex}` via per-role assignments
  (`drones=opus`).
  Tier→id: `opus`=claude-opus-4-8, `sonnet`=claude-sonnet-5, `haiku`=claude-haiku-4-5.
  Kimi tiers use the backend recipe: `kimi`=kimi-for-coding and `kimi3`=k3
  (design §5.2); `codex`=`gpt-5.6-terra` through the logged-in Codex
  subscription. The bounded S6 smoke has run; the three-rung M6 ladder and the
  cross-provider flight smoke remain separate outstanding work. Check
  `docs/PROJECT-STATE.md` before spending quota.
- **Repeats:** K per cell → success-rate + latency distribution.
- **Reset:** RTL soft-reset between cells; health check escalates to a fresh sim.
- **Flight link:** one MAVSDK `System` + telemetry sub is built once and reused across
  all cells (`runner.DroneHarness`); each cell gets a *fresh* backend client so
  repeats don't share conversation context.
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

## Historical Claude run recipe (verified 2026-07-01)

```bash
# 1) bring up a single-drone FLAT-world sim in the container (host):
docker run -d --name evals-sim \
  -e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500 \
  -v "$PWD:/workspace" \
  -v /tmp/evals-claude:/root/.claude -v /tmp/evals-claude.json:/root/.claude.json \
  -v /tmp/swarm-gz-fuel:/root/.gz/fuel \
  -e PX4_GZ_WORLD=default -e GZ_WORLD=default \
  squawd:dev bash -lc 'sim/launch/swarm_sim.sh'
# (first mount /tmp/evals-claude with a copy of ~/.claude/.credentials.json, + '{}' in the .json)
# wait until: ros2 topic list | grep -c vehicle_local_position  == 1

# 2) run the sweep INSIDE the container (ROS must be sourced so rclpy/px4_msgs resolve,
#    and keep $PYTHONPATH — do NOT overwrite it):
docker exec evals-sim bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash;
  cd /workspace && PYTHONPATH=/workspace:$PYTHONPATH GZ_WORLD=default \
  uv run --no-project python -m evals.run_evals \
    --tasks evals/tasks/reach_marker_single.yaml \
    --assignments "drones=opus;drones=haiku" --k 5'
```

CPU rendering is suitable only for tasks that do not require the camera. For
camera/perception tasks, follow the current container setup in
`docs/RUN-DEMO.md`; do not use the unsupported `run_swarm_demo.sh` agent path.

The provider-switch gate uses the camera-independent flat-world task
`tasks/smoke/backend_switch.yaml`. Run `--pilot --k 1` first, then one
`--assignments drones=codex` and one `drones=kimi` cell in fresh containers.
The task requires exactly `take_off → scan → report → land` and confirms PX4
disarm. Dated 2026-08-08 evidence, including the currently quota-blocked Kimi
cell, is in `docs/benchmarks/backend-switch-smoke-2026-08-08.md`.

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

## Perception grading (M5)

- **Deps split (design §3.8):** `Deps.oracle_truth` (GzPoses) feeds the sampler +
  oracle ONLY. Flight tools read `Deps.flight_contacts` — `VisionContacts` under
  `--feed vision` (Detector → VisionContacts, the production detect→lock→track
  path), or GzPoses under `--feed truth` (the explicit truth-fed control).
  `DroneHarness` never crosses those wires. Per cell, `VisionContacts.reset()`
  runs at soft_reset — no filter/ID leak across anchored repeats.
- **Perceive ladder (`tasks/perceive/*`):** the `perceive` world hosts ONE orange
  rover (`mov_true`) plus visually DISTINCT ground decoys (red tall hauler,
  blue-grey sled — the blob can't separate same-orange decoys). The
  `identified_target` oracle check grades the perception act: the first
  `track`/`goto` aimed at a `vis_*` id logs a `TargetLockEvent`, the harness
  associates that contact to oracle truth AT that sim moment, and the check
  compares the truth id (`truth: mov_true`). Report text is never graded.
  Dual gates as usual: `pilot` (scripted `track_vis` behavior — same contact
  path as the LLM) must PASS, `pilot_null` (blind) must FAIL.
- **Offline accuracy (`perceive_eval.accuracy_report`):** recorded frames
  timestamp-joined to truth by sim_stamp (50 ms tolerance): per-class
  precision/recall (IoU≥0.5), center error p50/p95 by truth range, ID-switch
  rate + track fragmentation.
- **Strategy A/B (§13 item 6):** `--assignments "drones=sonnet;drones=sonnet,strategy=intercept-lead"`
  appends a validated snippet (`agents/pilot/strategies/*.md`) to the system
  prompt for the named lane only; `evals/strategy_ab.lift_decision` activates a
  snippet ONLY on measured lift (snippet Wilson CI-low > base point rate, both
  lanes ≥3 scored cells).
- **Primitive statistics (§13 item 7):** `PRIMITIVES.md` per sweep —
  per-primitive call count, latency p50, stable error-code counts, grouped by
  model/detector/difficulty. Observational only.

## Scope

Single-drone only. Historical Commander/swarm code and tasks are preserved in
Git history, not in this harness.
