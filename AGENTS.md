# Repository guidance

## Scope and source of truth

- The active product is the single-drone pilot/cockpit system, not the historical
  Commander-led swarm.
- Read `README.md`, then `docs/architecture.md`, then
  `docs/PROJECT-STATE.md` before substantial work.
- Use `docs/RUN-DEMO.md` for the supported live demo.
- Treat `docs/superpowers/` and `docs/benchmarks/` as design/evidence records,
  not automatically current instructions.
- `gpt-review/` is a dated 2026-08-01 audit. Revalidate a finding against the
  current worktree before changing code, but do not hide unresolved risks.

## Working agreement

- The worktree is intentionally allowed to be dirty. Existing changes belong to
  the owner; preserve them and do not rewrite unrelated files.
- Do not commit, push, or create branches unless the owner explicitly requests
  it.
- Prefer bounded steps and report at milestones or blockers.
- Stop after three genuine fixes to one live gate without measurable
  convergence; record the result and request an independent review.
- Treat LLM requests, long simulator runs, model downloads, and GPU campaigns as
  explicit budgets. Run cheap checks before live gates.

## Architecture constraints

- Keep the LLM out of real-time control loops. The model chooses high-level
  tools; PX4/MAVSDK or classical controllers execute the fast loop.
- Production flight contacts come from `VisionContacts`. Gazebo mover truth is
  only for clocks, grading, and explicit `--feed truth` baselines.
- Preserve ENU/NED and simulation-time/wall-time boundaries. Put conversions in
  `World`, geo, or projection helpers rather than scattering sign swaps.
- Keep the cockpit a transport/UI adapter; it must not become a second flight or
  perception authority.
- Basic flight and estop must remain usable when optional vision/deep perception
  is unavailable.

## Safety review rules

- Flag any movement path that bypasses `Envelope`, active-tool cancellation, or
  PX4 geofence assumptions.
- Treat `run_mission` as arbitrary code execution, not as a sandbox.
- Preserve the single-owner `FlightOps` and independent estop-supervisor model.
- The cockpit is unauthenticated and currently suitable only for a trusted local
  simulation network. Do not describe it as production- or real-vehicle-safe.
- Never print, commit, or embed `.env`, `.deep_token`, Claude credentials, or API
  keys.

## Verification lanes

Run checks proportionate to the change and state exactly what ran.

```bash
git diff --check
bash -n scripts/*.sh sim/launch/*.sh evals/scripts/*.sh
uv run --extra dev --with pyyaml --with numpy \
  pytest tests/ --ignore=tests/integration -q
```

- Tests under `tests/integration/` can require local sockets, Docker context, a
  live deep sidecar, or model weights. Separate environmental skips/failures
  from unit regressions.
- Gazebo/PX4 evals are not ordinary unit tests. Use a fresh container for
  controlled gates and follow the task-specific benchmark/runbook.
- Before spending LLM quota, prove the same task with its scripted `--pilot`
  baseline and confirm the null lane fails when applicable.
- Do not quote a historical green-test count as current. Collect or run the
  relevant suite and date the result.

## Documentation expectations

- Update user-visible docs when entry points, environment variables, APIs,
  models, or supported scope change.
- Keep `docs/PROJECT-STATE.md` concise and current at each deliberate pause:
  active goal, worktree state, last verification, blockers, and next bounded
  step.
- Date benchmark evidence separately from implementation status.
- Label historical paths explicitly instead of leaving broken commands looking
  supported.
- Prefer repository-relative paths in committed documentation; avoid machine-
  specific absolute paths except where the local runbook deliberately documents
  this workstation.

## Project map

- `agents/pilot/run.py`: active composition root.
- `agents/flight/`: MAVSDK operations, tracking, tools, safety envelope.
- `agents/vision/`: fast perception, contacts/fusion, optional deep service.
- `agents/observatory/`: cockpit server and UI.
- `evals/`: declarative task evaluation and deterministic oracle.
- `sim/launch/swarm_sim.sh`: simulator launcher; despite its name, the supported
  application path is single-drone.
- `agents/swarm/` and `scripts/run_swarm_demo.sh`: historical/unsupported until
  a Commander and runnable assembly are restored.
