# Swarm Scaling (E2) + Commander Layer (C1/E3/E4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure coordination vs fleet size (w7 survey at N=2/4/8) and build the C1 commander architecture (Commander LLM dispatching per-drone LLM agents) to measure the delegation tax (E3) and the role-mix cost/quality frontier (E4).

**Architecture:** E2 reuses the operator layer with an N-parameterized zone-survey task (8 zones on a ring; N drones use the first N even zones). C1 adds `evals/commander.py`: a CommanderSession owning one commander client (tools: `situation`, `dispatch`, `done`) plus N drone worker loops, each a fresh single-drone client per dispatched task; reports flow back as follow-up commander turns. Same task YAMLs, same oracle — `run_evals --layer commander` overrides the harness path only.

**Tech Stack:** as before (pytest host tests; claude-agent-sdk; PX4/Gazebo docker live gates).

**Spec:** `docs/superpowers/specs/2026-07-04-swarm-coordination-evals-design.md` (§C1, §Experiments)

## Global Constraints

- Branch: `feat/dynamic-scenarios`. Commit per task with the session trailers used on this branch.
- Pure logic host-unit-testable without rclpy/mavsdk/gz.
- Every new task YAML ships `pilot:` (must-PASS) and, above entry, `null_pilot:` (must-FAIL). `within_step_budget` == `budget.max_steps` (test-enforced already).
- LLM budgets include the observation tax; fleet budgets scale it by N (takeoffs, scans).
- Concurrent drone clients in C1 MUST get per-agent `CLAUDE_CONFIG_DIR` via options env (shared ~/.claude.json corrupts at >4 agents — proven).
- Unique `ROS_DOMAIN_ID`+`GZ_PARTITION` per concurrent container; bound every wait.
- The oracle is never weakened to make a model pass; budget recalibration only with a measured artifact (observation-tax discipline).

## File Structure

- `evals/areas.py` — 8 survey zones on a ring (r=130, 60×60 boxes)
- `evals/tasks/swarm/w7_survey_n{2,4,8}.yaml` — the scaling capstone ×3
- `evals/commander.py` — NEW: CommanderSession (commander client + N drone workers + report loop)
- `evals/runner.py` — `run_cell` commander branch; `require_layer_supported` accepts commander when session available
- `evals/run_evals.py` — `--layer {spec,operator,commander}` override; commander assignments already parse (`commander=x,drones=y`)
- Tests: `tests/evals/test_areas_zones.py` (zone geometry), `tests/evals/test_commander.py` (session with fake clients), task-file additions

---

### Task 1: Survey zones + w7 YAMLs (n2/n4/n8) + geometry test

**Files:**
- Modify: `evals/areas.py`, `tests/evals/test_task_files.py`
- Create: `evals/tasks/swarm/w7_survey_n2.yaml`, `w7_survey_n4.yaml`, `w7_survey_n8.yaml`
- Test: `tests/evals/test_areas_zones.py` (create)

**Interfaces:**
- Produces: `AREAS["zone_0"] .. AREAS["zone_7"]` — 60×60 axis-aligned boxes centered on a ring of radius 130 at 45° spacings starting due east: centers `(130,0), (92,92), (0,130), (-92,92), (-130,0), (-92,-92), (0,-130), (92,-92)` (rounded ints). N=2 uses zones 0,4; N=4 zones 0,2,4,6; N=8 all — task k of fleet N surveys `zone_{k * (8//N)}`.
- Consumes: existing `coverage` check {area, min_pct, radius_m, cell_m}, `fleet_separation` (+exempt_near_spawn_m), `path_length`, `targets_covered` n/a.

- [ ] **Step 1: Write failing tests**

```python
# tests/evals/test_areas_zones.py
"""Survey-zone ring geometry for the w7 scaling capstone."""
import math

from evals.areas import AREAS, area_cells, point_in_area


def test_eight_zones_on_the_ring():
    for k in range(8):
        poly = AREAS[f"zone_{k}"]
        cx = sum(p[0] for p in poly) / 4
        cy = sum(p[1] for p in poly) / 4
        assert abs(math.hypot(cx, cy) - 130) < 3, (k, cx, cy)
        # 60x60 box
        es = sorted({p[0] for p in poly})
        ns = sorted({p[1] for p in poly})
        assert es[1] - es[0] == 60 and ns[1] - ns[0] == 60


def test_zones_fit_the_geofence_and_do_not_overlap():
    for k in range(8):
        for (e, n) in AREAS[f"zone_{k}"]:
            assert math.hypot(e, n) <= 240
    for k in range(8):
        cells = area_cells(f"zone_{k}", 20.0)
        assert len(cells) == 9      # 3x3 grid of 20m cells in a 60m box
        for (e, n) in cells:
            for j in range(8):
                if j != k:
                    assert not point_in_area(f"zone_{j}", e, n)
```

Append to `tests/evals/test_task_files.py`:

```python
def test_w7_scaling_family_loads_and_scales():
    from evals.spec import load_task

    for n in (2, 4, 8):
        t = load_task(f"evals/tasks/swarm/w7_survey_n{n}.yaml")
        assert t.setup.n_drones == n and t.target_layer == "operator"
        zones = [c["area"] for c in t.oracle if c["check"] == "coverage"]
        assert zones == [f"zone_{k * (8 // n)}" for k in range(n)]
        assert t.pilot and (n == 2 or t.null_pilot)
```

(and bump the swarm-count assertion 5 → 8 in the existing swarm test.)

- [ ] **Step 2: Run to verify failure** — `python -m pytest tests/evals/test_areas_zones.py tests/evals/test_task_files.py -q` → FAIL (`KeyError: 'zone_0'`, count 5≠8).

- [ ] **Step 3: Implement**

`evals/areas.py` — append inside `AREAS` (after the fence strips):

```python
    # w7 survey ring: 8 60x60 zones at r=130, 45deg apart, zone_0 due east.
    # Fleet N surveys zones {k*(8//N)}: N=2 -> E/W, N=4 -> the diagonals too,
    # N=8 -> all. Same world, same geometry, only N scales.
    **{f"zone_{k}": [(cx - 30.0, cy - 30.0), (cx + 30.0, cy - 30.0),
                     (cx + 30.0, cy + 30.0), (cx - 30.0, cy + 30.0)]
       for k, (cx, cy) in enumerate([(130.0, 0.0), (92.0, 92.0), (0.0, 130.0),
                                     (-92.0, 92.0), (-130.0, 0.0), (-92.0, -92.0),
                                     (0.0, -130.0), (92.0, -92.0)])},
```

(NOTE: `AREAS` is a plain dict literal — convert the trailing entries with a
`**{...}` comprehension as above, or write the 8 entries out longhand; either
is fine, tests pin the geometry.)

w7 YAML template — write n2 longhand; n4/n8 differ ONLY where marked:

```yaml
# evals/tasks/swarm/w7_survey_n2.yaml
# Scaling capstone: N drones, N zones on the survey ring — one zone each is
# the only plan that fits the budgets. Coverage is drone-agnostic; budgets and
# separation make solo tours and sloppy airspace fail. Same geometry at every
# N: only the fleet and the zone count scale.
id: w7_survey_n2            # n4/n8: id matches filename
target_layer: operator
suite: swarm
difficulty: {plan_depth: 3, coordination: 3, ambiguity: 1, spatial: 3, swarm: 5}
setup:
  world: default
  n_drones: 2               # n4: 4; n8: 8
  spawn: home
  seed_objects: []
prompt: "You operate ALL drones of this fleet. Survey every assigned zone:
  each is a 60x60 m box — zone centers: E130 N0 and E-130 N0. Cover at least
  60% of every zone (fly lawnmower passes ~25 m apart at 20 m altitude).
  Assign ONE drone per zone and fly them concurrently; the fleet fuel budget
  only fits one-zone-each. Keep drones 6 m apart outside the launch area."
  # n4 prompt lists 4 centers: E130 N0, N130 E0, E-130 N0, S: E0 N-130
  # n8 prompt lists all 8 centers
budget: {wall_clock_s: 300, max_steps: 16}     # n4: 420/24; n8: 600/40
oracle:
  - {check: alive}
  - {check: coverage, area: zone_0, min_pct: 60, radius_m: 15, cell_m: 20}
  - {check: coverage, area: zone_4, min_pct: 60, radius_m: 15, cell_m: 20}
  # n4 adds zone_2, zone_6; n8 adds zone_1..zone_7 (all 8) — order zone_{k*(8//N)}
  - {check: fleet_separation, margin_m: 6, exempt_near_spawn_m: 20, grace_s: 30}
  - {check: path_length, max_m: 800}           # n4: 1600; n8: 3200 (~400/drone)
  - {check: within_step_budget, max_steps: 16} # n4: 24; n8: 40
pilot:
  - {tool: take_off, args: {altitude: 20}}
  - {tool: take_off, drone: 1, args: {altitude: 18}}
  # n4/n8: one take_off per drone, alternating altitudes 20/18/22/16/... (layered transit)
  - {tool: set_speed, args: {speed: 10}}
  - {tool: set_speed, drone: 1, args: {speed: 10}}
  # 4 goto_all waves: zone entry + 3 lawnmower rows per drone (rows at cy-25, cy, cy+25)
  - {tool: goto_all, args: {moves: [{drone: 0, east: 100, north: -25, up: 20},
                                    {drone: 1, east: -100, north: -25, up: 18}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 160, north: -25, up: 20},
                                    {drone: 1, east: -160, north: -25, up: 18}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 160, north: 0, up: 20},
                                    {drone: 1, east: -160, north: 0, up: 18}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 100, north: 0, up: 20},
                                    {drone: 1, east: -100, north: 0, up: 18}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 100, north: 25, up: 20},
                                    {drone: 1, east: -100, north: 25, up: 18}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 160, north: 25, up: 20},
                                    {drone: 1, east: -160, north: 25, up: 18}]}}
  # (n4: same 6 waves with 4 moves each — zone rows per drone; n8: 8 moves each)
# n2 is the family's entry configuration: no null (w1 already proved split-
# or-die at N=2). n4/n8 null: HALF the fleet takes off and tries to cover all
# zones -> coverage + wall-clock fail:
# null_pilot (n4/n8 only):
#   - {tool: take_off, args: {altitude: 20}}
#   - {tool: take_off, drone: 1, args: {altitude: 18}}
#   - {tool: set_speed, args: {speed: 12}}
#   - {tool: set_speed, drone: 1, args: {speed: 12}}
#   - 2-drone tour of all zone centers via alternating goto_all waves
```

Author all three files fully (no comments-as-code: expand the n4/n8 variants
per the marked rules; lawnmower rows for zone k run along the zone's LONG
axis — for diagonal zones keep rows east-west at cy-25/cy/cy+25, entries at
cx∓30; verify coverage arithmetic: 3 rows × radius 15 at spacing 25 covers a
60-wide box ≥60% when rows span the full 60 m width).

- [ ] **Step 4: Run tests** — zones + task files + full suite green.
- [ ] **Step 5: Commit** — `feat(evals): w7 survey ring — the N-scaling capstone (n2/n4/n8)`.

---

### Task 2 (LIVE): w7 gates at N=2, 4, 8

Containers (create with the standard eval-container recipe; flat world):
- N=2: reuse `evals-fleet2` (domain 85, running).
- N=4: `evals-fleet4`, `SWARM_N=4`, `ROS_DOMAIN_ID=88`, `GZ_PARTITION=evalsfleet4`.
- N=8: `evals-fleet8`, `SWARM_N=8`, `ROS_DOMAIN_ID=89`, `GZ_PARTITION=evalsfleet8`.

- [ ] **Step 1:** Gate w7_survey_n2 on evals-fleet2 (`--pilot --k 2`, out /tmp/w7n2_gate). Expect pilot 2/2.
- [ ] **Step 2:** Launch evals-fleet4; bounded wait for 4 telemetry topics (the instance-0 spawn race may need one container restart; at N≥4 also verify all N spawned — `ros2 topic list | grep -c vehicle_local_position`). Gate w7_survey_n4 (`--pilot --k 2`): pilot 2/2, null 2/2 FAIL.
- [ ] **Step 3:** Same for evals-fleet8 / w7_survey_n8. N=8 reset is unproven: if the ferry/reset repeatedly fails between cells, that IS a finding — record it, fall back to container-restart-between-cells (drive_sweep pattern), don't weaken check_home.
- [ ] **Step 4:** Commit gate artifacts as `evals/out/pilot_w7/{n2,n4,n8}` — `data(evals): w7 scaling gates`.

---

### Task 3 (LIVE): E2 runs — opus + haiku at N=2/4/8

- [ ] w7_survey_n2 on evals-fleet2, w7_survey_n4 on evals-fleet4, w7_survey_n8 on evals-fleet8; for each: `--assignments drones=opus --k 2 --seed 11 --out evals/out/e2_opus_n{N}` then `drones=haiku` → `evals/out/e2_haiku_n{N}` (sequential per container, parallel across containers; 75-min caps; resume on wedge after container restart).
- [ ] Merge per-tier, render reports, commit `data(evals): E2 fleet scaling (N=2/4/8)`. Record the scaling curve (pass rate + steps + wall-clock vs N) in the ledger for the final writeup.

---

### Task 4: CommanderSession (C1 core) + unit tests

**Files:**
- Create: `evals/commander.py`
- Test: `tests/evals/test_commander.py` (create)

**Interfaces:**
- Consumes: `make_drone_options(i, system, world, bridge, n, cameras, report=cb, env=..., model=...)` (existing), `situation_text(world, bridge, n)` (agents/perception), `Trace` (evals/runner — reuse for event recording), SDK `tool`/`create_sdk_mcp_server`/`ClaudeAgentOptions`/`ClaudeSDKClient`.
- Produces: `CommanderSession(deps, systems, commander_model, drone_model, client_factory=None, drone_client_factory=None)` with `await run(prompt, deadline_s, max_steps) -> (trace, crashed, reason)` — the same triple `_drive` returns, so `run_cell` can splice it in. `trace.steps` counts COMMANDER tool calls; `trace.meta["drone_steps"]` total drone tool calls.

**Design (locked):**
- Commander MCP server `cmd` with three tools:
  - `situation()` → `situation_text(world, bridge, n)` + the last 10 report lines.
  - `dispatch(drone, task)` → coerce drone id (reuse `FleetOps._coerce_id`), put task on that drone's `asyncio.Queue`, return `"dispatched to drone_i"`. Dispatch to a drone with a task STILL RUNNING returns an error string (`"drone_i is busy"`) — no queuing pileups.
  - `done(summary)` → sets the session's done flag.
- Per-drone worker coroutine: `while True: task = await queue.get()`; builds a FRESH single-drone client (`drone_client_factory(i, task)` in tests; production = `ClaudeSDKClient(make_drone_options(...))` with `env={"CLAUDE_CONFIG_DIR": f"/tmp/claude-agent-{i}"}`), `query(task)`, drains the response counting tool calls into `drone_steps`, then `report_cb` fires with the drone's `report(...)` calls (the report tool callback appends `(i, msg)` to `self._reports`).
- Commander loop: `query(mission_prompt)`; drain (recording tool calls into the shared `Trace`); then repeat: if done → break; if new reports since last turn → `query("REPORTS:\n" + formatted)` ; else if all workers idle and no new reports for 20s → `query("STATUS: all drones idle, no new reports. Finish or re-dispatch.")`; every turn re-checks deadline and commander step budget; on exit cancel workers.
- The commander prompt (system prompt in commander options — write it in this task):

```
You are the COMMANDER of a fleet of {n} drones. You do not fly; you delegate.
TOOLS: situation() — live fleet map; dispatch(drone, task) — send ONE drone a
self-contained natural-language task (it flies autonomously and reports back);
done(summary) — end the mission when the objective is met.
Write dispatch tasks the way you'd brief a pilot who knows nothing else:
exact coordinates, altitudes, hold times, constraints. Drones cannot hear
each other; only you see the whole picture. Re-dispatch on bad reports.
Mind the fleet constraints in the mission (separation, budgets, windows).
```

- [ ] **Step 1: failing tests** — with fake commander/drone client factories (same ScriptedClient-style fakes as tests/evals/test_runner.py): (a) dispatch routes task text to the right worker and a report flows back into the next commander turn's query text; (b) `done()` ends the loop; (c) busy-drone dispatch returns the busy error without queueing; (d) commander steps counted separately from drone steps; (e) deadline exit cancels workers. Write ~5 tests, each small.
- [ ] **Step 2:** verify failing (module missing).
- [ ] **Step 3:** implement `evals/commander.py` (~150 lines) per the locked design.
- [ ] **Step 4:** tests + full suite green.
- [ ] **Step 5:** commit `feat(evals): CommanderSession — C1 dispatch architecture`.

---

### Task 5: Runner + CLI wiring for `--layer commander`

**Files:**
- Modify: `evals/runner.py`, `evals/run_evals.py`
- Test: `tests/evals/test_runner.py`, `tests/evals/test_run_evals.py` (append)

- [ ] `run_evals --layer {spec,operator,commander}` (default `spec` = honor YAML). Layer override rides in `assignment["_layer"]` (so cells/rows/resume keys carry it) — `expand` copies assignments verbatim, `assignment_label` will show it.
- [ ] `require_layer_supported(spec, layer=None)`: effective layer = override or spec.target_layer; commander now allowed.
- [ ] `run_cell`: when effective layer == commander: build `CommanderSession(deps, systems, commander_model=model_for(assignment,"commander") or model_for(assignment,"drones"), drone_model=model_for(assignment,"drones"))` and call `session.run(spec.prompt, spec.budget.wall_clock_s, spec.budget.max_steps)` in place of `_drive`; everything else (reset, anchor, sampler, settle, oracle, hold) identical. Row gains `layer` + `drone_steps` fields (additive, JSON-safe).
- [ ] Tests: layer override reaches require_layer_supported; commander branch selected with a fake session factory; row carries layer field. Full suite green; commit `feat(evals): --layer commander path through run_cell`.

---

### Task 6 (LIVE): C1 smoke + E3 delegation-tax runs

- [ ] Smoke: ONE commander cell, w1_split_reach, `--layer commander --assignments "commander=opus,drones=opus" --k 1` on evals-fleet2. Iterate harness bugs here (this is C1's live gate; expect dispatch-prose/timing issues — fix session mechanics, not the oracle).
- [ ] E3: w1/w2/w3/w5 (+w4 on evals-fleetdyn), `commander=opus,drones=opus`, K=2, seed 11 → `evals/out/e3_commander_opus`. Compare directly against E1's operator rows (same tasks, same tier): the delegation tax.
- [ ] Commit data + ledger notes.

---

### Task 7 (LIVE): E4 role-mix matrix

- [ ] On w1/w2/w3/w5 (K=2, seed 11), the four mixes: `commander=opus,drones=opus` (from E3), `commander=opus,drones=haiku`, `commander=haiku,drones=opus`, `commander=haiku,drones=haiku` → `evals/out/e4_<cmd>_<drone>`. Track cost_usd from transcripts — the frontier is pass-rate vs $.
- [ ] Commit data.

---

### Task 8: Reports + writeup + final review

- [ ] Merge E2/E3/E4, render RESULTS/LADDERS/TOOLS, write `docs/benchmarks/EVALS-SWARM-SCALING-COMMANDER-<date>.md`: the N-scaling curve, the delegation tax number, the role-mix frontier (pass-rate vs cost), tier signatures at N=8, harness findings. Commit; update ledger; dispatch the final whole-branch review for this plan's range.

## Self-Review

- Spec coverage: E2 (Tasks 1–3), C1 (4–5 build, 6 smoke), E3 (Task 6), E4 (Task 7), writeup (8). w6 pincer-relay intentionally NOT in scope (spec marks it calibration-dependent; ladder findings so far say w4-class dynamics need the tooling loop first — recorded as a deliberate descope).
- Placeholders: the w7 n4/n8 "per the marked rules" expansions and the ~5 commander tests are enumerated with exact semantics; implementers expand mechanically. No TBDs.
- Type consistency: `CommanderSession.run -> (trace, crashed, reason)` matches `_drive`'s contract consumed in run_cell; `_coerce_id` reuse named; layer override key `assignment["_layer"]` used consistently in Tasks 5 tests.
