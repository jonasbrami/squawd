# Agent Task-Eval Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a sim-state-graded harness that measures how hard a task the single-drone agent can accomplish, and how Claude model tier trades latency against correctness.

**Architecture:** A new `evals/` Python module, sibling to `bench/`. Pure modules (data types, areas, oracle, spec, matrix, report) are fully unit-tested; integration modules (sampler, reset, runner) reuse the live swarm stack (`agents.core.bus.RosBridge`, `agents.world.World`, `agents.swarm.drone.DroneAgent`) and are validated by a sim-gated smoke test. A "model assignment" is a config dict threaded into `ClaudeAgentOptions(model=...)`.

**Tech Stack:** Python 3.11+, `pyyaml`, `pytest`, `claude_agent_sdk`, `mavsdk`, `rclpy` (live only). Reuses `sim/launch/swarm_sim.sh` for the Gazebo+PX4 sim.

## Global Constraints

- Python target: 3.11+ (matches existing `agents/` — uses `str | None` unions). Copy verbatim.
- Pure modules (`worldstate.py`, `areas.py`, `oracle.py`, `spec.py`, `matrix.py`, `report.py`) MUST NOT import `rclpy`, `mavsdk`, or `claude_agent_sdk` — so they unit-test without ROS installed (same discipline as `agents/core/store.py`).
- Every long wait (sim launch, RTL, agent run) MUST be bounded by a hard deadline — no unbounded polls (project practice: time-bound long waits).
- Grading is **sim-state oracle only** — no LLM-judge. Every oracle check is a pure function over `WorldTrack`.
- An infra failure (sim crash, EKF reject, failed arm, connection loss) is tagged `infra_fail=True` and retried once; it is NEVER scored as a task failure. Reuse the `bench.run_bench.run_with_retry` pattern.
- Coverage is measured by **position-overflight** (a cell counts when a drone's ground position passed within radius R), not camera footprint.
- Oracle targets are **spec-declared logical coordinates** (deterministic). Physical Gazebo marker spawning and camera-footprint coverage are deferred to a later plan.
- Model tier → SDK id map (verbatim): `opus` → `claude-opus-4-8`, `sonnet` → `claude-sonnet-4-6`, `haiku` → `claude-haiku-4-5-20251001`.
- Out-dir convention (mirror `bench/`): `evals/out/<timestamp>/` with `results.jsonl` (one row per cell) + `RESULTS.md`.

**Scope of THIS plan:** single-drone layer only (spec build phases 1–4). Lifting the same oracle/report to the Commander and full swarm (phase 5) and the prompt/tooling iteration loop (phase 6) are separate follow-on plans.

---

### Task 1: WorldTrack data types

The decoupled, ROS-free ground-truth record the oracle grades. Accumulated during a run by the sampler (Task 5).

**Files:**
- Create: `evals/__init__.py` (empty)
- Create: `evals/worldstate.py`
- Test: `tests/evals/test_worldstate.py`
- Create: `tests/evals/__init__.py` (empty)

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `DronePose(e: float, n: float, alt: float, heading: float)` — frozen dataclass.
  - `Snapshot(t: float, poses: dict[int, DronePose])` — frozen dataclass.
  - `WorldTrack(snapshots: list[Snapshot], objects: dict[str, tuple[float, float]], n_drones: int, geofence_m: float)`.
  - `WorldTrack.min_dist_to(self, xy: tuple[float, float]) -> float` — smallest distance from ANY drone at ANY snapshot to point `xy`; `inf` if no snapshots.
  - `WorldTrack.max_dist_from_origin(self) -> float` — largest distance any drone reached from world origin `(0, 0)` (for the `alive`/geofence check); `0.0` if empty.
  - `WorldTrack.positions(self) -> list[tuple[float, float]]` — flat list of every `(e, n)` across all snapshots/drones.

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_worldstate.py
import math
from evals.worldstate import DronePose, Snapshot, WorldTrack


def _track():
    snaps = [
        Snapshot(t=0.0, poses={0: DronePose(0.0, 0.0, 0.0, 0.0)}),
        Snapshot(t=1.0, poses={0: DronePose(10.0, 0.0, 12.0, 0.0)}),
        Snapshot(t=2.0, poses={0: DronePose(10.0, 5.0, 12.0, 0.0)}),
    ]
    return WorldTrack(snapshots=snaps, objects={"tgt_a": (12.0, 5.0)}, n_drones=1, geofence_m=300.0)


def test_min_dist_to_uses_closest_snapshot():
    assert math.isclose(_track().min_dist_to((12.0, 5.0)), 2.0)


def test_min_dist_to_empty_is_inf():
    t = WorldTrack(snapshots=[], objects={}, n_drones=1, geofence_m=300.0)
    assert t.min_dist_to((0.0, 0.0)) == math.inf


def test_max_dist_from_origin():
    assert math.isclose(_track().max_dist_from_origin(), math.hypot(10.0, 5.0))


def test_positions_flattens_all():
    assert (10.0, 0.0) in _track().positions()
    assert len(_track().positions()) == 3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_worldstate.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.worldstate'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/worldstate.py
"""WorldTrack: the ROS-free ground-truth record the oracle grades.

A run's sampler (evals.sampler) appends a Snapshot per poll; the oracle
(evals.oracle) reads the finished track. Pure dataclasses + math so it imports
and unit-tests without rclpy/mavsdk (same discipline as agents.core.store)."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DronePose:
    e: float
    n: float
    alt: float
    heading: float


@dataclass(frozen=True)
class Snapshot:
    t: float
    poses: dict[int, "DronePose"]


@dataclass
class WorldTrack:
    snapshots: list[Snapshot]
    objects: dict[str, tuple[float, float]]
    n_drones: int
    geofence_m: float

    def min_dist_to(self, xy: tuple[float, float]) -> float:
        best = math.inf
        for s in self.snapshots:
            for p in s.poses.values():
                best = min(best, math.hypot(p.e - xy[0], p.n - xy[1]))
        return best

    def max_dist_from_origin(self) -> float:
        best = 0.0
        for s in self.snapshots:
            for p in s.poses.values():
                best = max(best, math.hypot(p.e, p.n))
        return best

    def positions(self) -> list[tuple[float, float]]:
        return [(p.e, p.n) for s in self.snapshots for p in s.poses.values()]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_worldstate.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/__init__.py evals/worldstate.py tests/evals/__init__.py tests/evals/test_worldstate.py
git commit -m "feat(evals): WorldTrack ground-truth data types"
```

---

### Task 2: Named areas (coverage regions)

World-frame polygons so specs reference `ne_quadrant` instead of inlining geometry. Provides the coverage denominator (grid cells inside a region).

**Files:**
- Create: `evals/areas.py`
- Test: `tests/evals/test_areas.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `AREAS: dict[str, list[tuple[float, float]]]` — name → polygon vertices (e, n), CCW.
  - `point_in_area(name: str, e: float, n: float) -> bool` — ray-cast point-in-polygon; raises `KeyError` on unknown name.
  - `area_cells(name: str, cell_m: float) -> list[tuple[float, float]]` — centers of `cell_m`-sized grid cells whose center lies inside the polygon (the coverage denominator).

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_areas.py
import pytest
from evals.areas import AREAS, point_in_area, area_cells


def test_ne_quadrant_registered():
    assert "ne_quadrant" in AREAS


def test_point_inside_and_outside():
    assert point_in_area("ne_quadrant", 50.0, 50.0) is True
    assert point_in_area("ne_quadrant", -50.0, -50.0) is False


def test_unknown_area_raises():
    with pytest.raises(KeyError):
        point_in_area("nowhere", 0.0, 0.0)


def test_area_cells_all_inside_and_nonempty():
    cells = area_cells("ne_quadrant", 20.0)
    assert len(cells) > 0
    assert all(point_in_area("ne_quadrant", e, n) for e, n in cells)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_areas.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.areas'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/areas.py
"""Named world-frame regions (ENU east/north) used by oracle coverage checks.

A region is a polygon (CCW vertices). `area_cells` discretizes it into grid-cell
centers — the denominator for position-overflight coverage. Keep regions here so
task specs stay terse and reusable. Add regions as scenarios need them."""

# ne_quadrant: a 200 m x 200 m box NE of home (home is world origin (0,0)).
AREAS: dict[str, list[tuple[float, float]]] = {
    "ne_quadrant": [(0.0, 0.0), (200.0, 0.0), (200.0, 200.0), (0.0, 200.0)],
}


def _point_in_poly(poly: list[tuple[float, float]], e: float, n: float) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        ei, ni = poly[i]
        ej, nj = poly[j]
        if ((ni > n) != (nj > n)) and (e < (ej - ei) * (n - ni) / (nj - ni) + ei):
            inside = not inside
        j = i
    return inside


def point_in_area(name: str, e: float, n: float) -> bool:
    return _point_in_poly(AREAS[name], e, n)


def area_cells(name: str, cell_m: float) -> list[tuple[float, float]]:
    poly = AREAS[name]
    es = [p[0] for p in poly]
    ns = [p[1] for p in poly]
    e0, e1, n0, n1 = min(es), max(es), min(ns), max(ns)
    cells: list[tuple[float, float]] = []
    e = e0 + cell_m / 2
    while e < e1:
        n = n0 + cell_m / 2
        while n < n1:
            if _point_in_poly(poly, e, n):
                cells.append((e, n))
            n += cell_m
        e += cell_m
    return cells
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_areas.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/areas.py tests/evals/test_areas.py
git commit -m "feat(evals): named coverage areas + grid discretization"
```

---

### Task 3: Oracle (sim-state graders)

Pure functions over `WorldTrack` that decide pass/fail. This is the heart of grading.

**Files:**
- Create: `evals/oracle.py`
- Test: `tests/evals/test_oracle.py`

**Interfaces:**
- Consumes: `evals.worldstate.WorldTrack`; `evals.areas.area_cells`, `point_in_area`.
- Produces:
  - `CheckResult(name: str, passed: bool, value: float, detail: str)` — frozen dataclass.
  - `GradeResult(passed: bool, checks: list[CheckResult])` — `passed` is `all(c.passed)`.
  - `CHECKS: dict[str, callable]` — registry mapping check name → `fn(track, params, run_meta) -> CheckResult`. `run_meta` is `dict` with at least `{"steps": int, "crashed": bool}`.
  - `grade(track: WorldTrack, oracle_specs: list[dict], run_meta: dict) -> GradeResult`.
  - Registered checks: `reached`, `coverage`, `alive`, `within_step_budget`.

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_oracle.py
from evals.worldstate import DronePose, Snapshot, WorldTrack
from evals.oracle import grade


def _track(reach=True):
    e = 118.0 if reach else 0.0
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 0.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(e, -40.0, 12.0, 0.0)}),
    ]
    return WorldTrack(snaps, {"tgt_a": (120.0, -40.0)}, n_drones=1, geofence_m=300.0)


META_OK = {"steps": 5, "crashed": False}


def test_reached_pass():
    g = grade(_track(True), [{"check": "reached", "target": "tgt_a", "tol_m": 15}], META_OK)
    assert g.passed


def test_reached_fail():
    g = grade(_track(False), [{"check": "reached", "target": "tgt_a", "tol_m": 15}], META_OK)
    assert not g.passed


def test_alive_fails_on_crash():
    g = grade(_track(True), [{"check": "alive"}], {"steps": 5, "crashed": True})
    assert not g.passed


def test_step_budget():
    spec = [{"check": "within_step_budget", "max_steps": 4}]
    assert not grade(_track(True), spec, {"steps": 5, "crashed": False}).passed
    assert grade(_track(True), spec, {"steps": 4, "crashed": False}).passed


def test_coverage_counts_overflown_cells():
    # Drone visits two cell centers of ne_quadrant; min_pct low enough to pass.
    snaps = [Snapshot(float(i), {0: DronePose(e, n, 12.0, 0.0)})
             for i, (e, n) in enumerate([(10, 10), (30, 10)])]
    t = WorldTrack(snaps, {}, n_drones=1, geofence_m=300.0)
    spec = [{"check": "coverage", "area": "ne_quadrant", "min_pct": 1, "radius_m": 15, "cell_m": 20}]
    assert grade(t, spec, META_OK).passed


def test_all_checks_must_pass():
    spec = [{"check": "reached", "target": "tgt_a", "tol_m": 15},
            {"check": "within_step_budget", "max_steps": 1}]
    assert not grade(_track(True), spec, {"steps": 5, "crashed": False}).passed
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_oracle.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.oracle'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/oracle.py
"""Sim-state oracle: pure functions over a WorldTrack that decide pass/fail.

Grading is deterministic ground truth only (no LLM-judge). A task succeeds when
EVERY check passes. Add checks to CHECKS; specs reference them by name. Each check
is fn(track, params, run_meta) -> CheckResult, where run_meta carries trace facts
the track doesn't (tool-call count, crash flag)."""
import math
from dataclasses import dataclass

from evals.areas import area_cells, point_in_area
from evals.worldstate import WorldTrack


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    value: float
    detail: str


@dataclass(frozen=True)
class GradeResult:
    passed: bool
    checks: list[CheckResult]


def _reached(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    xy = track.objects[p["target"]]
    d = track.min_dist_to(xy)
    tol = float(p["tol_m"])
    return CheckResult("reached", d <= tol, d, f"min dist {d:.1f}m to {p['target']} (tol {tol:g})")


def _coverage(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    cells = area_cells(p["area"], float(p.get("cell_m", 20.0)))
    r = float(p.get("radius_m", 15.0))
    pts = track.positions()
    covered = sum(1 for (ce, cn) in cells
                  if any(math.hypot(pe - ce, pn - cn) <= r for pe, pn in pts))
    pct = 100.0 * covered / len(cells) if cells else 0.0
    need = float(p["min_pct"])
    return CheckResult("coverage", pct >= need, pct,
                       f"{covered}/{len(cells)} cells = {pct:.0f}% (need {need:g}%)")


def _alive(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    far = track.max_dist_from_origin()
    ok = (not m.get("crashed", False)) and far <= track.geofence_m
    return CheckResult("alive", ok, far, f"crashed={m.get('crashed')} max_reach={far:.0f}m")


def _within_step_budget(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    steps = int(m.get("steps", 0))
    mx = int(p["max_steps"])
    return CheckResult("within_step_budget", steps <= mx, float(steps), f"{steps} steps (max {mx})")


CHECKS = {
    "reached": _reached,
    "coverage": _coverage,
    "alive": _alive,
    "within_step_budget": _within_step_budget,
}


def grade(track: WorldTrack, oracle_specs: list[dict], run_meta: dict) -> GradeResult:
    results = []
    for spec in oracle_specs:
        fn = CHECKS[spec["check"]]
        results.append(fn(track, spec, run_meta))
    return GradeResult(passed=all(c.passed for c in results), checks=results)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_oracle.py -v`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/oracle.py tests/evals/test_oracle.py
git commit -m "feat(evals): sim-state oracle (reached/coverage/alive/step-budget)"
```

---

### Task 4: Task spec loader

Loads + validates a task YAML into typed objects, rejecting unknown oracle checks early.

**Files:**
- Create: `evals/spec.py`
- Create: `evals/tasks/reach_marker_single.yaml`
- Test: `tests/evals/test_spec.py`

**Interfaces:**
- Consumes: `evals.oracle.CHECKS` (to validate check names).
- Produces:
  - `SeedObject(id: str, e: float, n: float)`, `SetupSpec(world: str, n_drones: int, spawn: str, seed_objects: list[SeedObject])`, `BudgetSpec(wall_clock_s: float, max_steps: int)`.
  - `TaskSpec(id, target_layer, difficulty: dict, setup: SetupSpec, prompt: str, budget: BudgetSpec, oracle: list[dict])`.
  - `TaskSpec.objects_map(self) -> dict[str, tuple[float,float]]` — `{id: (e, n)}` from `setup.seed_objects`, for building the `WorldTrack`.
  - `SpecError(Exception)`.
  - `load_task(path: str) -> TaskSpec`.

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_spec.py
import pytest
from evals.spec import load_task, SpecError

VALID = """
id: reach_marker_single
target_layer: single_drone
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: baylands
  n_drones: 1
  spawn: home
  seed_objects:
    - {kind: marker, id: tgt_a, east: 120, north: -40}
prompt: "Take off and fly to the marker tgt_a at east 120, north -40."
budget: {wall_clock_s: 120, max_steps: 20}
oracle:
  - {check: alive}
  - {check: reached, target: tgt_a, tol_m: 15}
"""


def _write(tmp_path, text):
    p = tmp_path / "t.yaml"
    p.write_text(text)
    return str(p)


def test_loads_valid(tmp_path):
    t = load_task(_write(tmp_path, VALID))
    assert t.id == "reach_marker_single"
    assert t.setup.n_drones == 1
    assert t.objects_map() == {"tgt_a": (120.0, -40.0)}
    assert t.budget.max_steps == 20


def test_unknown_check_rejected(tmp_path):
    bad = VALID.replace("check: reached", "check: teleported")
    with pytest.raises(SpecError):
        load_task(_write(tmp_path, bad))


def test_missing_field_rejected(tmp_path):
    bad = VALID.replace("target_layer: single_drone", "")
    with pytest.raises(SpecError):
        load_task(_write(tmp_path, bad))


def test_bundled_task_file_loads():
    t = load_task("evals/tasks/reach_marker_single.yaml")
    assert t.target_layer == "single_drone"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_spec.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.spec'`

- [ ] **Step 3: Write the implementation and the bundled task file**

```python
# evals/spec.py
"""TaskSpec: load + validate a task YAML into typed objects.

Validation fails fast (SpecError) on missing fields or unknown oracle checks, so a
sweep never wastes sim time on a malformed scenario. Layer-agnostic: target_layer
selects where the runner injects the prompt; the rest is shared."""
from dataclasses import dataclass

import yaml

from evals.oracle import CHECKS


class SpecError(Exception):
    pass


@dataclass(frozen=True)
class SeedObject:
    id: str
    e: float
    n: float


@dataclass(frozen=True)
class SetupSpec:
    world: str
    n_drones: int
    spawn: str
    seed_objects: list[SeedObject]


@dataclass(frozen=True)
class BudgetSpec:
    wall_clock_s: float
    max_steps: int


@dataclass(frozen=True)
class TaskSpec:
    id: str
    target_layer: str
    difficulty: dict
    setup: SetupSpec
    prompt: str
    budget: BudgetSpec
    oracle: list[dict]

    def objects_map(self) -> dict[str, tuple[float, float]]:
        return {o.id: (o.e, o.n) for o in self.setup.seed_objects}


def _require(d: dict, key: str, ctx: str):
    if not isinstance(d, dict) or key not in d:
        raise SpecError(f"missing '{key}' in {ctx}")
    return d[key]


def load_task(path: str) -> TaskSpec:
    with open(path) as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise SpecError(f"{path}: top level must be a mapping")

    s = _require(raw, "setup", path)
    seeds = [SeedObject(id=_require(o, "id", "seed_object"),
                        e=float(_require(o, "east", "seed_object")),
                        n=float(_require(o, "north", "seed_object")))
             for o in s.get("seed_objects", [])]
    setup = SetupSpec(world=_require(s, "world", "setup"),
                      n_drones=int(_require(s, "n_drones", "setup")),
                      spawn=s.get("spawn", "home"),
                      seed_objects=seeds)

    b = _require(raw, "budget", path)
    budget = BudgetSpec(wall_clock_s=float(_require(b, "wall_clock_s", "budget")),
                        max_steps=int(_require(b, "max_steps", "budget")))

    oracle = _require(raw, "oracle", path)
    if not isinstance(oracle, list) or not oracle:
        raise SpecError(f"{path}: 'oracle' must be a non-empty list")
    for chk in oracle:
        name = _require(chk, "check", "oracle entry")
        if name not in CHECKS:
            raise SpecError(f"{path}: unknown oracle check '{name}' "
                            f"(have {sorted(CHECKS)})")

    return TaskSpec(
        id=_require(raw, "id", path),
        target_layer=_require(raw, "target_layer", path),
        difficulty=_require(raw, "difficulty", path),
        setup=setup,
        prompt=_require(raw, "prompt", path),
        budget=budget,
        oracle=oracle,
    )
```

```yaml
# evals/tasks/reach_marker_single.yaml
id: reach_marker_single
target_layer: single_drone
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: baylands
  n_drones: 1
  spawn: home
  seed_objects:
    - {kind: marker, id: tgt_a, east: 120, north: -40}
prompt: "Take off to 12 m, then fly to the marker tgt_a located at world east 120, north -40, and hover there."
budget: {wall_clock_s: 120, max_steps: 20}
oracle:
  - {check: alive}
  - {check: reached, target: tgt_a, tol_m: 15}
  - {check: within_step_budget, max_steps: 20}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_spec.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/spec.py evals/tasks/reach_marker_single.yaml tests/evals/test_spec.py
git commit -m "feat(evals): task spec loader + first single-drone task"
```

---

### Task 5: Sampler (live bridge → WorldTrack)

Polls the live `World`/`RosBridge` at a fixed interval into a `WorldTrack`. First integration module; the polling helper is unit-tested with a fake.

**Files:**
- Create: `evals/sampler.py`
- Test: `tests/evals/test_sampler.py`

**Interfaces:**
- Consumes: `agents.world.World.drone_state(bridge, i) -> (e, n, alt, heading_rad) | None`; `evals.worldstate` types.
- Produces:
  - `snapshot_now(world, bridge, n_drones, t) -> Snapshot` — pure-ish: reads `world.drone_state` for each drone, includes only drones with a valid fix.
  - `class Sampler` with `__init__(self, world, bridge, n_drones, objects, geofence_m, interval=0.5)`, `async def run(self)` (loop appending snapshots until `stop()`), `def stop(self)`, `def track(self) -> WorldTrack`.

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_sampler.py
from evals.sampler import snapshot_now


class FakeWorld:
    def __init__(self, states):
        self._states = states  # dict[i] -> tuple|None

    def drone_state(self, bridge, i):
        return self._states.get(i)


def test_snapshot_skips_invalid_fix():
    world = FakeWorld({0: (10.0, 20.0, 12.0, 0.0), 1: None})
    snap = snapshot_now(world, bridge=None, n_drones=2, t=3.0)
    assert snap.t == 3.0
    assert set(snap.poses) == {0}
    assert snap.poses[0].e == 10.0 and snap.poses[0].alt == 12.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_sampler.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.sampler'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/sampler.py
"""Sampler: poll the live World/RosBridge into a WorldTrack during a run.

snapshot_now is unit-testable with a fake World (no ROS). Sampler.run is the async
loop the runner starts before injecting the task and stops after; it tolerates the
brief windows where a drone has no valid fix (just omits it from that snapshot)."""
import asyncio
import time

from evals.worldstate import DronePose, Snapshot, WorldTrack


def snapshot_now(world, bridge, n_drones: int, t: float) -> Snapshot:
    poses: dict[int, DronePose] = {}
    for i in range(n_drones):
        st = world.drone_state(bridge, i)
        if st is not None:
            poses[i] = DronePose(e=st[0], n=st[1], alt=st[2], heading=st[3])
    return Snapshot(t=t, poses=poses)


class Sampler:
    def __init__(self, world, bridge, n_drones, objects, geofence_m, interval=0.5):
        self._world = world
        self._bridge = bridge
        self._n = n_drones
        self._objects = dict(objects)
        self._geofence_m = geofence_m
        self._interval = interval
        self._snaps: list[Snapshot] = []
        self._running = False

    async def run(self) -> None:
        self._running = True
        t0 = time.monotonic()
        while self._running:
            self._snaps.append(snapshot_now(self._world, self._bridge, self._n,
                                            time.monotonic() - t0))
            await asyncio.sleep(self._interval)

    def stop(self) -> None:
        self._running = False

    def track(self) -> WorldTrack:
        return WorldTrack(snapshots=list(self._snaps), objects=self._objects,
                          n_drones=self._n, geofence_m=self._geofence_m)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_sampler.py -v`
Expected: PASS (1 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/sampler.py tests/evals/test_sampler.py
git commit -m "feat(evals): live world sampler -> WorldTrack"
```

---

### Task 6: Reset (RTL soft-reset + health check)

Returns drones home via PX4 RTL and reports whether the world is clean enough to reuse. The health-check logic is unit-tested; the live RTL is a documented helper.

**Files:**
- Create: `evals/reset.py`
- Test: `tests/evals/test_reset.py`

**Interfaces:**
- Consumes: `agents.world.World.spawn_x`, `World.spawn_spacing`, `World.world_xy(bridge, i)`; MAVSDK `System.action.return_to_launch()`.
- Produces:
  - `home_xy(world, i) -> tuple[float, float]` — `(world.spawn_x, world.spawn_spacing * i)`.
  - `ResetResult(ok: bool, reason: str)` — frozen dataclass.
  - `check_home(world, bridge, n, tol_m) -> ResetResult` — pure: every drone within `tol_m` of its `home_xy`, else `ok=False` naming the offender.
  - `async def soft_reset(systems, world, bridge, n, tol_m=5.0, timeout_s=60.0) -> ResetResult` — RTL every system, wait until all near home (bounded by `timeout_s`), then return `check_home`.

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_reset.py
from evals.reset import home_xy, check_home


class FakeWorld:
    spawn_x = 0.0
    spawn_spacing = 3.0

    def __init__(self, xy):
        self._xy = xy  # dict[i] -> (e, n, alt)

    def world_xy(self, bridge, i):
        return self._xy.get(i)


def test_home_xy():
    assert home_xy(FakeWorld({}), 2) == (0.0, 6.0)


def test_check_home_pass():
    w = FakeWorld({0: (0.2, 0.0, 0.1), 1: (0.0, 3.1, 0.1)})
    assert check_home(w, None, 2, tol_m=5.0).ok


def test_check_home_fail_names_drone():
    w = FakeWorld({0: (0.0, 0.0, 0.1), 1: (50.0, 3.0, 0.1)})
    r = check_home(w, None, 2, tol_m=5.0)
    assert not r.ok and "drone_1" in r.reason


def test_check_home_fail_on_missing_fix():
    r = check_home(FakeWorld({0: None}), None, 1, tol_m=5.0)
    assert not r.ok
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_reset.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.reset'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/reset.py
"""Soft reset between runs: RTL all drones home, then verify the world is clean.

RTL (return-to-launch) brings each drone back to its fixed spawn XY and lands WITHOUT
teleporting the vehicle, so the EKF stays converged (the failure mode that makes naive
soft-resets leaky). check_home is the health gate: if any drone isn't near home, the
caller escalates to a full sim teardown. Pure geometry here is unit-tested; the live
RTL loop is bounded by timeout_s."""
import asyncio
import math
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class ResetResult:
    ok: bool
    reason: str


def home_xy(world, i: int) -> tuple[float, float]:
    return (world.spawn_x, world.spawn_spacing * i)


def check_home(world, bridge, n: int, tol_m: float) -> ResetResult:
    for i in range(n):
        xy = world.world_xy(bridge, i)
        if xy is None:
            return ResetResult(False, f"drone_{i} has no fix")
        hx, hy = home_xy(world, i)
        d = math.hypot(xy[0] - hx, xy[1] - hy)
        if d > tol_m:
            return ResetResult(False, f"drone_{i} {d:.1f}m from home (tol {tol_m:g})")
    return ResetResult(True, "all drones home")


async def soft_reset(systems, world, bridge, n, tol_m=5.0, timeout_s=60.0) -> ResetResult:
    for s in systems:
        try:
            await s.action.return_to_launch()
        except Exception as e:
            return ResetResult(False, f"RTL command failed: {e}")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        r = check_home(world, bridge, n, tol_m)
        if r.ok:
            return r
        await asyncio.sleep(1.0)
    return check_home(world, bridge, n, tol_m)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_reset.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/reset.py tests/evals/test_reset.py
git commit -m "feat(evals): RTL soft-reset + home health check"
```

---

### Task 7: Runner (single-drone cell execution)

Drives ONE `(task × model × repeat)` cell end-to-end against a live sim: build the drone with the chosen model, inject the prompt under a deadline, count tool calls, grade. The trace-accounting helper is unit-tested with a fake message stream; full execution is sim-gated.

**Files:**
- Create: `evals/runner.py`
- Modify: `agents/flight/tools.py` (thread a `model` param into `make_drone_options` → `ClaudeAgentOptions`)
- Modify: `agents/swarm/drone.py` (thread `model` through `DroneAgent.__init__`)
- Test: `tests/evals/test_runner.py`

**Interfaces:**
- Consumes: `evals.spec.TaskSpec`; `evals.sampler.Sampler`; `evals.reset.soft_reset`; `evals.oracle.grade`; `agents.swarm.drone.DroneAgent`; `claude_agent_sdk.AssistantMessage`, `ToolUseBlock`.
- Produces:
  - `TIERS: dict[str, str]` — `{"opus": "claude-opus-4-8", "sonnet": "claude-sonnet-4-6", "haiku": "claude-haiku-4-5-20251001"}`.
  - `model_for(assignment: dict, role: str) -> str | None` — `TIERS[assignment[role]]` (role `"drones"` for single-drone), `None` if unset.
  - `class Trace` with `steps: int`, `first_action_t: float | None`, and `def observe(self, msg, now)` that increments `steps` per `ToolUseBlock` and stamps `first_action_t` once.
  - `@dataclass CellResult(task_id, assignment_label, repeat, passed, checks, latency_s, steps, infra_fail, failure_reason)` with `def to_row(self) -> dict`.
  - `async def run_cell(spec, assignment, repeat, deps) -> CellResult`, where `deps` bundles the live `world, bridge, cameras, systems` for soft-reset + sampling.

- [ ] **Step 1: Write the failing test (pure trace accounting)**

```python
# tests/evals/test_runner.py
from evals.runner import Trace, model_for, CellResult


class FakeTool:  # stands in for ToolUseBlock duck-typing in the test
    pass


def test_model_for_maps_tier():
    assert model_for({"drones": "haiku"}, "drones") == "claude-haiku-4-5-20251001"
    assert model_for({}, "drones") is None


def test_trace_counts_tooluse_and_stamps_first(monkeypatch):
    import evals.runner as r
    # Treat FakeTool as the ToolUseBlock type for this test.
    monkeypatch.setattr(r, "ToolUseBlock", FakeTool)

    class Msg:
        def __init__(self, content):
            self.content = content
    monkeypatch.setattr(r, "AssistantMessage", Msg)

    tr = Trace()
    tr.observe(Msg([FakeTool()]), now=5.0)
    tr.observe(Msg([FakeTool(), FakeTool()]), now=6.0)
    assert tr.steps == 3
    assert tr.first_action_t == 5.0


def test_cellresult_row_roundtrip():
    cr = CellResult("t1", "drones=haiku", 0, True, [], 12.3, 4, False, "")
    row = cr.to_row()
    assert row["task_id"] == "t1" and row["passed"] is True and row["steps"] == 4
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_runner.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.runner'`

- [ ] **Step 3a: Thread `model` through the drone options**

In `agents/flight/tools.py`, change the signature:

```python
def make_drone_options(i, drone, world, bridge, n, cameras, report, env=None, model=None):
```

and in the `return ClaudeAgentOptions(` call at the end of that function, add the `model` argument (place it right after `env=env or {},`):

```python
        env=env or {},
        model=model,
```

In `agents/swarm/drone.py`, thread it through `DroneAgent`:

```python
    def __init__(self, i: int, world, bridge, n: int, cameras, env=None, model=None) -> None:
```

and update the `make_drone_options(...)` call inside `__init__` to pass it:

```python
        self.client = ClaudeSDKClient(options=make_drone_options(
            i, self._system, world, bridge, n, cameras, self.report, env, model))
```

(Defaults keep the existing swarm callers unchanged.)

- [ ] **Step 3b: Write the runner**

```python
# evals/runner.py
"""Run ONE eval cell (task x model x repeat) against a live sim, single-drone layer.

Builds a DroneAgent with the chosen model, soft-resets the world, starts the sampler,
injects the task prompt at the drone's own Claude client, and bounds the run by the
spec's wall-clock + step budget. Captures latency + tool-call trace, then grades the
sampled WorldTrack. Infra failures (no fix, RTL/connection errors) are flagged, not
scored as task failures."""
import asyncio
import time
from dataclasses import dataclass, field

from claude_agent_sdk import AssistantMessage, ToolUseBlock

from agents.swarm.drone import DroneAgent
from evals.oracle import grade
from evals.reset import soft_reset
from evals.sampler import Sampler

TIERS = {
    "opus": "claude-opus-4-8",
    "sonnet": "claude-sonnet-4-6",
    "haiku": "claude-haiku-4-5-20251001",
}


def model_for(assignment: dict, role: str) -> str | None:
    tier = assignment.get(role)
    return TIERS[tier] if tier else None


def assignment_label(assignment: dict) -> str:
    return ",".join(f"{k}={v}" for k, v in sorted(assignment.items())) or "default"


class Trace:
    def __init__(self) -> None:
        self.steps = 0
        self.first_action_t: float | None = None

    def observe(self, msg, now: float) -> None:
        if not isinstance(msg, AssistantMessage):
            return
        for blk in msg.content:
            if isinstance(blk, ToolUseBlock):
                self.steps += 1
                if self.first_action_t is None:
                    self.first_action_t = now


@dataclass
class CellResult:
    task_id: str
    assignment_label: str
    repeat: int
    passed: bool
    checks: list = field(default_factory=list)
    latency_s: float = 0.0
    steps: int = 0
    infra_fail: bool = False
    failure_reason: str = ""

    def to_row(self) -> dict:
        return {
            "task_id": self.task_id,
            "assignment": self.assignment_label,
            "repeat": self.repeat,
            "passed": self.passed,
            "latency_s": round(self.latency_s, 2),
            "steps": self.steps,
            "infra_fail": self.infra_fail,
            "failure_reason": self.failure_reason,
            "checks": [{"name": c.name, "passed": c.passed, "detail": c.detail}
                       for c in self.checks],
        }


@dataclass
class Deps:
    """Live handles shared across cells in one sweep (built once by run_evals)."""
    world: object
    bridge: object
    cameras: object


async def _drive(client, prompt: str, deadline_s: float, max_steps: int) -> tuple[Trace, bool, str]:
    """Inject the prompt, drain the response, enforce deadline + step budget.
    Returns (trace, crashed, reason). crashed stays False here; the oracle's `alive`
    check derives crash from geofence breach + the reason string."""
    trace = Trace()
    t0 = time.monotonic()
    reason = ""

    async def _run():
        nonlocal reason
        await client.query(prompt)
        async for msg in client.receive_response():
            trace.observe(msg, time.monotonic() - t0)
            if trace.steps > max_steps:
                reason = "step budget exceeded"
                return
    try:
        await asyncio.wait_for(_run(), timeout=deadline_s)
    except asyncio.TimeoutError:
        reason = "wall-clock deadline"
    return trace, False, reason


async def run_cell(spec, assignment: dict, repeat: int, deps: Deps) -> CellResult:
    label = assignment_label(assignment)
    base = CellResult(spec.id, label, repeat, passed=False)
    n = spec.setup.n_drones  # 1 for single_drone tasks

    drone = DroneAgent(0, deps.world, deps.bridge, n, deps.cameras,
                       model=model_for(assignment, "drones"))
    try:
        await drone.connect()
    except Exception as e:
        base.infra_fail = True
        base.failure_reason = f"connect failed: {e}"
        return base

    rr = await soft_reset([drone._system], deps.world, deps.bridge, n)
    if not rr.ok:
        base.infra_fail = True
        base.failure_reason = f"reset unclean: {rr.reason}"
        return base

    sampler = Sampler(deps.world, deps.bridge, n, spec.objects_map(),
                      geofence_m=300.0)
    samp_task = asyncio.create_task(sampler.run())
    try:
        async with drone.client:
            trace, crashed, reason = await _drive(
                drone.client, spec.prompt, spec.budget.wall_clock_s, spec.budget.max_steps)
    except Exception as e:
        sampler.stop()
        await samp_task
        base.infra_fail = True
        base.failure_reason = f"agent run errored: {e}"
        return base
    finally:
        sampler.stop()
    await samp_task

    track = sampler.track()
    # Crash is inferred by the oracle's `alive` check (geofence breach); the deadline/
    # step-budget note lives in `reason`. So run_meta only carries steps + a False crash flag.
    run_meta = {"steps": trace.steps, "crashed": False}
    g = grade(track, spec.oracle, run_meta)

    base.passed = g.passed
    base.checks = g.checks
    base.steps = trace.steps
    base.latency_s = trace.first_action_t or 0.0
    base.failure_reason = reason
    return base
```

- [ ] **Step 4: Run the pure test to verify it passes**

Run: `python -m pytest tests/evals/test_runner.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Verify the existing swarm imports still resolve (no regression)**

Run: `python -c "import agents.flight.tools, agents.swarm.drone; print('ok')"`
Expected: prints `ok` (the new defaulted params don't break callers)

- [ ] **Step 6: Commit**

```bash
git add evals/runner.py agents/flight/tools.py agents/swarm/drone.py tests/evals/test_runner.py
git commit -m "feat(evals): single-drone cell runner + model threading"
```

---

### Task 8: Matrix expansion + resume

Pure expansion of the sweep grid and resume-skip logic. No sim.

**Files:**
- Create: `evals/matrix.py`
- Test: `tests/evals/test_matrix.py`

**Interfaces:**
- Consumes: `evals.runner.assignment_label`.
- Produces:
  - `@dataclass(frozen=True) Cell(task_id, assignment: dict, repeat: int)` with `def key(self) -> str` = `"{task_id}|{label}|{repeat}"`.
  - `expand(task_ids: list[str], assignments: list[dict], k: int) -> list[Cell]` — full cross product, repeats `0..k-1`.
  - `done_keys(rows: list[dict]) -> set[str]` — `"{task_id}|{assignment}|{repeat}"` for each already-recorded row (for resume).

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_matrix.py
from evals.matrix import expand, done_keys


def test_expand_cardinality():
    cells = expand(["t1", "t2"], [{"drones": "opus"}, {"drones": "haiku"}], k=3)
    assert len(cells) == 2 * 2 * 3


def test_cell_key_stable():
    c = expand(["t1"], [{"drones": "opus"}], k=1)[0]
    assert c.key() == "t1|drones=opus|0"


def test_done_keys_roundtrips_with_cell_key():
    cells = expand(["t1"], [{"drones": "opus"}], k=2)
    rows = [{"task_id": "t1", "assignment": "drones=opus", "repeat": 0}]
    done = done_keys(rows)
    assert cells[0].key() in done
    assert cells[1].key() not in done
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_matrix.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.matrix'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/matrix.py
"""Sweep grid: cross product of {tasks} x {model assignments} x K repeats, plus
resume-skip. Pure (no sim) so the schedule is testable and a killed sweep can be
restarted, skipping cells already in results.jsonl."""
from dataclasses import dataclass

from evals.runner import assignment_label


@dataclass(frozen=True)
class Cell:
    task_id: str
    assignment: dict
    repeat: int

    def key(self) -> str:
        return f"{self.task_id}|{assignment_label(self.assignment)}|{self.repeat}"


def expand(task_ids: list[str], assignments: list[dict], k: int) -> list[Cell]:
    return [Cell(t, a, r)
            for t in task_ids
            for a in assignments
            for r in range(k)]


def done_keys(rows: list[dict]) -> set[str]:
    return {f"{r['task_id']}|{r['assignment']}|{r['repeat']}" for r in rows}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_matrix.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/matrix.py tests/evals/test_matrix.py
git commit -m "feat(evals): sweep matrix expansion + resume keys"
```

---

### Task 9: Report aggregation

Pure aggregation of result rows into per-cell success-rate + latency, rendered as Markdown. No sim.

**Files:**
- Create: `evals/report.py`
- Test: `tests/evals/test_report.py`

**Interfaces:**
- Consumes: result-row dicts (`CellResult.to_row()` shape).
- Produces:
  - `@dataclass CellAgg(task_id, assignment, k, successes, success_rate, lat_p50, lat_p95, mean_steps, failure_breakdown: dict[str, int])`.
  - `aggregate(rows: list[dict]) -> list[CellAgg]` — groups by `(task_id, assignment)`, ignores `infra_fail=True` rows in the denominator, counts failure reasons.
  - `render_markdown(aggs: list[CellAgg]) -> str` — a table sorted by task then assignment.

- [ ] **Step 1: Write the failing test**

```python
# tests/evals/test_report.py
from evals.report import aggregate, render_markdown


ROWS = [
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 0, "passed": True,
     "latency_s": 3.0, "steps": 4, "infra_fail": False, "failure_reason": ""},
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 1, "passed": False,
     "latency_s": 5.0, "steps": 9, "infra_fail": False, "failure_reason": "wall-clock deadline"},
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 2, "passed": True,
     "latency_s": 4.0, "steps": 5, "infra_fail": True, "failure_reason": "reset unclean"},
]


def test_aggregate_excludes_infra_fail_from_denominator():
    agg = {a.assignment: a for a in aggregate(ROWS)}["drones=opus"]
    assert agg.k == 2            # the infra_fail row is dropped
    assert agg.successes == 1
    assert agg.success_rate == 0.5


def test_failure_breakdown_counts_reasons():
    agg = aggregate(ROWS)[0]
    assert agg.failure_breakdown.get("wall-clock deadline") == 1


def test_render_markdown_has_header_and_rate():
    md = render_markdown(aggregate(ROWS))
    assert "success_rate" in md
    assert "t1" in md
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_report.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.report'`

- [ ] **Step 3: Write minimal implementation**

```python
# evals/report.py
"""Aggregate eval result rows into per-cell success-rate + latency, render Markdown.

Pure (no sim): reads the results.jsonl rows produced by run_evals. infra_fail rows are
excluded from the success denominator (they're harness noise, not task outcomes) so the
accuracy numbers stay honest. Answers: complexity limit (success vs task), model
trade-off (success + latency per assignment)."""
from collections import defaultdict
from dataclasses import dataclass


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    k = (len(s) - 1) * pct
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


@dataclass
class CellAgg:
    task_id: str
    assignment: str
    k: int
    successes: int
    success_rate: float
    lat_p50: float
    lat_p95: float
    mean_steps: float
    failure_breakdown: dict[str, int]


def aggregate(rows: list[dict]) -> list[CellAgg]:
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        groups[(r["task_id"], r["assignment"])].append(r)

    out: list[CellAgg] = []
    for (task_id, assignment), grp in sorted(groups.items()):
        scored = [r for r in grp if not r.get("infra_fail")]
        k = len(scored)
        successes = sum(1 for r in scored if r.get("passed"))
        lats = [r["latency_s"] for r in scored]
        steps = [r["steps"] for r in scored]
        fails: dict[str, int] = defaultdict(int)
        for r in scored:
            if not r.get("passed"):
                fails[r.get("failure_reason") or "oracle check failed"] += 1
        out.append(CellAgg(
            task_id=task_id, assignment=assignment, k=k, successes=successes,
            success_rate=(successes / k if k else 0.0),
            lat_p50=_percentile(lats, 0.5), lat_p95=_percentile(lats, 0.95),
            mean_steps=(sum(steps) / k if k else 0.0),
            failure_breakdown=dict(fails)))
    return out


def render_markdown(aggs: list[CellAgg]) -> str:
    lines = ["# Agent Task-Eval Results", "",
             "| task | assignment | k | success_rate | lat_p50 | lat_p95 | mean_steps | failures |",
             "|------|-----------|---|--------------|---------|---------|------------|----------|"]
    for a in aggs:
        fb = ", ".join(f"{kk}×{vv}" for kk, vv in a.failure_breakdown.items()) or "-"
        lines.append(f"| {a.task_id} | {a.assignment} | {a.k} | {a.success_rate:.0%} | "
                     f"{a.lat_p50:.1f}s | {a.lat_p95:.1f}s | {a.mean_steps:.1f} | {fb} |")
    return "\n".join(lines) + "\n"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/evals/test_report.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add evals/report.py tests/evals/test_report.py
git commit -m "feat(evals): per-cell success-rate + latency report"
```

---

### Task 10: CLI orchestrator + sim-gated smoke + README

Wires it together: load tasks, expand the matrix, run each cell with infra retry, stream rows to `results.jsonl`, render `RESULTS.md`. Then a real single-drone smoke run against a live sim, bounded by a hard deadline.

**Files:**
- Create: `evals/run_evals.py`
- Create: `evals/README.md`
- Test: `tests/evals/test_run_evals.py` (pure helpers only)

**Interfaces:**
- Consumes: `evals.spec.load_task`, `evals.matrix.expand`/`done_keys`, `evals.runner.run_cell`/`Deps`, `evals.report.aggregate`/`render_markdown`; `agents.core.bus.RosBridge`, `agents.core.camera.GzCameras`, `agents.world.World`.
- Produces:
  - `parse_assignments(spec_str: str) -> list[dict]` — `"drones=opus;drones=haiku"` → `[{"drones": "opus"}, {"drones": "haiku"}]`.
  - `run_with_retry(coro_fn, attempts=2)` — async mirror of `bench.run_bench.run_with_retry`: retry only while result `.infra_fail`.
  - `async def main(args)` and an `argparse` CLI.

- [ ] **Step 1: Write the failing test (pure helpers)**

```python
# tests/evals/test_run_evals.py
import asyncio
from evals.run_evals import parse_assignments, run_with_retry
from evals.runner import CellResult


def test_parse_assignments():
    got = parse_assignments("drones=opus;drones=haiku")
    assert got == [{"drones": "opus"}, {"drones": "haiku"}]


def test_run_with_retry_stops_on_non_infra():
    calls = {"n": 0}

    async def fn():
        calls["n"] += 1
        return CellResult("t", "drones=opus", 0, passed=True)

    res = asyncio.run(run_with_retry(fn, attempts=3))
    assert res.passed and calls["n"] == 1


def test_run_with_retry_retries_infra_then_returns_last():
    calls = {"n": 0}

    async def fn():
        calls["n"] += 1
        return CellResult("t", "drones=opus", 0, passed=False, infra_fail=True)

    res = asyncio.run(run_with_retry(fn, attempts=2))
    assert res.infra_fail and calls["n"] == 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/evals/test_run_evals.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'evals.run_evals'`

- [ ] **Step 3: Write the orchestrator + README**

```python
# evals/run_evals.py
"""Agent task-eval orchestrator (single-drone layer).

For each (task x model-assignment x repeat) cell: soft-reset the world, run the drone
agent on the task under its budget, grade the sampled WorldTrack, append a row to
results.jsonl (infra failures retried once, never scored as task fails). Then render
RESULTS.md. Runs SEQUENTIALLY against ONE already-running sim (launch it first with
sim/launch/swarm_sim.sh, SWARM_N=1) — parallel sims would confound the latency metric.

Usage:
  # 1) bring up a single-drone sim in the swarm container (separate shell)
  # 2) inside that container/venv:
  python -m evals.run_evals \
      --tasks evals/tasks/reach_marker_single.yaml \
      --assignments "drones=opus;drones=haiku" \
      --k 5
"""
import argparse
import asyncio
import json
import os
import time

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.world import World
from evals.matrix import expand, done_keys
from evals.report import aggregate, render_markdown
from evals.runner import Deps, run_cell
from evals.spec import load_task


def parse_assignments(spec_str: str) -> list[dict]:
    out = []
    for chunk in spec_str.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        d = {}
        for pair in chunk.split(","):
            role, _, tier = pair.partition("=")
            d[role.strip()] = tier.strip()
        out.append(d)
    return out


async def run_with_retry(coro_fn, attempts: int = 2):
    """Await coro_fn() up to `attempts` times; retry only while the result is an
    infra failure. A real PASS/FAIL is returned immediately."""
    result = None
    for _ in range(max(1, attempts)):
        result = await coro_fn()
        if not (result and result.infra_fail):
            return result
    return result


def _load_rows(path: str) -> list[dict]:
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


async def main(args) -> None:
    specs = {os.path.splitext(os.path.basename(p))[0]: load_task(p) for p in args.tasks}
    assignments = parse_assignments(args.assignments)

    out_dir = args.out or os.path.join(
        os.path.dirname(__file__), "out", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    jsonl = os.path.join(out_dir, "results.jsonl")

    done = done_keys(_load_rows(jsonl))
    cells = expand(list(specs), assignments, args.k)

    bridge = RosBridge(node_name="evals_runner")
    world = World()
    n_max = max(s.setup.n_drones for s in specs.values())
    cameras = GzCameras(n_max)
    bridge.start()
    deps = Deps(world=world, bridge=bridge, cameras=cameras)

    print(f"evals: {len(cells)} cells -> {jsonl}", flush=True)
    with open(jsonl, "a") as fh:
        for cell in cells:
            if cell.key() in done:
                print(f"skip (done): {cell.key()}", flush=True)
                continue
            spec = specs[cell.task_id]
            res = await run_with_retry(
                lambda c=cell, s=spec: run_cell(s, c.assignment, c.repeat, deps))
            fh.write(json.dumps(res.to_row()) + "\n")
            fh.flush()
            print(f"{cell.key()}: passed={res.passed} infra_fail={res.infra_fail} "
                  f"steps={res.steps} lat={res.latency_s:.1f}s", flush=True)

    rows = _load_rows(jsonl)
    md = render_markdown(aggregate(rows))
    with open(os.path.join(out_dir, "RESULTS.md"), "w") as f:
        f.write(md)
    print(md, flush=True)
    bridge.shutdown()


def _cli() -> None:
    ap = argparse.ArgumentParser(description="Agent task-eval harness (single-drone).")
    ap.add_argument("--tasks", nargs="+", required=True, help="task YAML paths")
    ap.add_argument("--assignments", default="drones=opus",
                    help="';'-separated role=tier groups, e.g. 'drones=opus;drones=haiku'")
    ap.add_argument("--k", type=int, default=5, help="repeats per cell")
    ap.add_argument("--out", default=None, help="output dir (default evals/out/<ts>)")
    args = ap.parse_args()
    asyncio.run(main(args))


if __name__ == "__main__":
    _cli()
```

```markdown
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
```

- [ ] **Step 4: Run the pure test to verify it passes**

Run: `python -m pytest tests/evals/test_run_evals.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Run the full pure suite**

Run: `python -m pytest tests/evals/ -v`
Expected: PASS (all tasks' unit tests green)

- [ ] **Step 6: Sim-gated smoke (manual, hard-deadline bounded)**

Bring up a single-drone sim, then run one cell with K=1. Bound the whole thing with a hard timeout so a hung sim can never stall the session:

```bash
timeout 600 python -m evals.run_evals \
    --tasks evals/tasks/reach_marker_single.yaml \
    --assignments "drones=opus" \
    --k 1
```

Expected: prints a cell line `reach_marker_single|drones=opus|0: passed=... steps=... lat=...s`, writes `evals/out/<ts>/results.jsonl` (one row) and `RESULTS.md` (one table row). A `passed=True` proves the full path (reset → inject → fly → sample → oracle) works end to end. If `infra_fail=True`, the sim/connection is the issue — check the sim is up with `SWARM_N=1` — not the harness logic.

- [ ] **Step 7: Commit**

```bash
git add evals/run_evals.py evals/README.md tests/evals/test_run_evals.py
git commit -m "feat(evals): orchestrator CLI + sim-gated single-drone smoke + README"
```

---

## Self-Review

**Spec coverage** (design doc → tasks):
- Module layout (`spec/areas/oracle/reset/runner/matrix/report/run_evals`) → Tasks 1–10 (worldstate split out of `spec.py` as the shared data type; justified — oracle and sampler both depend on it without depending on the loader).
- Layer-agnostic task spec + 4 difficulty axes → Task 4 (`difficulty` dict carried verbatim; single-drone tasks authored now).
- Sim-state oracle (reached/coverage/formation/ordering/alive/step-budget) → Task 3 implements `reached/coverage/alive/within_step_budget`. `formation`/`ordering` are coordination/sequence checks only meaningful at the multi-drone (Commander/swarm) layer → deferred to the phase-5 plan, consistent with this plan's single-drone scope. Registry makes adding them additive.
- Position-overflight coverage + spec-seeded logical targets → Task 3 `_coverage`, Task 4 `seed_objects`.
- RTL soft-reset + home health check → Task 6.
- Model matrix incl. per-role assignment → Task 7 (`TIERS`, `model_for`, threaded into `ClaudeAgentOptions(model=...)`); `assignment` is a role→tier dict so per-role mixes already expressible.
- K repeats, success-rate + latency dist, failure breakdown → Tasks 8–9.
- Sequential execution, resumable, timestamped out-dir, infra-fail retry-once → Tasks 8 (`done_keys`) + 10 (`run_with_retry`, out-dir).
- Trace capture (steps, latency-to-first-action) → Task 7 `Trace`.

**Deferred (explicitly, with rationale):** Commander/swarm layers + `formation`/`ordering` checks + prompt/tooling iteration dimension (phases 5–6); physical Gazebo marker spawning + camera-footprint coverage (design "out of scope"). None are required for a working single-drone harness.

**Placeholder scan:** No TBD/TODO; every code step shows complete code; every test shows real assertions. `run_cell` builds a single `run_meta = {"steps": ..., "crashed": False}` — crash is inferred via the oracle's `alive`/geofence check, per design.

**Type consistency:** `WorldTrack`/`Snapshot`/`DronePose` identical across Tasks 1/3/5. `CheckResult.detail` used by `CellResult.to_row()` (Task 7) matches Task 3. `assignment_label` defined in Task 7, reused in Tasks 8/9. `CellResult.to_row()` keys (`task_id/assignment/repeat/passed/latency_s/steps/infra_fail/failure_reason`) match the row shape consumed by `report.aggregate` (Task 9), `matrix.done_keys` (Task 8), and `run_evals._load_rows` (Task 10). `model_for(assignment, "drones")` role key matches `parse_assignments` output (Task 10).
