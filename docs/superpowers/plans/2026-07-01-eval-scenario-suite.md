# Eval Scenario Suite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Grow the `evals/` harness from one anchor task into a graduated scenario suite (per-axis ladders + compound capstones) that makes the harness discriminate between Claude tiers.

**Architecture:** Five new pure sim-oracle checks over `WorldTrack` (`visited_all`, `ordering`, `altitude`, `clearance`, `dwell`), a `buildings` field on `WorldTrack` populated by the sampler, a `suite` tag on task specs + a report "ladder" pivot, and ~13 hand-authored task YAMLs across flat (`default`) and city worlds. Obstacle rungs are gated on a city-world altitude-frame validation.

**Tech Stack:** Python 3.11+, `pytest`, `pyyaml`. Reuses the validated single-drone harness (`evals/`), `agents/world.World`, and the `default`/`city` PX4 gz worlds.

## Global Constraints

- Python target: 3.11+ (`str | None` unions). Copy verbatim.
- Pure modules (`worldstate.py`, `oracle.py`, `areas.py`, `spec.py`, `report.py`) MUST NOT import `rclpy`/`mavsdk`/`claude_agent_sdk` — they unit-test without ROS.
- Grading is **sim-state oracle only**; every check is a pure `fn(track, params, run_meta) -> CheckResult` registered in `oracle.CHECKS`. A task passes iff ALL its checks pass.
- Targets are **spec-declared coordinates** (world ENU: east, north). No physical Gazebo markers.
- Worlds: flat `default` for plan-depth/spatial/ambiguity/`c1`; `city` for obstacle + `c2`. **`baylands` is banned** for evals (terrain offsets PX4's local-altitude frame ~570 m — see `evals/README.md`).
- Building box schema (from `<world>_boxes.json`, via `World.buildings`): `{name, x, y, w, d, h, color}` where `x`=east-center, `y`=north-center, `w`=east extent, `d`=north extent. `clearance` uses `w`/`d` as full extents (half-extent = `w/2`, `d/2`).
- Tier→id (verbatim): `opus`=claude-opus-4-8, `sonnet`=claude-sonnet-5, `haiku`=claude-haiku-4-5-20251001.
- Difficulty metadata: each task's `difficulty` dict carries its ladder axis at the rung's level (e.g. spatial rung 2 → `difficulty.spatial: 2`); the task's `suite` field names that axis so the report can pivot.
- Run with `python -m pytest tests/evals/ -v` from repo root; run the FULL evals suite once before each commit.
- Leave unrelated working-tree changes alone; commit only each task's listed files.

**Scope of THIS plan:** the scenario suite + its grading. Live multi-cell sweeps and the city-world live smoke are operator/controller steps (called out, not automated). Commander/swarm layers remain deferred.

---

### Task 1: `WorldTrack.buildings` + sampler building capture

Give the track the static building boxes `clearance` needs; have the sampler capture them once from `World.buildings`.

**Files:**
- Modify: `evals/worldstate.py`
- Modify: `evals/sampler.py`
- Test: `tests/evals/test_worldstate.py`, `tests/evals/test_sampler.py`

**Interfaces:**
- Consumes: `agents.world.World.buildings` (list of box dicts) at runtime; not imported in pure code.
- Produces:
  - `WorldTrack.buildings: list[dict]` (default `[]`) — building boxes for clearance grading.
  - `Sampler.track()` returns a `WorldTrack` whose `buildings` come from the world (captured at `Sampler.__init__`).

- [ ] **Step 1: Write the failing tests**

Add to `tests/evals/test_worldstate.py`:
```python
def test_worldtrack_buildings_defaults_empty():
    from evals.worldstate import WorldTrack
    t = WorldTrack(snapshots=[], objects={}, n_drones=1, geofence_m=300.0)
    assert t.buildings == []


def test_worldtrack_buildings_carried():
    from evals.worldstate import WorldTrack
    b = [{"name": "bldg_9", "x": 43.8, "y": 14.4, "w": 6.5, "d": 6.2}]
    t = WorldTrack(snapshots=[], objects={}, n_drones=1, geofence_m=300.0, buildings=b)
    assert t.buildings[0]["name"] == "bldg_9"
```

Add to `tests/evals/test_sampler.py`:
```python
def test_sampler_captures_buildings_from_world():
    from evals.sampler import Sampler

    class WorldWithBuildings:
        buildings = [{"name": "b0", "x": 1.0, "y": 2.0, "w": 3.0, "d": 4.0}]

        def drone_state(self, bridge, i):
            return None

    s = Sampler(WorldWithBuildings(), bridge=None, n_drones=1, objects={}, geofence_m=300.0)
    assert s.track().buildings == [{"name": "b0", "x": 1.0, "y": 2.0, "w": 3.0, "d": 4.0}]


def test_sampler_buildings_empty_when_world_has_none():
    from evals.sampler import Sampler

    class BareWorld:
        def drone_state(self, bridge, i):
            return None

    s = Sampler(BareWorld(), bridge=None, n_drones=1, objects={}, geofence_m=300.0)
    assert s.track().buildings == []
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest tests/evals/test_worldstate.py::test_worldtrack_buildings_carried tests/evals/test_sampler.py::test_sampler_captures_buildings_from_world -v`
Expected: FAIL (`TypeError: __init__() got an unexpected keyword argument 'buildings'` and `AssertionError`).

- [ ] **Step 3: Implement**

In `evals/worldstate.py`, change the import and add the field:
```python
from dataclasses import dataclass, field
```
In the `WorldTrack` dataclass, add after `geofence_m: float`:
```python
    buildings: list[dict] = field(default_factory=list)
```

In `evals/sampler.py`, capture buildings in `Sampler.__init__` and pass them in `track()`:
```python
    def __init__(self, world, bridge, n_drones, objects, geofence_m, interval=0.5):
        self._world = world
        self._bridge = bridge
        self._n = n_drones
        self._objects = dict(objects)
        self._geofence_m = geofence_m
        self._interval = interval
        self._buildings = list(getattr(world, "buildings", []) or [])
        self._snaps: list[Snapshot] = []
        self._running = False
```
and:
```python
    def track(self) -> WorldTrack:
        return WorldTrack(snapshots=list(self._snaps), objects=dict(self._objects),
                          n_drones=self._n, geofence_m=self._geofence_m,
                          buildings=list(self._buildings))
```

- [ ] **Step 4: Run to verify they pass**

Run: `python -m pytest tests/evals/test_worldstate.py tests/evals/test_sampler.py -v`
Expected: PASS (all, including the four new).

- [ ] **Step 5: Commit**

```bash
git add evals/worldstate.py evals/sampler.py tests/evals/test_worldstate.py tests/evals/test_sampler.py
git commit -m "feat(evals): WorldTrack.buildings + sampler building capture"
```

---

### Task 2: oracle checks `visited_all` + `ordering`

Multi-target reach (any order) and strict sequence — unlocks the plan-depth ladder.

**Files:**
- Modify: `evals/oracle.py`
- Test: `tests/evals/test_oracle.py`

**Interfaces:**
- Consumes: `WorldTrack.min_dist_to`, `WorldTrack.snapshots`, `WorldTrack.objects`.
- Produces: `CHECKS["visited_all"]`, `CHECKS["ordering"]`, and module-level helper `_first_reach_time(track, xy, tol) -> float | None`.

- [ ] **Step 1: Write the failing tests**

Add to `tests/evals/test_oracle.py`:
```python
def _route_track(reach_c=True):
    # Drone visits a(10,0) at t=1, b(10,10) at t=2, then c(0,10) at t=3 (if reach_c).
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(10.0, 0.0, 12.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(10.0, 10.0, 12.0, 0.0)}),
        Snapshot(3.0, {0: DronePose(0.0 if reach_c else 40.0, 10.0, 12.0, 0.0)}),
    ]
    objs = {"a": (10.0, 0.0), "b": (10.0, 10.0), "c": (0.0, 10.0)}
    return WorldTrack(snaps, objs, n_drones=1, geofence_m=300.0)


def test_visited_all_pass_and_fail():
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    spec = [{"check": "visited_all", "targets": ["a", "b", "c"], "tol_m": 3}]
    assert grade(_route_track(True), spec, ok).passed
    assert not grade(_route_track(False), spec, ok).passed  # c missed


def test_ordering_pass_when_in_sequence():
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    spec = [{"check": "ordering", "sequence": ["a", "b", "c"], "tol_m": 3}]
    assert grade(_route_track(True), spec, ok).passed


def test_ordering_fails_when_out_of_sequence():
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    # require c BEFORE a — the track reaches a first, so ordering must fail
    spec = [{"check": "ordering", "sequence": ["c", "a", "b"], "tol_m": 3}]
    assert not grade(_route_track(True), spec, ok).passed
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest tests/evals/test_oracle.py -k "visited_all or ordering" -v`
Expected: FAIL with `KeyError: 'visited_all'` (check not registered).

- [ ] **Step 3: Implement**

In `evals/oracle.py`, add the helper and checks, then register them:
```python
def _first_reach_time(track: WorldTrack, xy: tuple, tol: float) -> float | None:
    for s in track.snapshots:
        for pose in s.poses.values():
            if math.hypot(pose.e - xy[0], pose.n - xy[1]) <= tol:
                return s.t
    return None


def _visited_all(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    tol = float(p["tol_m"])
    targets = list(p["targets"])
    missed = [t for t in targets if track.min_dist_to(track.objects[t]) > tol]
    n = len(targets)
    return CheckResult("visited_all", not missed, float(n - len(missed)),
                       f"visited {n - len(missed)}/{n} within {tol:g}m; missed {missed}")


def _ordering(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    tol = float(p["tol_m"])
    seq = list(p["sequence"])
    times = [_first_reach_time(track, track.objects[t], tol) for t in seq]
    reached_all = all(x is not None for x in times)
    in_order = reached_all and all(times[i] < times[i + 1] for i in range(len(times) - 1))
    return CheckResult("ordering", in_order, float(sum(x is not None for x in times)),
                       f"first-reach times {times} for {seq} (need all set & increasing)")
```
Add both to the `CHECKS` dict:
```python
    "visited_all": _visited_all,
    "ordering": _ordering,
```

- [ ] **Step 4: Run to verify they pass**

Run: `python -m pytest tests/evals/test_oracle.py -v`
Expected: PASS (all, including the three new).

- [ ] **Step 5: Commit**

```bash
git add evals/oracle.py tests/evals/test_oracle.py
git commit -m "feat(evals): oracle visited_all + ordering checks"
```

---

### Task 3: oracle checks `altitude` + `dwell`

Altitude band at a target, and continuous hold near a target — unlocks spatial-altitude rungs and the dwell capstone.

**Files:**
- Modify: `evals/oracle.py`
- Test: `tests/evals/test_oracle.py`

**Interfaces:**
- Consumes: `WorldTrack.snapshots`, `WorldTrack.objects`.
- Produces: `CHECKS["altitude"]`, `CHECKS["dwell"]`, helpers `_closest_pose_to(track, xy) -> DronePose | None`, `_max_dwell(track, xy, tol) -> float`.

- [ ] **Step 1: Write the failing tests**

Add to `tests/evals/test_oracle.py`:
```python
def test_altitude_band_at_target():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    # Closest approach to tgt (100,0) is at t=2 where alt=20 -> in [18,22].
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 5.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(50.0, 0.0, 20.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(100.0, 0.0, 20.0, 0.0)}),
    ]
    t = WorldTrack(snaps, {"tgt": (100.0, 0.0)}, n_drones=1, geofence_m=300.0)
    ok = {"steps": 5, "crashed": False}
    assert grade(t, [{"check": "altitude", "target": "tgt", "min_m": 18, "max_m": 22}], ok).passed
    assert not grade(t, [{"check": "altitude", "target": "tgt", "min_m": 25, "max_m": 30}], ok).passed


def test_dwell_holds_long_enough():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    # Within 3m of tgt(0,0) from t=1..t=4 -> a 3s continuous hold.
    snaps = [
        Snapshot(0.0, {0: DronePose(50.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(1.0, 0.0, 12.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(0.5, 0.0, 12.0, 0.0)}),
        Snapshot(3.0, {0: DronePose(1.0, 0.0, 12.0, 0.0)}),
        Snapshot(4.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
    ]
    t = WorldTrack(snaps, {"tgt": (0.0, 0.0)}, n_drones=1, geofence_m=300.0)
    ok = {"steps": 5, "crashed": False}
    assert grade(t, [{"check": "dwell", "target": "tgt", "tol_m": 3, "hold_s": 2.5}], ok).passed
    assert not grade(t, [{"check": "dwell", "target": "tgt", "tol_m": 3, "hold_s": 5}], ok).passed
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest tests/evals/test_oracle.py -k "altitude or dwell" -v`
Expected: FAIL with `KeyError: 'altitude'`.

- [ ] **Step 3: Implement**

In `evals/oracle.py`, add helpers + checks and register:
```python
def _closest_pose_to(track: WorldTrack, xy: tuple):
    best, bd = None, math.inf
    for s in track.snapshots:
        for pose in s.poses.values():
            d = math.hypot(pose.e - xy[0], pose.n - xy[1])
            if d < bd:
                bd, best = d, pose
    return best


def _max_dwell(track: WorldTrack, xy: tuple, tol: float) -> float:
    best, run_start = 0.0, None
    for s in track.snapshots:
        inside = any(math.hypot(pose.e - xy[0], pose.n - xy[1]) <= tol
                     for pose in s.poses.values())
        if inside:
            if run_start is None:
                run_start = s.t
            best = max(best, s.t - run_start)
        else:
            run_start = None
    return best


def _altitude(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    lo, hi = float(p["min_m"]), float(p["max_m"])
    pose = _closest_pose_to(track, track.objects[p["target"]])
    alt = pose.alt if pose is not None else None
    ok = alt is not None and lo <= alt <= hi
    return CheckResult("altitude", ok, float(alt or 0.0),
                       f"alt {alt}m at {p['target']} (band [{lo:g},{hi:g}])")


def _dwell(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    tol, need = float(p["tol_m"]), float(p["hold_s"])
    held = _max_dwell(track, track.objects[p["target"]], tol)
    return CheckResult("dwell", held >= need, held,
                       f"held {held:.1f}s within {tol:g}m of {p['target']} (need {need:g}s)")
```
Register:
```python
    "altitude": _altitude,
    "dwell": _dwell,
```

- [ ] **Step 4: Run to verify they pass**

Run: `python -m pytest tests/evals/test_oracle.py -v`
Expected: PASS (all).

- [ ] **Step 5: Commit**

```bash
git add evals/oracle.py tests/evals/test_oracle.py
git commit -m "feat(evals): oracle altitude + dwell checks"
```

---

### Task 4: oracle check `clearance`

Min distance from the drone to any building footprint stayed ≥ margin — unlocks the obstacle ladder.

**Files:**
- Modify: `evals/oracle.py`
- Test: `tests/evals/test_oracle.py`

**Interfaces:**
- Consumes: `WorldTrack.buildings` (Task 1), `WorldTrack.snapshots`.
- Produces: `CHECKS["clearance"]`, helper `_min_building_clearance(track) -> float` (`inf` when no buildings, so flat worlds pass trivially).

- [ ] **Step 1: Write the failing tests**

Add to `tests/evals/test_oracle.py`:
```python
def test_clearance_passes_when_far_and_no_buildings():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    snaps = [Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)})]
    # No buildings -> inf clearance -> passes.
    assert grade(WorldTrack(snaps, {}, 1, 300.0), [{"check": "clearance", "margin_m": 5}], ok).passed


def test_clearance_fails_on_near_miss():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    # Building footprint centered (10,0), half-extents 3 x 3 -> edge at e=7.
    # Drone passes at e=8 -> clearance 1m < margin 5 -> fail.
    b = [{"name": "b0", "x": 10.0, "y": 0.0, "w": 6.0, "d": 6.0}]
    snaps = [Snapshot(0.0, {0: DronePose(8.0, 0.0, 12.0, 0.0)})]
    t = WorldTrack(snaps, {}, 1, 300.0, buildings=b)
    assert not grade(t, [{"check": "clearance", "margin_m": 5}], ok).passed


def test_clearance_passes_when_routed_around():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    b = [{"name": "b0", "x": 10.0, "y": 0.0, "w": 6.0, "d": 6.0}]
    # Drone passes at n=20 -> far from the box -> clearance ~17m >= 5 -> pass.
    snaps = [Snapshot(0.0, {0: DronePose(10.0, 20.0, 12.0, 0.0)})]
    t = WorldTrack(snaps, {}, 1, 300.0, buildings=b)
    assert grade(t, [{"check": "clearance", "margin_m": 5}], ok).passed
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest tests/evals/test_oracle.py -k clearance -v`
Expected: FAIL with `KeyError: 'clearance'`.

- [ ] **Step 3: Implement**

In `evals/oracle.py`, add helper + check and register:
```python
def _min_building_clearance(track: WorldTrack) -> float:
    if not track.buildings:
        return math.inf
    best = math.inf
    for s in track.snapshots:
        for pose in s.poses.values():
            for b in track.buildings:
                dx = max(abs(pose.e - b["x"]) - b["w"] / 2.0, 0.0)
                dy = max(abs(pose.n - b["y"]) - b["d"] / 2.0, 0.0)
                best = min(best, math.hypot(dx, dy))
    return best


def _clearance(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    margin = float(p["margin_m"])
    d = _min_building_clearance(track)
    shown = "inf" if d == math.inf else f"{d:.1f}m"
    return CheckResult("clearance", d >= margin, 0.0 if d == math.inf else d,
                       f"min clearance {shown} (margin {margin:g})")
```
Register:
```python
    "clearance": _clearance,
```

- [ ] **Step 4: Run to verify they pass**

Run: `python -m pytest tests/evals/test_oracle.py -v`
Expected: PASS (all).

- [ ] **Step 5: Commit**

```bash
git add evals/oracle.py tests/evals/test_oracle.py
git commit -m "feat(evals): oracle clearance check (building footprint distance)"
```

---

### Task 5: `suite` tag + result-row plumbing + report ladder pivot

Carry each task's ladder axis into the result row and render a per-axis success-rate pivot (the knee view).

**Files:**
- Modify: `evals/spec.py`
- Modify: `evals/runner.py`
- Modify: `evals/report.py`
- Test: `tests/evals/test_spec.py`, `tests/evals/test_runner.py`, `tests/evals/test_report.py`

**Interfaces:**
- Consumes: `TaskSpec.difficulty` (dict), `TaskSpec.suite`.
- Produces:
  - `TaskSpec.suite: str | None` (default `None`).
  - `CellResult.difficulty: dict` and `CellResult.suite: str | None`; `to_row()` emits `"difficulty"` and `"suite"`.
  - `report.render_ladders(rows: list[dict]) -> str` — per-suite table: rows = rungs (`difficulty[suite]`), cols = assignments, cells = success-rate; excludes `infra_fail` and rows without a `suite`.

- [ ] **Step 1: Write the failing tests**

Add to `tests/evals/test_spec.py`:
```python
def test_suite_field_loads_and_defaults(tmp_path):
    from evals.spec import load_task
    with_suite = VALID.replace("target_layer: single_drone",
                               "target_layer: single_drone\nsuite: spatial")
    assert load_task(_write(tmp_path, with_suite)).suite == "spatial"
    assert load_task(_write(tmp_path, VALID)).suite is None
```

Add to `tests/evals/test_runner.py`:
```python
def test_cellresult_row_carries_suite_and_difficulty():
    from evals.runner import CellResult
    cr = CellResult("t1", "drones=opus", 0, True, [], 3.0, 4, False, "",
                    difficulty={"spatial": 2}, suite="spatial")
    row = cr.to_row()
    assert row["suite"] == "spatial" and row["difficulty"] == {"spatial": 2}
```

Add to `tests/evals/test_report.py`:
```python
def test_render_ladders_pivots_by_rung_and_tier():
    from evals.report import render_ladders
    rows = [
        {"task_id": "s1", "assignment": "drones=haiku", "passed": True, "infra_fail": False,
         "suite": "spatial", "difficulty": {"spatial": 1}, "latency_s": 3, "steps": 4, "repeat": 0},
        {"task_id": "s3", "assignment": "drones=haiku", "passed": False, "infra_fail": False,
         "suite": "spatial", "difficulty": {"spatial": 3}, "latency_s": 3, "steps": 4, "repeat": 0},
        {"task_id": "x", "assignment": "drones=haiku", "passed": True, "infra_fail": False,
         "suite": None, "difficulty": {}, "latency_s": 3, "steps": 4, "repeat": 0},
    ]
    md = render_ladders(rows)
    assert "spatial" in md
    assert "drones=haiku" in md
    # rung 1 = 100%, rung 3 = 0%
    assert "100%" in md and "0%" in md
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest tests/evals/test_spec.py::test_suite_field_loads_and_defaults tests/evals/test_runner.py::test_cellresult_row_carries_suite_and_difficulty tests/evals/test_report.py::test_render_ladders_pivots_by_rung_and_tier -v`
Expected: FAIL (`TypeError`/`AttributeError`/`ImportError`).

- [ ] **Step 3: Implement**

In `evals/spec.py`, add `suite` to the `TaskSpec` dataclass (after `oracle: list[dict]`):
```python
    suite: str | None = None
```
and set it in `load_task`'s `return TaskSpec(...)` (add the kwarg):
```python
        oracle=oracle,
        suite=raw.get("suite"),
```

In `evals/runner.py`, add two fields to `CellResult` (after `failure_reason: str = ""`):
```python
    difficulty: dict = field(default_factory=dict)
    suite: str | None = None
```
add them to `to_row()` (inside the returned dict):
```python
            "difficulty": self.difficulty,
            "suite": self.suite,
```
and populate them in `run_cell`, immediately after `base = CellResult(spec.id, label, repeat, passed=False)`:
```python
    base.difficulty = dict(spec.difficulty)
    base.suite = spec.suite
```

In `evals/report.py`, add:
```python
def render_ladders(rows: list[dict]) -> str:
    """Per-suite pivot: success-rate by rung (difficulty[suite]) x assignment — the knee view.
    Skips infra_fail rows and rows without a suite."""
    from collections import defaultdict
    suites: dict[str, dict] = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    assigns: set[str] = set()
    for r in rows:
        if r.get("infra_fail"):
            continue
        suite = r.get("suite")
        if not suite:
            continue
        rung = (r.get("difficulty") or {}).get(suite, 0)
        a = r["assignment"]
        assigns.add(a)
        suites[suite][rung][a].append(bool(r.get("passed")))

    cols = sorted(assigns)
    lines = ["# Ladders (success-rate by rung x tier)"]
    for suite in sorted(suites):
        lines += ["", f"## {suite} ladder", "",
                  "| rung | " + " | ".join(cols) + " |",
                  "|------|" + "|".join(["------"] * len(cols)) + "|"]
        for rung in sorted(suites[suite]):
            cells = []
            for a in cols:
                res = suites[suite][rung].get(a, [])
                cells.append(f"{100.0 * sum(res) / len(res):.0f}%" if res else "-")
            lines.append(f"| {rung} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"
```

Also, in `evals/run_evals.py` `main()`, after writing `RESULTS.md`, also write the ladder view (find the block that renders `render_markdown` and append):
```python
        from evals.report import render_ladders
        with open(os.path.join(out_dir, "LADDERS.md"), "w") as f:
            f.write(render_ladders(rows))
```

- [ ] **Step 4: Run to verify they pass**

Run: `python -m pytest tests/evals/ -v`
Expected: PASS (all).

- [ ] **Step 5: Commit**

```bash
git add evals/spec.py evals/runner.py evals/report.py evals/run_evals.py tests/evals/test_spec.py tests/evals/test_runner.py tests/evals/test_report.py
git commit -m "feat(evals): suite tag + result-row plumbing + report ladder pivot"
```

---

### Task 6: flat-world scenario suite (plan-depth, spatial, ambiguity, capstone c1)

Author the flat-world ladders + `c1`. All grade on the validated `default` world.

**Files:**
- Create: `evals/tasks/plan_depth/p1_route2.yaml`, `p2_route3.yaml`, `p3_route4.yaml`
- Create: `evals/tasks/spatial/s1_dist60.yaml`, `s2_dist130.yaml`, `s3_dist250.yaml`, `s4_altband.yaml`
- Create: `evals/tasks/ambiguity/am1_explicit.yaml`, `am2_relative.yaml`, `am3_search.yaml`
- Create: `evals/tasks/capstone/c1_recon_patrol.yaml`
- Test: `tests/evals/test_task_files.py`

**Interfaces:**
- Consumes: `evals.spec.load_task`; the oracle checks from Tasks 2–4; `evals.areas.AREAS` (`ne_quadrant`).
- Produces: 11 task YAMLs, each with a `suite` tag and `difficulty[suite]` = rung level.

- [ ] **Step 1: Write the failing test**

Create `tests/evals/test_task_files.py`:
```python
import glob
from evals.spec import load_task


def test_all_flat_world_tasks_load_and_are_flat():
    paths = (glob.glob("evals/tasks/plan_depth/*.yaml")
             + glob.glob("evals/tasks/spatial/*.yaml")
             + glob.glob("evals/tasks/ambiguity/*.yaml")
             + glob.glob("evals/tasks/capstone/c1_*.yaml"))
    assert len(paths) == 11
    for p in paths:
        t = load_task(p)
        assert t.setup.world in ("default", "lawn"), f"{p} must use a flat world"
        assert t.setup.n_drones == 1
        assert t.suite in ("plan_depth", "spatial", "ambiguity", "capstone")
        assert t.difficulty.get(t.suite) is not None, f"{p} missing difficulty[{t.suite}]"
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest tests/evals/test_task_files.py -v`
Expected: FAIL (`assert 0 == 11` — no files yet).

- [ ] **Step 3: Create the task files**

`evals/tasks/plan_depth/p1_route2.yaml`:
```yaml
id: p1_route2
target_layer: single_drone
suite: plan_depth
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: a, east: 60, north: 0}
    - {id: b, east: 60, north: 60}
prompt: "Take off to 12 m, then fly to point a (east 60, north 0), then to point b (east 60, north 60), in that order. Hover at b."
budget: {wall_clock_s: 150, max_steps: 12}
oracle:
  - {check: alive}
  - {check: ordering, sequence: [a, b], tol_m: 12}
  - {check: within_step_budget, max_steps: 12}
```

`evals/tasks/plan_depth/p2_route3.yaml`:
```yaml
id: p2_route3
target_layer: single_drone
suite: plan_depth
difficulty: {plan_depth: 2, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: a, east: 60, north: 0}
    - {id: b, east: 60, north: 60}
    - {id: c, east: 0, north: 60}
prompt: "Take off to 12 m, then visit these points in order and hover at the last: a (east 60, north 0), b (east 60, north 60), c (east 0, north 60)."
budget: {wall_clock_s: 180, max_steps: 16}
oracle:
  - {check: alive}
  - {check: ordering, sequence: [a, b, c], tol_m: 12}
  - {check: within_step_budget, max_steps: 16}
```

`evals/tasks/plan_depth/p3_route4.yaml`:
```yaml
id: p3_route4
target_layer: single_drone
suite: plan_depth
difficulty: {plan_depth: 3, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: a, east: 60, north: 0}
    - {id: b, east: 60, north: 60}
    - {id: c, east: 0, north: 60}
    - {id: d, east: 0, north: 0}
prompt: "Take off to 12 m, then patrol these points in order, returning to the last: a (east 60, north 0), b (east 60, north 60), c (east 0, north 60), d (east 0, north 0)."
budget: {wall_clock_s: 220, max_steps: 20}
oracle:
  - {check: alive}
  - {check: ordering, sequence: [a, b, c, d], tol_m: 12}
  - {check: within_step_budget, max_steps: 20}
```

`evals/tasks/spatial/s1_dist60.yaml`:
```yaml
id: s1_dist60
target_layer: single_drone
suite: spatial
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 1}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: tgt, east: 60, north: 0}
prompt: "Take off to 12 m and fly to the marker tgt at east 60, north 0. Hover there."
budget: {wall_clock_s: 120, max_steps: 8}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 10}
  - {check: within_step_budget, max_steps: 8}
```

`evals/tasks/spatial/s2_dist130.yaml`:
```yaml
id: s2_dist130
target_layer: single_drone
suite: spatial
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: tgt, east: 130, north: 0}
prompt: "Take off to 12 m and fly to the marker tgt at east 130, north 0. Hover there."
budget: {wall_clock_s: 150, max_steps: 8}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 10}
  - {check: within_step_budget, max_steps: 8}
```

`evals/tasks/spatial/s3_dist250.yaml`:
```yaml
id: s3_dist250
target_layer: single_drone
suite: spatial
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 3}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: tgt, east: 250, north: 0}
prompt: "Take off to 12 m and fly to the marker tgt at east 250, north 0. Hover there."
budget: {wall_clock_s: 200, max_steps: 8}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 10}
  - {check: within_step_budget, max_steps: 8}
```

`evals/tasks/spatial/s4_altband.yaml`:
```yaml
id: s4_altband
target_layer: single_drone
suite: spatial
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 4}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: tgt, east: 100, north: 0}
prompt: "Take off, climb to 20 m, then fly to the marker tgt at east 100, north 0 and hover there at 20 m."
budget: {wall_clock_s: 160, max_steps: 10}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 12}
  - {check: altitude, target: tgt, min_m: 18, max_m: 22}
  - {check: within_step_budget, max_steps: 10}
```

`evals/tasks/ambiguity/am1_explicit.yaml`:
```yaml
id: am1_explicit
target_layer: single_drone
suite: ambiguity
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: tgt, east: 120, north: -40}
prompt: "Take off to 12 m and fly to the marker tgt at world east 120, north -40. Hover there."
budget: {wall_clock_s: 150, max_steps: 10}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 15}
  - {check: within_step_budget, max_steps: 10}
```

`evals/tasks/ambiguity/am2_relative.yaml`:
```yaml
id: am2_relative
target_layer: single_drone
suite: ambiguity
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 3, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    # Implied point: ~120 m at compass north-east (45 deg) from home -> (85, 85).
    - {id: ne, east: 85, north: 85}
prompt: "Take off to 12 m, then fly roughly 120 metres to the north-east and stop. Hover where you end up."
budget: {wall_clock_s: 150, max_steps: 10}
oracle:
  - {check: alive}
  - {check: reached, target: ne, tol_m: 30}
  - {check: within_step_budget, max_steps: 10}
```

`evals/tasks/ambiguity/am3_search.yaml`:
```yaml
id: am3_search
target_layer: single_drone
suite: ambiguity
difficulty: {plan_depth: 2, coordination: 1, ambiguity: 4, spatial: 2}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects: []
prompt: "Take off to 15 m and search the north-east area (roughly the 200 m x 200 m region north-east of home), covering as much of it as you can."
budget: {wall_clock_s: 240, max_steps: 25}
oracle:
  - {check: alive}
  - {check: coverage, area: ne_quadrant, min_pct: 60, radius_m: 20, cell_m: 20}
  - {check: within_step_budget, max_steps: 25}
```

`evals/tasks/capstone/c1_recon_patrol.yaml`:
```yaml
id: c1_recon_patrol
target_layer: single_drone
suite: capstone
difficulty: {plan_depth: 3, coordination: 1, ambiguity: 2, spatial: 3}
setup:
  world: default
  n_drones: 1
  spawn: home
  seed_objects:
    - {id: a, east: 50, north: 0}
    - {id: b, east: 50, north: 50}
    - {id: c, east: 0, north: 50}
prompt: "Take off and climb to about 18 m. Patrol these points in order at that altitude: a (east 50, north 0), b (east 50, north 50), c (east 0, north 50). Then hold your position over c for at least 12 seconds."
budget: {wall_clock_s: 240, max_steps: 20}
oracle:
  - {check: alive}
  - {check: ordering, sequence: [a, b, c], tol_m: 12}
  - {check: altitude, target: c, min_m: 14, max_m: 22}
  - {check: dwell, target: c, tol_m: 12, hold_s: 8}
  - {check: within_step_budget, max_steps: 20}
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest tests/evals/test_task_files.py -v`
Expected: PASS (11 files load; all flat; suites/difficulty valid).

- [ ] **Step 5: Commit**

```bash
git add evals/tasks/plan_depth evals/tasks/spatial evals/tasks/ambiguity evals/tasks/capstone/c1_recon_patrol.yaml tests/evals/test_task_files.py
git commit -m "feat(evals): flat-world scenario suite (plan-depth/spatial/ambiguity/c1)"
```

---

### Task 7: city-world gate + obstacle ladder + capstone c2 + README

Validate the `city` world, then author obstacle rungs + `c2` using real building coordinates, and document the city bring-up. Obstacle authoring proceeds only if the city altitude-frame gate passes.

**Files:**
- Create: `evals/tasks/obstacle/o1_one_bldg.yaml`, `o2_cluster.yaml`
- Create: `evals/tasks/capstone/c2_obstacle_patrol.yaml`
- Modify: `evals/README.md`
- Modify: `tests/evals/test_task_files.py`

**Interfaces:**
- Consumes: `evals.spec.load_task`; the `clearance` check (Task 4); city building coords from `PX4-Autopilot/Tools/simulation/gz/worlds/city_boxes.json` (`bldg_9` at E=43.8,N=14.4; `bldg_13` at E=75.8,N=45.2; `bldg_12` at E=83.6,N=-31.3).
- Produces: 3 city task YAMLs + a README `city` bring-up section.

- [ ] **Step 1: CITY-WORLD GATE (controller/operator, live, bounded — do this FIRST)**

Bring up a `city` sim and confirm the altitude frame is clean (no baylands-style offset) before authoring obstacle tasks. From the repo root on the host:
```bash
# reuse the cred/fuel mounts from evals/README.md
docker rm -f evals-city >/dev/null 2>&1 || true
docker run -d --name evals-city \
  -e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500 \
  -v "$PWD:/workspace" -v /tmp/evals-claude:/root/.claude -v /tmp/evals-claude.json:/root/.claude.json \
  -v /tmp/swarm-gz-fuel:/root/.gz/fuel \
  -e SWARM_N=1 -e PX4_GZ_WORLD=city -e GZ_WORLD=city \
  squawd:dev bash -lc 'sim/launch/swarm_sim.sh'
# wait (bounded) until vehicle_local_position appears, then read grounded z:
timeout 25 docker exec evals-city bash -lc 'source /opt/ros/jazzy/setup.bash; timeout 8 ros2 topic echo --once /px4_0/fmu/out/vehicle_local_position 2>/dev/null | grep "^z:"'
```
Expected: `z:` between roughly -2 and +2 (grounded ≈ 0). **If `z` is a large magnitude (e.g. -500+), the city world has the baylands altitude bug: STOP — mark obstacle + c2 as held in the README, skip Steps 2–4, and finish the task by committing only the README note.** If `z ≈ 0`, proceed.

- [ ] **Step 2: Write the failing test**

Add to `tests/evals/test_task_files.py`:
```python
def test_city_obstacle_tasks_load_and_use_clearance():
    paths = glob.glob("evals/tasks/obstacle/*.yaml") + ["evals/tasks/capstone/c2_obstacle_patrol.yaml"]
    assert len(paths) == 3
    for p in paths:
        t = load_task(p)
        assert t.setup.world == "city"
        assert any(c["check"] == "clearance" for c in t.oracle), f"{p} must grade clearance"
        assert t.setup.n_drones == 1
```

- [ ] **Step 3: Run to verify it fails**

Run: `python -m pytest tests/evals/test_task_files.py::test_city_obstacle_tasks_load_and_use_clearance -v`
Expected: FAIL (`assert 0 == 3`).

- [ ] **Step 4: Create the city task files**

`evals/tasks/obstacle/o1_one_bldg.yaml` (target beyond `bldg_9` on the +E,+N line from home):
```yaml
id: o1_one_bldg
target_layer: single_drone
suite: obstacle
difficulty: {plan_depth: 1, coordination: 1, ambiguity: 1, spatial: 2}
setup:
  world: city
  n_drones: 1
  spawn: home
  seed_objects:
    # bldg_9 (E43.8,N14.4) sits on the straight line home->tgt; the drone must route around it.
    - {id: tgt, east: 80, north: 26}
prompt: "Take off to 12 m and fly to the marker tgt at east 80, north 26. There are buildings in the way; keep clear of them (do not fly through or clip a building). Hover at tgt."
budget: {wall_clock_s: 200, max_steps: 16}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 12}
  - {check: clearance, margin_m: 4}
  - {check: within_step_budget, max_steps: 16}
```

`evals/tasks/obstacle/o2_cluster.yaml` (target beyond a NE cluster `bldg_9`/`bldg_10`/`bldg_13`):
```yaml
id: o2_cluster
target_layer: single_drone
suite: obstacle
difficulty: {plan_depth: 2, coordination: 1, ambiguity: 1, spatial: 3}
setup:
  world: city
  n_drones: 1
  spawn: home
  seed_objects:
    # Beyond bldg_13 (E75.8,N45.2); path from home threads the bldg_9/10/13 cluster.
    - {id: tgt, east: 95, north: 55}
prompt: "Take off to 15 m and fly to the marker tgt at east 95, north 55. Several buildings lie between you and the target; route around them and keep well clear. Hover at tgt."
budget: {wall_clock_s: 240, max_steps: 20}
oracle:
  - {check: alive}
  - {check: reached, target: tgt, tol_m: 12}
  - {check: clearance, margin_m: 4}
  - {check: within_step_budget, max_steps: 20}
```

`evals/tasks/capstone/c2_obstacle_patrol.yaml` (two ordered waypoints either side of the cluster):
```yaml
id: c2_obstacle_patrol
target_layer: single_drone
suite: capstone
difficulty: {plan_depth: 3, coordination: 1, ambiguity: 2, spatial: 3}
setup:
  world: city
  n_drones: 1
  spawn: home
  seed_objects:
    # p south-east of the cluster, q north-east of it; the drone crosses the built-up strip.
    - {id: p, east: 88, north: -20}
    - {id: q, east: 95, north: 55}
prompt: "Take off to 15 m. Fly to point p (east 88, north -20), then to point q (east 95, north 55), in that order, keeping clear of all buildings. Hover at q."
budget: {wall_clock_s: 260, max_steps: 22}
oracle:
  - {check: alive}
  - {check: ordering, sequence: [p, q], tol_m: 12}
  - {check: clearance, margin_m: 4}
  - {check: reached, target: q, tol_m: 12}
  - {check: within_step_budget, max_steps: 22}
```

- [ ] **Step 5: Run to verify it passes**

Run: `python -m pytest tests/evals/test_task_files.py -v`
Expected: PASS (both flat and city task tests).

- [ ] **Step 6: Update the README**

In `evals/README.md`, under the "Run" section, add a city variant note:
```markdown
### City world (obstacle scenarios)

Obstacle rungs (`evals/tasks/obstacle/*`, `capstone/c2_*`) run on the `city` world.
Bring it up exactly like the flat sim but with `PX4_GZ_WORLD=city GZ_WORLD=city`
(container name e.g. `evals-city`). `city` is flat-ground with buildings, so its
altitude frame is clean (verified: grounded `z ≈ 0`). Run the obstacle subset with
`--tasks evals/tasks/obstacle/*.yaml evals/tasks/capstone/c2_obstacle_patrol.yaml`
against that container; results append into the same out-dir as the flat suite for
one merged report.
```
(If the city gate in Step 1 FAILED, instead add: "**Obstacle scenarios are held**: the
`city` world exhibits a PX4 local-altitude offset like baylands (grounded z ≈ <value>);
obstacle/`c2` await a fix or an alternative flat-with-buildings world.")

- [ ] **Step 7: (Optional, controller/operator) live clearance smoke**

If the gate passed, run one `o1` cell against the city sim to confirm `clearance` grades end-to-end (bounded):
```bash
timeout 340 docker exec evals-city bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash; cd /workspace && PYTHONPATH=/workspace:$PYTHONPATH SWARM_N=1 GZ_WORLD=city uv run --no-project python -m evals.run_evals --tasks evals/tasks/obstacle/o1_one_bldg.yaml --assignments "drones=opus" --k 1 --out /workspace/evals/out/city_smoke'
```
Expected: a graded row (pass or fail) with a `clearance` check present and `infra_fail=False`. Record the outcome in the commit message / ledger. Stop the container with `docker rm -f evals-city`.

- [ ] **Step 8: Commit**

```bash
git add evals/tasks/obstacle evals/tasks/capstone/c2_obstacle_patrol.yaml evals/README.md tests/evals/test_task_files.py
git commit -m "feat(evals): city obstacle ladder + c2 capstone (gated on city altitude frame)"
```

---

## Self-Review

**Spec coverage** (design → tasks):
- Five new checks (`visited_all`, `ordering`, `altitude`, `clearance`, `dwell`) → Tasks 2–4. (Design note: `altitude` was specified with an optional target; the plan makes `target` **required** — simpler and sufficient for `s4`/`c1`, pair with `reached` when a target is meant. Documented here as the one deliberate simplification.)
- `WorldTrack.buildings` + sampler capture → Task 1.
- `suite` tag + report knee-view (`render_ladders`) → Task 5.
- Scenario suite (anchor reused; plan-depth/spatial/ambiguity/`c1` on flat; obstacle/`c2` on city) → Tasks 6–7.
- Two-world orchestration + city gate → Task 7 (gate in Step 1; README in Step 6).
- Testing: every check TDD'd pure; sampler building capture faked; every YAML load-tested; city clearance live smoke (Task 7 Step 7).

**Deferred/out-of-scope (per spec):** coordination/multi-drone; physical marker spawning; camera-perception grading; a YAML rung generator. `p4` (5-leg return) and `am4` (sweep+finish) from the design's "~15–18" are trimmed to a first cut (p1–p3, am1–am3) per the user's "it's a start"; adding them later is one YAML file each, no code change.

**Placeholder scan:** No TBD/TODO. Every code step shows complete code; every YAML is complete; every test asserts real behavior. The city obstacle coordinates are concrete (from `city_boxes.json`); the one conditional is the explicit city-gate branch in Task 7 Step 1 (a real decision point, not a placeholder).

**Type consistency:** `CheckResult(name, passed, value, detail)` used consistently across new checks (Tasks 2–4) and matches Task-3-era `to_row` serialization. `WorldTrack(..., buildings=[])` kwarg matches between Task 1 (definition), Task 4 (`clearance`), and the sampler. `render_ladders(rows)` (Task 5) consumes the `suite`/`difficulty` keys that `CellResult.to_row` (Task 5) emits and that the YAMLs (Tasks 6–7) set. `difficulty[suite]` is the rung in both the YAMLs and `render_ladders`. `ordering`/`dwell`/`altitude` params (`sequence`/`tol_m`, `target`/`tol_m`/`hold_s`, `target`/`min_m`/`max_m`) match between check code and the task YAMLs.
