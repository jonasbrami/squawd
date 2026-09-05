# Swarm Coordination Evals (C0 operator) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the eval harness from single-drone to N-drone "operator" cells (one LLM flies the whole fleet) with a w1–w5 coordination ladder, dual-baseline pilot gates, and the E1 tier screening.

**Architecture:** One ClaudeSDKClient receives N per-drone MCP tool namespaces (`mcp__d0__*`, `mcp__d1__*`) plus a fleet-level `goto_all` that moves drones CONCURRENTLY (sequential blocking gotos would serialize the fleet). The oracle gains fleet checks (`targets_covered`, `fleet_separation`, `simultaneous`, `within_window`) plus an optional `drone:` filter on existing checks — all pure functions over the already-N-aware `Snapshot.poses`. Reset/sampler/scan are N-aware today; the runner's single-drone gate lifts only for `target_layer: operator`.

**Tech Stack:** Python 3.11+ host tests (pytest), claude-agent-sdk MCP tools, MAVSDK/PX4 SITL + Gazebo in docker (live gates), existing `evals/` harness conventions.

**Spec:** `docs/superpowers/specs/2026-07-04-swarm-coordination-evals-design.md`

## Global Constraints

- Branch: `feat/dynamic-scenarios` (continue on it; do NOT touch main).
- Pure logic must be host-unit-testable without rclpy/mavsdk/gz (existing discipline).
- Every new task YAML ships `pilot:` (must-PASS) and, except entry rungs, `null_pilot:` (must-FAIL).
- Fleet moves must be able to run CONCURRENTLY (`goto_all`); never force serialization through tool design.
- LLM budgets include the observation tax: +1–2 ToolSearch steps, ~5 s deliberation per step (dynamic-suite lesson).
- Sim containers: unique `ROS_DOMAIN_ID` + `GZ_PARTITION` per concurrent container; time-bound every wait (`timeout`, bounded loops).
- Commit after every task with the session trailer used on this branch.
- `within_step_budget` values in YAML `oracle:` must equal `budget.max_steps`.

## File Structure

- `evals/oracle.py` — add 4 fleet checks + `drone:` filter on `reached`/`ordering`/`final_pos` (existing file, existing style)
- `agents/flight/fleet.py` — NEW: `FleetOps` (list of FlightOps + `goto_all`)
- `agents/flight/tools.py` — extract `_drone_server(i, ops, cameras, report, name)`; add `make_operator_options(...)`
- `evals/pilot.py` — fleet-aware `ScriptedClient` (per-step `drone:`, fleet tools)
- `evals/runner.py` — `FleetHarness` (replaces `DroneHarness` usage; N=1 compatible), gate by `target_layer`
- `evals/run_evals.py` — wire FleetHarness + fleet pilot builder
- `evals/tasks/swarm/w{1..5}_*.yaml` — the ladder
- `tests/evals/test_oracle_fleet.py`, `tests/test_fleet_ops.py`, `tests/test_operator_tools.py`, additions to `tests/evals/test_pilot.py`, `tests/evals/test_runner.py`, `tests/evals/test_task_files.py`

---

### Task 1: Fleet oracle checks — `targets_covered`, `fleet_separation`, + `drone:` filter

**Files:**
- Modify: `evals/oracle.py`
- Test: `tests/evals/test_oracle_fleet.py` (create)

**Interfaces:**
- Consumes: `Snapshot.poses: dict[int, DronePose]` (already multi-drone), `WorldTrack.objects`.
- Produces: oracle check names `targets_covered` (params: `targets: list[str]`, `tol_m`), `fleet_separation` (params: `margin_m`, optional `grace_s: 0.0`, optional `use_3d: false`); existing `reached`/`ordering`/`final_pos` accept optional `drone: int` (filter to that drone's poses only). Helper `def _sel(poses: dict, p: dict) -> list` returning `[poses[p["drone"]]] if "drone" in p and p["drone"] in poses else list(poses.values())`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/evals/test_oracle_fleet.py
"""Fleet oracle checks: multi-drone coverage, own-fleet separation, drone filters."""
from evals.oracle import grade
from evals.worldstate import DronePose, Snapshot, WorldTrack

META = {"steps": 5, "crashed": False}


def _snap(t, positions, alts=None):
    """positions: dict[drone_id] -> (e, n); alts: dict[drone_id] -> alt (default 10)."""
    alts = alts or {}
    return Snapshot(t, {i: DronePose(e, n, alts.get(i, 10.0), 0.0)
                        for i, (e, n) in positions.items()})


def _track(snaps, objects=None):
    return WorldTrack(snaps, objects or {}, n_drones=2, geofence_m=300.0)


class TestTargetsCovered:
    OBJS = {"t1": (100.0, 0.0), "t2": (-100.0, 0.0)}

    def test_split_coverage_passes(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(10, {0: (100, 0), 1: (-100, 0)})], self.OBJS)
        g = grade(t, [{"check": "targets_covered", "targets": ["t1", "t2"],
                       "tol_m": 12}], META)
        assert g.passed

    def test_one_target_missed_fails_and_names_it(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(10, {0: (100, 0), 1: (50, 0)})], self.OBJS)
        g = grade(t, [{"check": "targets_covered", "targets": ["t1", "t2"],
                       "tol_m": 12}], META)
        assert not g.passed
        assert "t2" in g.checks[0].detail

    def test_single_drone_covering_both_passes(self):
        # coverage is drone-agnostic (budgets punish solo runs, not this check)
        t = _track([_snap(0, {0: (100, 0), 1: (0, 3)}),
                    _snap(10, {0: (-100, 0), 1: (0, 3)})], self.OBJS)
        g = grade(t, [{"check": "targets_covered", "targets": ["t1", "t2"],
                       "tol_m": 12}], META)
        assert g.passed


class TestFleetSeparation:
    def test_close_pass_fails_2d(self):
        t = _track([_snap(0, {0: (0, 0), 1: (100, 0)}),
                    _snap(10, {0: (50, 0), 1: (54, 0)})])
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8}], META)
        assert not g.passed and abs(g.checks[0].value - 4.0) < 1e-9

    def test_altitude_layering_passes_3d(self):
        t = _track([_snap(10, {0: (50, 0), 1: (52, 0)}, alts={0: 10.0, 1: 22.0})])
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8,
                       "use_3d": True}], META)
        assert g.passed

    def test_grace_excuses_spawn_adjacency(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(60, {0: (50, 0), 1: (-50, 0)})])
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8,
                       "grace_s": 30}], META)
        assert g.passed

    def test_single_drone_track_passes_vacuously(self):
        t = WorldTrack([_snap(0, {0: (0, 0)})], {}, n_drones=1, geofence_m=300.0)
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8}], META)
        assert g.passed


class TestDroneFilter:
    OBJS = {"pad_n": (60.0, 80.0), "pad_s": (60.0, -80.0)}

    def test_reached_by_specific_drone(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(10, {0: (60, 80), 1: (0, 3)})], self.OBJS)
        assert grade(t, [{"check": "reached", "target": "pad_n", "tol_m": 10,
                          "drone": 0}], META).passed
        assert not grade(t, [{"check": "reached", "target": "pad_n", "tol_m": 10,
                              "drone": 1}], META).passed

    def test_per_drone_ordering_grades_the_swap(self):
        snaps = [_snap(0, {0: (60, 80), 1: (60, -80)}),
                 _snap(10, {0: (60, 0), 1: (60, 0)}),
                 _snap(20, {0: (60, -80), 1: (60, 80)})]
        t = _track(snaps, self.OBJS)
        assert grade(t, [{"check": "ordering", "sequence": ["pad_n", "pad_s"],
                          "tol_m": 10, "drone": 0}], META).passed
        assert grade(t, [{"check": "ordering", "sequence": ["pad_s", "pad_n"],
                          "tol_m": 10, "drone": 1}], META).passed
        assert not grade(t, [{"check": "ordering", "sequence": ["pad_s", "pad_n"],
                              "tol_m": 10, "drone": 0}], META).passed
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/evals/test_oracle_fleet.py -q`
Expected: FAIL/ERROR with `KeyError: 'targets_covered'` (unknown check name).

- [ ] **Step 3: Implement in `evals/oracle.py`**

Add the selector helper right after `GradeResult` (before `_reached`):

```python
def _sel(poses: dict, p: dict) -> list:
    """Poses considered by a check: one drone when the check carries `drone:`
    (per-drone assignments — e.g. grading a swap), else the whole fleet."""
    if "drone" in p:
        pose = poses.get(int(p["drone"]))
        return [pose] if pose is not None else []
    return list(poses.values())
```

Thread it through the three existing checks (mechanical edits — each currently
iterates `s.poses.values()` or calls a helper that does):

- `_reached`: replace `track.min_dist_to(xy)` with an inline loop
  `d = min((math.hypot(q.e - xy[0], q.n - xy[1]) for s in track.snapshots
  for q in _sel(s.poses, p)), default=math.inf)`.
- `_first_reach_time`: add parameter `p: dict | None = None`; replace
  `for pose in s.poses.values():` with `for pose in _sel(s.poses, p or {}):`;
  `_ordering` passes its own `p` into every `_first_reach_time` call.
- `_final_pos`: replace `last.poses.values()` with `_sel(last.poses, p)`.

Add the two fleet checks before the `CHECKS` dict:

```python
def _targets_covered(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Every listed target visited within tol by SOME drone. Drone-agnostic on
    purpose: budgets (fleet path, wall clock) punish solo tours, not this check."""
    tol = float(p["tol_m"])
    missed = []
    for t in p["targets"]:
        xy = track.objects[t]
        d = min((math.hypot(q.e - xy[0], q.n - xy[1])
                 for s in track.snapshots for q in s.poses.values()),
                default=math.inf)
        if d > tol:
            missed.append(t)
    got = len(p["targets"]) - len(missed)
    return CheckResult("targets_covered", not missed, float(got),
                       f"covered {got}/{len(p['targets'])} within {tol:g}m; "
                       f"missed {missed}")


def _fleet_separation(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Min pairwise distance between OWN drones stays >= margin. 2D by default
    (airspace hygiene); use_3d for tasks where altitude layering is the legal
    dodge. grace_s excuses spawn adjacency (drones spawn 3 m apart)."""
    margin = float(p["margin_m"])
    grace = float(p.get("grace_s", 0.0))
    use_3d = bool(p.get("use_3d", False))
    worst = math.inf
    for s in track.snapshots:
        if s.t < grace:
            continue
        ids = sorted(s.poses)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                qa, qb = s.poses[ids[a]], s.poses[ids[b]]
                d = math.hypot(qa.e - qb.e, qa.n - qb.n)
                if use_3d:
                    d = math.hypot(d, qa.alt - qb.alt)
                worst = min(worst, d)
    ok = worst >= margin or worst == math.inf
    shown = "inf" if worst == math.inf else f"{worst:.1f}m"
    return CheckResult("fleet_separation", ok, 0.0 if worst == math.inf else worst,
                       f"min own-fleet separation {shown} (margin {margin:g}"
                       f"{', 3D' if use_3d else ''})")
```

Register both in `CHECKS`:

```python
    "targets_covered": _targets_covered,
    "fleet_separation": _fleet_separation,
```

- [ ] **Step 4: Run tests**

Run: `python -m pytest tests/evals/test_oracle_fleet.py tests/evals/test_oracle.py tests/evals/test_oracle_dynamic.py -q`
Expected: all PASS (existing oracle tests must not regress).

- [ ] **Step 5: Commit**

```bash
git add evals/oracle.py tests/evals/test_oracle_fleet.py
git commit -m "feat(evals): fleet oracle checks — targets_covered, fleet_separation, per-drone filter"
```

---

### Task 2: Timing oracle checks — `simultaneous`, `within_window`

**Files:**
- Modify: `evals/oracle.py`
- Test: `tests/evals/test_oracle_fleet.py` (append)

**Interfaces:**
- Consumes: `_mover_sep(s, name)` (exists, dynamic checks), `_sel`, `track.objects`.
- Produces: check `simultaneous` (params `marks: list[{target, tol_m}]`) — passes iff some single snapshot has DISTINCT drones covering all marks; check `within_window` (params `events: list[{type: "reach", target, tol_m} | {type: "intercept", mover, tol_m}]`, `window_s`) — passes iff every event occurs and max(first-times) − min(first-times) ≤ window_s.

- [ ] **Step 1: Write the failing tests (append to `tests/evals/test_oracle_fleet.py`)**

```python
class TestSimultaneous:
    OBJS = {"mark_a": (80.0, 80.0), "mark_b": (80.0, -80.0)}
    SPEC = [{"check": "simultaneous",
             "marks": [{"target": "mark_a", "tol_m": 10},
                       {"target": "mark_b", "tol_m": 10}]}]

    def test_both_marks_same_snapshot_distinct_drones_passes(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(30, {0: (80, 80), 1: (80, -80)})], self.OBJS)
        assert grade(t, self.SPEC, META).passed

    def test_sequential_solo_visits_fail(self):
        t = _track([_snap(10, {0: (80, 80), 1: (0, 3)}),
                    _snap(60, {0: (80, -80), 1: (0, 3)})], self.OBJS)
        assert not grade(t, self.SPEC, META).passed

    def test_one_drone_cannot_satisfy_two_marks(self):
        # marks 12m apart, one drone within tol of both — still needs a partner
        objs = {"mark_a": (80.0, 6.0), "mark_b": (80.0, -6.0)}
        t = _track([_snap(30, {0: (80, 0), 1: (0, 3)})], objs)
        assert not grade(t, [{"check": "simultaneous",
                              "marks": [{"target": "mark_a", "tol_m": 10},
                                        {"target": "mark_b", "tol_m": 10}]}],
                         META).passed


class TestWithinWindow:
    def _movers(self, t, positions, movers):
        return Snapshot(t, {i: DronePose(e, n, 10.0, 0.0)
                            for i, (e, n) in positions.items()},
                        {k: (v[0], v[1], 8.0) for k, v in movers.items()})

    def test_two_intercepts_inside_window_pass(self):
        snaps = [self._movers(40, {0: (50, 100), 1: (70, -100)},
                              {"mov_0": (52, 100), "mov_1": (72, -100)})]
        t = WorldTrack(snaps, {}, n_drones=2, geofence_m=300.0)
        spec = [{"check": "within_window", "window_s": 25,
                 "events": [{"type": "intercept", "mover": "mov_0", "tol_m": 12},
                            {"type": "intercept", "mover": "mov_1", "tol_m": 12}]}]
        assert grade(t, spec, META).passed

    def test_spread_events_fail_window(self):
        snaps = [self._movers(10, {0: (52, 100), 1: (0, 3)},
                              {"mov_0": (50, 100), "mov_1": (300, 300)}),
                 self._movers(80, {0: (0, 0), 1: (72, -100)},
                              {"mov_0": (300, 300), "mov_1": (70, -100)})]
        t = WorldTrack(snaps, {}, n_drones=2, geofence_m=300.0)
        spec = [{"check": "within_window", "window_s": 25,
                 "events": [{"type": "intercept", "mover": "mov_0", "tol_m": 12},
                            {"type": "intercept", "mover": "mov_1", "tol_m": 12}]}]
        g = grade(t, spec, META)
        assert not g.passed and "70.0s apart" in g.checks[0].detail

    def test_missing_event_fails(self):
        t = WorldTrack([self._movers(10, {0: (0, 0)}, {"mov_0": (300, 300)})],
                       {}, n_drones=1, geofence_m=300.0)
        spec = [{"check": "within_window", "window_s": 25,
                 "events": [{"type": "intercept", "mover": "mov_0", "tol_m": 12}]}]
        assert not grade(t, spec, META).passed
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/evals/test_oracle_fleet.py -q`
Expected: new tests FAIL with `KeyError: 'simultaneous'`.

- [ ] **Step 3: Implement (append to `evals/oracle.py`, before `CHECKS`)**

```python
def _simultaneous(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Some single snapshot where DISTINCT drones cover all marks at once —
    the coordinated-timing primitive. Distinctness is a permutation match
    (fleet sizes here are small)."""
    import itertools
    marks = p["marks"]
    for s in track.snapshots:
        ids = sorted(s.poses)
        if len(ids) < len(marks):
            continue
        for perm in itertools.permutations(ids, len(marks)):
            ok = True
            for mk, did in zip(marks, perm):
                xy = track.objects[mk["target"]]
                q = s.poses[did]
                if math.hypot(q.e - xy[0], q.n - xy[1]) > float(mk["tol_m"]):
                    ok = False
                    break
            if ok:
                return CheckResult("simultaneous", True, s.t,
                                   f"all {len(marks)} marks held at t={s.t:.1f}s")
    return CheckResult("simultaneous", False, 0.0,
                       f"no snapshot with {len(marks)} marks covered by "
                       f"distinct drones")


def _event_time(track: WorldTrack, ev: dict) -> float | None:
    tol = float(ev["tol_m"])
    for s in track.snapshots:
        if ev["type"] == "reach":
            xy = track.objects[ev["target"]]
            if any(math.hypot(q.e - xy[0], q.n - xy[1]) <= tol
                   for q in s.poses.values()):
                return s.t
        elif ev["type"] == "intercept":
            d = _mover_sep(s, ev["mover"])
            if d is not None and d <= tol:
                return s.t
    return None


def _within_window(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """All listed events (first occurrence each) within window_s of one another
    — forces a fleet SPLIT when the event sites are far apart."""
    times = [_event_time(track, ev) for ev in p["events"]]
    if any(t is None for t in times):
        missing = [p["events"][i] for i, t in enumerate(times) if t is None]
        return CheckResult("within_window", False, 0.0,
                           f"events never occurred: {missing}")
    spread = max(times) - min(times)
    win = float(p["window_s"])
    return CheckResult("within_window", spread <= win, spread,
                       f"events {spread:.1f}s apart (window {win:g}s)")
```

Register in `CHECKS`:

```python
    "simultaneous": _simultaneous,
    "within_window": _within_window,
```

- [ ] **Step 4: Run tests**

Run: `python -m pytest tests/evals/test_oracle_fleet.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add evals/oracle.py tests/evals/test_oracle_fleet.py
git commit -m "feat(evals): timing oracle checks — simultaneous, within_window"
```

---

### Task 3: `FleetOps` + concurrent `goto_all`

**Files:**
- Create: `agents/flight/fleet.py`
- Test: `tests/test_fleet_ops.py` (create)

**Interfaces:**
- Consumes: `FlightOps` instances (constructed elsewhere; here they are duck-typed: need `.goto(target, east, north, up, heading, wait)` coroutine and `.i`).
- Produces: `class FleetOps` — `FleetOps(ops_list)`, `fleet.drone(i) -> FlightOps`, `fleet.n`, `await fleet.goto_all(moves) -> str` where `moves = [{"drone": int, "east": float, "north": float, "up": float}, ...]`. Tasks 4–6 rely on exactly these names.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_fleet_ops.py
"""FleetOps: concurrent multi-drone movement primitive."""
import asyncio

import pytest

from agents.flight.fleet import FleetOps


class FakeOps:
    def __init__(self, i, delay=0.0, fail=False):
        self.i = i
        self.delay = delay
        self.fail = fail
        self.calls = []
        self.t_start = None

    async def goto(self, target="", east=None, north=None, up=None,
                   heading="travel", wait=True):
        self.t_start = asyncio.get_event_loop().time()
        self.calls.append((east, north, up))
        await asyncio.sleep(self.delay)
        if self.fail:
            raise ValueError(f"drone_{self.i} boom")
        return f"drone_{self.i} arrived E{east} N{north}"


def test_goto_all_moves_run_concurrently():
    a, b = FakeOps(0, delay=0.2), FakeOps(1, delay=0.2)
    fleet = FleetOps([a, b])

    async def run():
        t0 = asyncio.get_event_loop().time()
        out = await fleet.goto_all([
            {"drone": 0, "east": 10, "north": 0, "up": 12},
            {"drone": 1, "east": -10, "north": 0, "up": 12}])
        return asyncio.get_event_loop().time() - t0, out

    dur, out = asyncio.run(run())
    assert dur < 0.35            # concurrent, not 0.4 sequential
    assert "drone_0 arrived" in out and "drone_1 arrived" in out


def test_goto_all_reports_per_drone_errors_without_losing_others():
    a, b = FakeOps(0), FakeOps(1, fail=True)
    fleet = FleetOps([a, b])
    out = asyncio.run(fleet.goto_all([
        {"drone": 0, "east": 5, "north": 5, "up": 10},
        {"drone": 1, "east": 6, "north": 6, "up": 10}]))
    assert "drone_0 arrived" in out
    assert "ERROR" in out and "boom" in out


def test_goto_all_rejects_unknown_drone():
    fleet = FleetOps([FakeOps(0)])
    with pytest.raises(ValueError, match="unknown drone 3"):
        asyncio.run(fleet.goto_all([{"drone": 3, "east": 0, "north": 0, "up": 10}]))


def test_drone_accessor_and_n():
    ops = [FakeOps(0), FakeOps(1)]
    fleet = FleetOps(ops)
    assert fleet.n == 2 and fleet.drone(1) is ops[1]
```

- [ ] **Step 2: Verify failure**

Run: `python -m pytest tests/test_fleet_ops.py -q`
Expected: `ModuleNotFoundError: agents.flight.fleet`.

- [ ] **Step 3: Implement `agents/flight/fleet.py`**

```python
"""FleetOps: one object owning every drone's FlightOps, plus the fleet-level
movement primitive.

`goto_all` exists because the per-drone `goto` BLOCKS until arrival: an
operator commanding drones one tool call at a time would serialize the fleet
— the harness, not the model, would forbid coordination (the blocking-goto
lesson at fleet granularity). goto_all issues every move concurrently and
returns when ALL arrive, reporting per-drone outcomes (one drone's error
never hides the others')."""
import asyncio


class FleetOps:
    def __init__(self, ops_list) -> None:
        self._ops = list(ops_list)

    @property
    def n(self) -> int:
        return len(self._ops)

    def drone(self, i: int):
        if not 0 <= int(i) < len(self._ops):
            raise ValueError(f"unknown drone {i} (fleet of {len(self._ops)})")
        return self._ops[int(i)]

    async def goto_all(self, moves: list[dict]) -> str:
        tasks = []
        for mv in moves:
            ops = self.drone(mv["drone"])   # validate BEFORE launching any move
            tasks.append((mv["drone"], ops.goto(
                east=mv.get("east"), north=mv.get("north"), up=mv.get("up"),
                wait=True)))
        results = await asyncio.gather(*(t for _, t in tasks),
                                       return_exceptions=True)
        lines = []
        for (i, _), r in zip(tasks, results):
            if isinstance(r, BaseException):
                lines.append(f"drone_{i} ERROR: {r}")
            else:
                lines.append(str(r))
        return "\n".join(lines)
```

- [ ] **Step 4: Run tests**

Run: `python -m pytest tests/test_fleet_ops.py -q`
Expected: 4 PASS.

- [ ] **Step 5: Commit**

```bash
git add agents/flight/fleet.py tests/test_fleet_ops.py
git commit -m "feat(flight): FleetOps.goto_all — concurrent fleet movement primitive"
```

---

### Task 4: `make_operator_options` — one client, N tool namespaces + fleet server

**Files:**
- Modify: `agents/flight/tools.py`
- Test: `tests/test_operator_tools.py` (create)

**Interfaces:**
- Consumes: `FleetOps` (Task 3), `FlightOps`, the existing `@tool` wrappers in `make_drone_options`.
- Produces: `make_operator_options(systems, world, bridge, n, cameras, gzposes=None, env=None, model=None) -> (ClaudeAgentOptions, FleetOps)`. Options carry MCP servers `d0..d{n-1}` (full per-drone toolset each) + server `fleet` with tool `goto_all`; `allowed_tools` covers all of them. Task 6's `FleetHarness.client_for` calls exactly this.

**Approach:** `make_drone_options` currently inlines 12 `@tool` wrappers, then builds server+options (lines 27–200). Extract the wrapper block into `_drone_server(i, ops, cameras, report)` returning `(server, allowed_names)`; `make_drone_options` becomes a thin caller (same behavior, same options — existing tests/`swarm` unaffected); `make_operator_options` calls it N times.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_operator_tools.py
"""Operator options: one client, N per-drone namespaces + fleet goto_all."""
from agents.flight.tools import make_operator_options


def test_operator_options_carry_n_namespaces_and_fleet_server():
    opts, fleet = make_operator_options(
        systems=[object(), object()], world=None, bridge=None, n=2,
        cameras=None)
    assert set(opts.mcp_servers) == {"d0", "d1", "fleet"}
    for name in ("mcp__d0__goto", "mcp__d1__scan", "mcp__fleet__goto_all"):
        assert name in opts.allowed_tools
    assert fleet.n == 2


def test_operator_prompt_frames_the_whole_fleet():
    opts, _ = make_operator_options(
        systems=[object(), object()], world=None, bridge=None, n=2,
        cameras=None)
    sp = opts.system_prompt
    assert "ALL" in sp and "goto_all" in sp and "d0" in sp and "d1" in sp
```

- [ ] **Step 2: Verify failure**

Run: `python -m pytest tests/test_operator_tools.py -q`
Expected: `ImportError: cannot import name 'make_operator_options'`.

- [ ] **Step 3: Refactor + implement in `agents/flight/tools.py`**

3a. Rename the body of `make_drone_options` from the first `@tool` (line 27)
through the `create_sdk_mcp_server(...)` call (line 145) into:

```python
def _drone_server(i, ops, cameras, report):
    """The 12 per-drone tools bound to one FlightOps; returns (server, allowed)."""
    name = f"drone_{i}"
    # ... the existing 12 @tool definitions, verbatim, unchanged ...
    server = create_sdk_mcp_server(
        name=f"d{i}", tools=[take_off, fly, goto, orbit, hover, set_speed, face,
                             land, report_tool, look, scan, run_mission])
    allowed = [f"mcp__d{i}__{t}" for t in
               ("take_off", "fly", "goto", "orbit", "hover", "set_speed", "face",
                "land", "report", "look", "scan", "run_mission")]
    return server, allowed
```

`make_drone_options` becomes:

```python
def make_drone_options(i, drone, world, bridge, n, cameras, report, env=None,
                       model=None, gzposes=None):
    ops = FlightOps(drone, world, bridge, i, n, gzposes=gzposes)
    server, allowed = _drone_server(i, ops, cameras, report)
    return ClaudeAgentOptions(
        mcp_servers={f"d{i}": server},
        allowed_tools=allowed,
        setting_sources=[], env=env or {}, model=model,
        system_prompt=( ... the existing per-drone system prompt, verbatim ... ),
    )
```

3b. Add the operator builder at the end of the file:

```python
def make_operator_options(systems, world, bridge, n, cameras, gzposes=None,
                          env=None, model=None):
    """ONE client flying ALL n drones: per-drone namespaces d0..d{n-1} plus a
    fleet server whose goto_all moves drones CONCURRENTLY (sequential blocking
    gotos would serialize the fleet). Returns (options, fleet_ops)."""
    from agents.flight.fleet import FleetOps

    ops_list = [FlightOps(systems[i], world, bridge, i, n, gzposes=gzposes)
                for i in range(n)]
    fleet = FleetOps(ops_list)
    servers, allowed = {}, []
    for i in range(n):
        server, names = _drone_server(i, ops_list[i], cameras,
                                      report=lambda _m: None)
        servers[f"d{i}"] = server
        allowed += names

    @tool("goto_all",
          "Move SEVERAL drones at once: moves=[{drone, east, north, up}, ...]. "
          "Issues every move concurrently and returns when ALL arrive, with a "
          "per-drone result line. This is the primitive for coordinated legs — "
          "one-at-a-time goto calls make the other drones WAIT.",
          {"moves": {"type": "array", "items": {"type": "object", "properties": {
              "drone": {"type": "number"}, "east": {"type": "number"},
              "north": {"type": "number"}, "up": {"type": "number"}}}}})
    async def goto_all(args):
        try:
            return _ok(await fleet.goto_all(args.get("moves", [])))
        except Exception as e:
            return _err(f"goto_all failed: {e}")

    servers["fleet"] = create_sdk_mcp_server(name="fleet", tools=[goto_all])
    allowed.append("mcp__fleet__goto_all")
    drone_words = ", ".join(f"d{i}" for i in range(n))
    return ClaudeAgentOptions(
        mcp_servers=servers, allowed_tools=allowed, setting_sources=[],
        env=env or {}, model=model,
        system_prompt=(
            f"You are the OPERATOR of a fleet of {n} drones. You fly ALL of "
            f"them yourself: each drone has its own tool namespace ({drone_words} "
            f"— e.g. d1's goto is mcp__d1__goto), and mcp__fleet__goto_all moves "
            "several drones AT ONCE (per-drone goto/fly BLOCK until arrival, so "
            "moving drones one at a time leaves the rest parked — use goto_all "
            "for coordinated legs).\n"
            "PLAN: before your first move, assign each drone to its goals "
            "EXPLICITLY (which drone takes which target, at which altitude) and "
            "check the assignment against every constraint — separation minimums, "
            "fleet path budgets, timing windows. Keep your drones apart unless "
            "the task says otherwise; give crossing routes different altitudes.\n"
            "SENSE: each drone's scan/look reports from ITS position (other "
            "drones appear as 'drone_j' contacts). MOVE/MISSION semantics per "
            "drone are identical to a single drone's tools."),
    ), fleet
```

- [ ] **Step 4: Run tests (new + regressions)**

Run: `python -m pytest tests/test_operator_tools.py tests/test_drone_tools.py tests/evals -q`
Expected: all PASS (`make_drone_options` behavior unchanged).

- [ ] **Step 5: Commit**

```bash
git add agents/flight/tools.py tests/test_operator_tools.py
git commit -m "feat(flight): make_operator_options — one client, N drone namespaces + fleet goto_all"
```

---

### Task 5: Fleet-aware pilot — `drone:` step field + fleet tools

**Files:**
- Modify: `evals/pilot.py`
- Test: `tests/evals/test_pilot.py` (append)

**Interfaces:**
- Consumes: `FleetOps` (`.drone(i)`, `.goto_all`, `.n`).
- Produces: `ScriptedClient(ops_provider, script)` where `ops_provider()` now returns a **FleetOps**; steps may carry `drone: <int>` (default 0) routing the tool to `fleet.drone(i)`; a step whose tool exists on FleetOps itself (`goto_all`) runs fleet-level; behavior steps accept `drone:` too (the behavior receives `fleet.drone(i)`). `pilot_client_builder(harness, deps)` builds the FleetOps from `harness.systems_list()` (Task 6).

- [ ] **Step 1: Write the failing tests (append to `tests/evals/test_pilot.py`)**

```python
def test_scripted_client_routes_steps_by_drone_field():
    import asyncio
    from agents.flight.fleet import FleetOps
    from evals.pilot import ScriptedClient

    class MiniOps:
        def __init__(self, i):
            self.i = i
            self.calls = []

        async def take_off(self, altitude=10.0):
            self.calls.append(("take_off", altitude))
            return f"drone_{self.i} airborne"

        async def goto(self, target="", east=None, north=None, up=None,
                       heading="travel", wait=True):
            self.calls.append(("goto", east, north))
            return f"drone_{self.i} arrived"

    a, b = MiniOps(0), MiniOps(1)
    fleet = FleetOps([a, b])

    async def provider():
        return fleet

    async def run():
        client = ScriptedClient(provider, [
            {"tool": "take_off", "args": {"altitude": 12}},              # default d0
            {"tool": "take_off", "drone": 1, "args": {"altitude": 12}},
            {"tool": "goto_all", "args": {"moves": [
                {"drone": 0, "east": 10, "north": 0, "up": 12},
                {"drone": 1, "east": -10, "north": 0, "up": 12}]}},
        ])
        return [m async for m in client.receive_response()]

    msgs = asyncio.run(run())
    assert len(msgs) == 6                     # 3 steps -> 3 (use, result) pairs
    assert a.calls[0] == ("take_off", 12) and b.calls[0] == ("take_off", 12)
    assert a.calls[1] == ("goto", 10, 0) and b.calls[1] == ("goto", -10, 0)
```

- [ ] **Step 2: Verify failure**

Run: `python -m pytest tests/evals/test_pilot.py -q`
Expected: new test FAILS (`ScriptedClient` calls `getattr(ops, tool)` on FleetOps and finds no `take_off`).

- [ ] **Step 3: Implement in `evals/pilot.py`**

In `ScriptedClient.receive_response`, replace the two resolution sites:

```python
        # fleet routing: `ops` is a FleetOps — a step's `drone:` picks the
        # body (default 0); tools that live on the fleet itself (goto_all)
        # run fleet-level. Behaviors bind one drone's FlightOps.
```

- behavior branch: `fn(ops, ...)` becomes
  `fn(ops.drone(step.get("drone", 0)), step.get("args", {}))`.
- tool branch: replace `fn = getattr(ops, tool, None)` with:

```python
            target = ops if hasattr(ops, tool) else ops.drone(step.get("drone", 0))
            fn = getattr(target, tool, None)
```

In `pilot_client_builder`, replace `ops_provider` with:

```python
    async def ops_provider():
        from agents.flight.fleet import FleetOps
        from agents.flight.ops import FlightOps
        systems = await harness.systems_list()
        n = len(systems)
        return FleetOps([FlightOps(s, deps.world, deps.bridge, i, n,
                                   gzposes=deps.gzposes)
                         for i, s in enumerate(systems)])
```

(`harness.systems_list()` arrives in Task 6; until then this file change keeps
the old single-system path working because Task 6 lands `systems_list` on the
harness before anything live runs — unit tests here use fakes.)

- [ ] **Step 4: Run tests**

Run: `python -m pytest tests/evals/test_pilot.py tests/test_fleet_ops.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add evals/pilot.py tests/evals/test_pilot.py
git commit -m "feat(evals): fleet-aware pilot — drone: step routing + fleet-level tools"
```

---

### Task 6: `FleetHarness` + operator gate in the runner

**Files:**
- Modify: `evals/runner.py`, `evals/run_evals.py`
- Test: `tests/evals/test_runner.py` (append)

**Interfaces:**
- Consumes: `make_operator_options` (Task 4), existing `DroneHarness` internals (`evals/runner.py:154-203`), `require_single_drone` (`evals/runner.py:240-246`), `run_cell` (`evals/runner.py:277+`), `soft_reset(systems, ...)`.
- Produces: `FleetHarness(deps, n, agent_factory=None, client_builder=None)` with `await systems_list() -> list`, `await system() -> systems_list()[0]` (back-compat), `client_for(model)`; `require_layer_supported(spec)` replacing `require_single_drone`; `run_cell` resets/holds ALL systems. `run_evals` constructs `FleetHarness(deps, n_max)`.

- [ ] **Step 1: Write the failing tests (append to `tests/evals/test_runner.py`)**

```python
def test_fleet_harness_connects_n_agents_once():
    import asyncio
    from evals.runner import Deps, FleetHarness

    made = []

    class FakeAgent:
        def __init__(self, i):
            self.i = i
            self._system = f"sys{i}"
            self.connects = 0

        async def connect(self):
            self.connects += 1

    def factory(i):
        a = FakeAgent(i)
        made.append(a)
        return a

    h = FleetHarness(Deps(world=None, bridge=None, cameras=None), n=2,
                     agent_factory=factory)

    async def run():
        s1 = await h.systems_list()
        s2 = await h.systems_list()
        return s1, s2

    s1, s2 = asyncio.run(run())
    assert s1 == ["sys0", "sys1"] and s2 is not s1 and s2 == s1
    assert [a.connects for a in made] == [1, 1]          # built + connected once
    assert asyncio.run(h.system()) == "sys0"             # back-compat accessor


def test_layer_gate_allows_operator_multidrone_only():
    import pytest
    from evals.runner import require_layer_supported

    class Setup:
        n_drones = 2

    class Spec:
        id = "w1"
        setup = Setup()
        target_layer = "single_drone"

    with pytest.raises(ValueError, match="n_drones==1"):
        require_layer_supported(Spec())
    Spec.target_layer = "operator"
    require_layer_supported(Spec())                      # no raise
    Spec.target_layer = "commander"
    with pytest.raises(ValueError, match="not built"):
        require_layer_supported(Spec())
```

- [ ] **Step 2: Verify failure**

Run: `python -m pytest tests/evals/test_runner.py -q`
Expected: `ImportError: FleetHarness`.

- [ ] **Step 3: Implement in `evals/runner.py`**

3a. Rename `DroneHarness` → `FleetHarness` (keep `DroneHarness = FleetHarness`
alias line for older imports/tests). Generalize:

```python
class FleetHarness:
    """Owns the persistent MAVSDK links for drones 0..n-1, built ONCE and
    reused across cells (the per-cell System/subscription leak fix, now per
    drone). Hands out a FRESH client per cell: single-drone specs get the
    classic one-drone options; operator specs get make_operator_options."""

    def __init__(self, deps: Deps, n: int = 1, agent_factory=None,
                 client_builder=None) -> None:
        self._deps = deps
        self._n = n
        self._agent_factory = agent_factory
        self._client_builder = client_builder
        self._agents: list | None = None

    def _make_agent(self, i: int):
        if self._agent_factory is not None:
            return self._agent_factory(i)
        from agents.swarm.drone import DroneAgent
        return DroneAgent(i, self._deps.world, self._deps.bridge, self._n,
                          self._deps.cameras, model=None)

    async def _ensure(self) -> list:
        if self._agents is None:
            agents = [self._make_agent(i) for i in range(self._n)]
            for a in agents:
                await a.connect()
            self._agents = agents
        return self._agents

    async def systems_list(self) -> list:
        return [a._system for a in await self._ensure()]

    async def system(self):
        return (await self.systems_list())[0]

    def client_for(self, model, n_drones: int = 1):
        if self._client_builder is not None:
            return self._client_builder(model)
        from claude_agent_sdk import ClaudeSDKClient
        if n_drones <= 1:
            from agents.flight import make_drone_options
            opts = make_drone_options(0, self._agents[0]._system,
                                      self._deps.world, self._deps.bridge, 1,
                                      self._deps.cameras,
                                      report=lambda _m: None, env=None,
                                      model=model, gzposes=self._deps.gzposes)
        else:
            from agents.flight.tools import make_operator_options
            opts, _fleet = make_operator_options(
                [a._system for a in self._agents], self._deps.world,
                self._deps.bridge, n_drones, self._deps.cameras,
                gzposes=self._deps.gzposes, model=model)
        return ClaudeSDKClient(options=opts)
```

3b. Replace `require_single_drone` with (keep the old name as an alias if
anything imports it):

```python
def require_layer_supported(spec) -> None:
    """single_drone cells must be n==1; operator cells may be multi-drone;
    the commander layer is a later phase — reject loudly, never silently."""
    layer = getattr(spec, "target_layer", "single_drone")
    if layer == "operator":
        return
    if layer == "commander":
        raise ValueError(f"target_layer 'commander' not built yet (task {spec.id})")
    if spec.setup.n_drones != 1:
        raise ValueError(
            f"single_drone runner requires n_drones==1, got {spec.setup.n_drones} "
            f"(task {spec.id})")
```

3c. In `run_cell`: change the signature type hint to `FleetHarness`, call
`require_layer_supported(spec)`, and:

- `system = await harness.system()` → `systems = await harness.systems_list()`
  (wrap in the same try/except infra path).
- `await soft_reset([system], ...)` → `await soft_reset(systems, deps.world, deps.bridge, n)`.
- `harness.client_for(model_for(assignment, "drones"))` →
  `harness.client_for(model_for(assignment, "drones"), n_drones=n)`.
- the post-turn halt (`grep -n 'action.hold' evals/runner.py`): wrap in a loop —

```python
        for s in systems:
            try:
                await asyncio.wait_for(s.action.hold(), timeout=5)
            except Exception:
                pass
```

(preserve the existing comment; keep per-system isolation so one dead link
doesn't skip the others' halt.)

3d. In `evals/run_evals.py`, the construction site
(`harness = DroneHarness(deps)`) becomes:

```python
    harness = FleetHarness(deps, n=n_max)  # shared flight links, fresh client per cell
```

with the matching import change (`from evals.runner import ... FleetHarness ...`).

- [ ] **Step 4: Run tests**

Run: `python -m pytest tests/evals -q`
Expected: all PASS (existing runner/run_evals tests keep passing through the alias + n=1 default).

- [ ] **Step 5: Commit**

```bash
git add evals/runner.py evals/run_evals.py tests/evals/test_runner.py
git commit -m "feat(evals): FleetHarness + operator layer gate — N-drone cells"
```

---

### Task 7: Task YAMLs w1–w3 + machine-verified geometry

**Files:**
- Create: `evals/tasks/swarm/w1_split_reach.yaml`, `evals/tasks/swarm/w2_allocation.yaml`, `evals/tasks/swarm/w3_crossing.yaml`
- Test: `tests/evals/test_task_files.py` (append)

**Interfaces:**
- Consumes: oracle checks from Tasks 1–2, pilot `drone:` routing from Task 5, `goto_all` pilot steps.
- Produces: three `target_layer: operator`, `suite: swarm`, `setup.world: default`, `n_drones: 2` YAMLs with `pilot:`/`null_pilot:`.

Geometry (verified below in the test):
- **w1**: T1 (150, 40), T2 (−150, −40). Solo flight ≥ 60 s + return leg ≥ 62 s at 5 m/s → wall_clock 120 s kills solo; split needs ~40 s. `targets_covered` tol 12, steps 14.
- **w2**: targets A(120, 20), B(140, −30), C(−100, 60), D(−90, −70). Optimal split (east pair / west pair) ≈ 422 m fleet total; interleaved assignment ≥ 700 m; best solo tour ≈ 533 m. Fleet `path_length` max 500 sits between. tol 12, steps 16, wall 240.
- **w3**: pads pad_n (60, 80), pad_s (60, −80). Stage there, then SWAP simultaneously — straight same-altitude swap meets head-on midway. Per-drone `ordering` forces the swap; `fleet_separation` margin 8, `use_3d: true`, `grace_s: 45` (spawn adjacency + staging); steps 14, wall 240.

- [ ] **Step 1: Write the failing test (append to `tests/evals/test_task_files.py`)**

```python
def test_swarm_tasks_load_operator_layer_and_verified_geometry():
    import glob
    import math
    from evals.spec import load_task

    paths = sorted(glob.glob("evals/tasks/swarm/*.yaml"))
    assert len(paths) == 3   # w1-w3 (w4/w5 arrive with the dynamic fleet rung)
    for p in paths:
        t = load_task(p)
        assert t.target_layer == "operator" and t.suite == "swarm"
        assert t.setup.n_drones == 2
        assert t.pilot, f"{p} needs a pilot"

    # w2 allocation numbers: budget must separate optimal from interleaved+solo
    A, B, C, D = (120, 20), (140, -30), (-100, 60), (-90, -70)
    s0, s1 = (0, 0), (0, 3)
    d = math.dist
    optimal = d(s0, A) + d(A, B) + d(s1, C) + d(C, D)
    interleaved = d(s0, A) + d(A, C) + d(s1, B) + d(B, D)
    solo = d(s0, C) + d(C, D) + d(D, B) + d(B, A)
    assert optimal < 460 < 500 < min(interleaved, solo), \
        (optimal, interleaved, solo)
```

- [ ] **Step 2: Verify failure**

Run: `python -m pytest tests/evals/test_task_files.py -q`
Expected: new test FAILS (`len(paths) == 3` → 0).

- [ ] **Step 3: Write the YAMLs**

```yaml
# evals/tasks/swarm/w1_split_reach.yaml
# Entry rung: two targets in OPPOSITE directions, wall clock too short for one
# drone to visit both (solo needs ~130s of flight; budget 120s; split ~45s).
id: w1_split_reach
target_layer: operator
suite: swarm
difficulty: {plan_depth: 2, coordination: 2, ambiguity: 1, spatial: 1, swarm: 1}
setup:
  world: default
  n_drones: 2
  spawn: home
  seed_objects:
    - {id: t1, east: 150, north: 40}
    - {id: t2, east: -150, north: -40}
prompt: "You operate BOTH drones. Reach beacon t1 at E150 N40 AND beacon t2 at
  E-150 N-40 — they are in opposite directions and time is short, so send one
  drone to each, simultaneously."
budget: {wall_clock_s: 120, max_steps: 14}
oracle:
  - {check: alive}
  - {check: targets_covered, targets: [t1, t2], tol_m: 12}
  - {check: within_step_budget, max_steps: 14}
pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: take_off, drone: 1, args: {altitude: 12}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 150, north: 40, up: 12},
                                    {drone: 1, east: -150, north: -40, up: 12}]}}
```

```yaml
# evals/tasks/swarm/w2_allocation.yaml
# Allocation rung: 4 targets, 2 east / 2 west. The fleet path budget (500m,
# sampled 2D sum over both drones) sits between the optimal pairing (~422m)
# and both the interleaved assignment (~700m) and the best solo tour (~533m)
# — machine-verified in test_task_files. Null = the interleaved assignment.
id: w2_allocation
target_layer: operator
suite: swarm
difficulty: {plan_depth: 3, coordination: 3, ambiguity: 1, spatial: 2, swarm: 2}
setup:
  world: default
  n_drones: 2
  spawn: home
  seed_objects:
    - {id: ta, east: 120, north: 20}
    - {id: tb, east: 140, north: -30}
    - {id: tc, east: -100, north: 60}
    - {id: td, east: -90, north: -70}
prompt: "You operate BOTH drones. Visit all four checkpoints (ta E120 N20,
  tb E140 N-30, tc E-100 N60, td E-90 N-70). Fleet fuel is shared and tight:
  the SUM of both drones' distance flown must stay under 500 m — assign the
  checkpoints to drones so nobody backtracks across the map."
budget: {wall_clock_s: 240, max_steps: 16}
oracle:
  - {check: alive}
  - {check: targets_covered, targets: [ta, tb, tc, td], tol_m: 12}
  - {check: path_length, max_m: 500}
  - {check: within_step_budget, max_steps: 16}
pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: take_off, drone: 1, args: {altitude: 12}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 120, north: 20, up: 12},
                                    {drone: 1, east: -100, north: 60, up: 12}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 140, north: -30, up: 12},
                                    {drone: 1, east: -90, north: -70, up: 12}]}}
null_pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: take_off, drone: 1, args: {altitude: 12}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 120, north: 20, up: 12},
                                    {drone: 1, east: 140, north: -30, up: 12}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: -100, north: 60, up: 12},
                                    {drone: 1, east: -90, north: -70, up: 12}]}}
```

```yaml
# evals/tasks/swarm/w3_crossing.yaml
# Deconfliction rung: stage at opposite pads, then SWAP. The naive simultaneous
# straight swap meets head-on midway at the same altitude (null). The legal
# dodge is altitude layering — separation is graded in 3D. Per-drone ordering
# forces the actual swap (targets_covered alone would pass without crossing).
id: w3_crossing
target_layer: operator
suite: swarm
difficulty: {plan_depth: 3, coordination: 4, ambiguity: 1, spatial: 3, swarm: 3}
setup:
  world: default
  n_drones: 2
  spawn: home
  seed_objects:
    - {id: pad_n, east: 60, north: 80}
    - {id: pad_s, east: 60, north: -80}
prompt: "You operate BOTH drones. First: drone 0 to pad_n (E60 N80), drone 1 to
  pad_s (E60 N-80). Then SWAP them: drone 0 ends at pad_s, drone 1 ends at
  pad_n, moving at the same time. Your drones must NEVER come within 8 m of
  each other — plan the crossing (hint: altitude is yours to use)."
budget: {wall_clock_s: 240, max_steps: 14}
oracle:
  - {check: alive}
  - {check: ordering, sequence: [pad_n, pad_s], tol_m: 12, drone: 0}
  - {check: ordering, sequence: [pad_s, pad_n], tol_m: 12, drone: 1}
  - {check: fleet_separation, margin_m: 8, use_3d: true, grace_s: 45}
  - {check: within_step_budget, max_steps: 14}
pilot:
  - {tool: take_off, args: {altitude: 18}}
  - {tool: take_off, drone: 1, args: {altitude: 10}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 60, north: 80, up: 18},
                                    {drone: 1, east: 60, north: -80, up: 10}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 60, north: -80, up: 18},
                                    {drone: 1, east: 60, north: 80, up: 10}]}}
null_pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: take_off, drone: 1, args: {altitude: 12}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 60, north: 80, up: 12},
                                    {drone: 1, east: 60, north: -80, up: 12}]}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 60, north: -80, up: 12},
                                    {drone: 1, east: 60, north: 80, up: 12}]}}
```

- [ ] **Step 4: Run tests**

Run: `python -m pytest tests/evals/test_task_files.py -q`
Expected: PASS (including the geometry inequalities).

- [ ] **Step 5: Commit**

```bash
git add evals/tasks/swarm tests/evals/test_task_files.py
git commit -m "feat(evals): swarm ladder w1-w3 (split, allocation, crossing) with dual baselines"
```

---

### Task 8 (LIVE): N=2 flat container + w1–w3 dual-baseline gates

No unit tests — a bounded live checklist. Requires docker + the `squawd:dev` image on the host.

- [ ] **Step 1: Launch the fleet sim (flat world, 2 drones)**

```bash
mkdir -p /tmp/evals-claude && cp ~/.claude/.credentials.json /tmp/evals-claude/ \
  && [ -f /tmp/evals-claude.json ] || echo '{}' > /tmp/evals-claude.json
docker run -d --name evals-fleet2 \
  -e RENDER_BACKEND=cpu -e PX4_MODEL=gz_x500 \
  -v "$PWD:/workspace" \
  -v /tmp/evals-claude:/root/.claude -v /tmp/evals-claude.json:/root/.claude.json \
  -v /tmp/swarm-gz-fuel:/root/.gz/fuel \
  -e SWARM_N=2 -e PX4_GZ_WORLD=default -e GZ_WORLD=default \
  -e ROS_DOMAIN_ID=85 -e GZ_PARTITION=evalsfleet2 \
  squawd:dev bash -lc 'sim/launch/swarm_sim.sh'
```

Wait (bounded) until BOTH drones publish:

```bash
for i in $(seq 1 30); do sleep 10; \
  n=$(docker exec evals-fleet2 bash -lc 'source /opt/ros/jazzy/setup.bash; timeout 10 ros2 topic list 2>/dev/null | grep -c vehicle_local_position'); \
  [ "$n" = "2" ] && echo up && break; done
```

Expected: `up` within ~2 min. If only 1 drone appears: the PX4 instance-0
spawn race (known gotcha) — `docker exec evals-fleet2 bash -lc 'cd /workspace/PX4-Autopilot && px4 -i 0 ...'` per `docs`/memory, or restart the container once.

- [ ] **Step 2: Run the dual-baseline gate**

```bash
timeout 3000 docker exec evals-fleet2 bash -lc 'source /opt/ros/jazzy/setup.bash; \
  source /opt/px4_ws/install/setup.bash; cd /workspace && \
  PYTHONPATH=/workspace:$PYTHONPATH SWARM_N=2 GZ_WORLD=default \
  uv run --no-project --with pyyaml python -m evals.run_evals \
    --tasks evals/tasks/swarm/*.yaml --pilot --k 2 --out /tmp/fleet_gate'
```

Expected gate reading (else iterate task geometry/budgets, the gate's whole
purpose): `drones=pilot` rows PASS 2/2 on w1, w2, w3; `drones=pilot_null`
rows FAIL 2/2 on w2 (path_length > 500) and w3 (fleet_separation < 8).
Also verify the N=2 reset held across ≥ 10 consecutive cells (no infra_fail).

- [ ] **Step 3: Commit gate artifacts**

```bash
docker cp evals-fleet2:/tmp/fleet_gate evals/out/pilot_swarm
git add evals/out/pilot_swarm && git commit -m "data(evals): swarm w1-w3 dual-baseline gate"
```

---

### Task 9: w4/w5 YAMLs (dynamic + timing rungs)

**Files:**
- Create: `evals/tasks/swarm/w4_double_intercept.yaml`, `evals/tasks/swarm/w5_sync_mark.yaml`
- Test: `tests/evals/test_task_files.py` (modify the count in the swarm test: 3 → 5, and move w4's world assertion)

**Interfaces:** consumes `within_window`/`simultaneous` (Task 2), the dynamic world's movers `mov_0` (courier, N edge) and `mov_1` (rover, SE plaza — 3.5 m/s circle), pilot behaviors (`lead_chaser` etc. usable per-drone via `drone:`).

- [ ] **Step 1: Update the swarm task-count test** — `assert len(paths) == 5`; assert `load_task("evals/tasks/swarm/w4_double_intercept.yaml").setup.world == "dynamic"` and w1–w3+w5 use `default`. Run; expect FAIL.

- [ ] **Step 2: Write the YAMLs**

```yaml
# evals/tasks/swarm/w4_double_intercept.yaml
# Cooperative dynamics: tag the courier (N edge) AND the rover (SE plaza)
# within 25s of each other. The sites are ~230m apart: a solo drone tags them
# >=45s apart (gate-verified null) — only a split fleet makes the window.
id: w4_double_intercept
target_layer: operator
suite: swarm
difficulty: {plan_depth: 3, coordination: 4, ambiguity: 2, spatial: 3, swarm: 4, dynamic: 3}
setup:
  world: dynamic
  n_drones: 2
  spawn: home
  seed_objects: []
prompt: "You operate BOTH drones. Two contacts must be tagged nearly
  simultaneously (within 25 seconds of each other): get a drone within 12 m of
  contact mov_0 (patrolling the far north) and within 12 m of contact mov_1
  (circling the plaza to your southeast). One drone cannot do both in time —
  split up, observe each contact's motion, and time your tags together."
budget: {wall_clock_s: 300, max_steps: 18}
oracle:
  - {check: alive}
  - {check: within_window, window_s: 25, events: [
      {type: intercept, mover: mov_0, tol_m: 12},
      {type: intercept, mover: mov_1, tol_m: 12}]}
  - {check: within_step_budget, max_steps: 18}
pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: take_off, drone: 1, args: {altitude: 12}}
  - {tool: set_speed, args: {speed: 10}}
  - {tool: set_speed, drone: 1, args: {speed: 10}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 100, north: 100, up: 12},
                                    {drone: 1, east: 70, north: -65, up: 12}]}}
  - {behavior: lead_chaser, args: {mover: mov_0, rounds: 4, lead_s: 8, alt: 12}}
  - {behavior: lead_chaser, drone: 1, args: {mover: mov_1, rounds: 4, lead_s: 8, alt: 12}}
null_pilot:
  # solo attempt: tag the courier, then transit ~230m and tag the rover
  - {tool: take_off, args: {altitude: 12}}
  - {tool: set_speed, args: {speed: 12}}
  - {behavior: lead_chaser, args: {mover: mov_0, rounds: 3, lead_s: 8, alt: 12}}
  - {behavior: lead_chaser, args: {mover: mov_1, rounds: 5, lead_s: 8, alt: 12}}
```

NOTE for the implementer: the pilot's sequential `lead_chaser` steps run one
drone at a time — drone 1 waits while drone 0 chases. If the gate shows the
window missed because of serialization, replace the two chaser steps with a
single `goto_all` to each mover's predicted position (positions are
deterministic post-anchor: courier at phase t on the N100 line, rover on the
r=35 circle) and re-gate; keep whichever passes 2/2.

```yaml
# evals/tasks/swarm/w5_sync_mark.yaml
# Coordinated timing: both marks held in the SAME sampler snapshot by DISTINCT
# drones. Marks are 160m apart — no solo pass exists by construction.
id: w5_sync_mark
target_layer: operator
suite: swarm
difficulty: {plan_depth: 2, coordination: 4, ambiguity: 1, spatial: 2, swarm: 4}
setup:
  world: default
  n_drones: 2
  spawn: home
  seed_objects:
    - {id: mark_a, east: 80, north: 80}
    - {id: mark_b, east: 80, north: -80}
prompt: "You operate BOTH drones. A synchronized survey: one drone must be
  within 10 m of mark_a (E80 N80) AT THE SAME MOMENT the other is within 10 m
  of mark_b (E80 N-80). Position both, then bring them onto the marks
  together and hold a few seconds."
budget: {wall_clock_s: 240, max_steps: 16}
oracle:
  - {check: alive}
  - {check: simultaneous, marks: [{target: mark_a, tol_m: 10},
                                  {target: mark_b, tol_m: 10}]}
  - {check: within_step_budget, max_steps: 16}
pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: take_off, drone: 1, args: {altitude: 12}}
  - {tool: goto_all, args: {moves: [{drone: 0, east: 80, north: 80, up: 12},
                                    {drone: 1, east: 80, north: -80, up: 12}]}}
  - {tool: hover, args: {seconds: 8}}
null_pilot:
  # solo: visits both marks, never simultaneously
  - {tool: take_off, args: {altitude: 12}}
  - {tool: set_speed, args: {speed: 12}}
  - {tool: goto, args: {east: 80, north: 80, up: 12}}
  - {tool: goto, args: {east: 80, north: -80, up: 12}}
  - {tool: hover, args: {seconds: 8}}
```

- [ ] **Step 3: Run tests** — `python -m pytest tests/evals/test_task_files.py -q` → PASS.

- [ ] **Step 4: Commit**

```bash
git add evals/tasks/swarm tests/evals/test_task_files.py
git commit -m "feat(evals): swarm rungs w4 double-intercept (dynamic) + w5 sync-mark (timing)"
```

---

### Task 10 (LIVE): dynamic N=2 container + w4/w5 gates

- [ ] **Step 1: Launch** — same `docker run` as Task 8 with: name `evals-fleetdyn`, `SWARM_N=2`, `PX4_GZ_WORLD=dynamic`, `GZ_WORLD=dynamic`, `ROS_DOMAIN_ID=95`, `GZ_PARTITION=evalsfleetdyn`. Bounded wait for 2 telemetry topics as in Task 8.
- [ ] **Step 2: Gate w4/w5** — same `run_evals --pilot --k 2` command with `--tasks evals/tasks/swarm/w4_double_intercept.yaml evals/tasks/swarm/w5_sync_mark.yaml GZ_WORLD=dynamic SWARM_N=2`, out `/tmp/fleet_gate_dyn`. Expected: pilots 2/2 PASS, nulls 2/2 FAIL (w4 null must miss the 25 s window; w5 null must fail `simultaneous`). Iterate per the w4 NOTE if serialization bites.
- [ ] **Step 3: Commit** — `docker cp` into `evals/out/pilot_swarm_dyn`, commit `data(evals): swarm w4/w5 dual-baseline gate`.

---

### Task 11 (LIVE): E1 screening — 3 tiers × w1–w5 × K=2

- [ ] **Step 1: Containers** — flat fleet sims ×3 for tier parallelism
  (`evals-fleet2` domain 85 exists; add `evals-fleet2-b` domain 86,
  `evals-fleet2-c` domain 87, each with its own `/tmp/evals-claude-*` creds
  copy). w4 runs on `evals-fleetdyn` (95) afterwards, sequentially per tier
  (it's 1 task — cheap).
- [ ] **Step 2: Sweeps** — for w1/w2/w3/w5 per tier (note `SWARM_N=2` must be
  in the sweep env; `drive_sweep.sh` reads `GZ_WORLD` from env already —
  export `SWARM_N` the same way or run `run_evals` directly with the Task 8
  command shape, `--assignments drones=<tier> --k 2 --seed 11`, out
  `evals/out/swarm_<tier>`). Then w4 on the dynamic container per tier into
  the same out dir (resume-safe by design).
- [ ] **Step 3: Merge + report** — concatenate the three `results.jsonl` +
  `transcripts.jsonl` into `evals/out/swarm_v1_merged/`, render via
  `evals.report` (`aggregate`/`render_markdown`/`render_ladders`/`render_tools`
  — same snippet used for `dyn_v2_merged`).
- [ ] **Step 4: Write up + commit** — `docs/benchmarks/EVALS-SWARM-<date>.md`:
  per-rung table, tier signatures from transcripts (does anyone use `goto_all`?
  who serializes the fleet?), budget-artifact audit (the dynamic-suite lesson:
  check step-budget kills for observation tax before believing them), next
  knobs. Commit results + doc.

---

## Self-Review

- **Spec coverage:** C0 operator (Tasks 3–6), all four axes (w1/w2 allocation+entry, w3 deconfliction, w4 coop-dynamic, w5 timing — Tasks 7, 9), dual baselines everywhere a rung isn't entry (w2–w5), N=2 fleet infra + gates (Tasks 8, 10), E1 (Task 11). Deferred per spec build order: w6/w7, N=4/8 scaling (E2), C1 commander (E3/E4) — separate plan after E1 lands.
- **Placeholders:** the only intentionally open item is the w4 pilot serialization NOTE, which includes its concrete fallback (goto_all to predicted positions) — a live-gate decision, not a TBD.
- **Type consistency:** `FleetOps(ops_list)` / `.drone(i)` / `.goto_all(moves)` consistent across Tasks 3/4/5; `systems_list()` consistent across Tasks 5/6; check names in YAMLs (Tasks 7/9) match `CHECKS` registrations (Tasks 1/2); `require_layer_supported` referenced only after Task 6 defines it.
