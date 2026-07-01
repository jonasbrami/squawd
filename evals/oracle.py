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


CHECKS = {
    "reached": _reached,
    "coverage": _coverage,
    "alive": _alive,
    "within_step_budget": _within_step_budget,
    "visited_all": _visited_all,
    "ordering": _ordering,
}


def grade(track: WorldTrack, oracle_specs: list[dict], run_meta: dict) -> GradeResult:
    results = []
    for spec in oracle_specs:
        fn = CHECKS[spec["check"]]
        results.append(fn(track, spec, run_meta))
    return GradeResult(passed=all(c.passed for c in results), checks=results)
