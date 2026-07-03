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


def _first_reach_time(track: WorldTrack, xy: tuple, tol: float,
                      after: float | None = None) -> float | None:
    """First time any drone comes within `tol` of `xy`. If `after` is given, only
    consider samples strictly after that time — this lets `ordering` chain reaches so a
    waypoint that coincides with an earlier position (e.g. return-to-home) is matched at
    its later, in-sequence visit rather than at t=0."""
    for s in track.snapshots:
        if after is not None and s.t <= after:
            continue
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
    # Greedy chain: reach seq[0], then seq[1] strictly after that, etc. This is the
    # correct meaning of "visit in this order" and, unlike independent per-waypoint
    # first-reach, it doesn't false-fail when a waypoint coincides with an earlier
    # position (e.g. a return-to-home final leg sitting on the t=0 spawn point).
    times: list[float | None] = []
    prev: float | None = None
    for t in seq:
        rt = _first_reach_time(track, track.objects[t], tol, after=prev)
        times.append(rt)
        if rt is None:
            break
        prev = rt
    in_order = len(times) == len(seq) and all(x is not None for x in times)
    return CheckResult("ordering", in_order, float(sum(x is not None for x in times)),
                       f"chained-reach times {times} for {seq} (each after the previous)")


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


def _not_reached(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Inverse of reached: the track must NEVER come within tol of the target. For
    distractor rejection — pick tol/geometry so transit legs of the correct route
    can't clip it (verify at spec-authoring time)."""
    d = track.min_dist_to(track.objects[p["target"]])
    tol = float(p["tol_m"])
    return CheckResult("not_reached", d > tol, d,
                       f"min dist {d:.1f}m to {p['target']} (must stay > {tol:g}m)")


def _avoid_area(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """No sampled position inside the named area. Optional grace_s excuses early
    samples when the area abuts the spawn point."""
    grace = float(p.get("grace_s", 0.0))
    hits = sum(1 for s in track.snapshots if s.t >= grace
               for pose in s.poses.values()
               if point_in_area(p["area"], pose.e, pose.n))
    return CheckResult("avoid_area", hits == 0, float(hits),
                       f"{hits} samples inside {p['area']} (must be 0)")


def _path_length(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Total 2D distance flown (per drone, summed over sampled legs). Sampling
    slightly under-counts true path on curves — leave ~3-5% slack between the
    prompt's budget and max_m, calibrated on a reference run."""
    total, prev = 0.0, {}
    for s in track.snapshots:
        for did, pose in s.poses.items():
            if did in prev:
                total += math.hypot(pose.e - prev[did][0], pose.n - prev[did][1])
            prev[did] = (pose.e, pose.n)
    mx = float(p["max_m"])
    return CheckResult("path_length", total <= mx, total,
                       f"flew {total:.0f}m (max {mx:g}m)")


def _alt_ceiling(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Track-wide altitude ceiling (`altitude` only checks the closest pose to one
    target; this binds for the WHOLE flight)."""
    mx = float(p["max_m"])
    worst = max((pose.alt for s in track.snapshots
                 for pose in s.poses.values()), default=0.0)
    return CheckResult("alt_ceiling", worst <= mx, worst,
                       f"max alt {worst:.1f}m (ceiling {mx:g}m)")


def _min_visited(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """At least min_count of the listed targets visited within tol. For budgeted
    'visit as many as you can' tasks where visited_all (all-or-nothing) can't
    grade the tradeoff."""
    tol = float(p["tol_m"])
    need = int(p["min_count"])
    got = sum(1 for t in p["targets"] if track.min_dist_to(track.objects[t]) <= tol)
    return CheckResult("min_visited", got >= need, float(got),
                       f"visited {got}/{len(p['targets'])} within {tol:g}m (need >= {need})")


def _final_pos(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """The flight must END within tol of the target (last sample, post-settle).
    Grades 'then return to X' without pinning the visit order the way a chained
    `ordering` would — `reached` can't do it when X coincides with spawn."""
    xy = track.objects[p["target"]]
    tol = float(p["tol_m"])
    last = track.snapshots[-1] if track.snapshots else None
    d = min((math.hypot(pose.e - xy[0], pose.n - xy[1])
             for pose in last.poses.values()), default=math.inf) if last else math.inf
    shown = "inf" if d == math.inf else f"{d:.1f}m"
    return CheckResult("final_pos", d <= tol, 0.0 if d == math.inf else d,
                       f"ended {shown} from {p['target']} (tol {tol:g})")


def _clearance(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    margin = float(p["margin_m"])
    d = _min_building_clearance(track)
    shown = "inf" if d == math.inf else f"{d:.1f}m"
    return CheckResult("clearance", d >= margin, 0.0 if d == math.inf else d,
                       f"min clearance {shown} (margin {margin:g})")


CHECKS = {
    "reached": _reached,
    "coverage": _coverage,
    "alive": _alive,
    "within_step_budget": _within_step_budget,
    "visited_all": _visited_all,
    "ordering": _ordering,
    "altitude": _altitude,
    "dwell": _dwell,
    "clearance": _clearance,
    "not_reached": _not_reached,
    "avoid_area": _avoid_area,
    "path_length": _path_length,
    "alt_ceiling": _alt_ceiling,
    "final_pos": _final_pos,
    "min_visited": _min_visited,
}


def grade(track: WorldTrack, oracle_specs: list[dict], run_meta: dict) -> GradeResult:
    results = []
    for spec in oracle_specs:
        fn = CHECKS[spec["check"]]
        results.append(fn(track, spec, run_meta))
    return GradeResult(passed=all(c.passed for c in results), checks=results)
