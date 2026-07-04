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


def _sel(poses: dict, p: dict) -> list:
    """Poses considered by a check: one drone when the check carries `drone:`
    (per-drone assignments — e.g. grading a swap), else the whole fleet."""
    if "drone" in p:
        pose = poses.get(int(p["drone"]))
        return [pose] if pose is not None else []
    return list(poses.values())


def _reached(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    xy = track.objects[p["target"]]
    d = min((math.hypot(q.e - xy[0], q.n - xy[1]) for s in track.snapshots
             for q in _sel(s.poses, p)), default=math.inf)
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
                      after: float | None = None, p: dict | None = None) -> float | None:
    """First time any drone comes within `tol` of `xy`. If `after` is given, only
    consider samples strictly after that time — this lets `ordering` chain reaches so a
    waypoint that coincides with an earlier position (e.g. return-to-home) is matched at
    its later, in-sequence visit rather than at t=0."""
    for s in track.snapshots:
        if after is not None and s.t <= after:
            continue
        for pose in _sel(s.poses, p or {}):
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
        rt = _first_reach_time(track, track.objects[t], tol, after=prev, p=p)
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
             for pose in _sel(last.poses, p)), default=math.inf) if last else math.inf
    shown = "inf" if d == math.inf else f"{d:.1f}m"
    return CheckResult("final_pos", d <= tol, 0.0 if d == math.inf else d,
                       f"ended {shown} from {p['target']} (tol {tol:g})")


def _clearance(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    margin = float(p["margin_m"])
    d = _min_building_clearance(track)
    shown = "inf" if d == math.inf else f"{d:.1f}m"
    return CheckResult("clearance", d >= margin, 0.0 if d == math.inf else d,
                       f"min clearance {shown} (margin {margin:g})")


def _mover_sep(s, name: str) -> float | None:
    """Min 2D distance from any drone pose to mover `name` within ONE snapshot —
    drone and mover positions were captured the same tick, so dynamic checks
    built on this are immune to sim-time vs wall-clock drift."""
    mv = s.movers.get(name)
    if mv is None or not s.poses:
        return None
    return min(math.hypot(p.e - mv[0], p.n - mv[1]) for p in s.poses.values())


def _intercept(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Come within tol of the mover at some instant. Optional deadline: at the
    moment of contact the mover must still be > zone_radius_m from the objects
    point zone_target (perimeter-defense: 'catch it BEFORE it reaches X')."""
    tol = float(p["tol_m"])
    name = p["mover"]
    zone = track.objects[p["zone_target"]] if "zone_target" in p else None
    zr = float(p.get("zone_radius_m", 0.0))
    best, hit = math.inf, False
    for s in track.snapshots:
        d = _mover_sep(s, name)
        if d is None:
            continue
        best = min(best, d)
        if d <= tol:
            if zone is None:
                hit = True
                break
            mv = s.movers[name]
            if math.hypot(mv[0] - zone[0], mv[1] - zone[1]) > zr:
                hit = True
                break
    shown = "inf" if best == math.inf else f"{best:.1f}m"
    extra = f" before {p['zone_target']}+{zr:g}m" if zone is not None else ""
    return CheckResult("intercept", hit, 0.0 if best == math.inf else best,
                       f"min separation {shown} to {name} (tol {tol:g}){extra}")


def _dwell_moving(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Stay within tol of the mover for >= hold_s CONTIGUOUS seconds. One
    outside/missing sample doesn't break the run (2 Hz sampling can drop a
    tick); two consecutive do."""
    tol, need = float(p["tol_m"]), float(p["hold_s"])
    name = p["mover"]
    best, run_start, gap = 0.0, None, 0
    for s in track.snapshots:
        d = _mover_sep(s, name)
        if d is not None and d <= tol:
            if run_start is None:
                run_start = s.t
            gap = 0
            best = max(best, s.t - run_start)
        else:
            gap += 1
            if gap >= 2:
                run_start = None
    return CheckResult("dwell_moving", best >= need, best,
                       f"held {best:.1f}s within {tol:g}m of {name} (need {need:g}s)")


def _avoid_moving(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Never come within margin of the mover (moving keep-out bubble).
    grace_s excuses early samples, same convention as avoid_area."""
    margin = float(p["margin_m"])
    grace = float(p.get("grace_s", 0.0))
    name = p["mover"]
    worst, hits = math.inf, 0
    for s in track.snapshots:
        if s.t < grace:
            continue
        d = _mover_sep(s, name)
        if d is None:
            continue
        worst = min(worst, d)
        if d < margin:
            hits += 1
    shown = "inf" if worst == math.inf else f"{worst:.1f}m"
    return CheckResult("avoid_moving", hits == 0, 0.0 if worst == math.inf else worst,
                       f"{hits} samples within {margin:g}m of {name} (closest {shown})")


def _escort(track: WorldTrack, p: dict, m: dict) -> CheckResult:
    """Station-keeping on a mover: from the FIRST joined sample (<= tol) to the
    end of the track, the fraction of samples within tol must be >= min_fraction
    AND no continuous gap outside tol may exceed max_gap_s. Measured from first
    join so takeoff/transit to the convoy isn't charged against the escort."""
    tol = float(p["tol_m"])
    need_frac = float(p["min_fraction"])
    max_gap = float(p["max_gap_s"])
    name = p["mover"]
    seps = [(s.t, _mover_sep(s, name)) for s in track.snapshots]
    seps = [(t, d) for t, d in seps if d is not None]
    joined = next((i for i, (_, d) in enumerate(seps) if d <= tol), None)
    if joined is None:
        return CheckResult("escort", False, 0.0, f"never within {tol:g}m of {name}")
    window = seps[joined:]
    inside = sum(1 for _, d in window if d <= tol)
    frac = inside / len(window)
    worst_gap, gap_start = 0.0, None
    for t, d in window:
        if d > tol:
            gap_start = t if gap_start is None else gap_start
            worst_gap = max(worst_gap, t - gap_start)
        else:
            gap_start = None
    ok = frac >= need_frac and worst_gap <= max_gap
    return CheckResult("escort", ok, frac,
                       f"{frac:.0%} of samples within {tol:g}m of {name} after join "
                       f"(need {need_frac:.0%}); worst gap {worst_gap:.1f}s "
                       f"(max {max_gap:g}s)")


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
    "targets_covered": _targets_covered,
    "fleet_separation": _fleet_separation,
    "intercept": _intercept,
    "dwell_moving": _dwell_moving,
    "avoid_moving": _avoid_moving,
    "escort": _escort,
}


def grade(track: WorldTrack, oracle_specs: list[dict], run_meta: dict) -> GradeResult:
    results = []
    for spec in oracle_specs:
        fn = CHECKS[spec["check"]]
        results.append(fn(track, spec, run_meta))
    return GradeResult(passed=all(c.passed for c in results), checks=results)
