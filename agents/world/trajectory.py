"""Analytic mover trajectories: pos/vel as pure functions of seconds-since-anchor.

The SAME spec dict drives four consumers — the world generator (authoring), the
in-sim mover plugin (sim/plugins/mover_system.py, evaluated per physics step),
scan (live contact positions), and the oracle's sampled-vs-analytic cross-check.
Keeping them on one function is what makes mover motion gradable at all.

Spec forms (all speeds m/s, points [x, y] in world metres):
  {"type": "line", "p0": [..], "p1": [..], "speed_mps": v, "mode": "bounce"|"once"}
  {"type": "waypoint_loop", "pts": [[..], ...], "speed_mps": v}   # closed loop
  {"type": "circle", "center": [..], "radius_m": r, "speed_mps": v,
   "ccw": true, "phase0_deg": 0}
"""
import math


def _leg_lengths(pts: list[list[float]]) -> list[float]:
    return [math.dist(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]


def period_s(traj: dict) -> float:
    kind = traj["type"]
    v = float(traj["speed_mps"])
    if kind == "line":
        length = math.dist(traj["p0"], traj["p1"])
        return math.inf if traj.get("mode") == "once" else 2.0 * length / v
    if kind == "waypoint_loop":
        return sum(_leg_lengths(traj["pts"])) / v
    if kind == "circle":
        return 2.0 * math.pi * float(traj["radius_m"]) / v
    raise ValueError(f"unknown trajectory type {kind!r}")


def _lerp(a: list[float], b: list[float], frac: float) -> tuple[float, float]:
    return (a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac)


def pos_xy(traj: dict, t: float) -> tuple[float, float]:
    t = max(0.0, t)
    kind = traj["type"]
    v = float(traj["speed_mps"])
    if kind == "line":
        p0, p1 = traj["p0"], traj["p1"]
        length = math.dist(p0, p1)
        if traj.get("mode") == "once":
            s = min(v * t, length)
        else:
            s = (v * t) % (2.0 * length)
            if s > length:                 # return leg
                s = 2.0 * length - s
        return _lerp(p0, p1, s / length if length else 0.0)
    if kind == "waypoint_loop":
        pts, legs = traj["pts"], _leg_lengths(traj["pts"])
        s = (v * t) % sum(legs)
        for i, leg in enumerate(legs):
            if s <= leg or i == len(legs) - 1:
                return _lerp(pts[i], pts[(i + 1) % len(pts)], s / leg if leg else 0.0)
            s -= leg
    if kind == "circle":
        cx, cy = traj["center"]
        r = float(traj["radius_m"])
        sign = 1.0 if traj.get("ccw", True) else -1.0
        ang = math.radians(traj.get("phase0_deg", 0.0)) + sign * (v / r) * t
        return (cx + r * math.cos(ang), cy + r * math.sin(ang))
    raise ValueError(f"unknown trajectory type {kind!r}")


def vel_xy(traj: dict, t: float) -> tuple[float, float]:
    t = max(0.0, t)
    kind = traj["type"]
    v = float(traj["speed_mps"])
    if kind == "line":
        p0, p1 = traj["p0"], traj["p1"]
        length = math.dist(p0, p1)
        if length == 0.0:
            return (0.0, 0.0)
        ux, uy = (p1[0] - p0[0]) / length, (p1[1] - p0[1]) / length
        if traj.get("mode") == "once":
            return (ux * v, uy * v) if v * t < length else (0.0, 0.0)
        outbound = (v * t) % (2.0 * length) <= length
        sign = 1.0 if outbound else -1.0
        return (sign * ux * v, sign * uy * v)
    if kind == "waypoint_loop":
        pts, legs = traj["pts"], _leg_lengths(traj["pts"])
        s = (v * t) % sum(legs)
        for i, leg in enumerate(legs):
            if s <= leg or i == len(legs) - 1:
                a, b = pts[i], pts[(i + 1) % len(pts)]
                if leg == 0.0:
                    return (0.0, 0.0)
                return ((b[0] - a[0]) / leg * v, (b[1] - a[1]) / leg * v)
            s -= leg
    if kind == "circle":
        r = float(traj["radius_m"])
        sign = 1.0 if traj.get("ccw", True) else -1.0
        ang = math.radians(traj.get("phase0_deg", 0.0)) + sign * (v / r) * t
        return (-sign * v * math.sin(ang), sign * v * math.cos(ang))
    raise ValueError(f"unknown trajectory type {kind!r}")
