"""Associate the first vision-targeting tool call with simulator truth.

The runner records the first ``track``/``goto`` call aimed at a ``vis_*``
contact, associates its measured position to truth at that instant, and gives
the result to the deterministic ``identified_target`` oracle check.
"""
import math


MAX_ASSOC_M = 25.0


def _contact_xy(contacts, name: str):
    """Return a contact's current measured position, if it has one."""
    obs_fn = getattr(contacts, "observation", None)
    if callable(obs_fn):
        try:
            view = obs_fn(name)
        except (KeyError, ValueError):
            view = None
        if view is not None and view.e is not None and view.n is not None:
            return view.e, view.n
    pos = contacts.poses().get(name)
    return None if pos is None else (pos[0], pos[1])


def associate_to_truth(xy: tuple, truth, gate_m: float = MAX_ASSOC_M):
    """Return the nearest truth mover within ``gate_m``."""
    best, best_d = None, None
    for name, pos in (truth.poses() or {}).items():
        distance = math.hypot(xy[0] - pos[0], xy[1] - pos[1])
        if distance <= gate_m and (best_d is None or distance < best_d):
            best, best_d = name, distance
    return best, best_d


def note_target_lock(trace, contacts, truth) -> None:
    """Record the first vision contact selected by ``track`` or ``goto``."""
    if "target_lock" in trace.meta:
        return
    for event in trace.events:
        if event.get("type") != "tool_call":
            continue
        tool = event.get("name", "").rsplit("__", 1)[-1]
        target = (event.get("args") or {}).get("target") or ""
        if tool not in ("track", "goto") or not str(target).startswith("vis_"):
            continue
        measured_xy = _contact_xy(contacts, str(target))
        truth_id, error = (None, None)
        if measured_xy is not None:
            truth_id, error = associate_to_truth(measured_xy, truth)
            error = round(error, 2) if truth_id is not None else None
        trace.meta["target_lock"] = {
            "contact_id": str(target),
            "sim_stamp": float(contacts.sim_time()),
            "tool": tool,
            "measured_xy": measured_xy,
            "truth_id": truth_id,
            "assoc_err_m": error,
        }
        return
