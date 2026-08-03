"""pilot/cmd.py — /pilot/cmd -> CommandArbiter (design v0.3 §5, W3a).

The UI's locked-object ops arrive as JSON on /pilot/cmd (std_msgs/String,
CMD_QOS). This supervisor parses, validates and dispatches them into THE
CommandArbiter (operator lease > LLM), mirroring estop_supervisor's
subscribe/dispatch pattern. Malformed or unknown ops are logged and dropped
— a bad click never crashes the pilot.

Op schema (the shared contract with the UI workstream):
  {"op":"lock","contact":name}     -> ops.track(name, mode="shadow",
                                                hold_altitude=True)
  {"op":"orbit","contact":name,"radius_m":R,"rate_dps":w}
                                   -> ops.track(name, mode="orbit",
                                                hold_altitude=True, ...)
  {"op":"standoff","contact":name,"range_m":R}
                                   -> ops.track(name, mode="shadow", range_m=R,
                                                hold_altitude=True)
  {"op":"stop"}                    -> cancel the lease + hold (NO estop latch)
  {"op":"resume","contact":name}   -> re-lock: ops.track(name, mode="shadow",
                                                hold_altitude=True)

std_msgs and the bus are imported lazily inside the supervisor (same pattern
as estop.py) so validation and the dispatch map unit-test without ROS.
"""
import asyncio
import json

from agents.core.store import TopicLog

# Validation bounds (shared with the UI): the 7 m keep-out bubble in
# agents/flight/ops.py plus margin floors every radius/range at 8 m.
RADIUS_MIN_M, RADIUS_MAX_M = 8.0, 40.0
RATE_MIN_DPS, RATE_MAX_DPS = 2.0, 45.0
RANGE_MIN_M, RANGE_MAX_M = 8.0, 40.0

_OPERATOR_OPS = ("lock", "orbit", "standoff", "stop", "resume")


def _bounded(data, key, lo, hi):
    try:
        v = float(data.get(key))
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be a number in [{lo:g}, {hi:g}]") from None
    if not lo <= v <= hi:
        raise ValueError(f"{key} {v:g} outside [{lo:g}, {hi:g}]")
    return v


def validate_op(data):
    """One parsed /pilot/cmd payload -> a normalized op dict. Raises
    ValueError with a legible reason on anything malformed — the supervisor
    logs and drops those (never crashes, never preempts the slot)."""
    if not isinstance(data, dict):
        raise ValueError("op must be a JSON object")
    name = data.get("op")
    if name not in _OPERATOR_OPS:
        raise ValueError(f"unknown op {name!r}")
    op = {"op": name}
    if name == "stop":
        return op
    contact = str(data.get("contact") or "").strip()
    if not contact:
        raise ValueError(f"op {name!r} needs a contact")
    op["contact"] = contact
    if name == "orbit":
        op["radius_m"] = _bounded(data, "radius_m", RADIUS_MIN_M, RADIUS_MAX_M)
        op["rate_dps"] = _bounded(data, "rate_dps", RATE_MIN_DPS, RATE_MAX_DPS)
    elif name == "standoff":
        op["range_m"] = _bounded(data, "range_m", RANGE_MIN_M, RANGE_MAX_M)
    return op


def make_run_op(ops):
    """The arbiter's FlightOps binding: one validated op -> the right
    ops.track(...) call (the dispatch map in the module docstring). `stop`
    holds via the idempotent public hold surface — deliberately NOT the
    estop latch: the LLM is free again the moment the hold lands."""
    async def run_op(op):
        name = op["op"]
        if name == "stop":
            return await ops.emergency_hold()
        # W3 codex §4: the operator's shadow ops hold the COMMANDED altitude
        # — the M3b beam-geometry profile sagged the COCO demo pursuit out
        # of the car's detection envelope (ops.track's default keeps the
        # profile for the LLM/mover path). R2: orbit takes the flag too —
        # the radial floor keys off hold_altitude, and an orbit inside the
        # blind cone LOST-breaks exactly like a shadow.
        if name in ("lock", "resume"):
            return await ops.track(op["contact"], mode="shadow",
                                   hold_altitude=True)
        if name == "standoff":
            return await ops.track(op["contact"], mode="shadow",
                                   range_m=op["range_m"], hold_altitude=True)
        if name == "orbit":
            return await ops.track(op["contact"], mode="orbit",
                                   radius_m=op["radius_m"],
                                   rate_dps=op["rate_dps"],
                                   hold_altitude=True)
        raise ValueError(f"unknown op {name!r}")   # validate_op gates first
    return run_op


async def cmd_supervisor(bridge, arbiter, *, msg_type=None, cmd_qos=None,
                         chat_qos=None) -> None:
    """Independent asyncio task mirroring estop_supervisor: /pilot/cmd JSON
    -> validate -> arbiter.submit_operator, ack'd on /pilot/chat. std_msgs/
    bus are resolved lazily at runtime; tests inject fakes."""
    if msg_type is None or cmd_qos is None or chat_qos is None:
        from std_msgs.msg import String as _S
        from agents.core.bus import CHAT_QOS as _CHAT, CMD_QOS as _CMD
        msg_type = _S if msg_type is None else msg_type
        cmd_qos = _CMD if cmd_qos is None else cmd_qos
        chat_qos = _CHAT if chat_qos is None else chat_qos
    log = TopicLog(bridge, "/pilot/cmd", msg_type, cmd_qos)
    seen = len(log.all())                    # never act on latched history
    while True:
        await asyncio.sleep(0.2)
        new, seen = log.since(seen)
        for line in new:
            try:
                op = validate_op(json.loads(line))
            except ValueError as e:          # malformed JSON or bad op
                print(f"/pilot/cmd dropped: {e}", flush=True)
                continue
            res = await arbiter.submit_operator(op)
            m = msg_type()
            m.data = (f"cmd {op['op']}: ok" if res.get("ok")
                      else f"cmd {op['op']}: {res.get('error')}")
            bridge.publish("/pilot/chat", msg_type, m, chat_qos)
