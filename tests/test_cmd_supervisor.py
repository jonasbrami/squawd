"""/pilot/cmd supervisor contract (design v0.3 §5, W3a): op-JSON validation
bounds, the run_op dispatch map into FlightOps.track, the supervisor's
subscribe/dispatch loop (malformed ops dropped, never a crash), and the
arbiter integration — stop releases the lease, guard_llm blocks wired LLM
tools while the lease holds, and the generation-guarded clear survives a
preemption end-to-end. Fake ops/bridge only: no LLM, no sim."""
import asyncio
import json

import pytest

from agents.flight.tools import _handler, _ok
from agents.pilot.arbiter import CommandArbiter
from agents.pilot.cmd import (RANGE_MAX_M, RANGE_MIN_M, RATE_MAX_DPS,
                              RATE_MIN_DPS, RADIUS_MAX_M, RADIUS_MIN_M,
                              cmd_supervisor, make_run_op, validate_op)
from agents.pilot.estop import ActiveToolRegistry


class FakeOps:
    """Records track calls (which block on a one-shot gate when armed, so a
    lease stays held until the test releases it) and the estop surface."""

    def __init__(self):
        self.track_calls = []
        self.holds = []
        self.gate = None

    async def track(self, target="", mode="shadow", **kw):
        self.track_calls.append((target, mode, kw))
        gate, self.gate = self.gate, None        # only the FIRST call blocks
        if gate is not None:
            await gate.wait()
        return f"drone_0 {mode} {target}"

    async def emergency_hold(self):
        self.holds.append("emergency_hold")
        return "drone_0 HOLDING (estop)"

    async def emergency_land(self):
        return "drone_0 LANDING (estop)"


class FakeBridge:
    """Minimal bridge double for TopicLog + publish (same shape as test_estop)."""

    def __init__(self):
        self._cb = None
        self.published = []

    def subscribe(self, topic, msg_type, qos=None, callback=None):
        self._cb = callback

    def publish(self, topic, msg_type, msg, qos=None):
        self.published.append((topic, msg.data))

    def feed(self, text):
        class M:
            data = text
        self._cb(M())


class FakeString:
    def __init__(self):
        self.data = ""


# ---------- validate_op: the schema + bounds ----------

def test_validate_op_accepts_the_schema():
    assert validate_op({"op": "lock", "contact": "vis_car_0"}) == \
        {"op": "lock", "contact": "vis_car_0"}
    assert validate_op({"op": "resume", "contact": "vis_car_0"}) == \
        {"op": "resume", "contact": "vis_car_0"}
    assert validate_op({"op": "stop"}) == {"op": "stop"}
    assert validate_op({"op": "orbit", "contact": "vis_car_0",
                        "radius_m": 15, "rate_dps": 15}) == \
        {"op": "orbit", "contact": "vis_car_0",
         "radius_m": 15.0, "rate_dps": 15.0}
    assert validate_op({"op": "standoff", "contact": "vis_car_0",
                        "range_m": 12}) == \
        {"op": "standoff", "contact": "vis_car_0", "range_m": 12.0}


def test_validate_op_requires_contact_for_the_tracking_ops():
    for op in ("lock", "orbit", "standoff", "resume"):
        with pytest.raises(ValueError, match="contact"):
            validate_op({"op": op, "radius_m": 15, "rate_dps": 15,
                         "range_m": 15})


def test_validate_op_enforces_the_bounds():
    bad = [
        {"op": "orbit", "contact": "c", "radius_m": RADIUS_MIN_M - 0.1,
         "rate_dps": 15},
        {"op": "orbit", "contact": "c", "radius_m": RADIUS_MAX_M + 0.1,
         "rate_dps": 15},
        {"op": "orbit", "contact": "c", "radius_m": 15,
         "rate_dps": RATE_MIN_DPS - 0.1},
        {"op": "orbit", "contact": "c", "radius_m": 15,
         "rate_dps": RATE_MAX_DPS + 0.1},
        {"op": "orbit", "contact": "c"},                      # missing radius
        {"op": "orbit", "contact": "c", "radius_m": "wide",
         "rate_dps": 15},
        {"op": "standoff", "contact": "c", "range_m": RANGE_MIN_M - 0.1},
        {"op": "standoff", "contact": "c", "range_m": RANGE_MAX_M + 0.1},
        {"op": "standoff", "contact": "c"},                   # missing range
    ]
    for payload in bad:
        with pytest.raises(ValueError):
            validate_op(payload)
    # the boundary values themselves pass (8 m = 7 m keep-out + margin)
    validate_op({"op": "orbit", "contact": "c", "radius_m": RADIUS_MIN_M,
                 "rate_dps": RATE_MIN_DPS})
    validate_op({"op": "orbit", "contact": "c", "radius_m": RADIUS_MAX_M,
                 "rate_dps": RATE_MAX_DPS})
    validate_op({"op": "standoff", "contact": "c", "range_m": RANGE_MIN_M})


def test_validate_op_rejects_unknown_ops_and_non_objects():
    with pytest.raises(ValueError, match="unknown op"):
        validate_op({"op": "dance", "contact": "c"})
    for payload in (None, "lock", ["lock"], 3.14):
        with pytest.raises(ValueError, match="JSON object"):
            validate_op(payload)


# ---------- make_run_op: the dispatch map ----------

def test_run_op_dispatches_each_op_to_the_right_track_call():
    ops = FakeOps()
    run_op = make_run_op(ops)
    asyncio.run(run_op({"op": "lock", "contact": "vis_car_0"}))
    asyncio.run(run_op({"op": "resume", "contact": "vis_car_1"}))
    asyncio.run(run_op({"op": "standoff", "contact": "vis_car_2",
                        "range_m": 12.0}))
    asyncio.run(run_op({"op": "orbit", "contact": "vis_car_3",
                        "radius_m": 18.0, "rate_dps": 20.0}))
    assert ops.track_calls == [
        ("vis_car_0", "shadow", {"hold_altitude": True}),  # lock -> shadow
        ("vis_car_1", "shadow", {"hold_altitude": True}),  # resume -> re-lock
        ("vis_car_2", "shadow", {"range_m": 12.0,
                                 "hold_altitude": True}),  # standoff
        ("vis_car_3", "orbit", {"radius_m": 18.0, "rate_dps": 20.0,
                                "hold_altitude": True}),   # orbit (R2 floor)
    ]
    assert ops.holds == []                              # no hold without stop


def test_run_op_orbit_passes_hold_altitude_for_the_r2_radial_floor():
    """W3 codex R2: the operator orbit opts into hold_altitude so FlightOps'
    radial floor (R_min by commanded hold altitude) applies to orbits too —
    an orbit inside the blind cone LOST-breaks exactly like a shadow."""
    ops = FakeOps()
    asyncio.run(make_run_op(ops)({"op": "orbit", "contact": "vis_car_0",
                                  "radius_m": 20.0, "rate_dps": 8.0}))
    assert ops.track_calls == [("vis_car_0", "orbit",
                                {"radius_m": 20.0, "rate_dps": 8.0,
                                 "hold_altitude": True})]


def test_run_op_stop_holds_without_track_or_estop_latch():
    ops = FakeOps()
    msg = asyncio.run(make_run_op(ops)({"op": "stop"}))
    assert msg == "drone_0 HOLDING (estop)"     # the idempotent hold surface
    assert ops.holds == ["emergency_hold"]
    assert ops.track_calls == []                # stop is NOT a tracking op


# ---------- cmd_supervisor: subscribe/dispatch ----------

async def _run_supervisor(bridge, arbiter):
    sup = asyncio.create_task(cmd_supervisor(
        bridge, arbiter, msg_type=FakeString, cmd_qos=object(),
        chat_qos=object()))
    for _ in range(200):                          # wait for the subscription
        if bridge._cb is not None:
            break
        await asyncio.sleep(0.01)
    return sup


async def _stop_supervisor(sup):
    sup.cancel()
    try:
        await sup
    except asyncio.CancelledError:
        pass


def test_supervisor_dispatches_json_to_the_arbiter_and_acks():
    ops, bridge = FakeOps(), FakeBridge()
    arb = CommandArbiter(ActiveToolRegistry(), ops, make_run_op(ops))

    async def main():
        gate = ops.gate = asyncio.Event()         # the lease stays held
        sup = await _run_supervisor(bridge, arb)
        bridge.feed(json.dumps({"op": "orbit", "contact": "vis_car_0",
                                "radius_m": 15, "rate_dps": 15}))
        await asyncio.sleep(0.4)
        assert arb.lease_held
        assert ops.track_calls == [("vis_car_0", "orbit",
                                    {"radius_m": 15.0, "rate_dps": 15.0,
                                     "hold_altitude": True})]
        assert ("/pilot/chat", "cmd orbit: ok") in bridge.published
        gate.set()
        await asyncio.sleep(0.1)
        assert not arb.lease_held                 # lease ends on completion
        await _stop_supervisor(sup)

    asyncio.run(main())


def test_supervisor_drops_malformed_and_unknown_ops_without_crashing():
    ops, bridge = FakeOps(), FakeBridge()
    arb = CommandArbiter(ActiveToolRegistry(), ops, make_run_op(ops))

    async def main():
        sup = await _run_supervisor(bridge, arb)
        for line in ("not json at all", '{"op":"dance"}',
                     '{"op":"orbit","contact":"c","radius_m":3}',
                     '{"op":"lock"}', '[1,2,3]', '"just a string"'):
            bridge.feed(line)
        await asyncio.sleep(0.5)
        assert ops.track_calls == []              # nothing dispatched
        assert bridge.published == []             # nothing acked
        assert not arb.lease_held
        # ...and the supervisor is alive enough to take a valid op next
        bridge.feed('{"op":"lock","contact":"vis_car_0"}')
        await asyncio.sleep(0.4)
        assert ops.track_calls == [("vis_car_0", "shadow",
                                    {"hold_altitude": True})]
        await _stop_supervisor(sup)

    asyncio.run(main())


def test_supervisor_stop_releases_the_lease_and_holds():
    ops, bridge = FakeOps(), FakeBridge()
    arb = CommandArbiter(ActiveToolRegistry(), ops, make_run_op(ops))

    async def main():
        ops.gate = asyncio.Event()                # never set: lock stays held
        sup = await _run_supervisor(bridge, arb)
        bridge.feed('{"op":"lock","contact":"vis_car_0"}')
        await asyncio.sleep(0.4)
        assert arb.lease_held
        bridge.feed('{"op":"stop"}')
        await asyncio.sleep(0.4)
        assert ops.holds == ["emergency_hold"]    # held, no emergency latch
        assert not arb.estopped
        assert not arb.lease_held                 # the lock op was cancelled
        assert ("/pilot/chat", "cmd stop: ok") in bridge.published
        arb.guard_llm("goto")                     # the LLM is free again
        # resume re-locks through the same path
        bridge.feed('{"op":"resume","contact":"vis_car_0"}')
        await asyncio.sleep(0.4)
        assert ops.track_calls[-1] == ("vis_car_0", "shadow",
                                       {"hold_altitude": True})
        await _stop_supervisor(sup)

    asyncio.run(main())


def test_supervisor_ops_preempt_each_other_through_the_lease():
    ops, bridge = FakeOps(), FakeBridge()
    arb = CommandArbiter(ActiveToolRegistry(), ops, make_run_op(ops))

    async def main():
        ops.gate = asyncio.Event()                # only the first op blocks
        sup = await _run_supervisor(bridge, arb)
        bridge.feed('{"op":"lock","contact":"vis_car_0"}')
        await asyncio.sleep(0.4)
        bridge.feed('{"op":"standoff","contact":"vis_car_0","range_m":12}')
        await asyncio.sleep(0.4)
        assert ops.track_calls == [
            ("vis_car_0", "shadow", {"hold_altitude": True}),
            ("vis_car_0", "shadow", {"range_m": 12.0,
                                     "hold_altitude": True})]
        await _stop_supervisor(sup)

    asyncio.run(main())


# ---------- arbiter guard + registry gen wiring (the _handler seam) ----------

def test_guard_blocks_a_wired_llm_tool_while_lease_held_passes_after_release():
    ops = FakeOps()
    reg = ActiveToolRegistry()
    arb = CommandArbiter(reg, ops, make_run_op(ops))

    async def main():
        gate = ops.gate = asyncio.Event()
        await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        await asyncio.sleep(0.05)
        assert arb.lease_held

        async def tool_body(args):
            return _ok("flew")

        res = await _handler("track", reg, tool_body, arb.guard_llm)({})
        assert res["is_error"] is True
        assert res["content"][0]["text"].startswith(
            "OPERATOR_ACTIVE: operator active")
        # rejected BEFORE register(): the lease keeps the slot
        assert reg._task is arb._lease
        gate.set()
        await asyncio.sleep(0.05)
        assert not arb.lease_held
        res = await _handler("track", reg, tool_body, arb.guard_llm)({})
        assert res == {"content": [{"type": "text", "text": "flew"}]}

    asyncio.run(main())


def test_gen_aware_clear_keeps_the_lease_slot_when_a_preempted_tool_finishes():
    """End-to-end W0.4 race through the REAL _handler: the preempted LLM
    tool's finally-clear must not wipe the operator lease from the slot."""
    ops = FakeOps()
    reg = ActiveToolRegistry()
    arb = CommandArbiter(reg, ops, make_run_op(ops))

    async def main():
        gate = ops.gate = asyncio.Event()
        started = asyncio.Event()

        async def tool_body(args):
            started.set()
            await asyncio.sleep(60)
            return _ok("unreachable")

        tool_task = asyncio.create_task(
            _handler("hover", reg, tool_body, arb.guard_llm)({}))
        await started.wait()
        res = await arb.submit_operator({"op": "orbit", "contact": "vis_car_0",
                                         "radius_m": 15.0, "rate_dps": 15.0})
        assert res == {"ok": True, "op": "orbit"}
        tool_res = await tool_task                # preempted mid-flight
        assert tool_res["is_error"] is True
        assert tool_res["content"][0]["text"].startswith(
            "ESTOPPED: operator halted hover")
        await asyncio.sleep(0.05)
        assert reg._task is arb._lease            # stale clear did NOT steal
        assert arb.lease_held
        gate.set()
        await asyncio.sleep(0.05)

    asyncio.run(main())
