"""Command arbiter contract (design v0.3, W0.4): ONE serialized owner —
estop (latched) > operator lease > LLM — around THE shared
ActiveToolRegistry. Dummy registered async tasks only: no LLM, no sim."""
import asyncio

import pytest

from agents.flight.errors import OperatorActiveError
from agents.flight.tools import _handler
from agents.pilot.arbiter import CommandArbiter
from agents.pilot.estop import ActiveToolRegistry


class FakeOps:
    def __init__(self):
        self.calls = []

    async def emergency_hold(self):
        self.calls.append("emergency_hold")
        return "drone_0 HOLDING (estop)"

    async def emergency_land(self):
        self.calls.append("emergency_land")
        return "drone_0 LANDING (estop)"


def make_llm_tool(reg, observed):
    """A registered, cancellable stand-in for an in-flight LLM tool task."""
    async def llm_tool():
        reg.register(asyncio.current_task())
        try:
            await asyncio.sleep(60)
        except asyncio.CancelledError:
            observed.append("llm cancelled")
            raise
    return llm_tool


def make_run_op(observed, gate=None):
    """The W3 FlightOps binding stand-in: records ops; non-stop ops block on
    `gate` (an asyncio.Event) so tests control op completion."""
    async def run_op(op):
        observed.append(f"op:{op['op']}")
        if gate is not None and op["op"] != "stop":
            await gate.wait()
        return f"{op['op']} done"
    return run_op


# (a) operator > LLM: preemption + registry ownership transfer

def test_operator_op_preempts_running_llm_task():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        gate = asyncio.Event()
        arb = CommandArbiter(reg, ops, make_run_op(observed, gate))
        llm = asyncio.create_task(make_llm_tool(reg, observed)())
        await asyncio.sleep(0.05)
        res = await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        assert res == {"ok": True, "op": "lock"}
        await asyncio.sleep(0.05)
        assert observed == ["llm cancelled", "op:lock"]   # preemption observed
        assert llm.done()
        assert arb.lease_held and reg._task is arb._lease  # ownership moved
        gate.set()                                        # op completes
        await asyncio.sleep(0.05)
        assert not arb.lease_held             # lease ends on op completion

    asyncio.run(main())


def test_unknown_operator_op_rejected_without_preemption():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        arb = CommandArbiter(reg, ops, make_run_op(observed))
        llm = asyncio.create_task(make_llm_tool(reg, observed)())
        await asyncio.sleep(0.05)
        res = await arb.submit_operator({"op": "dance"})
        assert res["ok"] is False and "INVALID_PARAM" in res["error"]
        assert observed == []                             # LLM untouched
        assert not arb.lease_held
        await reg.cancel_current()

    asyncio.run(main())


# (b) the lease gate: LLM rejected while held, accepted after release

def test_llm_rejected_while_lease_held_accepted_after_release():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        gate = asyncio.Event()
        arb = CommandArbiter(reg, ops, make_run_op(observed, gate))
        await arb.submit_operator({"op": "orbit", "contact": "vis_car_0"})
        await asyncio.sleep(0.05)
        assert arb.lease_held
        with pytest.raises(OperatorActiveError) as e:
            arb.guard_llm("track")
        assert e.value.code == "OPERATOR_ACTIVE"
        # the structured tool-wrapper mapping W3 wires (guard BEFORE register)
        async def wired(args):
            arb.guard_llm("track")
            return {"content": [{"type": "text", "text": "unreachable"}]}
        res = await _handler("track", reg, wired)({})
        assert res["is_error"] is True
        assert res["content"][0]["text"].startswith(
            "OPERATOR_ACTIVE: operator active")
        gate.set()                                        # lease released
        await asyncio.sleep(0.05)
        assert not arb.lease_held
        arb.guard_llm("track")                            # no raise now

    asyncio.run(main())


def test_stop_op_cancels_the_running_op_holds_and_releases_the_lease():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        gate = asyncio.Event()
        arb = CommandArbiter(reg, ops, make_run_op(observed, gate))
        await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        await asyncio.sleep(0.05)
        assert arb.lease_held
        res = await arb.submit_operator({"op": "stop"})
        assert res == {"ok": True, "op": "stop"}
        await asyncio.sleep(0.05)
        assert observed == ["op:lock", "op:stop"]   # lock cancelled, stop ran
        assert not arb.lease_held                  # explicit stop ends the lease
        arb.guard_llm("goto")                      # LLM free again

    asyncio.run(main())


# (c) lease timeout

def test_lease_timeout_cancels_the_op_holds_and_releases():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        gate = asyncio.Event()                    # never set: op outlives lease
        arb = CommandArbiter(reg, ops, make_run_op(observed, gate),
                             lease_s=0.1)
        await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        await asyncio.sleep(0.05)
        assert arb.lease_held
        await asyncio.sleep(0.3)
        assert not arb.lease_held                          # timeout released
        assert ops.calls == ["emergency_hold"]   # held, never left drifting
        arb.guard_llm("goto")                              # LLM free again

    asyncio.run(main())


# (d) estop > all: cancels mid-op, latches, stays latched

def test_estop_mid_operator_op_cancels_it_and_latches():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        gate = asyncio.Event()
        arb = CommandArbiter(reg, ops, make_run_op(observed, gate))
        await arb.submit_operator({"op": "orbit", "contact": "vis_car_0"})
        await asyncio.sleep(0.05)
        msg = await arb.estop()
        await asyncio.sleep(0.05)
        assert msg == "estop: drone_0 HOLDING (estop) (tool cancelled: True)"
        assert ops.calls == ["emergency_hold"]
        assert arb.estopped is True
        assert not arb.lease_held                  # lease cancelled too
        # stays latched: neither LLM nor operator runs until release()
        with pytest.raises(OperatorActiveError):
            arb.guard_llm("goto")
        res = await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        assert res["ok"] is False and "ESTOPPED" in res["error"]
        await asyncio.sleep(0.1)
        assert arb.estopped is True                # nothing stale un-latches it
        arb.release()
        assert arb.estopped is False
        res = await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        assert res["ok"] is True
        gate.set()
        await asyncio.sleep(0.05)

    asyncio.run(main())


def test_estop_land_action_latches_and_lands():
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        arb = CommandArbiter(reg, ops, make_run_op(observed))
        msg = await arb.estop("land")
        assert ops.calls == ["emergency_land"]
        assert msg == "estop: drone_0 LANDING (estop) (tool cancelled: False)"
        assert arb.estopped is True

    asyncio.run(main())


# (e) generation invalidation: stale completions can't steal the slot

def test_generation_invalidation_blocks_stale_completion_clear():
    """The codex registry race: a preempted LLM tool's finally-clear must NOT
    wipe the operator op from the slot — the generation captured at
    register() guards it (the W3 wrapper pattern)."""
    reg, ops, observed = ActiveToolRegistry(), FakeOps(), []

    async def main():
        gate = asyncio.Event()
        arb = CommandArbiter(reg, ops, make_run_op(observed, gate))

        async def llm_tool():
            gen = reg.register(asyncio.current_task())
            try:
                await asyncio.sleep(60)
            finally:
                reg.clear(gen)              # guarded: stale gen is a no-op

        llm = asyncio.create_task(llm_tool())
        await asyncio.sleep(0.05)
        await arb.submit_operator({"op": "lock", "contact": "vis_car_0"})
        await asyncio.sleep(0.05)
        assert llm.done()                            # preempted; its clear ran
        assert reg._task is arb._lease               # ...and did NOT steal it
        msg = await arb.estop()                      # estop still reaches the op
        assert "tool cancelled: True" in msg

    asyncio.run(main())


def test_registry_clear_with_stale_generation_is_a_noop():
    reg = ActiveToolRegistry()

    async def main():
        g1 = reg.register(asyncio.current_task())
        reg.clear(g1 + 1)                            # stale: not the slot's gen
        assert reg._task is asyncio.current_task()
        reg.clear(g1)                                # the owner's own clear works
        assert reg._task is None
        reg.register(asyncio.current_task())
        reg.clear()                                  # no-arg: original contract
        assert reg._task is None

    asyncio.run(main())
