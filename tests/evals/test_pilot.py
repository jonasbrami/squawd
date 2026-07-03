"""The scripted reference pilot must drive the same Trace/oracle path as a real
agent: real SDK messages out, tools executed in order against FlightOps."""
import asyncio

from evals.pilot import ScriptedClient
from evals.runner import Trace


class FakeOps:
    def __init__(self):
        self.calls = []

    async def take_off(self, altitude=10.0):
        self.calls.append(("take_off", altitude))
        return f"airborne at {altitude}m"

    async def goto(self, target="", east=None, north=None, up=None,
                   heading="travel", wait=True):
        self.calls.append(("goto", east, north))
        return "arrived"


def _run(client):
    async def go():
        tr = Trace()
        async with client:
            await client.query("ignored")
            t = 0.0
            async for msg in client.receive_response():
                t += 1.0
                tr.observe(msg, t)
        return tr
    return asyncio.run(go())


def test_pilot_executes_script_in_order_and_traces_steps():
    ops = FakeOps()

    async def provider():
        return ops

    script = [{"tool": "take_off", "args": {"altitude": 12}},
              {"tool": "goto", "args": {"east": 60, "north": 0, "up": 12}},
              {"tool": "goto", "args": {"east": 60, "north": 60, "up": 12}}]
    tr = _run(ScriptedClient(provider, script))
    assert ops.calls == [("take_off", 12), ("goto", 60, 0), ("goto", 60, 60)]
    assert tr.steps == 3
    calls = [e for e in tr.events if e["type"] == "tool_call"]
    assert [c["name"] for c in calls] == ["pilot__take_off", "pilot__goto", "pilot__goto"]
    assert all("arrived" in c["result"] or "airborne" in c["result"] for c in calls)


def test_pilot_rejects_unknown_or_private_tool():
    async def provider():
        return FakeOps()

    import pytest
    with pytest.raises(ValueError):
        _run(ScriptedClient(provider, [{"tool": "nope"}]))
    with pytest.raises(ValueError):
        _run(ScriptedClient(provider, [{"tool": "_halt"}]))


def test_spec_parses_optional_pilot(tmp_path):
    from evals.spec import load_task
    p = tmp_path / "t.yaml"
    p.write_text("""
id: t
target_layer: single_drone
suite: spatial
difficulty: {spatial: 1}
setup: {world: default, n_drones: 1, spawn: home, seed_objects: []}
prompt: "x"
budget: {wall_clock_s: 60, max_steps: 5}
oracle:
  - {check: alive}
pilot:
  - {tool: take_off, args: {altitude: 12}}
  - {tool: goto, args: {east: 10, north: 0, up: 12}}
""")
    t = load_task(str(p))
    assert t.pilot[0]["tool"] == "take_off"
    assert t.pilot[1]["args"]["east"] == 10


def test_spec_pilot_defaults_to_none(tmp_path):
    from evals.spec import load_task
    p = tmp_path / "t.yaml"
    p.write_text("""
id: t
target_layer: single_drone
suite: spatial
difficulty: {spatial: 1}
setup: {world: default, n_drones: 1, spawn: home, seed_objects: []}
prompt: "x"
budget: {wall_clock_s: 60, max_steps: 5}
oracle:
  - {check: alive}
""")
    assert load_task(str(p)).pilot is None


# ---- dynamic behaviors ----

class FakeGz:
    """Scripted mover positions: pops the next position per poses() call."""
    def __init__(self, seq, name="mov_x"):
        self._seq = list(seq)
        self._name = name
        self._t = 0.0

    def poses(self):
        p = self._seq.pop(0) if len(self._seq) > 1 else self._seq[0]
        self._t += 5.0
        return {self._name: (p[0], p[1], 8.0)}

    def sim_time(self):
        return self._t


class BehaviorOps:
    def __init__(self, gz, pos=(0.0, 0.0)):
        self.gzposes = gz
        self.calls = []
        self._pos = pos
        self._speed = 5.0
        self.world = self
        self.bridge = None
        self.i = 0

    def drone_state(self, bridge, i):
        return (self._pos[0], self._pos[1], 12.0, 0.0)

    async def goto(self, east=None, north=None, up=None, **kw):
        self.calls.append(("goto", east, north))
        return "ARRIVED"

    async def hover(self, seconds=0.0):
        self.calls.append(("hover", seconds))
        return "held"

    async def set_speed(self, speed=5.0):
        self._speed = speed
        self.calls.append(("set_speed", speed))
        return f"speed {speed}"


async def _drain(gen):
    return [step async for step in gen]


def test_naive_chaser_gotos_current_position():
    import asyncio
    from evals.pilot import naive_chaser

    gz = FakeGz([(10, 0), (20, 0), (30, 0)])
    ops = BehaviorOps(gz)
    steps = asyncio.run(
        _drain(naive_chaser(ops, {"mover": "mov_x", "rounds": 3, "alt": 12})))
    assert [c[1] for c in ops.calls] == [10, 20, 30]   # always where it WAS
    assert len(steps) == 3 and steps[0][0] == "goto"


def test_lead_chaser_aims_ahead_of_motion():
    import asyncio
    from evals.pilot import lead_chaser

    gz = FakeGz([(10, 0), (20, 0), (30, 0)])   # 2 m/s east at 5s sim ticks
    ops = BehaviorOps(gz)
    asyncio.run(
        _drain(lead_chaser(ops, {"mover": "mov_x", "rounds": 3, "lead_s": 10, "alt": 12})))
    # round 1 has no velocity estimate (aims at 10); later rounds lead by v*10 = 20m
    assert ops.calls[0][1] == 10
    assert ops.calls[1][1] == 20 + 20 and ops.calls[2][1] == 30 + 20


def test_lead_intercept_solves_head_on_meeting():
    import asyncio
    from evals.pilot import lead_intercept

    # mover at x=100 flying WEST at 2 m/s (100 -> 90 over the 5s observation);
    # drone at origin, speed 8 -> meet at x = 90 - 2t = 8t => t = 9, x = 72,
    # minus the 2.5s accel-lag margin along the course -> aim x = 67
    gz = FakeGz([(100, 0), (90, 0)])
    ops = BehaviorOps(gz)
    asyncio.run(
        _drain(lead_intercept(ops, {"mover": "mov_x", "speed_mps": 8,
                                    "obs_s": 0.01, "alt": 12})))
    assert ops.calls[0] == ("set_speed", 8)
    tool, east, north = ops.calls[1]
    assert tool == "goto" and abs(east - 67.0) < 1.0 and abs(north) < 1e-6


def test_await_gap_hovers_until_receding_past_threshold():
    import asyncio
    from evals.pilot import await_gap

    # n climbs: 5, 12, 18, 26, 31 — first receding sample past 25 is 31
    gz = FakeGz([(110, 5), (110, 12), (110, 18), (110, 26), (110, 31)])
    ops = BehaviorOps(gz)
    steps = asyncio.run(
        _drain(await_gap(ops, {"mover": "mov_x", "coord": "n", "min_value": 25,
                               "poll_s": 2, "timeout_s": 30})))
    assert "gap open" in steps[-1][2]
    assert all(c[0] == "hover" for c in ops.calls)


def test_scripted_client_executes_behavior_steps():
    import asyncio
    from evals.pilot import ScriptedClient

    gz = FakeGz([(10, 0), (20, 0)])
    ops = BehaviorOps(gz)

    async def provider():
        return ops

    async def run():
        client = ScriptedClient(provider, [
            {"tool": "hover", "args": {"seconds": 1}},
            {"behavior": "naive_chaser", "args": {"mover": "mov_x", "rounds": 2}},
        ])
        return [m async for m in client.receive_response()]

    msgs = asyncio.run(run())
    # 1 static step + 2 behavior-yielded calls = 3 (tool_use, result) pairs
    assert len(msgs) == 6
    assert ops.calls[0] == ("hover", 1)
    assert ops.calls[1][0] == "goto"
