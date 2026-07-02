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
