from claude_agent_sdk import AssistantMessage, ToolUseBlock

from evals.runner import Trace, model_for, CellResult


def test_model_for_maps_tier():
    assert model_for({"drones": "haiku"}, "drones") == "claude-haiku-4-5-20251001"
    assert model_for({"drones": "sonnet"}, "drones") == "claude-sonnet-5"
    assert model_for({"drones": "opus"}, "drones") == "claude-opus-4-8"
    assert model_for({}, "drones") is None


def test_trace_counts_tooluse_and_stamps_first():
    def blk(i):
        return ToolUseBlock(id=f"t{i}", name="mcp__d0__goto", input={})

    def msg(*blocks):
        return AssistantMessage(content=list(blocks), model="m")

    tr = Trace()
    tr.observe(msg(blk(1)), now=5.0)
    tr.observe(msg(blk(2), blk(3)), now=6.0)
    assert tr.steps == 3
    assert tr.first_action_t == 5.0


def test_cellresult_row_roundtrip():
    cr = CellResult("t1", "drones=haiku", 0, True, [], 12.3, 4, False, "")
    row = cr.to_row()
    assert row["task_id"] == "t1" and row["passed"] is True and row["steps"] == 4


def test_cellresult_row_latency_none():
    cr = CellResult("t1", "drones=haiku", 0, False, [], None, 0, False, "wall-clock deadline")
    assert cr.to_row()["latency_s"] is None


def test_cellresult_row_carries_suite_and_difficulty():
    from evals.runner import CellResult
    cr = CellResult("t1", "drones=opus", 0, True, [], 3.0, 4, False, "",
                    difficulty={"spatial": 2}, suite="spatial")
    row = cr.to_row()
    assert row["suite"] == "spatial" and row["difficulty"] == {"spatial": 2}


def test_require_single_drone_rejects_multi():
    import pytest
    from evals.runner import require_single_drone
    from types import SimpleNamespace
    spec = SimpleNamespace(id="t", setup=SimpleNamespace(n_drones=4))
    with pytest.raises(ValueError):
        require_single_drone(spec)
    require_single_drone(SimpleNamespace(id="t", setup=SimpleNamespace(n_drones=1)))  # no raise


def test_settle_returns_when_drone_stops_moving():
    """Once the drone holds position (speed < threshold), settle returns before the
    deadline instead of burning the whole budget."""
    import asyncio
    import time
    from evals.runner import _settle

    class MovingThenStillWorld:
        # East advances 5 m/poll for 3 samples, then holds at 15.
        def __init__(self):
            self._seq = [0.0, 5.0, 10.0, 15.0, 15.0, 15.0, 15.0]
            self._i = 0

        def world_xy(self, bridge, i):
            e = self._seq[min(self._i, len(self._seq) - 1)]
            self._i += 1
            return (e, 0.0, 12.0)

    async def go():
        t0 = time.monotonic()
        # Generous deadline; settle should return well before it once still.
        await _settle(MovingThenStillWorld(), bridge=None, n=1,
                      deadline=t0 + 5.0, still_speed=0.8, poll=0.01)
        return time.monotonic() - t0

    elapsed = asyncio.run(go())
    assert elapsed < 4.0  # returned on stillness, not on the 5 s deadline


def test_settle_stops_at_deadline_when_never_still():
    """A drone that never stops moving makes settle run until the deadline (bounded)."""
    import asyncio
    import time
    from evals.runner import _settle

    class AlwaysMovingWorld:
        def __init__(self):
            self._e = 0.0

        def world_xy(self, bridge, i):
            self._e += 10.0  # 10 m per poll -> always "moving"
            return (self._e, 0.0, 12.0)

    async def go():
        t0 = time.monotonic()
        await _settle(AlwaysMovingWorld(), bridge=None, n=1,
                      deadline=t0 + 0.1, still_speed=0.8, poll=0.01)
        return time.monotonic() - t0

    elapsed = asyncio.run(go())
    assert elapsed >= 0.1  # ran until the deadline


def test_droneharness_caches_system_once_and_yields_fresh_clients():
    """The leak fix: the connected System is built exactly once and reused; each
    cell still gets a distinct client (fresh session), so no context bleed."""
    import asyncio
    from evals.runner import Deps, DroneHarness

    connects = {"n": 0}

    class FakeAgent:
        def __init__(self):
            self._system = object()

        async def connect(self):
            connects["n"] += 1

    agents_made = []

    def agent_factory(i):
        a = FakeAgent()
        agents_made.append(a)
        return a

    clients = []

    def client_builder(model):
        c = ("client", model)
        clients.append(c)
        return c

    h = DroneHarness(Deps(world=None, bridge=None, cameras=None),
                     agent_factory=agent_factory, client_builder=client_builder)

    async def go():
        s1 = await h.system()
        s2 = await h.system()
        assert s1 is s2                       # System built + connected once, reused
        c1 = h.client_for("claude-opus-4-8")
        c2 = h.client_for("claude-opus-4-8")
        assert c1 is not c2                    # fresh client per cell (same model)
        return s1

    s = asyncio.run(go())
    assert connects["n"] == 1                  # connect() called exactly once
    assert len(agents_made) == 1               # only one DroneAgent ever built
    assert len(clients) == 2                   # one client per client_for call
    assert s is agents_made[0]._system


def test_fleet_harness_connects_n_agents_once():
    import asyncio
    from evals.runner import Deps, FleetHarness

    made = []

    class FakeAgent:
        def __init__(self, i):
            self.i = i
            self._system = f"sys{i}"
            self.connects = 0

        async def connect(self):
            self.connects += 1

    def factory(i):
        a = FakeAgent(i)
        made.append(a)
        return a

    h = FleetHarness(Deps(world=None, bridge=None, cameras=None), n=2,
                     agent_factory=factory)

    async def run():
        s1 = await h.systems_list()
        s2 = await h.systems_list()
        return s1, s2

    s1, s2 = asyncio.run(run())
    assert s1 == ["sys0", "sys1"] and s2 is not s1 and s2 == s1
    assert [a.connects for a in made] == [1, 1]          # built + connected once
    assert asyncio.run(h.system()) == "sys0"             # back-compat accessor


def test_layer_gate_allows_operator_multidrone_only():
    import pytest
    from evals.runner import require_layer_supported

    class Setup:
        n_drones = 2

    class Spec:
        id = "w1"
        setup = Setup()
        target_layer = "single_drone"

    with pytest.raises(ValueError, match="n_drones==1"):
        require_layer_supported(Spec())
    Spec.target_layer = "operator"
    require_layer_supported(Spec())                      # no raise
    Spec.target_layer = "commander"
    with pytest.raises(ValueError, match="not built"):
        require_layer_supported(Spec())
