from agents.flight.backend import ToolCall, ToolResult

from evals.runner import Trace, model_for, CellResult


def test_model_for_maps_tier():
    assert model_for({"drones": "haiku"}, "drones") == "claude-haiku-4-5-20251001"
    assert model_for({"drones": "sonnet"}, "drones") == "claude-sonnet-5"
    assert model_for({"drones": "opus"}, "drones") == "claude-opus-4-8"
    assert model_for({}, "drones") is None


def test_trace_counts_tooluse_and_stamps_first():
    def blk(i):
        return ToolCall(id=f"t{i}", name="mcp__d0__goto", input={}, model="m")

    tr = Trace()
    tr.observe(blk(1), now=5.0)
    tr.observe(blk(2), now=6.0)
    tr.observe(blk(3), now=6.0)
    assert tr.steps == 3
    assert tr.first_action_t == 5.0


def test_completed_land_requires_final_successful_result():
    from evals.runner import _completed_land

    tr = Trace()
    tr.observe(ToolCall(id="l", name="mcp__pilot__land", input={}, model="m"), 1)
    assert not _completed_land(tr)
    tr.observe(ToolResult(tool_use_id="l", content="landed", is_error=False), 2)
    assert _completed_land(tr)
    tr.observe(ToolCall(id="s", name="mcp__pilot__scan", input={}, model="m"), 3)
    tr.observe(ToolResult(tool_use_id="s", content="ok", is_error=False), 4)
    assert not _completed_land(tr)


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
        await _settle(MovingThenStillWorld(), bridge=None,
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
        await _settle(AlwaysMovingWorld(), bridge=None,
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

    class FakeSystem:
        def __init__(self):
            self.param = None

        async def connect(self):
            connects["n"] += 1

    systems_made = []

    def system_factory():
        system = FakeSystem()
        systems_made.append(system)
        return system

    clients = []

    def client_builder(model):
        c = ("client", model)
        clients.append(c)
        return c

    h = DroneHarness(Deps(world=None, bridge=None, cameras=None),
                     system_factory=system_factory, client_builder=client_builder)

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
    assert len(systems_made) == 1              # only one System ever built
    assert len(clients) == 2                   # one client per client_for call
    assert s is systems_made[0]
