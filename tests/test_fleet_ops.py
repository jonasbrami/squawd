"""FleetOps: concurrent multi-drone movement primitive."""
import asyncio

import pytest

from agents.flight.fleet import FleetOps


class FakeOps:
    def __init__(self, i, delay=0.0, fail=False):
        self.i = i
        self.delay = delay
        self.fail = fail
        self.calls = []
        self.t_start = None

    async def goto(self, target="", east=None, north=None, up=None,
                   heading="travel", wait=True):
        self.t_start = asyncio.get_event_loop().time()
        self.calls.append((east, north, up))
        await asyncio.sleep(self.delay)
        if self.fail:
            raise ValueError(f"drone_{self.i} boom")
        return f"drone_{self.i} arrived E{east} N{north}"


def test_goto_all_moves_run_concurrently():
    a, b = FakeOps(0, delay=0.2), FakeOps(1, delay=0.2)
    fleet = FleetOps([a, b])

    async def run():
        t0 = asyncio.get_event_loop().time()
        out = await fleet.goto_all([
            {"drone": 0, "east": 10, "north": 0, "up": 12},
            {"drone": 1, "east": -10, "north": 0, "up": 12}])
        return asyncio.get_event_loop().time() - t0, out

    dur, out = asyncio.run(run())
    assert dur < 0.35            # concurrent, not 0.4 sequential
    assert "drone_0 arrived" in out and "drone_1 arrived" in out


def test_goto_all_reports_per_drone_errors_without_losing_others():
    a, b = FakeOps(0), FakeOps(1, fail=True)
    fleet = FleetOps([a, b])
    out = asyncio.run(fleet.goto_all([
        {"drone": 0, "east": 5, "north": 5, "up": 10},
        {"drone": 1, "east": 6, "north": 6, "up": 10}]))
    assert "drone_0 arrived" in out
    assert "ERROR" in out and "boom" in out


def test_goto_all_rejects_unknown_drone():
    fleet = FleetOps([FakeOps(0)])
    with pytest.raises(ValueError, match="unknown drone 3"):
        asyncio.run(fleet.goto_all([{"drone": 3, "east": 0, "north": 0, "up": 10}]))


def test_drone_accessor_and_n():
    ops = [FakeOps(0), FakeOps(1)]
    fleet = FleetOps(ops)
    assert fleet.n == 2 and fleet.drone(1) is ops[1]


def test_drone_accepts_model_style_string_ids():
    """Operator LLMs pass the names the harness taught them — d0/d1 namespaces,
    drone_1 scan contacts. goto_all must accept them (observed live: opus's
    correct layered plan was rejected for '\"drone\":\"d0\"')."""
    ops = [FakeOps(0), FakeOps(1)]
    fleet = FleetOps(ops)
    for alias in (1, "1", "d1", "drone_1", "Drone_1"):
        assert fleet.drone(alias) is ops[1], alias
    out = asyncio.run(fleet.goto_all([
        {"drone": "d0", "east": 1, "north": 0, "up": 10},
        {"drone": "drone_1", "east": 2, "north": 0, "up": 10}]))
    assert "drone_0 arrived" in out and "drone_1 arrived" in out


def test_drone_still_rejects_garbage_ids():
    import pytest
    fleet = FleetOps([FakeOps(0)])
    for bad in ("dx", "drone_", "seven", None):
        with pytest.raises((ValueError, TypeError)):
            fleet.drone(bad)
