"""track / track_all: MCP wiring + FleetOps fan-out + FlightOps.track guard
clauses. No sim/ROS/MAVSDK connection — fakes throughout, mirroring
test_fleet_ops.py / test_drone_tools.py / test_operator_tools.py."""
import asyncio

import pytest

from agents.flight.fleet import FleetOps
from agents.flight.ops import FlightOps
from agents.flight.tools import make_operator_options
from agents.flight import make_drone_options


class FakeOps:
    """Mirrors tests/test_fleet_ops.py's FakeOps, but for track."""

    def __init__(self, i, delay=0.0, fail=False):
        self.i = i
        self.delay = delay
        self.fail = fail
        self.calls = []

    async def track(self, target="", mode="shadow", alt=12.0, duration_s=60.0,
                    within_m=15.0, speed=12.0, standoff_east=0.0,
                    standoff_north=0.0):
        self.calls.append((target, mode))
        await asyncio.sleep(self.delay)
        if self.fail:
            raise ValueError(f"drone_{self.i} boom")
        return f"drone_{self.i} shadowed {target}"


def test_track_all_runs_concurrently():
    a, b = FakeOps(0, delay=0.2), FakeOps(1, delay=0.2)
    fleet = FleetOps([a, b])

    async def run():
        t0 = asyncio.get_event_loop().time()
        out = await fleet.track_all([
            {"drone": 0, "target": "mov_0", "mode": "shadow"},
            {"drone": 1, "target": "mov_1", "mode": "intercept"}])
        return asyncio.get_event_loop().time() - t0, out

    dur, out = asyncio.run(run())
    assert dur < 0.35            # concurrent, not 0.4 sequential
    assert "drone_0 shadowed mov_0" in out and "drone_1 shadowed mov_1" in out


def test_track_all_reports_per_drone_error_without_losing_others():
    a, b = FakeOps(0), FakeOps(1, fail=True)
    fleet = FleetOps([a, b])
    out = asyncio.run(fleet.track_all([
        {"drone": 0, "target": "mov_0"},
        {"drone": 1, "target": "mov_1"}]))
    assert "drone_0 shadowed mov_0" in out
    assert "track ERROR" in out and "boom" in out


def test_track_all_rejects_empty():
    fleet = FleetOps([FakeOps(0)])
    with pytest.raises(ValueError, match="tracks is empty"):
        asyncio.run(fleet.track_all([]))


def test_track_all_rejects_unknown_drone():
    fleet = FleetOps([FakeOps(0)])
    with pytest.raises(ValueError, match="unknown drone 3"):
        asyncio.run(fleet.track_all([{"drone": 3, "target": "mov_0"}]))


def test_drone_server_registers_and_allows_track():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "mcp__d0__track" in opts.allowed_tools


def test_drone_prompt_mentions_track():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "track" in opts.system_prompt


def test_operator_options_allow_track_all():
    opts, fleet = make_operator_options(
        systems=[object(), object()], world=None, bridge=None, n=2,
        cameras=None)
    assert "mcp__fleet__track_all" in opts.allowed_tools
    assert "mcp__d0__track" in opts.allowed_tools
    assert fleet.n == 2


def test_operator_prompt_mentions_track_all():
    opts, _ = make_operator_options(
        systems=[object(), object()], world=None, bridge=None, n=2,
        cameras=None)
    assert "track_all" in opts.system_prompt


class FakeGzPoses:
    """Live mover feed: poses() -> {name: (e, n)}, sim_time() -> float."""

    def __init__(self, poses=None, t=0.0):
        self._poses = poses or {}
        self._t = t

    def poses(self):
        return dict(self._poses)

    def sim_time(self):
        return self._t


def test_track_requires_dynamic_world():
    ops = FlightOps(drone=None, world=None, bridge=None, i=0, n=1, gzposes=None)
    with pytest.raises(ValueError, match="dynamic world"):
        asyncio.run(ops.track(target="mov_0"))


def test_track_rejects_unknown_mover():
    gzposes = FakeGzPoses(poses={"mov_0": (10.0, 5.0)})
    ops = FlightOps(drone=None, world=None, bridge=None, i=0, n=1, gzposes=gzposes)
    with pytest.raises(ValueError, match="unknown moving contact"):
        asyncio.run(ops.track(target="mov_no_such_thing"))
