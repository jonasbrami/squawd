"""Blocking goto/fly: the move tools return on ARRIVAL by default (wait=True),
so an ordered route is one call per leg — no fire-and-forget setpoint overrides.
`wait=False` restores the old return-immediately behavior for acting mid-flight."""
import asyncio

import pytest

import agents.flight.ops as ops_mod
from agents.flight.ops import FlightOps
from agents.flight import make_drone_options


class FakeAction:
    def __init__(self):
        self.goto_calls = []

    async def goto_location(self, lat, lon, alt, yaw):
        self.goto_calls.append((lat, lon, alt, yaw))


class FakeTelemetry:
    async def position(self):
        class P:
            latitude_deg = 47.0
            longitude_deg = 8.0
            absolute_altitude_m = 500.0
        while True:
            yield P()


class FakeDrone:
    def __init__(self):
        self.action = FakeAction()
        self.telemetry = FakeTelemetry()


class FakeWorld:
    """world_xy pops through `positions` then repeats the last one forever."""
    def __init__(self, positions):
        self.positions = list(positions)

    def world_xy(self, bridge, i):
        if len(self.positions) > 1:
            return self.positions.pop(0)
        return self.positions[0]

    def resolve_xy(self, target, bridge, n):
        return None

    def drone_state(self, bridge, i):
        return None


def _ops(positions):
    return FlightOps(FakeDrone(), FakeWorld(positions), bridge=None, i=0, n=1)


@pytest.fixture(autouse=True)
def fast_polling(monkeypatch):
    monkeypatch.setattr(ops_mod, "ARRIVE_POLL_S", 0.01)
    monkeypatch.setattr(ops_mod, "ARRIVE_MIN_TIMEOUT_S", 0.1)
    monkeypatch.setattr(ops_mod, "ARRIVE_MARGIN", 0.02)


def test_await_arrival_returns_arrived_when_position_converges():
    ops = _ops([(0.0, 0.0, 10.0), (5.0, 5.0, 10.0), (20.0, 30.0, 10.0)])
    out = asyncio.run(ops._await_arrival(20.0, 30.0, 10.0))
    assert "arrived" in out


def test_await_arrival_reports_enroute_on_timeout_without_raising():
    ops = _ops([(0.0, 0.0, 10.0)])          # never moves
    out = asyncio.run(ops._await_arrival(200.0, 0.0, 10.0))
    assert "ENROUTE" in out


def test_await_arrival_checks_altitude_when_given():
    # Reaches the XY but stays 8m below the target altitude -> not arrived.
    ops = _ops([(20.0, 30.0, 2.0)])
    out = asyncio.run(ops._await_arrival(20.0, 30.0, 10.0))
    assert "ENROUTE" in out


def test_goto_waits_for_arrival_by_default():
    ops = _ops([(0.0, 0.0, 10.0), (0.0, 0.0, 10.0), (0.0, 0.0, 10.0),
                (30.0, 40.0, 12.0)])
    out = asyncio.run(ops.goto(east=30, north=40, up=12))
    assert "arrived" in out


def test_goto_wait_false_returns_immediately():
    ops = _ops([(0.0, 0.0, 10.0)])          # never reaches the target
    out = asyncio.run(ops.goto(east=30, north=40, up=12, wait=False))
    assert "moving" in out and "ENROUTE" not in out


def test_fly_waits_for_relative_arrival():
    # Start at (0,0,10); fly east +30 -> arrival gate at (30, 0, 10).
    ops = _ops([(0.0, 0.0, 10.0), (0.0, 0.0, 10.0), (30.0, 0.0, 10.0)])
    out = asyncio.run(ops.fly(east=30))
    assert "arrived" in out


def test_goto_tool_schema_and_prompt_state_blocking_semantics():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "ARRIVE" in opts.system_prompt          # prompt states goto returns on arrival
    assert "wait=false" in opts.system_prompt.lower()
