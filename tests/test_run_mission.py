import asyncio
import math

import pytest

from mavsdk.mission import MissionError, MissionResult

from agents.flight.ops import FlightOps, DEFAULT_MISSION_TIMEOUT_S


def _denied():
    return MissionError(MissionResult(MissionResult.Result.DENIED, "Denied"),
                        "start_mission()")


class FakeMission:
    def __init__(self, deny_starts=0):
        self.uploaded = None
        self.started = False
        self.paused = False
        self.start_calls = 0
        self._deny_starts = deny_starts

    async def upload_mission(self, plan):
        self.uploaded = plan

    async def start_mission(self):
        self.start_calls += 1
        if self.start_calls <= self._deny_starts:
            raise _denied()
        self.started = True

    async def pause_mission(self):
        self.paused = True


class FakeAction:
    def __init__(self):
        self.armed = False
        self.held = False

    async def arm(self):
        self.armed = True

    async def hold(self):
        self.held = True


class _Pos:
    latitude_deg = 47.0
    longitude_deg = 8.0
    absolute_altitude_m = 500.0


class FakeTelemetry:
    async def position(self):
        yield _Pos()


class FakeWorld:
    """world_xy returns None so _world_to_geo takes the origin path (no offset math)."""
    def world_xy(self, bridge, i):
        return None


class FakeDrone:
    def __init__(self, deny_starts=0):
        self.mission = FakeMission(deny_starts)
        self.action = FakeAction()
        self.telemetry = FakeTelemetry()


def _ops(deny_starts=0):
    return FlightOps(FakeDrone(deny_starts), world=FakeWorld(), bridge=None)


async def test_success_returns_logs_and_value():
    err, text = await _ops().run_mission('log("hello")\nreturn "done"')
    assert err is False
    assert "hello" in text
    assert "done" in text


async def test_runs_full_lifecycle_against_drone():
    ops = _ops()
    code = (
        "from mavsdk.mission import MissionPlan\n"
        "items = [mission_item(latitude_deg=1.0, longitude_deg=2.0, "
        "relative_altitude_m=15.0, speed_m_s=5.0)]\n"
        "await drone.mission.upload_mission(MissionPlan(items))\n"
        "await drone.action.arm()\n"
        "await drone.mission.start_mission()\n"
        "return 'flown'"
    )
    err, text = await ops.run_mission(code)
    assert err is False
    assert ops.drone.action.armed is True
    assert ops.drone.mission.started is True
    assert ops.drone.mission.uploaded is not None
    assert "flown" in text


async def test_none_return_reports_completed():
    err, text = await _ops().run_mission("x = 1 + 1")
    assert err is False
    assert "completed" in text.lower()


async def test_runtime_exception_returns_traceback():
    err, text = await _ops().run_mission("raise ValueError('boom')")
    assert err is True
    assert "ValueError" in text
    assert "boom" in text


async def test_syntax_error_returns_error_not_raise():
    err, text = await _ops().run_mission("def : this is not python")
    assert err is True
    assert "Error" in text


async def test_timeout_halts_vehicle():
    ops = _ops()
    err, text = await ops.run_mission("import asyncio\nawait asyncio.sleep(100)",
                                      timeout=0.05)
    assert err is True
    assert "timed out" in text
    assert ops.drone.mission.paused is True


async def test_cancel_halts_vehicle():
    ops = _ops()
    task = asyncio.ensure_future(
        ops.run_mission("import asyncio\nawait asyncio.sleep(100)"))
    # let run_mission start and reach the await
    await asyncio.sleep(0.05)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert ops.drone.mission.paused is True


def test_default_timeout_value():
    assert DEFAULT_MISSION_TIMEOUT_S == 180.0


async def test_world_to_geo_accepts_kwargs():
    # The tool docstring + system prompt advertise world_to_geo(east, north, up);
    # calling by keyword must not raise TypeError.
    g = await _ops()._world_to_geo(east=10.0, north=5.0, up=12.0)
    assert g.latitude_deg == 47.0 and g.longitude_deg == 8.0


async def test_world_to_geo_positional_still_works():
    g = await _ops()._world_to_geo(10.0, 5.0, 12.0)
    assert g.latitude_deg == 47.0


async def test_arm_and_start_retries_through_denied():
    # PX4 DENIES the first start_mission right after arm; the helper must retry.
    ops = _ops(deny_starts=2)
    await ops._arm_and_start(delay=0)
    assert ops.drone.action.armed is True
    assert ops.drone.mission.started is True
    assert ops.drone.mission.start_calls == 3  # 2 denied + 1 success


async def test_arm_and_start_reraises_if_never_succeeds():
    ops = _ops(deny_starts=99)
    with pytest.raises(MissionError):
        await ops._arm_and_start(retries=3, delay=0)
    assert ops.drone.mission.start_calls == 3


async def test_arm_and_start_bound_in_namespace():
    ops = _ops(deny_starts=1)
    err, text = await ops.run_mission("await arm_and_start(delay=0)\nreturn 'ok'")
    assert err is False
    assert ops.drone.mission.started is True
    assert "ok" in text
