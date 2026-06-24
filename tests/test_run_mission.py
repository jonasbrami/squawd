import asyncio
import math

import pytest

from agents.flight.ops import FlightOps, DEFAULT_MISSION_TIMEOUT_S


class FakeMission:
    def __init__(self):
        self.uploaded = None
        self.started = False
        self.paused = False

    async def upload_mission(self, plan):
        self.uploaded = plan

    async def start_mission(self):
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


class FakeDrone:
    def __init__(self):
        self.mission = FakeMission()
        self.action = FakeAction()


def _ops():
    return FlightOps(FakeDrone(), world=None, bridge=None, i=0, n=1)


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
