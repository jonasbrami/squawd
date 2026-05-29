# tests/test_executor.py
from dronebot.control.executor import CommandExecutor, CommandResult
from dronebot.control.state import StateStore
from dronebot.control.safety import SafetyGuard, SafetyLimits
from dronebot.control.geo import GeoPoint

LIMITS = SafetyLimits(30.0, 100.0, 60.0, 2.0, 20.0)
HOME = GeoPoint(47.3977, 8.5456, 500.0)


class FakeController:
    def __init__(self):
        self.calls = []

    async def arm(self): self.calls.append("arm")
    async def takeoff(self, alt): self.calls.append(("takeoff", alt))
    async def land(self): self.calls.append("land")
    async def goto(self, target, yaw_deg=float("nan")): self.calls.append(("goto", target))


def armed_store(in_air=False):
    s = StateStore()
    s.set_connection(True)
    s.set_armed(True)
    s.set_in_air(in_air)
    s.set_flight_mode("HOLD")
    s.set_home(HOME)
    s.set_position(HOME)
    return s


async def test_takeoff_blocked_by_safety_returns_error_result():
    ex = CommandExecutor(FakeController(), armed_store(in_air=False), SafetyGuard(LIMITS))
    result = await ex.takeoff(999.0)
    assert isinstance(result, CommandResult)
    assert result.ok is False
    assert "cap" in result.message or "maximum" in result.message


async def test_takeoff_ok_calls_controller():
    fake = FakeController()
    ex = CommandExecutor(fake, armed_store(in_air=False), SafetyGuard(LIMITS))
    result = await ex.takeoff(10.0)
    assert result.ok is True
    assert ("takeoff", 10.0) in fake.calls


async def test_goto_relative_translates_and_calls_controller():
    fake = FakeController()
    ex = CommandExecutor(fake, armed_store(in_air=True), SafetyGuard(LIMITS))
    result = await ex.goto_relative(north_m=10.0, east_m=0.0, up_m=0.0)
    assert result.ok is True
    assert fake.calls and fake.calls[-1][0] == "goto"


async def test_goto_relative_outside_geofence_blocked():
    fake = FakeController()
    ex = CommandExecutor(fake, armed_store(in_air=True), SafetyGuard(LIMITS))
    result = await ex.goto_relative(north_m=500.0, east_m=0.0, up_m=0.0)
    assert result.ok is False
    assert not any(c[0] == "goto" for c in fake.calls if isinstance(c, tuple))
