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
    async def hold(self): self.calls.append("hold")
    async def return_to_launch(self): self.calls.append("rtl")
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


async def test_arm_blocked_when_disconnected():
    fake = FakeController()
    store = StateStore()
    store.set_connection(False)
    ex = CommandExecutor(fake, store, SafetyGuard(LIMITS))
    result = await ex.arm()
    assert result.ok is False
    assert "arm" not in fake.calls


def _goto_calls(fake):
    return sum(1 for c in fake.calls if isinstance(c, tuple) and c[0] == "goto")


async def test_second_goto_blocked_while_first_in_progress():
    # droneserver's crash class: don't stack positioning commands.
    fake = FakeController()
    store = armed_store(in_air=True)
    ex = CommandExecutor(fake, store, SafetyGuard(LIMITS))
    first = await ex.goto_relative(10.0, 0.0, 0.0)
    assert first.ok is True
    # drone has NOT reached the target (still at HOME)
    second = await ex.goto_relative(0.0, 10.0, 0.0)
    assert second.ok is False
    assert "still executing" in second.message
    assert _goto_calls(fake) == 1


async def test_goto_allowed_after_reaching_target():
    fake = FakeController()
    store = armed_store(in_air=True)
    ex = CommandExecutor(fake, store, SafetyGuard(LIMITS))
    await ex.goto_relative(10.0, 0.0, 0.0)
    # simulate arrival ~10 m north of home
    store.set_position(GeoPoint(HOME.latitude_deg + 10.0 / 111320.0,
                                HOME.longitude_deg, HOME.absolute_altitude_m))
    second = await ex.goto_relative(0.0, 10.0, 0.0)
    assert second.ok is True
    assert _goto_calls(fake) == 2


async def test_hold_clears_interlock():
    fake = FakeController()
    store = armed_store(in_air=True)
    ex = CommandExecutor(fake, store, SafetyGuard(LIMITS))
    await ex.goto_relative(10.0, 0.0, 0.0)
    await ex.hold()  # override / terminator clears the interlock
    again = await ex.goto_relative(0.0, 10.0, 0.0)  # still at HOME, but hold cleared it
    assert again.ok is True


async def test_goto_blocked_while_takeoff_climbing():
    fake = FakeController()
    store = armed_store(in_air=False)
    ex = CommandExecutor(fake, store, SafetyGuard(LIMITS))
    await ex.takeoff(10.0)
    store.set_in_air(True)  # airborne but still low (rel_alt ~0)
    blocked = await ex.goto_relative(10.0, 0.0, 0.0)
    assert blocked.ok is False
    assert "still executing takeoff" in blocked.message
    # reached takeoff altitude (10 m above home)
    store.set_position(GeoPoint(HOME.latitude_deg, HOME.longitude_deg,
                                HOME.absolute_altitude_m + 10.0))
    ok = await ex.goto_relative(10.0, 0.0, 0.0)
    assert ok.ok is True
