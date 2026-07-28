"""Estop arbiter contract (ICD §7.1): cancel the tool task (turn survives),
shielded cleanup, emergency action through the shared FlightOps, generation
counter, confirmation on /pilot/chat."""
import asyncio

from agents.pilot.estop import ActiveToolRegistry, estop_supervisor


class FakeOps:
    def __init__(self):
        self.calls = []

    async def emergency_hold(self):
        self.calls.append("emergency_hold")
        return "drone_0 HOLDING (estop)"

    async def emergency_land(self):
        self.calls.append("emergency_land")
        return "drone_0 LANDING (estop)"


class FakeBridge:
    """Minimal bridge double for TopicLog + publish_str."""

    def __init__(self):
        self._cb = None
        self.published = []

    def subscribe(self, topic, msg_type, qos=None, callback=None):
        self._cb = callback

    def publish(self, topic, msg_type, msg, qos=None):
        self.published.append((topic, msg.data))

    def feed(self, text):
        class M:
            data = text
        self._cb(M())


def test_registry_cancel_produces_estopped_result_and_bumps_generation():
    reg = ActiveToolRegistry()
    g0 = reg.generation

    async def long_tool():
        reg.register(asyncio.current_task())
        try:
            await asyncio.sleep(60)
        except asyncio.CancelledError:
            return "ESTOPPED: operator halted hover"
        finally:
            reg.clear()

    async def main():
        t = asyncio.create_task(long_tool())
        await asyncio.sleep(0.05)
        cancelled = await reg.cancel_current()
        return cancelled, await t

    cancelled, result = asyncio.run(main())
    assert cancelled is True
    assert result == "ESTOPPED: operator halted hover"
    assert reg.generation > g0


def test_cancel_with_nothing_active_is_false():
    reg = ActiveToolRegistry()
    assert asyncio.run(reg.cancel_current()) is False


class FakeString:
    def __init__(self):
        self.data = ""


def test_supervisor_cancels_tool_then_holds_and_confirms():
    ops = FakeOps()
    bridge = FakeBridge()
    reg = ActiveToolRegistry()
    started = asyncio.Event()

    async def long_tool():
        reg.register(asyncio.current_task())
        started.set()
        try:
            await asyncio.sleep(60)
        except asyncio.CancelledError:
            return "ESTOPPED"
        finally:
            reg.clear()

    async def main():
        tool_task = asyncio.create_task(long_tool())
        sup = asyncio.create_task(
            estop_supervisor(bridge, reg, ops, msg_type=FakeString,
                                   cmd_qos=object(), chat_qos=object()))
        for _ in range(200):                      # wait for the subscription
            if bridge._cb is not None and started.is_set():
                break
            await asyncio.sleep(0.01)
        bridge.feed("hold")
        await asyncio.sleep(0.3)
        sup.cancel()
        try:
            await sup
        except asyncio.CancelledError:
            pass
        return await tool_task

    result = asyncio.run(main())
    assert result == "ESTOPPED"
    assert ops.calls == ["emergency_hold"]
    assert ("/pilot/chat", "estop: drone_0 HOLDING (estop) (tool cancelled: True)") \
        in bridge.published


def test_supervisor_land_action_lands():
    ops = FakeOps()
    bridge = FakeBridge()
    reg = ActiveToolRegistry()

    async def main():
        sup = asyncio.create_task(
            estop_supervisor(bridge, reg, ops, msg_type=FakeString,
                                   cmd_qos=object(), chat_qos=object()))
        for _ in range(200):                      # wait for the subscription
            if bridge._cb is not None:
                break
            await asyncio.sleep(0.01)
        bridge.feed("land")
        await asyncio.sleep(0.3)
        sup.cancel()
        try:
            await sup
        except asyncio.CancelledError:
            pass

    asyncio.run(main())
    assert ops.calls == ["emergency_land"]
