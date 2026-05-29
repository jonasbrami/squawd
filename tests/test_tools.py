# tests/test_tools.py
from dronebot.agent.tools import make_takeoff_tool, make_status_tool
from dronebot.control.executor import CommandResult


class FakeExecutor:
    def __init__(self, result): self._result = result; self.called = False
    async def takeoff(self, altitude_m): self.called = True; return self._result
    def status(self): return CommandResult(True, "all good")


async def test_takeoff_tool_reports_success():
    ex = FakeExecutor(CommandResult(True, "taking off to 10m (climbing)"))
    tool_fn = make_takeoff_tool(ex)
    out = await tool_fn({"altitude_m": 10.0})
    assert ex.called
    assert out.get("is_error") in (None, False)
    assert "climbing" in out["content"][0]["text"]


async def test_takeoff_tool_reports_refusal_as_error():
    ex = FakeExecutor(CommandResult(False, "refused: not armed"))
    tool_fn = make_takeoff_tool(ex)
    out = await tool_fn({"altitude_m": 10.0})
    assert out["is_error"] is True
    assert "not armed" in out["content"][0]["text"]


async def test_status_tool():
    ex = FakeExecutor(CommandResult(True, "x"))
    out = await make_status_tool(ex)({})
    assert "all good" in out["content"][0]["text"]
