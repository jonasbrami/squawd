"""ICD §9 error taxonomy: every code has a producible path through the tool
wrapper's mapping, IN ORDER (CancelledError first — it's BaseException)."""
import asyncio

import pytest

from agents.flight.errors import (BlockedError, InvalidParamError, NotReadyError,
                                  ToolFailure)
from agents.flight.tools import _handler


def make(name, fn):
    return _handler(name, None, fn)


def run(h, args=None):
    return asyncio.run(h(args or {}))


def test_typed_failures_map_to_their_codes():
    assert run(make("goto", lambda a: (_ for _ in ()).throw(
        InvalidParamError("bad target"))))["content"][0]["text"] == "INVALID_PARAM: bad target"
    assert run(make("track", lambda a: (_ for _ in ()).throw(
        NotReadyError("no contact feed"))))["content"][0]["text"] == "NOT_READY: no contact feed"
    assert run(make("goto", lambda a: (_ for _ in ()).throw(
        BlockedError("still enroute"))))["content"][0]["text"] == "BLOCKED: still enroute"
    assert run(make("x", lambda a: (_ for _ in ()).throw(
        ToolFailure("BLOCKED", "stuck"))))["is_error"] is True


def test_timeout_maps_to_timeout_code():
    async def slow(args):
        raise asyncio.TimeoutError()
    assert run(make("run_mission", slow))["content"][0]["text"].startswith("TIMEOUT:")


def test_cancelled_maps_to_estopped_not_internal():
    async def hanging(args):
        await asyncio.sleep(60)
        return {"content": [{"type": "text", "text": "unreachable"}]}

    async def main():
        t = asyncio.create_task(make("hover", hanging)({}))
        await asyncio.sleep(0.05)
        t.cancel()
        return await t

    res = asyncio.run(main())
    assert res["content"][0]["text"] == "ESTOPPED: operator halted hover"
    assert res["is_error"] is True


def test_unknown_errors_become_internal():
    async def boom(args):
        raise RuntimeError("weird")
    res = run(make("track", boom))
    assert res["content"][0]["text"] == "INTERNAL: RuntimeError: weird"
    assert res["is_error"] is True
