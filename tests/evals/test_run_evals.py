import asyncio
from evals.run_evals import parse_assignments, run_with_retry
from evals.runner import CellResult


def test_parse_assignments():
    got = parse_assignments("drones=opus;drones=haiku")
    assert got == [{"drones": "opus"}, {"drones": "haiku"}]


def test_run_with_retry_stops_on_non_infra():
    calls = {"n": 0}

    async def fn():
        calls["n"] += 1
        return CellResult("t", "drones=opus", 0, passed=True)

    res = asyncio.run(run_with_retry(fn, attempts=3))
    assert res.passed and calls["n"] == 1


def test_run_with_retry_retries_infra_then_returns_last():
    calls = {"n": 0}

    async def fn():
        calls["n"] += 1
        return CellResult("t", "drones=opus", 0, passed=False, infra_fail=True)

    res = asyncio.run(run_with_retry(fn, attempts=2))
    assert res.infra_fail and calls["n"] == 2
