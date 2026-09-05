import asyncio
from evals.run_evals import apply_layer_override, parse_assignments, run_with_retry
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


def test_apply_layer_override_default_spec_is_noop():
    assignments = [{"drones": "opus"}, {"drones": "haiku"}]
    assert apply_layer_override(assignments, "spec") == assignments
    assert apply_layer_override(assignments, "spec") is assignments


def test_apply_layer_override_injects_layer_key_into_every_assignment():
    assignments = [{"drones": "opus"}, {"drones": "haiku", "commander": "sonnet"}]
    got = apply_layer_override(assignments, "commander")
    assert got == [{"drones": "opus", "_layer": "commander"},
                   {"drones": "haiku", "commander": "sonnet", "_layer": "commander"}]
    # original list of dicts is untouched (no in-place mutation)
    assert assignments == [{"drones": "opus"}, {"drones": "haiku", "commander": "sonnet"}]


def test_cli_layer_flag_default_and_choices():
    import pytest
    from evals.run_evals import _build_arg_parser

    ap = _build_arg_parser()
    args = ap.parse_args(["--tasks", "x.yaml"])
    assert args.layer == "spec"

    # dropped layers are rejected at the CLI, not just at the runner gate
    with pytest.raises(SystemExit):
        ap.parse_args(["--tasks", "x.yaml", "--layer", "commander"])
    with pytest.raises(SystemExit):
        ap.parse_args(["--tasks", "x.yaml", "--layer", "operator"])
    with pytest.raises(SystemExit):
        ap.parse_args(["--tasks", "x.yaml", "--layer", "bogus"])


def test_infra_fuse_trips_on_consecutive_failures_only():
    from evals.run_evals import InfraFuse

    fuse = InfraFuse(limit=2)
    assert not fuse.update(True)        # 1 consecutive
    assert fuse.update(True)            # 2 consecutive -> tripped
    fuse = InfraFuse(limit=2)
    fuse.update(True)
    assert not fuse.update(False)       # success resets the count
    assert not fuse.update(True)
