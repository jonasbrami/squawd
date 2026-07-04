"""Operator options: one client, N per-drone namespaces + fleet goto_all."""
from agents.flight.tools import make_operator_options


def test_operator_options_carry_n_namespaces_and_fleet_server():
    opts, fleet = make_operator_options(
        systems=[object(), object()], world=None, bridge=None, n=2,
        cameras=None)
    assert set(opts.mcp_servers) == {"d0", "d1", "fleet"}
    for name in ("mcp__d0__goto", "mcp__d1__scan", "mcp__fleet__goto_all"):
        assert name in opts.allowed_tools
    assert fleet.n == 2


def test_operator_prompt_frames_the_whole_fleet():
    opts, _ = make_operator_options(
        systems=[object(), object()], world=None, bridge=None, n=2,
        cameras=None)
    sp = opts.system_prompt
    assert "ALL" in sp and "goto_all" in sp and "d0" in sp and "d1" in sp
