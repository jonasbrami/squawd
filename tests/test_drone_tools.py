from agents.flight import FlightOps, make_pilot_options


def _opts():
    return make_pilot_options(FlightOps(None, None, None),
                              report=lambda m: None)


def test_run_mission_tool_registered_and_allowed():
    assert "mcp__pilot__run_mission" in _opts().allowed_tools


def test_existing_primitives_still_registered():
    opts = _opts()
    for name in ("take_off", "goto", "orbit", "land", "scan", "report"):
        assert f"mcp__pilot__{name}" in opts.allowed_tools


def test_system_prompt_mentions_run_mission():
    assert "run_mission" in _opts().system_prompt
