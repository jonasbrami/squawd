from agents.flight import make_drone_options


def test_run_mission_tool_registered_and_allowed():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "mcp__d0__run_mission" in opts.allowed_tools


def test_existing_primitives_still_registered():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    for name in ("take_off", "goto", "orbit", "land", "look", "scan", "report"):
        assert f"mcp__d0__{name}" in opts.allowed_tools


def test_system_prompt_mentions_run_mission():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "run_mission" in opts.system_prompt
