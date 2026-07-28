"""M1 tool-surface contracts (ICD §5.5/§0.6): exactly the 12 M1 tools, full JSON
schemas (required arrays + additionalProperties:false), detect registered only
when a detect_text is composed (M2)."""
from agents.flight import FlightOps, make_pilot_options
from agents.flight.tools import _schema

M1_TOOLS = ["take_off", "fly", "goto", "orbit", "hover", "set_speed", "face",
            "land", "report", "scan", "run_mission", "track"]


def _opts(**kw):
    return make_pilot_options(FlightOps(None, None, None, 0, 1),
                              report=lambda m: None, **kw)


def test_exposed_tool_list_is_exactly_the_12_m1_tools():
    opts = _opts()
    assert sorted(opts.allowed_tools) == sorted(f"mcp__pilot__{t}" for t in M1_TOOLS)


def test_detect_registers_only_when_composed():
    class FakeOps:
        pass
    no_detect = make_pilot_options(FakeOps(), report=lambda m: None)
    assert "mcp__pilot__detect" not in no_detect.allowed_tools
    with_detect = make_pilot_options(FakeOps(), detect_text=lambda c: "nothing",
                                     report=lambda m: None)
    assert "mcp__pilot__detect" in with_detect.allowed_tools


def test_schema_helper_emits_required_and_closed_objects():
    s = _schema({"speed": {"type": "number"}}, ["speed"])
    assert s["type"] == "object"
    assert s["required"] == ["speed"]
    assert s["additionalProperties"] is False
    s2 = _schema({"altitude": {"type": "number"}})
    assert s2["required"] == [] and s2["additionalProperties"] is False


def test_builtin_cli_tools_disabled():
    opts = _opts()
    assert opts.tools == []


def test_extra_prompt_appends_only_when_given():
    """Strategy-snippet A/B seam (§13 item 6): extra_prompt rides the system
    prompt for that one options instance; the default stays byte-identical."""
    from agents.flight.tools import PILOT_SYSTEM_PROMPT
    assert _opts().system_prompt == PILOT_SYSTEM_PROMPT
    opts = _opts(extra_prompt="snippet text")
    assert opts.system_prompt == PILOT_SYSTEM_PROMPT + "\n\nsnippet text"
