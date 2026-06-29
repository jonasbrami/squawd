from evals.runner import Trace, model_for, CellResult


class FakeTool:  # stands in for ToolUseBlock duck-typing in the test
    pass


def test_model_for_maps_tier():
    assert model_for({"drones": "haiku"}, "drones") == "claude-haiku-4-5-20251001"
    assert model_for({}, "drones") is None


def test_trace_counts_tooluse_and_stamps_first(monkeypatch):
    import evals.runner as r
    # Treat FakeTool as the ToolUseBlock type for this test.
    monkeypatch.setattr(r, "ToolUseBlock", FakeTool)

    class Msg:
        def __init__(self, content):
            self.content = content
    monkeypatch.setattr(r, "AssistantMessage", Msg)

    tr = Trace()
    tr.observe(Msg([FakeTool()]), now=5.0)
    tr.observe(Msg([FakeTool(), FakeTool()]), now=6.0)
    assert tr.steps == 3
    assert tr.first_action_t == 5.0


def test_cellresult_row_roundtrip():
    cr = CellResult("t1", "drones=haiku", 0, True, [], 12.3, 4, False, "")
    row = cr.to_row()
    assert row["task_id"] == "t1" and row["passed"] is True and row["steps"] == 4
