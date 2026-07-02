"""Per-cell transcript capture: Trace records every tool call (name/args/result/
duration), agent text, and end-of-run usage — all from the SDK message stream the
runner already iterates. This is what lets tool-choice mechanisms (goto bursts,
run_mission use) be OBSERVED per tier instead of inferred from step counts."""
from claude_agent_sdk import (AssistantMessage, ResultMessage, TextBlock,
                              ToolResultBlock, ToolUseBlock, UserMessage)

from evals.runner import CellResult, Trace


def _assistant(blocks):
    return AssistantMessage(content=blocks, model="claude-haiku-4-5-20251001")


def test_trace_records_tool_call_events_with_args():
    tr = Trace()
    tr.observe(_assistant([ToolUseBlock(id="t1", name="mcp__d0__goto",
                                        input={"east": 60, "north": 0})]), now=3.0)
    assert tr.steps == 1
    (ev,) = tr.events
    assert ev["type"] == "tool_call" and ev["name"] == "mcp__d0__goto"
    assert ev["args"] == {"east": 60, "north": 0} and ev["t"] == 3.0


def test_trace_pairs_result_and_duration_by_tool_use_id():
    tr = Trace()
    tr.observe(_assistant([ToolUseBlock(id="t1", name="mcp__d0__goto",
                                        input={"east": 60})]), now=3.0)
    tr.observe(UserMessage(content=[ToolResultBlock(
        tool_use_id="t1", content="drone_0 -> E60 N0 alt 12; arrived (1.2m off target)",
        is_error=False)]), now=15.5)
    (ev,) = tr.events
    assert "arrived" in ev["result"] and ev["is_error"] is False
    assert ev["dur_s"] == 12.5


def test_trace_records_agent_text_and_skips_blank():
    tr = Trace()
    tr.observe(_assistant([TextBlock(text="Heading to point a now.")]), now=1.0)
    tr.observe(_assistant([TextBlock(text="   ")]), now=2.0)
    assert tr.events == [{"type": "text", "t": 1.0, "text": "Heading to point a now."}]


def test_trace_summarizes_image_results_never_stores_b64():
    tr = Trace()
    tr.observe(_assistant([ToolUseBlock(id="t1", name="mcp__d0__look", input={})]), now=1.0)
    tr.observe(UserMessage(content=[ToolResultBlock(
        tool_use_id="t1",
        content=[{"type": "image", "data": "A" * 100_000, "mimeType": "image/jpeg"}],
        is_error=False)]), now=2.0)
    (ev,) = tr.events
    assert "image" in ev["result"] and len(ev["result"]) < 600


def test_trace_captures_usage_from_result_message():
    tr = Trace()
    tr.observe(ResultMessage(subtype="success", duration_ms=42000, duration_api_ms=9000,
                             is_error=False, num_turns=7, session_id="s",
                             total_cost_usd=0.21,
                             usage={"input_tokens": 1000, "output_tokens": 400}),
               now=42.0)
    assert tr.usage == {"input_tokens": 1000, "output_tokens": 400}
    assert tr.cost_usd == 0.21 and tr.num_turns == 7 and tr.api_ms == 9000


def test_trace_captures_model_from_assistant_message():
    tr = Trace()
    tr.observe(_assistant([TextBlock(text="ok")]), now=1.0)
    assert tr.model == "claude-haiku-4-5-20251001"


def test_cellresult_transcript_row_keyed_like_results_row():
    cr = CellResult("t1", "drones=haiku", 2, True)
    cr.transcript = {"model": "m", "events": [{"type": "text", "t": 1.0, "text": "x"}]}
    row = cr.to_transcript_row()
    assert (row["task_id"], row["assignment"], row["repeat"]) == ("t1", "drones=haiku", 2)
    assert row["events"] == [{"type": "text", "t": 1.0, "text": "x"}]


def test_client_failure_is_detected_from_trace():
    """A cell where the SDK client never ran the model (auth failure, synthetic
    error message) must be flagged infra, not scored as a task FAIL."""
    from evals.runner import client_failed

    tr = Trace()
    tr.observe(AssistantMessage(content=[TextBlock(
        text="Failed to authenticate. API Error: 401 Invalid authentication credentials")],
        model="<synthetic>"), now=3.0)
    assert client_failed(tr)

    ok = Trace()
    ok.observe(_assistant([ToolUseBlock(id="t1", name="mcp__d0__goto", input={})]), now=3.0)
    assert not client_failed(ok)

    empty = Trace()          # stream produced nothing at all
    assert client_failed(empty)
