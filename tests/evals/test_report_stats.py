"""Statistics upgrade: Wilson CIs on every rate (K=3 alone distinguishes nothing —
0/3 vs 3/3 is Fisher p=0.10), steps conditioned on success, goal-condition (fractional
check) success, and transcript-derived tool metrics (mix / goto-burst / patience)."""
from evals.report import (aggregate, aggregate_transcripts, render_markdown,
                          render_tools, wilson_ci)


def test_wilson_ci_known_values():
    lo, hi = wilson_ci(3, 3)
    assert abs(lo - 0.438) < 0.01 and hi == 1.0     # 3/3 -> [43.8%, 100%]
    lo, hi = wilson_ci(0, 3)
    assert lo == 0.0 and abs(hi - 0.562) < 0.01     # 0/3 -> [0%, 56.2%]
    assert wilson_ci(0, 0) == (0.0, 1.0)


def _row(passed, steps, checks=None, ttfa=3.0, **kw):
    r = {"task_id": "t1", "assignment": "drones=opus", "repeat": 0, "passed": passed,
         "latency_s": ttfa, "steps": steps, "infra_fail": False, "failure_reason": ""}
    if checks is not None:
        r["checks"] = checks
    r.update(kw)
    return r


def test_aggregate_carries_wilson_bounds():
    agg = aggregate([_row(True, 4), _row(True, 5), _row(False, 9)])[0]
    assert agg.successes == 2 and agg.k == 3
    assert 0.19 < agg.ci_lo < 0.22 and 0.93 < agg.ci_hi < 0.95   # 2/3 Wilson


def test_steps_conditioned_on_success():
    agg = aggregate([_row(True, 4), _row(True, 6), _row(False, 12)])[0]
    assert agg.steps_to_success == 5.0          # mean over passing rows only
    assert agg.steps_on_fail == 12.0
    only_fail = aggregate([_row(False, 12)])[0]
    assert only_fail.steps_to_success is None


def test_gcs_fraction_of_checks_passed():
    checks_2of3 = [{"name": "alive", "passed": True, "detail": ""},
                   {"name": "ordering", "passed": False, "detail": ""},
                   {"name": "dwell", "passed": True, "detail": ""}]
    checks_all = [{"name": "alive", "passed": True, "detail": ""}]
    agg = aggregate([_row(False, 5, checks=checks_2of3), _row(True, 4, checks=checks_all)])[0]
    assert abs(agg.gcs - (2 / 3 + 1.0) / 2) < 1e-9


def test_render_markdown_shows_ci_not_bare_rate():
    md = render_markdown(aggregate([_row(True, 4), _row(True, 5), _row(False, 9)]))
    assert "2/3" in md and "[" in md            # rate shown with its interval
    assert "p95" not in md                      # no tail statistics at tiny K


TROWS = [
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 0,
     "usage": {"output_tokens": 400}, "cost_usd": 0.2,
     "events": [
         {"type": "tool_call", "t": 2.0, "name": "mcp__d0__take_off", "args": {}},
         {"type": "tool_call", "t": 12.0, "name": "mcp__d0__goto", "args": {}},
         {"type": "tool_call", "t": 13.5, "name": "mcp__d0__goto", "args": {}},
         {"type": "tool_call", "t": 15.0, "name": "mcp__d0__goto", "args": {}},
         {"type": "tool_call", "t": 60.0, "name": "mcp__d0__hover", "args": {}},
     ]},
    {"task_id": "t1", "assignment": "drones=haiku", "repeat": 0,
     "usage": {"output_tokens": 150}, "cost_usd": 0.01,
     "events": [
         {"type": "tool_call", "t": 2.0, "name": "mcp__d0__take_off", "args": {}},
         {"type": "tool_call", "t": 30.0, "name": "mcp__d0__goto", "args": {}},
         {"type": "text", "t": 55.0, "text": "waiting"},
         {"type": "tool_call", "t": 60.0, "name": "mcp__d0__goto", "args": {}},
     ]},
]


def test_transcript_agg_tool_mix_and_burst():
    aggs = {a.assignment: a for a in aggregate_transcripts(TROWS)}
    opus = aggs["drones=opus"]
    assert opus.tool_mix == {"take_off": 1, "goto": 3, "hover": 1}
    # two gotos issued <5s after the previous goto -> burst score 2
    assert opus.goto_burst == 2
    haiku = aggs["drones=haiku"]
    assert haiku.goto_burst == 0                # 30s apart: patient
    assert haiku.out_tokens == 150


def test_transcript_agg_median_gap():
    haiku = {a.assignment: a for a in aggregate_transcripts(TROWS)}["drones=haiku"]
    assert haiku.gap_p50 == 29.0                # gaps 28, 30 between tool calls


def test_render_tools_lists_mix():
    md = render_tools(aggregate_transcripts(TROWS))
    assert "goto" in md and "burst" in md
