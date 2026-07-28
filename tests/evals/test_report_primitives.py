"""Primitive statistics (design §13 item 7, observational only): per-primitive
latency + stable error-code counts, grouped by model / detector / difficulty
(difficulty joined from results rows on the shared cell triple)."""
from evals.report import (primitive_stats, render_primitive_stats,
                          _primitive_err_code)


def test_err_code_parsing():
    assert _primitive_err_code("INVALID_PARAM: bad east", True) == "INVALID_PARAM"
    assert _primitive_err_code("LOST: vis_target_0 dropped", False) == "LOST"
    assert _primitive_err_code("arrived (0.3m off target)", False) is None
    assert _primitive_err_code("weird failure", True) == "OTHER"
    assert _primitive_err_code(None, True) == "OTHER"
    assert _primitive_err_code(None, False) is None


TROWS = [
    {"task_id": "d4", "assignment": "drones=opus", "repeat": 0,
     "detector": "ColorBlobBackend",
     "events": [
         {"type": "tool_call", "t": 1.0, "name": "mcp__pilot__take_off",
          "result": "airborne at 12m", "is_error": False, "dur_s": 8.0},
         {"type": "tool_call", "t": 10.0, "name": "mcp__pilot__track",
          "result": "LOST: mov_1 dropped out", "is_error": False, "dur_s": 60.0},
         {"type": "tool_call", "t": 71.0, "name": "mcp__pilot__track",
          "result": "INTERCEPTED at 12.1m", "is_error": False, "dur_s": 40.0},
         {"type": "tool_call", "t": 112.0, "name": "mcp__pilot__goto",
          "result": "INVALID_PARAM: east required", "is_error": True, "dur_s": 0.1},
         {"type": "text", "t": 113.0, "text": "replanning"},
     ]},
    {"task_id": "d4", "assignment": "drones=haiku", "repeat": 0,
     "events": [
         {"type": "tool_call", "t": 1.0, "name": "pilot__track",
          "result": "TIMEOUT: track exceeded", "is_error": True, "dur_s": 120.0},
     ]},
]

RROWS = [
    {"task_id": "d4", "assignment": "drones=opus", "repeat": 0,
     "difficulty": {"dynamic": 4}, "suite": "dynamic"},
]


def test_primitive_stats_groups_and_counts():
    aggs = {(a.primitive, a.model): a for a in primitive_stats(TROWS, RROWS)}
    tr = aggs[("track", "drones=opus")]
    assert tr.calls == 2 and tr.dur_p50 == 50.0        # median of 40/60
    assert tr.errors == {"LOST": 1}
    assert tr.detector == "ColorBlobBackend"
    assert tr.difficulty == "dynamic=4"                # joined from results rows
    gt = aggs[("goto", "drones=opus")]
    assert gt.errors == {"INVALID_PARAM": 1}
    hk = aggs[("track", "drones=haiku")]
    assert hk.detector == "-" and hk.difficulty == "-"  # dims absent -> '-'
    assert hk.errors == {"TIMEOUT": 1}


def test_render_primitive_stats_is_observational_markdown():
    md = render_primitive_stats(primitive_stats(TROWS, RROWS))
    assert "observational only" in md
    assert "| track | drones=opus | ColorBlobBackend | dynamic=4 | 2 | 50.0s | LOST×1 |" in md
