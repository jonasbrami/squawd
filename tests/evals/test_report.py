from evals.report import aggregate, render_markdown


ROWS = [
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 0, "passed": True,
     "latency_s": 3.0, "steps": 4, "infra_fail": False, "failure_reason": ""},
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 1, "passed": False,
     "latency_s": 5.0, "steps": 9, "infra_fail": False, "failure_reason": "wall-clock deadline"},
    {"task_id": "t1", "assignment": "drones=opus", "repeat": 2, "passed": True,
     "latency_s": 4.0, "steps": 5, "infra_fail": True, "failure_reason": "reset unclean"},
]


def test_aggregate_excludes_infra_fail_from_denominator():
    agg = {a.assignment: a for a in aggregate(ROWS)}["drones=opus"]
    assert agg.k == 2            # the infra_fail row is dropped
    assert agg.successes == 1
    assert agg.success_rate == 0.5


def test_failure_breakdown_counts_reasons():
    agg = aggregate(ROWS)[0]
    assert agg.failure_breakdown.get("wall-clock deadline") == 1


def test_render_markdown_has_header_and_rate():
    md = render_markdown(aggregate(ROWS))
    assert "success_rate" in md
    assert "t1" in md
