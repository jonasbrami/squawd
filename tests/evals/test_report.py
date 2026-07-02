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


def test_latency_none_excluded_from_percentiles():
    rows = [
        {"task_id": "t1", "assignment": "drones=opus", "repeat": 0, "passed": True,
         "latency_s": 4.0, "steps": 5, "infra_fail": False, "failure_reason": ""},
        {"task_id": "t1", "assignment": "drones=opus", "repeat": 1, "passed": False,
         "latency_s": None, "steps": 0, "infra_fail": False, "failure_reason": "wall-clock deadline"},
    ]
    agg = aggregate(rows)[0]
    assert agg.k == 2                  # both scored (neither is infra_fail)
    assert agg.ttfa_p50 == 4.0         # the None row is excluded from latency stats


def test_render_ladders_pivots_by_rung_and_tier():
    from evals.report import render_ladders
    rows = [
        {"task_id": "s1", "assignment": "drones=haiku", "passed": True, "infra_fail": False,
         "suite": "spatial", "difficulty": {"spatial": 1}, "latency_s": 3, "steps": 4, "repeat": 0},
        {"task_id": "s3", "assignment": "drones=haiku", "passed": False, "infra_fail": False,
         "suite": "spatial", "difficulty": {"spatial": 3}, "latency_s": 3, "steps": 4, "repeat": 0},
        {"task_id": "x", "assignment": "drones=haiku", "passed": True, "infra_fail": False,
         "suite": None, "difficulty": {}, "latency_s": 3, "steps": 4, "repeat": 0},
    ]
    md = render_ladders(rows)
    assert "spatial" in md
    assert "drones=haiku" in md
    # rung 1 = 100%, rung 3 = 0%
    assert "100%" in md and "0%" in md
