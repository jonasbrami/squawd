"""M5 load-bearing ④ (ICD §11): strategy-snippet A/B infrastructure — a
snippet activates ONLY on measured lift (design §13 item 6): its Wilson 95%
lower bound must beat the base lane's point success rate, with >= MIN_K scored
cells on BOTH sides."""
import pytest

from evals.strategy_ab import (MIN_K, StrategyError, lift_decision,
                               load_snippet)


def test_load_snippet_reads_and_validates_real_snippet():
    text = load_snippet("intercept-lead")
    assert "track" in text and "intercept" in text


def test_load_snippet_rejects_missing_traversal_and_unknown_tools(tmp_path):
    with pytest.raises(StrategyError):
        load_snippet("no-such-strategy")
    with pytest.raises(StrategyError):
        load_snippet("../secrets")
    (tmp_path / "empty.md").write_text("   \n")
    with pytest.raises(StrategyError, match="empty"):
        load_snippet("empty", tmp_path)
    (tmp_path / "bad.md").write_text("Call `explode(now)` then `track(x)`.")
    with pytest.raises(StrategyError, match="explode"):
        load_snippet("bad", tmp_path)


def _rows(passed, k):
    return [{"passed": i < passed, "infra_fail": False} for i in range(k)]


def test_lift_decision_activates_only_on_measured_lift():
    # snippet 5/5 (Wilson lo ~44%) beats base 2/5 (40%) at CI-low -> ACTIVATE
    d = lift_decision(_rows(2, 5), _rows(5, 5))
    assert d["activate"] is True and "measured lift" in d["reason"]
    assert d["snippet"]["k"] == 5 and d["base"]["rate"] == 0.4


def test_lift_decision_refuses_overlap_regression_and_thin_k():
    # no separation: base 4/5, snippet 3/5 -> stays inactive
    assert lift_decision(_rows(4, 5), _rows(3, 5))["activate"] is False
    # identical lanes -> no lift
    assert lift_decision(_rows(4, 5), _rows(4, 5))["activate"] is False
    # thin samples: 1/1 vs 1/1 proves nothing at MIN_K
    d = lift_decision(_rows(1, 1), _rows(1, 1))
    assert d["activate"] is False and "insufficient data" in d["reason"]
    assert MIN_K >= 3


def test_lift_decision_excludes_infra_rows_from_both_lanes():
    # 5 apparent passes but only ONE of them scored (rest infra_fail) -> k=1
    snip = [{"passed": True, "infra_fail": False}] + \
           [{"passed": True, "infra_fail": True} for _ in range(4)]
    d = lift_decision(_rows(2, 5), snip)
    assert d["activate"] is False and "insufficient data" in d["reason"]
    assert d["snippet"]["k"] == 1
