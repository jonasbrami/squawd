"""Rangefinder contracts (ICD §2.5): bundle reduction (min, EDGE_MIX, no-return),
impairment clamps, robust_at window/staleness/Hampel, read-time STALE."""
import math
import random

from agents.core.rangefinder import (EDGE_MIX, OUT_OF_RANGE, STALE, VALID,
                                     GzRangeProvider, SimImpairment)


def prov(**kw):
    return GzRangeProvider("test", **kw)


def test_reduce_min_of_bundle():
    p = prov()
    s = p.feed(10.0, [5.0, 5.2, 5.1, 5.0, 5.1, 5.2, 5.0, 5.1, 5.2], max_m=100.0)
    assert s.range_m == 5.0 and s.status == VALID and s.seq == 1


def test_reduce_no_return_is_not_free_space():
    p = prov()
    s = p.feed(10.0, [math.inf] * 9)
    assert s.range_m is None and s.status == OUT_OF_RANGE


def test_reduce_edge_mix_on_high_spread():
    p = prov(edge_spread_m=0.3)
    s = p.feed(10.0, [5.0, 5.0, 5.0, 9.0, 9.0, 9.0, 5.0, 5.0, 5.0])
    assert s.status == EDGE_MIX and s.range_m == 5.0


def test_impairment_dropout_and_eff_max():
    imp = SimImpairment(dropout_p=1.0)
    rng, q, st = imp.apply([5.0], 5.0, None)
    assert rng is None and st != VALID
    imp = SimImpairment(dropout_p=0.0, eff_max_m=10.0)
    rng, q, st = imp.apply([50.0], 50.0, None)
    assert rng is None                              # beyond effective max
    rng, q, st = imp.apply([5.0], 5.0, None)
    assert st == VALID and 4.0 < rng < 6.0 and 0.0 <= q <= 1.0


def test_robust_at_rejects_outlier_cluster():
    p = prov()
    for k in range(6):
        p.feed(10.00 + 0.02 * k, [5.0 + 0.01 * k] * 9)
    p.feed(10.12, [30.0] * 9)                      # wild outlier
    s = p.robust_at(10.12, window_s=0.2)
    assert s is not None and s.range_m < 10.0      # Hampel keeps the cluster


def test_robust_at_staleness_none():
    p = prov()
    p.feed(10.0, [5.0] * 9)
    assert p.robust_at(11.0, window_s=0.2, sync_tolerance_s=0.05) is None


def test_robust_at_empty_window_none():
    p = prov()
    assert p.robust_at(10.0) is None


def test_latest_marks_stale():
    p = prov()
    s = p.feed(10.0, [5.0] * 9, receive_t=0.0)     # ancient receive time
    out = p.latest()
    assert out.status == STALE
