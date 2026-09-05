import math

import pytest

from agents.flight.track import (TargetEstimator, TrackLog, clamp_ref_alt,
                                 control_ref, intercept_t_go)


def _fed(samples):
    est = TargetEstimator()
    for t, e, n in samples:
        est.update(t, e, n)
    return est


def test_estimator_constant_velocity():
    est = _fed([(0.0, 0.0, 0.0), (0.1, 0.3, -0.2), (0.2, 0.6, -0.4),
                (0.3, 0.9, -0.6)])
    assert est.ready
    assert est.ve == pytest.approx(3.0, abs=0.01)
    assert est.vn == pytest.approx(-2.0, abs=0.01)
    assert est.speed() == pytest.approx(math.hypot(3.0, 2.0), abs=0.02)


def test_estimator_skips_repeated_stamp():
    est = _fed([(0.0, 0.0, 0.0), (0.0, 5.0, 5.0)])   # same stamp: no velocity
    assert not est.ready
    est.update(0.5, 1.0, 0.0)
    assert est.ready
    assert est.ve == pytest.approx(2.0, abs=0.01)


def test_estimator_smooths_velocity_change():
    # v jumps 2 -> 4 m/s east; EMA moves toward 4 without reaching it in one step
    est = _fed([(0.0, 0.0, 0.0), (1.0, 2.0, 0.0), (2.0, 6.0, 0.0)])
    assert 2.0 < est.ve < 4.0


def test_intercept_t_go_stationary_target():
    # 100m away, target still, speed 10 -> 10s
    assert intercept_t_go(100.0, 0.0, 0.0, 0.0, 10.0) == pytest.approx(10.0)


def test_intercept_t_go_satisfies_collision_equation():
    r_e, r_n, v_e, v_n, s = 100.0, 40.0, 0.0, 4.0, 10.0
    t = intercept_t_go(r_e, r_n, v_e, v_n, s)
    assert t is not None and t > 0
    # at t, target displacement from drone equals s*t
    d = math.hypot(r_e + v_e * t, r_n + v_n * t)
    assert d == pytest.approx(s * t, rel=1e-6)


def test_intercept_t_go_unreachable():
    # target receding at the speed cap: never closes
    assert intercept_t_go(100.0, 0.0, 10.0, 0.0, 10.0) is None


def test_control_ref_shadow_is_target_plus_standoff_with_ff():
    est = _fed([(0.0, 50.0, 0.0), (1.0, 53.0, 0.0)])
    ref_e, ref_n, ff_ve, ff_vn = control_ref(
        "shadow", 0.0, 0.0, 53.0, 0.0, est, 12.0, standoff_e=-5.0)
    assert (ref_e, ref_n) == (48.0, 0.0)
    assert ff_ve == pytest.approx(3.0, abs=0.01)
    assert ff_vn == pytest.approx(0.0, abs=0.01)


def test_control_ref_intercept_leads_the_target():
    est = _fed([(0.0, 100.0, 0.0), (1.0, 100.0, 4.0)])   # northbound 4 m/s
    ref_e, ref_n, ff_ve, ff_vn = control_ref(
        "intercept", 0.0, 0.0, 100.0, 4.0, est, 10.0)
    assert ref_n > 4.0                                   # aims AHEAD of the target
    assert math.hypot(ff_ve, ff_vn) == pytest.approx(10.0, rel=1e-6)


def test_control_ref_intercept_fallback_before_estimate():
    est = TargetEstimator()                              # not ready
    ref_e, ref_n, ff_ve, ff_vn = control_ref(
        "intercept", 0.0, 0.0, 60.0, 80.0, est, 12.0)
    assert (ref_e, ref_n) == (60.0, 80.0)                # tail-chase fallback
    assert math.hypot(ff_ve, ff_vn) == pytest.approx(12.0, rel=1e-6)


class _W:
    buildings = [{"name": "b", "x": 0.0, "y": 0.0, "w": 20.0, "d": 20.0, "h": 30.0}]


def test_clamp_ref_alt_raises_inside_footprint():
    assert clamp_ref_alt(_W(), 5.0, -5.0, 12.0) == 33.0


def test_clamp_ref_alt_leaves_clear_refs():
    assert clamp_ref_alt(_W(), 50.0, 0.0, 12.0) == 12.0
    assert clamp_ref_alt(_W(), 0.0, 0.0, 40.0) == 40.0


def test_tracklog_contiguous_dwell_resets():
    log = TrackLog(15.0)
    for t, gap in [(0, 5), (1, 5), (2, 5), (3, 20), (4, 5), (5, 5)]:
        log.sample(float(t), float(gap))
    assert log.best_dwell == pytest.approx(2.0)          # 0..2, reset at t=3
    assert log.min_gap == 5.0
    assert log.mean_gap() == pytest.approx((5 * 5 + 20) / 6)
