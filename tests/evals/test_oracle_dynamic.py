"""Dynamic oracle predicates: drone-vs-mover geometry within snapshots."""
from evals.oracle import grade
from evals.worldstate import DronePose, Snapshot, WorldTrack

META = {"steps": 5, "crashed": False}


def _snap(t, drone_xy, mover_xy=None, name="mov_0"):
    movers = {name: (mover_xy[0], mover_xy[1], 8.0)} if mover_xy else {}
    return Snapshot(t, {0: DronePose(drone_xy[0], drone_xy[1], 10.0, 0.0)}, movers)


def _track(snaps, objects=None):
    return WorldTrack(snaps, objects or {}, geofence_m=300.0)


class TestIntercept:
    def test_passes_when_separation_dips_below_tol(self):
        t = _track([_snap(0, (0, 0), (50, 0)),
                    _snap(1, (42, 0), (46, 0)),
                    _snap(2, (80, 0), (40, 0))])
        g = grade(t, [{"check": "intercept", "mover": "mov_0", "tol_m": 10}], META)
        assert g.passed and abs(g.checks[0].value - 4.0) < 1e-9

    def test_fails_when_never_close(self):
        t = _track([_snap(0, (0, 0), (100, 100)), _snap(1, (10, 0), (90, 100))])
        g = grade(t, [{"check": "intercept", "mover": "mov_0", "tol_m": 10}], META)
        assert not g.passed

    def test_deadline_rejects_contact_after_zone_entry(self):
        # contact happens when the mover is already within 50m of the tower
        objs = {"tower": (0.0, 0.0)}
        t = _track([_snap(0, (0, 0), (200, 0)),
                    _snap(1, (45, 0), (40, 0))], objs)   # mover 40m from tower
        spec = [{"check": "intercept", "mover": "mov_0", "tol_m": 10,
                 "zone_target": "tower", "zone_radius_m": 50}]
        assert not grade(t, spec, META).passed

    def test_deadline_accepts_contact_outside_zone(self):
        objs = {"tower": (0.0, 0.0)}
        t = _track([_snap(0, (0, 0), (200, 0)),
                    _snap(1, (95, 0), (100, 0))], objs)  # mover 100m out
        spec = [{"check": "intercept", "mover": "mov_0", "tol_m": 10,
                 "zone_target": "tower", "zone_radius_m": 50}]
        assert grade(t, spec, META).passed

    def test_missing_mover_samples_are_skipped(self):
        t = _track([_snap(0, (0, 0)), _snap(1, (5, 0), (8, 0))])
        g = grade(t, [{"check": "intercept", "mover": "mov_0", "tol_m": 10}], META)
        assert g.passed


class TestDwellMoving:
    def test_contiguous_hold_passes(self):
        snaps = [_snap(float(t), (t * 2.0, 0), (t * 2.0 + 5, 0)) for t in range(12)]
        g = grade(_track(snaps),
                  [{"check": "dwell_moving", "mover": "mov_0", "tol_m": 10, "hold_s": 10}],
                  META)
        assert g.passed and g.checks[0].value == 11.0

    def test_single_dropout_does_not_break_the_run(self):
        snaps = [_snap(float(t), (0, 0), (5, 0)) for t in range(6)]
        snaps[3] = _snap(3.0, (0, 0), (50, 0))     # one sample outside
        g = grade(_track(snaps),
                  [{"check": "dwell_moving", "mover": "mov_0", "tol_m": 10, "hold_s": 5}],
                  META)
        assert g.passed

    def test_two_consecutive_dropouts_reset_the_run(self):
        snaps = [_snap(float(t), (0, 0), (5, 0)) for t in range(12)]
        snaps[5] = _snap(5.0, (0, 0), (50, 0))
        snaps[6] = _snap(6.0, (0, 0), (50, 0))
        g = grade(_track(snaps),
                  [{"check": "dwell_moving", "mover": "mov_0", "tol_m": 10, "hold_s": 8}],
                  META)
        assert not g.passed      # longest contiguous run is 5s, not 11s

    def test_intermittent_proximity_fails(self):
        snaps = [_snap(float(t), (0, 0), (5, 0) if t % 3 == 0 else (60, 0))
                 for t in range(12)]
        g = grade(_track(snaps),
                  [{"check": "dwell_moving", "mover": "mov_0", "tol_m": 10, "hold_s": 6}],
                  META)
        assert not g.passed


class TestAvoidMoving:
    def test_clean_track_passes(self):
        t = _track([_snap(0, (0, 0), (100, 0)), _snap(1, (10, 0), (90, 0))])
        g = grade(t, [{"check": "avoid_moving", "mover": "mov_0", "margin_m": 20}], META)
        assert g.passed

    def test_single_violation_fails(self):
        t = _track([_snap(0, (0, 0), (100, 0)), _snap(1, (85, 0), (90, 0))])
        g = grade(t, [{"check": "avoid_moving", "mover": "mov_0", "margin_m": 20}], META)
        assert not g.passed and g.checks[0].value == 5.0

    def test_grace_excuses_early_samples(self):
        t = _track([_snap(0, (0, 0), (5, 0)), _snap(20, (0, 0), (200, 0))])
        g = grade(t, [{"check": "avoid_moving", "mover": "mov_0", "margin_m": 20,
                       "grace_s": 5}], META)
        assert g.passed


class TestEscort:
    def test_high_fraction_after_join_passes(self):
        # 5 samples in transit, then 20 joined with one brief 2-sample gap
        snaps = [_snap(float(t), (t * 10.0, 200), (0, 0)) for t in range(5)]
        snaps += [_snap(5.0 + t, (t, 0), (t + 5, 0)) for t in range(20)]
        snaps[10] = _snap(10.0, (100, 200), (10, 0))
        g = grade(_track(snaps),
                  [{"check": "escort", "mover": "mov_0", "tol_m": 15,
                    "min_fraction": 0.8, "max_gap_s": 10}], META)
        assert g.passed

    def test_never_joining_fails(self):
        snaps = [_snap(float(t), (200, 200), (0, 0)) for t in range(5)]
        g = grade(_track(snaps),
                  [{"check": "escort", "mover": "mov_0", "tol_m": 15,
                    "min_fraction": 0.8, "max_gap_s": 10}], META)
        assert not g.passed

    def test_long_gap_fails_even_with_good_fraction(self):
        joined = [_snap(float(t), (t, 0), (t + 5, 0)) for t in range(40)]
        for k in range(10, 22):                     # 12s hole in the middle
            joined[k] = _snap(float(k), (500, 500), (k + 5, 0))
        g = grade(_track(joined),
                  [{"check": "escort", "mover": "mov_0", "tol_m": 15,
                    "min_fraction": 0.5, "max_gap_s": 10}], META)
        assert not g.passed
