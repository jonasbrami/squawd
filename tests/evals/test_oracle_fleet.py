"""Fleet oracle checks: multi-drone coverage, own-fleet separation, drone filters."""
from evals.oracle import grade
from evals.worldstate import DronePose, Snapshot, WorldTrack

META = {"steps": 5, "crashed": False}


def _snap(t, positions, alts=None):
    """positions: dict[drone_id] -> (e, n); alts: dict[drone_id] -> alt (default 10)."""
    alts = alts or {}
    return Snapshot(t, {i: DronePose(e, n, alts.get(i, 10.0), 0.0)
                        for i, (e, n) in positions.items()})


def _track(snaps, objects=None):
    return WorldTrack(snaps, objects or {}, n_drones=2, geofence_m=300.0)


class TestTargetsCovered:
    OBJS = {"t1": (100.0, 0.0), "t2": (-100.0, 0.0)}

    def test_split_coverage_passes(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(10, {0: (100, 0), 1: (-100, 0)})], self.OBJS)
        g = grade(t, [{"check": "targets_covered", "targets": ["t1", "t2"],
                       "tol_m": 12}], META)
        assert g.passed

    def test_one_target_missed_fails_and_names_it(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(10, {0: (100, 0), 1: (50, 0)})], self.OBJS)
        g = grade(t, [{"check": "targets_covered", "targets": ["t1", "t2"],
                       "tol_m": 12}], META)
        assert not g.passed
        assert "t2" in g.checks[0].detail

    def test_single_drone_covering_both_passes(self):
        # coverage is drone-agnostic (budgets punish solo runs, not this check)
        t = _track([_snap(0, {0: (100, 0), 1: (0, 3)}),
                    _snap(10, {0: (-100, 0), 1: (0, 3)})], self.OBJS)
        g = grade(t, [{"check": "targets_covered", "targets": ["t1", "t2"],
                       "tol_m": 12}], META)
        assert g.passed


class TestFleetSeparation:
    def test_close_pass_fails_2d(self):
        t = _track([_snap(0, {0: (0, 0), 1: (100, 0)}),
                    _snap(10, {0: (50, 0), 1: (54, 0)})])
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8}], META)
        assert not g.passed and abs(g.checks[0].value - 4.0) < 1e-9

    def test_altitude_layering_passes_3d(self):
        t = _track([_snap(10, {0: (50, 0), 1: (52, 0)}, alts={0: 10.0, 1: 22.0})])
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8,
                       "use_3d": True}], META)
        assert g.passed

    def test_grace_excuses_spawn_adjacency(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(60, {0: (50, 0), 1: (-50, 0)})])
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8,
                       "grace_s": 30}], META)
        assert g.passed

    def test_single_drone_track_passes_vacuously(self):
        t = WorldTrack([_snap(0, {0: (0, 0)})], {}, n_drones=1, geofence_m=300.0)
        g = grade(t, [{"check": "fleet_separation", "margin_m": 8}], META)
        assert g.passed


class TestDroneFilter:
    OBJS = {"pad_n": (60.0, 80.0), "pad_s": (60.0, -80.0)}

    def test_reached_by_specific_drone(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(10, {0: (60, 80), 1: (0, 3)})], self.OBJS)
        assert grade(t, [{"check": "reached", "target": "pad_n", "tol_m": 10,
                          "drone": 0}], META).passed
        assert not grade(t, [{"check": "reached", "target": "pad_n", "tol_m": 10,
                              "drone": 1}], META).passed

    def test_per_drone_ordering_grades_the_swap(self):
        snaps = [_snap(0, {0: (60, 80), 1: (60, -80)}),
                 _snap(10, {0: (60, 0), 1: (60, 0)}),
                 _snap(20, {0: (60, -80), 1: (60, 80)})]
        t = _track(snaps, self.OBJS)
        assert grade(t, [{"check": "ordering", "sequence": ["pad_n", "pad_s"],
                          "tol_m": 10, "drone": 0}], META).passed
        assert grade(t, [{"check": "ordering", "sequence": ["pad_s", "pad_n"],
                          "tol_m": 10, "drone": 1}], META).passed
        assert not grade(t, [{"check": "ordering", "sequence": ["pad_s", "pad_n"],
                              "tol_m": 10, "drone": 0}], META).passed


class TestSimultaneous:
    OBJS = {"mark_a": (80.0, 80.0), "mark_b": (80.0, -80.0)}
    SPEC = [{"check": "simultaneous",
             "marks": [{"target": "mark_a", "tol_m": 10},
                       {"target": "mark_b", "tol_m": 10}]}]

    def test_both_marks_same_snapshot_distinct_drones_passes(self):
        t = _track([_snap(0, {0: (0, 0), 1: (0, 3)}),
                    _snap(30, {0: (80, 80), 1: (80, -80)})], self.OBJS)
        assert grade(t, self.SPEC, META).passed

    def test_sequential_solo_visits_fail(self):
        t = _track([_snap(10, {0: (80, 80), 1: (0, 3)}),
                    _snap(60, {0: (80, -80), 1: (0, 3)})], self.OBJS)
        assert not grade(t, self.SPEC, META).passed

    def test_one_drone_cannot_satisfy_two_marks(self):
        # marks 12m apart, one drone within tol of both — still needs a partner
        objs = {"mark_a": (80.0, 6.0), "mark_b": (80.0, -6.0)}
        t = _track([_snap(30, {0: (80, 0), 1: (0, 3)})], objs)
        assert not grade(t, [{"check": "simultaneous",
                              "marks": [{"target": "mark_a", "tol_m": 10},
                                        {"target": "mark_b", "tol_m": 10}]}],
                         META).passed


class TestWithinWindow:
    def _movers(self, t, positions, movers):
        return Snapshot(t, {i: DronePose(e, n, 10.0, 0.0)
                            for i, (e, n) in positions.items()},
                        {k: (v[0], v[1], 8.0) for k, v in movers.items()})

    def test_two_intercepts_inside_window_pass(self):
        snaps = [self._movers(40, {0: (50, 100), 1: (70, -100)},
                              {"mov_0": (52, 100), "mov_1": (72, -100)})]
        t = WorldTrack(snaps, {}, n_drones=2, geofence_m=300.0)
        spec = [{"check": "within_window", "window_s": 25,
                 "events": [{"type": "intercept", "mover": "mov_0", "tol_m": 12},
                            {"type": "intercept", "mover": "mov_1", "tol_m": 12}]}]
        assert grade(t, spec, META).passed

    def test_spread_events_fail_window(self):
        snaps = [self._movers(10, {0: (52, 100), 1: (0, 3)},
                              {"mov_0": (50, 100), "mov_1": (300, 300)}),
                 self._movers(80, {0: (0, 0), 1: (72, -100)},
                              {"mov_0": (300, 300), "mov_1": (70, -100)})]
        t = WorldTrack(snaps, {}, n_drones=2, geofence_m=300.0)
        spec = [{"check": "within_window", "window_s": 25,
                 "events": [{"type": "intercept", "mover": "mov_0", "tol_m": 12},
                            {"type": "intercept", "mover": "mov_1", "tol_m": 12}]}]
        g = grade(t, spec, META)
        assert not g.passed and "70.0s apart" in g.checks[0].detail

    def test_missing_event_fails(self):
        t = WorldTrack([self._movers(10, {0: (0, 0)}, {"mov_0": (300, 300)})],
                       {}, n_drones=1, geofence_m=300.0)
        spec = [{"check": "within_window", "window_s": 25,
                 "events": [{"type": "intercept", "mover": "mov_0", "tol_m": 12}]}]
        assert not grade(t, spec, META).passed


class TestFleetSeparationSpawnExemption:
    def test_pad_climb_through_is_exempt_but_midfield_is_not(self):
        """Sequential takeoffs climb through each other's altitude 3m apart on
        the pads — exempt (positional). The same proximity mid-field fails."""
        spec = [{"check": "fleet_separation", "margin_m": 8, "use_3d": True,
                 "exempt_near_spawn_m": 15}]
        pads = [_snap(0, {0: (0, 0), 1: (0, 3)}, alts={0: 20.0, 1: 0.2}),
                _snap(50, {0: (0, 0), 1: (0, 3)}, alts={0: 20.0, 1: 20.0}),
                _snap(90, {0: (60, 40), 1: (60, -40)}, alts={0: 20.0, 1: 40.0})]
        assert grade(_track(pads), spec, META).passed
        midfield = pads[:1] + [_snap(60, {0: (60, 0), 1: (60, 3)},
                                     alts={0: 20.0, 1: 20.0})]
        assert not grade(_track(midfield), spec, META).passed

    def test_returning_to_land_close_together_is_exempt(self):
        # the terminal area works both ways: launch AND recovery legs pass
        # through it 3m apart by construction — never graded there
        spec = [{"check": "fleet_separation", "margin_m": 8,
                 "exempt_near_spawn_m": 15}]
        snaps = [_snap(0, {0: (0, 0), 1: (0, 3)}),
                 _snap(40, {0: (100, 0), 1: (-100, 3)}),
                 _snap(80, {0: (0, 0), 1: (0, 6)})]
        assert grade(_track(snaps), spec, META).passed
