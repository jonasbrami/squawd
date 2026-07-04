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
