from evals.worldstate import DronePose, Snapshot, WorldTrack
from evals.oracle import grade


def _track(reach=True):
    e = 118.0 if reach else 0.0
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 0.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(e, -40.0, 12.0, 0.0)}),
    ]
    return WorldTrack(snaps, {"tgt_a": (120.0, -40.0)}, n_drones=1, geofence_m=300.0)


META_OK = {"steps": 5, "crashed": False}


def test_reached_pass():
    g = grade(_track(True), [{"check": "reached", "target": "tgt_a", "tol_m": 15}], META_OK)
    assert g.passed


def test_reached_fail():
    g = grade(_track(False), [{"check": "reached", "target": "tgt_a", "tol_m": 15}], META_OK)
    assert not g.passed


def test_alive_fails_on_crash():
    g = grade(_track(True), [{"check": "alive"}], {"steps": 5, "crashed": True})
    assert not g.passed


def test_step_budget():
    spec = [{"check": "within_step_budget", "max_steps": 4}]
    assert not grade(_track(True), spec, {"steps": 5, "crashed": False}).passed
    assert grade(_track(True), spec, {"steps": 4, "crashed": False}).passed


def test_coverage_counts_overflown_cells():
    # Drone visits two cell centers of ne_quadrant; min_pct low enough to pass.
    snaps = [Snapshot(float(i), {0: DronePose(e, n, 12.0, 0.0)})
             for i, (e, n) in enumerate([(10, 10), (30, 10)])]
    t = WorldTrack(snaps, {}, n_drones=1, geofence_m=300.0)
    spec = [{"check": "coverage", "area": "ne_quadrant", "min_pct": 1, "radius_m": 15, "cell_m": 20}]
    assert grade(t, spec, META_OK).passed


def test_all_checks_must_pass():
    spec = [{"check": "reached", "target": "tgt_a", "tol_m": 15},
            {"check": "within_step_budget", "max_steps": 1}]
    assert not grade(_track(True), spec, {"steps": 5, "crashed": False}).passed


def _route_track(reach_c=True):
    # Drone visits a(10,0) at t=1, b(10,10) at t=2, then c(0,10) at t=3 (if reach_c).
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(10.0, 0.0, 12.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(10.0, 10.0, 12.0, 0.0)}),
        Snapshot(3.0, {0: DronePose(0.0 if reach_c else 40.0, 10.0, 12.0, 0.0)}),
    ]
    objs = {"a": (10.0, 0.0), "b": (10.0, 10.0), "c": (0.0, 10.0)}
    return WorldTrack(snaps, objs, n_drones=1, geofence_m=300.0)


def test_visited_all_pass_and_fail():
    ok = {"steps": 5, "crashed": False}
    spec = [{"check": "visited_all", "targets": ["a", "b", "c"], "tol_m": 3}]
    assert grade(_route_track(True), spec, ok).passed
    assert not grade(_route_track(False), spec, ok).passed  # c missed


def test_ordering_pass_when_in_sequence():
    ok = {"steps": 5, "crashed": False}
    spec = [{"check": "ordering", "sequence": ["a", "b", "c"], "tol_m": 3}]
    assert grade(_route_track(True), spec, ok).passed


def test_ordering_fails_when_out_of_sequence():
    ok = {"steps": 5, "crashed": False}
    # require c BEFORE a — the track reaches a first, so ordering must fail
    spec = [{"check": "ordering", "sequence": ["c", "a", "b"], "tol_m": 3}]
    assert not grade(_route_track(True), spec, ok).passed


def test_altitude_band_at_target():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    # Closest approach to tgt (100,0) is at t=2 where alt=20 -> in [18,22].
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 5.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(50.0, 0.0, 20.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(100.0, 0.0, 20.0, 0.0)}),
    ]
    t = WorldTrack(snaps, {"tgt": (100.0, 0.0)}, n_drones=1, geofence_m=300.0)
    ok = {"steps": 5, "crashed": False}
    assert grade(t, [{"check": "altitude", "target": "tgt", "min_m": 18, "max_m": 22}], ok).passed
    assert not grade(t, [{"check": "altitude", "target": "tgt", "min_m": 25, "max_m": 30}], ok).passed


def test_dwell_holds_long_enough():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    # Within 3m of tgt(0,0) from t=1..t=4 -> a 3s continuous hold.
    snaps = [
        Snapshot(0.0, {0: DronePose(50.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(1.0, 0.0, 12.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(0.5, 0.0, 12.0, 0.0)}),
        Snapshot(3.0, {0: DronePose(1.0, 0.0, 12.0, 0.0)}),
        Snapshot(4.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
    ]
    t = WorldTrack(snaps, {"tgt": (0.0, 0.0)}, n_drones=1, geofence_m=300.0)
    ok = {"steps": 5, "crashed": False}
    assert grade(t, [{"check": "dwell", "target": "tgt", "tol_m": 3, "hold_s": 2.5}], ok).passed
    assert not grade(t, [{"check": "dwell", "target": "tgt", "tol_m": 3, "hold_s": 5}], ok).passed
