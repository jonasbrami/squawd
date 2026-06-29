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
