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


def test_ordering_pass_when_final_waypoint_is_home():
    """A return-to-home route ends at spawn. The drone is trivially within tol of the
    home waypoint at t=0 too, but 'visited in order' means each waypoint is reached in a
    temporal CHAIN — d after c — so the t=0 presence at home must not break monotonicity."""
    ok = {"steps": 5, "crashed": False}
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),   # at home == d
        Snapshot(1.0, {0: DronePose(10.0, 0.0, 12.0, 0.0)}),  # a
        Snapshot(2.0, {0: DronePose(10.0, 10.0, 12.0, 0.0)}), # b
        Snapshot(3.0, {0: DronePose(0.0, 10.0, 12.0, 0.0)}),  # c
        Snapshot(4.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),   # back to d (home)
    ]
    objs = {"a": (10.0, 0.0), "b": (10.0, 10.0), "c": (0.0, 10.0), "d": (0.0, 0.0)}
    track = WorldTrack(snaps, objs, n_drones=1, geofence_m=300.0)
    spec = [{"check": "ordering", "sequence": ["a", "b", "c", "d"], "tol_m": 3}]
    assert grade(track, spec, ok).passed


def test_ordering_fails_when_intermediate_waypoint_skipped():
    """Greedy chaining must still reject a route that skips an intermediate waypoint —
    the genuine fire-and-forget failure mode (only the last point is ever reached)."""
    ok = {"steps": 5, "crashed": False}
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(0.0, 10.0, 12.0, 0.0)}),  # jumps straight to c
    ]
    objs = {"a": (10.0, 0.0), "b": (10.0, 10.0), "c": (0.0, 10.0)}
    track = WorldTrack(snaps, objs, n_drones=1, geofence_m=300.0)
    spec = [{"check": "ordering", "sequence": ["a", "b", "c"], "tol_m": 3}]
    assert not grade(track, spec, ok).passed


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


def test_clearance_passes_when_far_and_no_buildings():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    snaps = [Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)})]
    # No buildings -> inf clearance -> passes.
    assert grade(WorldTrack(snaps, {}, 1, 300.0), [{"check": "clearance", "margin_m": 5}], ok).passed


def test_clearance_fails_on_near_miss():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    # Building footprint centered (10,0), half-extents 3 x 3 -> east edge at e=13.
    # Drone passes at e=13.5 -> clearance ~0.5m < margin 5 -> fail (true near-miss, outside the box).
    b = [{"name": "b0", "x": 10.0, "y": 0.0, "w": 6.0, "d": 6.0}]
    snaps = [Snapshot(0.0, {0: DronePose(13.5, 0.0, 12.0, 0.0)})]
    t = WorldTrack(snaps, {}, 1, 300.0, buildings=b)
    assert not grade(t, [{"check": "clearance", "margin_m": 5}], ok).passed


def test_clearance_passes_when_routed_around():
    from evals.worldstate import DronePose, Snapshot, WorldTrack
    from evals.oracle import grade
    ok = {"steps": 5, "crashed": False}
    b = [{"name": "b0", "x": 10.0, "y": 0.0, "w": 6.0, "d": 6.0}]
    # Drone passes at n=20 -> far from the box -> clearance ~17m >= 5 -> pass.
    snaps = [Snapshot(0.0, {0: DronePose(10.0, 20.0, 12.0, 0.0)})]
    t = WorldTrack(snaps, {}, 1, 300.0, buildings=b)
    assert grade(t, [{"check": "clearance", "margin_m": 5}], ok).passed


# ---- discrimination-ladder checks: not_reached / avoid_area / path_length / alt_ceiling ----

def _ok_meta():
    return {"steps": 5, "crashed": False}


def test_not_reached_passes_when_kept_clear_and_fails_on_approach():
    # Track visits (10,0); decoy at (100,100) stays clear, decoy at (12,0) is approached.
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(10.0, 0.0, 12.0, 0.0)}),
    ]
    objs = {"far_decoy": (100.0, 100.0), "near_decoy": (12.0, 0.0)}
    t = WorldTrack(snaps, objs, n_drones=1, geofence_m=300.0)
    assert grade(t, [{"check": "not_reached", "target": "far_decoy", "tol_m": 25}], _ok_meta()).passed
    assert not grade(t, [{"check": "not_reached", "target": "near_decoy", "tol_m": 25}], _ok_meta()).passed


def test_avoid_area_fails_on_incursion_and_passes_outside():
    # ne_quadrant is (0,0)..(200,200); one sample inside it must fail the check.
    inside = [Snapshot(0.0, {0: DronePose(-5.0, -5.0, 12.0, 0.0)}),
              Snapshot(1.0, {0: DronePose(50.0, 50.0, 12.0, 0.0)})]
    outside = [Snapshot(0.0, {0: DronePose(-5.0, -5.0, 12.0, 0.0)}),
               Snapshot(1.0, {0: DronePose(-50.0, -50.0, 12.0, 0.0)})]
    t_in = WorldTrack(inside, {}, n_drones=1, geofence_m=300.0)
    t_out = WorldTrack(outside, {}, n_drones=1, geofence_m=300.0)
    spec = [{"check": "avoid_area", "area": "ne_quadrant"}]
    assert not grade(t_in, spec, _ok_meta()).passed
    assert grade(t_out, spec, _ok_meta()).passed


def test_avoid_area_grace_skips_spawn_samples():
    # Spawn is inside the area; grace_s excuses the early samples only.
    snaps = [Snapshot(0.0, {0: DronePose(50.0, 50.0, 0.0, 0.0)}),
             Snapshot(5.0, {0: DronePose(-50.0, -50.0, 12.0, 0.0)})]
    t = WorldTrack(snaps, {}, n_drones=1, geofence_m=300.0)
    spec = [{"check": "avoid_area", "area": "ne_quadrant", "grace_s": 2.0}]
    assert grade(t, spec, _ok_meta()).passed


def test_path_length_sums_2d_legs():
    # 30m east then 40m north = 70m flown; max 60 fails, max 80 passes.
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 12.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(30.0, 0.0, 12.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(30.0, 40.0, 12.0, 0.0)}),
    ]
    t = WorldTrack(snaps, {}, n_drones=1, geofence_m=300.0)
    assert grade(t, [{"check": "path_length", "max_m": 80}], _ok_meta()).passed
    assert not grade(t, [{"check": "path_length", "max_m": 60}], _ok_meta()).passed


def test_alt_ceiling_binds_whole_flight():
    # Climbs to 18m mid-flight even though it ends at 10m -> ceiling 15 fails.
    snaps = [
        Snapshot(0.0, {0: DronePose(0.0, 0.0, 10.0, 0.0)}),
        Snapshot(1.0, {0: DronePose(10.0, 0.0, 18.0, 0.0)}),
        Snapshot(2.0, {0: DronePose(20.0, 0.0, 10.0, 0.0)}),
    ]
    t = WorldTrack(snaps, {}, n_drones=1, geofence_m=300.0)
    assert not grade(t, [{"check": "alt_ceiling", "max_m": 15}], _ok_meta()).passed
    assert grade(t, [{"check": "alt_ceiling", "max_m": 20}], _ok_meta()).passed
