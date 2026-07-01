import math
from evals.worldstate import DronePose, Snapshot, WorldTrack


def _track():
    snaps = [
        Snapshot(t=0.0, poses={0: DronePose(0.0, 0.0, 0.0, 0.0)}),
        Snapshot(t=1.0, poses={0: DronePose(10.0, 0.0, 12.0, 0.0)}),
        Snapshot(t=2.0, poses={0: DronePose(10.0, 5.0, 12.0, 0.0)}),
    ]
    return WorldTrack(snapshots=snaps, objects={"tgt_a": (12.0, 5.0)}, n_drones=1, geofence_m=300.0)


def test_min_dist_to_uses_closest_snapshot():
    assert math.isclose(_track().min_dist_to((12.0, 5.0)), 2.0)


def test_min_dist_to_empty_is_inf():
    t = WorldTrack(snapshots=[], objects={}, n_drones=1, geofence_m=300.0)
    assert t.min_dist_to((0.0, 0.0)) == math.inf


def test_max_dist_from_origin():
    assert math.isclose(_track().max_dist_from_origin(), math.hypot(10.0, 5.0))


def test_positions_flattens_all():
    assert (10.0, 0.0) in _track().positions()
    assert len(_track().positions()) == 3


def test_worldtrack_buildings_defaults_empty():
    from evals.worldstate import WorldTrack
    t = WorldTrack(snapshots=[], objects={}, n_drones=1, geofence_m=300.0)
    assert t.buildings == []


def test_worldtrack_buildings_carried():
    from evals.worldstate import WorldTrack
    b = [{"name": "bldg_9", "x": 43.8, "y": 14.4, "w": 6.5, "d": 6.2}]
    t = WorldTrack(snapshots=[], objects={}, n_drones=1, geofence_m=300.0, buildings=b)
    assert t.buildings[0]["name"] == "bldg_9"
