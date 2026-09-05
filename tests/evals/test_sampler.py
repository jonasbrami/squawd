from evals.sampler import snapshot_now


class FakeWorld:
    def __init__(self, states):
        self._states = states  # dict[i] -> tuple|None

    def drone_state(self, bridge, i):
        return self._states.get(i)


def test_snapshot_skips_invalid_fix():
    world = FakeWorld({0: (10.0, 20.0, 12.0, 0.0)})
    snap = snapshot_now(world, bridge=None, t=3.0)
    assert snap.t == 3.0
    assert set(snap.poses) == {0}
    assert snap.poses[0].e == 10.0 and snap.poses[0].alt == 12.0


def test_sampler_captures_buildings_from_world():
    from evals.sampler import Sampler

    class WorldWithBuildings:
        buildings = [{"name": "b0", "x": 1.0, "y": 2.0, "w": 3.0, "d": 4.0}]

        def drone_state(self, bridge, i):
            return None

    s = Sampler(WorldWithBuildings(), bridge=None, objects={}, geofence_m=300.0)
    assert s.track().buildings == [{"name": "b0", "x": 1.0, "y": 2.0, "w": 3.0, "d": 4.0}]


class FakeGzPoses:
    def __init__(self, poses):
        self._poses = poses

    def poses(self):
        return dict(self._poses)


def test_snapshot_captures_movers_same_tick():
    world = FakeWorld({0: (10.0, 20.0, 12.0, 0.0)})
    gz = FakeGzPoses({"mov_0": (55.0, -10.0, 8.0)})
    snap = snapshot_now(world, bridge=None, t=1.0, gzposes=gz)
    assert snap.movers == {"mov_0": (55.0, -10.0, 8.0)}
    assert snap.poses[0].e == 10.0


def test_snapshot_movers_empty_without_reader():
    world = FakeWorld({0: (0.0, 0.0, 5.0, 0.0)})
    snap = snapshot_now(world, bridge=None, t=0.0)
    assert snap.movers == {}


def test_sampler_buildings_empty_when_world_has_none():
    from evals.sampler import Sampler

    class BareWorld:
        def drone_state(self, bridge, i):
            return None

    s = Sampler(BareWorld(), bridge=None, objects={}, geofence_m=300.0)
    assert s.track().buildings == []
