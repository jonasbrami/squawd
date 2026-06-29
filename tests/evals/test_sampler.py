from evals.sampler import snapshot_now


class FakeWorld:
    def __init__(self, states):
        self._states = states  # dict[i] -> tuple|None

    def drone_state(self, bridge, i):
        return self._states.get(i)


def test_snapshot_skips_invalid_fix():
    world = FakeWorld({0: (10.0, 20.0, 12.0, 0.0), 1: None})
    snap = snapshot_now(world, bridge=None, n_drones=2, t=3.0)
    assert snap.t == 3.0
    assert set(snap.poses) == {0}
    assert snap.poses[0].e == 10.0 and snap.poses[0].alt == 12.0
