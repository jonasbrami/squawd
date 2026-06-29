from evals.reset import home_xy, check_home


class FakeWorld:
    spawn_x = 0.0
    spawn_spacing = 3.0

    def __init__(self, xy):
        self._xy = xy  # dict[i] -> (e, n, alt)

    def world_xy(self, bridge, i):
        return self._xy.get(i)


def test_home_xy():
    assert home_xy(FakeWorld({}), 2) == (0.0, 6.0)


def test_check_home_pass():
    w = FakeWorld({0: (0.2, 0.0, 0.1), 1: (0.0, 3.1, 0.1)})
    assert check_home(w, None, 2, tol_m=5.0).ok


def test_check_home_fail_names_drone():
    w = FakeWorld({0: (0.0, 0.0, 0.1), 1: (50.0, 3.0, 0.1)})
    r = check_home(w, None, 2, tol_m=5.0)
    assert not r.ok and "drone_1" in r.reason


def test_check_home_fail_on_missing_fix():
    r = check_home(FakeWorld({0: None}), None, 1, tol_m=5.0)
    assert not r.ok
