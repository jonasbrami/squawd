from bench import sweep


def test_knee_midrange():
    calls = []

    def passes(n):
        calls.append(n)
        return n <= 7

    assert sweep.find_knee(passes, n_cap=32, seed=1) == 7
    # memoized: no N evaluated twice
    assert len(calls) == len(set(calls))


def test_knee_all_pass_returns_cap():
    assert sweep.find_knee(lambda n: True, n_cap=32, seed=1) == 32


def test_knee_none_pass_returns_zero():
    assert sweep.find_knee(lambda n: False, n_cap=32, seed=1) == 0


def test_knee_seed_above_capacity_searches_down():
    assert sweep.find_knee(lambda n: n <= 5, n_cap=32, seed=8) == 5


def test_knee_exact_at_cap():
    assert sweep.find_knee(lambda n: n <= 32, n_cap=32, seed=4) == 32
