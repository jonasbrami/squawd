from evals.matrix import expand, done_keys


def test_expand_cardinality():
    cells = expand(["t1", "t2"], [{"drones": "opus"}, {"drones": "haiku"}], k=3)
    assert len(cells) == 2 * 2 * 3


def test_cell_key_stable():
    c = expand(["t1"], [{"drones": "opus"}], k=1)[0]
    assert c.key() == "t1|drones=opus|0"


def test_done_keys_roundtrips_with_cell_key():
    cells = expand(["t1"], [{"drones": "opus"}], k=2)
    rows = [{"task_id": "t1", "assignment": "drones=opus", "repeat": 0}]
    done = done_keys(rows)
    assert cells[0].key() in done
    assert cells[1].key() not in done


def test_shuffled_is_deterministic_and_order_independent_of_done_keys():
    """Cell order is shuffled (repeats interleaved so drift isn't confounded with
    cell identity) but deterministically per seed, and resume keys don't depend
    on order."""
    from evals.matrix import expand, shuffled

    cells = expand(["t1", "t2", "t3"], [{"drones": "opus"}, {"drones": "haiku"}], 3)
    a = shuffled(cells, seed=7)
    b = shuffled(cells, seed=7)
    c = shuffled(cells, seed=8)
    assert a == b                       # same seed, same order
    assert a != c                       # different seed, different order
    assert sorted(x.key() for x in a) == sorted(x.key() for x in cells)  # same set
