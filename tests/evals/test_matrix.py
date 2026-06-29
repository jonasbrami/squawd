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
