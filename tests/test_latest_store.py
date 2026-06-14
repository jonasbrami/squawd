import threading
from agents.common.latest_store import LatestStore


def test_get_returns_none_before_any_set():
    store = LatestStore()
    assert store.get("/topic") is None


def test_set_then_get_returns_latest():
    store = LatestStore()
    store.set("/pos", {"z": -1.0})
    store.set("/pos", {"z": -2.0})
    assert store.get("/pos") == {"z": -2.0}


def test_independent_topics():
    store = LatestStore()
    store.set("/a", 1)
    store.set("/b", 2)
    assert store.get("/a") == 1 and store.get("/b") == 2


def test_concurrent_writes_do_not_crash_and_last_wins():
    store = LatestStore()

    def writer(n):
        for i in range(1000):
            store.set("/t", (n, i))

    threads = [threading.Thread(target=writer, args=(n,)) for n in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    val = store.get("/t")
    assert isinstance(val, tuple) and val[1] == 999
