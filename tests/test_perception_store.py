# tests/test_perception_store.py
from dronebot.perception.provider import PerceptionSnapshot, Obstacle
from dronebot.perception.store import PerceptionStore


def test_store_starts_empty():
    store = PerceptionStore()
    assert store.latest() is None


def test_store_returns_last_snapshot_and_summary():
    store = PerceptionStore()
    snap = PerceptionSnapshot(
        timestamp=1.0,
        jpeg_frame=b"\xff\xd8fake",
        obstacles=[Obstacle(direction="ahead", distance_m=4.0),
                   Obstacle(direction="left", distance_m=9.0)],
    )
    store.update(snap)
    assert store.latest() is snap
    # nearest obstacle leads the summary
    assert "4" in store.surroundings_summary()
    assert "ahead" in store.surroundings_summary()


def test_summary_when_clear():
    store = PerceptionStore()
    store.update(PerceptionSnapshot(timestamp=1.0, jpeg_frame=None, obstacles=[]))
    assert "clear" in store.surroundings_summary().lower()
