# tests/test_state.py
from dronebot.control.state import StateStore
from dronebot.control.geo import GeoPoint
from dronebot.control.safety import DroneSnapshot


def test_initial_state_is_disconnected():
    store = StateStore()
    snap = store.snapshot()
    assert isinstance(snap, DroneSnapshot)
    assert snap.is_connected is False
    assert snap.has_position is False


def test_updates_are_reflected_in_snapshot():
    store = StateStore()
    store.set_connection(True)
    store.set_armed(True)
    store.set_in_air(True)
    store.set_flight_mode("HOLD")
    store.set_home(GeoPoint(1.0, 2.0, 100.0))
    store.set_position(GeoPoint(1.0, 2.0, 110.0))
    snap = store.snapshot()
    assert snap.is_connected and snap.is_armed and snap.in_air
    assert snap.flight_mode == "HOLD"
    assert snap.has_position is True
    assert snap.position.absolute_altitude_m == 110.0


def test_battery_and_link_health_tracked():
    store = StateStore()
    store.set_battery(0.87)
    store.mark_telemetry_seen(timestamp=123.0)
    assert store.battery_remaining == 0.87
    assert store.last_telemetry_ts == 123.0
