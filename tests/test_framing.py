# tests/test_framing.py
from dronebot.web.framing import mjpeg_part, telemetry_frame
from dronebot.control.state import StateStore
from dronebot.control.geo import GeoPoint
from dronebot.perception.store import PerceptionStore
from dronebot.perception.provider import PerceptionSnapshot, Obstacle


def test_mjpeg_part_has_boundary_and_payload():
    part = mjpeg_part(b"\xff\xd8jpeg")
    assert part.startswith(b"--frame\r\n")
    assert b"Content-Type: image/jpeg" in part
    assert b"Content-Length: 6" in part
    assert part.endswith(b"\xff\xd8jpeg\r\n")


def test_telemetry_frame_populated():
    state = StateStore()
    state.set_connection(True)
    state.set_armed(True)
    state.set_in_air(True)
    state.set_flight_mode("HOLD")
    state.set_battery(0.8)
    state.set_home(GeoPoint(47.0, 8.0, 500.0))
    state.set_position(GeoPoint(47.0, 8.0, 510.0))
    perception = PerceptionStore()
    perception.update(PerceptionSnapshot(timestamp=1.0, jpeg_frame=None,
                                         obstacles=[Obstacle("ahead", 4.0)]))
    frame = telemetry_frame(state, perception)
    assert frame["connected"] and frame["armed"] and frame["in_air"]
    assert frame["flight_mode"] == "HOLD"
    assert frame["battery"] == 0.8
    assert frame["rel_alt"] == 10.0
    assert frame["position"]["lat"] == 47.0
    assert "4" in frame["surroundings"]


def test_telemetry_frame_no_fix():
    frame = telemetry_frame(StateStore(), PerceptionStore())
    assert frame["position"] is None
    assert frame["rel_alt"] is None
    assert frame["connected"] is False
