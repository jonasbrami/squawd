"""Cockpit server endpoint tests (M4, ICD §8.2) against the REAL handlers via
server.build_app + fakes — no ROS, no sim.

Covers the three load-bearing M4 contracts end to end:
- /state surfaces detector-down as the SENSING DEGRADED banner (§3.7);
- POST /estop publishes /pilot/estop whose payload drives the pilot's REAL
  estop arbiter (agents.pilot.estop, reused unmodified) to cancel an in-flight
  tool task and hold the drone;
- /command -> /pilot/user_input, /chat cursor semantics, /ws_detections
  verbatim relay.
"""
import asyncio
import json
from types import SimpleNamespace

from starlette.testclient import TestClient

from agents.observatory import overlay
from agents.observatory.server import build_app
from agents.observatory.video import VideoHub
from agents.pilot.estop import ActiveToolRegistry, estop_supervisor


class FakeString:
    def __init__(self):
        self.data = ""


class FakeBridge:
    """RosBridge-shaped fake: records subscriptions/publishes, serves canned
    latest() values, and can feed subscription callbacks like the rclpy thread."""

    def __init__(self):
        self.published = []          # (topic, data, qos)
        self._cbs = {}
        self._latest = {}

    def subscribe(self, topic, msg_type, qos=None, callback=None):
        if callback is not None:
            self._cbs[topic] = callback

    def publish(self, topic, msg_type, msg, qos=None):
        self.published.append((topic, msg.data, qos))

    def latest(self, topic):
        return self._latest.get(topic)

    def set_latest(self, topic, data):
        m = SimpleNamespace(data=data)
        self._latest[topic] = m
        return m

    def feed(self, topic, text):
        self._cbs[topic](SimpleNamespace(data=text))


class FakeCameras:
    def __init__(self, frame=None):
        self._f = frame

    def snapshot(self, i):
        return self._f


SNAP = json.dumps({
    "schema_version": 1, "sim_stamp": 42.0, "seq": 7,
    "frame_w": 640, "frame_h": 360,
    "dets": [{"cls": "target", "conf": 0.8, "xyxy": [10, 20, 60, 80], "tid": 3}],
    "contacts": [{"name": "vis_target_0", "cls": "target", "conf": 0.8,
                  "e": 3.0, "n": 12.0, "z": 5.0, "position_src": "measured",
                  "ve": 0.0, "vn": 0.0, "bearing_deg": 14.0,
                  "elevation_deg": -20.0, "range_m": 12.4, "range_src": "tof",
                  "range_conf": 0.9, "health": "MEASURED", "age_s": 0.1}],
    "detector": {"healthy": True, "latency_ms": 31.2},
    "beam": {"status": "LOCKED", "target": "vis_target_0", "range_m": 12.4},
    "track": {"state": "WORLD_TRACKED", "target": "vis_target_0", "gap_m": 3.1},
})


def _app(bridge, cams=None):
    cams = cams or FakeCameras()
    hub = VideoHub(cams, 0, maxpx=320, interval=0.01)
    return build_app(bridge, cams, hub,
                     msg_type=FakeString, cmd_qos="CMD", chat_qos="CHAT")


# ---- /state ----

def test_state_assembles_pose_attitude_battery_and_perception_health():
    br = FakeBridge()
    br._latest["/px4_0/fmu/out/vehicle_local_position"] = SimpleNamespace(
        x=10.0, y=-5.0, z=-12.4, vx=3.0, vy=4.0, vz=-1.0, heading=0.0)
    br._latest["/px4_0/fmu/out/vehicle_attitude"] = SimpleNamespace(
        q=[1.0, 0.0, 0.0, 0.0])
    br._latest["/px4_0/fmu/out/vehicle_status"] = SimpleNamespace(
        arming_state=2, nav_state=14)
    br._latest["/px4_0/fmu/out/battery_status"] = SimpleNamespace(
        remaining=0.78, voltage_v=15.62, warning=0)
    br.set_latest("/pilot/detections", SNAP)
    frame = SimpleNamespace(seq=57, sim_stamp=42.1)
    with TestClient(_app(br, FakeCameras(frame))) as client:
        d = client.get("/state").json()
    assert d["alt"] == 12.4 and d["speed"] == 5.0 and d["mode"] == "OFFBOARD"
    assert (d["roll"], d["pitch"], d["yaw"]) == (0.0, 0.0, 0.0)
    assert d["batt_pct"] == 78
    assert d["detector"]["healthy"] is True
    assert d["beam"]["status"] == "LOCKED"
    assert d["track"]["state"] == "WORLD_TRACKED"
    assert d["contacts"][0]["range_m"] == 12.4
    assert d["cam_seq"] == 57 and d["cam_stamp"] == 42.1
    assert d["banner"] is None


def test_state_surfaces_detector_down_as_degraded_banner():
    """M4 gate: killed detector -> the cockpit says SENSING DEGRADED."""
    br = FakeBridge()
    dead = json.loads(SNAP)
    dead["detector"] = {"healthy": False, "latency_ms": 0.0}
    br.set_latest("/pilot/detections", json.dumps(dead))
    with TestClient(_app(br)) as client:
        d = client.get("/state").json()
    assert d["detector"]["healthy"] is False
    assert d["banner"] == overlay.SENSING_DEGRADED


def test_state_is_null_safe_with_no_messages():
    with TestClient(_app(FakeBridge())) as client:
        d = client.get("/state").json()
    assert d["mode"] is None and d["detector"] is None and d["banner"] is None
    assert d["cam"] is False


def test_state_tolerates_unparseable_detections_payload():
    br = FakeBridge()
    br.set_latest("/pilot/detections", "{not json")
    with TestClient(_app(br)) as client:
        d = client.get("/state").json()
    assert d["detector"] is None and d["banner"] is None


# ---- /command + /chat ----

def test_command_publishes_user_input_and_echoes_to_chat():
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        r = client.post("/command", json={"text": "take off to 12m"})
        assert r.json() == {"ok": True}
        assert br.published == [("/pilot/user_input", "take off to 12m", "CMD")]
        j = client.get("/chat?since=0").json()
    assert j["lines"] == ["you: take off to 12m"]
    assert j["next"] == 1


def test_command_with_empty_text_publishes_nothing():
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        client.post("/command", json={"text": "   "})
    assert br.published == []


def test_chat_cursor_returns_only_new_lines():
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        br.feed("/pilot/chat", "pilot: airborne at 12m")
        br.feed("/pilot/chat", "pilot: tracking vis_target_0")
        j1 = client.get("/chat?since=0").json()
        assert j1["lines"] == ["pilot: airborne at 12m", "pilot: tracking vis_target_0"]
        br.feed("/pilot/chat", "estop: drone_0 HOLDING (estop)")
        j2 = client.get(f"/chat?since={j1['next']}").json()
        assert j2["lines"] == ["estop: drone_0 HOLDING (estop)"]


# ---- /estop ----

def test_estop_publishes_action_and_rejects_bad_action():
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        r = client.post("/estop", json={"action": "land"})
        assert r.json() == {"ok": True, "action": "land"}
        bad = client.post("/estop", json={"action": "orbit"})
        assert bad.status_code == 400
        client.post("/estop", json={})              # defaults to hold
    assert br.published == [("/pilot/estop", "land", "CMD"),
                            ("/pilot/estop", "hold", "CMD")]


def test_estop_button_cancels_in_flight_tool_via_real_arbiter():
    """M4 gate (cockpit side): the payload POST /estop puts on /pilot/estop
    drives the M1 arbiter to cancel an in-flight tool task and hold."""
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        client.post("/estop", json={"action": "hold"})
    topic, payload, _qos = br.published[-1]
    assert topic == "/pilot/estop"

    class FakeOps:
        def __init__(self):
            self.calls = []

        async def emergency_hold(self):
            self.calls.append("emergency_hold")
            return "drone_0 HOLDING (estop)"

    async def pilot_side():
        """The pilot process: real registry + real supervisor over its bridge."""
        ops, pilot_br, reg = FakeOps(), FakeBridge(), ActiveToolRegistry()
        started = asyncio.Event()

        async def long_track_tool():
            reg.register(asyncio.current_task())
            started.set()
            try:
                await asyncio.sleep(60)                 # a 120 s track, in flight
            except asyncio.CancelledError:
                return "ESTOPPED"
            finally:
                reg.clear()

        tool_task = asyncio.create_task(long_track_tool())
        sup = asyncio.create_task(
            estop_supervisor(pilot_br, reg, ops, msg_type=FakeString,
                             cmd_qos=object(), chat_qos=object()))
        for _ in range(200):
            if "/pilot/estop" in pilot_br._cbs and started.is_set():
                break
            await asyncio.sleep(0.01)
        pilot_br.feed("/pilot/estop", payload)          # the cockpit's own bytes
        await asyncio.sleep(0.3)
        sup.cancel()
        try:
            await sup
        except asyncio.CancelledError:
            pass
        return await tool_task, ops

    (result, ops) = asyncio.run(pilot_side())
    assert result == "ESTOPPED"                       # tool task cancelled
    assert ops.calls == ["emergency_hold"]            # drone held mid-track


# ---- /ws_detections ----

def test_ws_detections_relays_latest_then_updates_verbatim():
    br = FakeBridge()
    br.set_latest("/pilot/detections", SNAP)
    with TestClient(_app(br)) as client:
        with client.websocket_connect("/ws_detections") as ws:
            assert ws.receive_text() == SNAP            # latched latest on join
            updated = json.loads(SNAP)
            updated["seq"] = 8
            updated["sim_stamp"] = 42.1
            text = json.dumps(updated)
            br.set_latest("/pilot/detections", text)    # next publication
            assert ws.receive_text() == text            # verbatim, byte for byte
