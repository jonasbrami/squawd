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
    "dets": [{"cls": "target", "conf": 0.8, "xyxy": [10, 20, 60, 80]}],
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


def test_ws_detections_relays_det_masks_verbatim():
    """W2 (design §4): a snapshot whose dets carry mask RLE relays byte for
    byte — the overlay's mask drawing reads exactly what the pilot sent."""
    snap = json.loads(SNAP)
    snap["dets"] = [{"cls": "car", "conf": 0.7, "xyxy": [10, 20, 60, 80],
                     "mask": {"rle": "gICAwP8=", "w": 50, "h": 60}}]
    text = json.dumps(snap)
    br = FakeBridge()
    br.set_latest("/pilot/detections", text)
    with TestClient(_app(br)) as client:
        with client.websocket_connect("/ws_detections") as ws:
            assert ws.receive_text() == text


# ---- /api/lock (W0.3: server-side crowd-safe hit-test -> /pilot/cmd) ----

def _lock_snap(stamp=42.0, named_boxes=(("vis_car_0", [10, 10, 50, 50]),)):
    s = json.loads(SNAP)
    s["sim_stamp"] = stamp
    s["contacts"] = [{"name": n, "cls": "car", "bbox_xyxy": b}
                     for n, b in named_boxes]
    return json.dumps(s)


def _lock_app(br, stamp=42.1):
    cams = FakeCameras(SimpleNamespace(seq=1, sim_stamp=stamp))
    return _app(br, cams)


def test_lock_unique_hit_publishes_one_pilot_cmd():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _lock_snap())
    with TestClient(_lock_app(br)) as client:
        r = client.post("/api/lock", json={"x": 30, "y": 30})
        assert r.status_code == 200
        assert r.json() == {"ok": True, "contact": "vis_car_0",
                            "bbox_xyxy": [10, 10, 50, 50]}
    assert br.published == [
        ("/pilot/cmd", '{"op": "lock", "contact": "vis_car_0"}', "CMD")]


def test_lock_ambiguous_crowd_is_409_and_publishes_nothing():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _lock_snap(named_boxes=(
        ("vis_car_0", [10, 10, 60, 60]), ("vis_car_1", [20, 20, 70, 70]))))
    with TestClient(_lock_app(br)) as client:
        r = client.post("/api/lock", json={"x": 30, "y": 30})
        assert r.status_code == 409
        assert r.json() == {"ok": False, "reason": "ambiguous"}
    assert br.published == []


def test_lock_miss_is_409_and_publishes_nothing():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _lock_snap())
    with TestClient(_lock_app(br)) as client:
        r = client.post("/api/lock", json={"x": 300, "y": 300})
        assert r.status_code == 409
        assert r.json() == {"ok": False, "reason": "miss"}
    assert br.published == []


def test_lock_stale_snapshot_is_409_and_publishes_nothing():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _lock_snap(stamp=40.0))  # 2.1 s old
    with TestClient(_lock_app(br)) as client:
        r = client.post("/api/lock", json={"x": 30, "y": 30})
        assert r.status_code == 409
        assert r.json() == {"ok": False, "reason": "stale"}
    assert br.published == []


def test_lock_without_coordinates_is_400():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _lock_snap())
    with TestClient(_lock_app(br)) as client:
        assert client.post("/api/lock", json={"x": 30}).status_code == 400
        assert client.post("/api/lock", json={}).status_code == 400
    assert br.published == []


# ---- /api/cmd (W3b: validated locked-object ops -> /pilot/cmd verbatim) ----

def test_cmd_each_valid_op_publishes_once_verbatim():
    br = FakeBridge()
    ops = [
        {"op": "lock", "contact": "vis_car_0"},
        {"op": "orbit", "contact": "vis_car_0", "radius_m": 15, "rate_dps": 15},
        {"op": "standoff", "contact": "vis_car_0", "range_m": 12.5},
        {"op": "stop"},
        {"op": "resume", "contact": "vis_car_0"},
    ]
    with TestClient(_app(br)) as client:
        for op in ops:
            r = client.post("/api/cmd", json=op)
            assert r.status_code == 200
            assert r.json() == {"ok": True, "op": op["op"]}
    assert br.published == [("/pilot/cmd", json.dumps(op), "CMD") for op in ops]


def test_cmd_accepts_bound_edges():
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        assert client.post("/api/cmd", json={"op": "orbit", "contact": "c",
                           "radius_m": 8, "rate_dps": 2}).status_code == 200
        assert client.post("/api/cmd", json={"op": "orbit", "contact": "c",
                           "radius_m": 40, "rate_dps": 45}).status_code == 200
        assert client.post("/api/cmd", json={"op": "standoff", "contact": "c",
                           "range_m": 40}).status_code == 200
    assert len(br.published) == 3


def test_cmd_invalid_ops_are_400_and_publish_nothing():
    br = FakeBridge()
    bad = [
        {},                                                      # no op
        {"op": "dance"},                                         # unknown op
        {"op": "orbit", "contact": "c", "rate_dps": 15},         # no radius
        {"op": "orbit", "contact": "c", "radius_m": 15},         # no rate
        {"op": "standoff", "contact": "c"},                      # no range
        {"op": "resume"},                                        # no contact
        {"op": "orbit", "contact": "  ", "radius_m": 15, "rate_dps": 15},
        {"op": "orbit", "contact": "c", "radius_m": 7, "rate_dps": 15},
        {"op": "orbit", "contact": "c", "radius_m": 41, "rate_dps": 15},
        {"op": "orbit", "contact": "c", "radius_m": 15, "rate_dps": 1},
        {"op": "orbit", "contact": "c", "radius_m": 15, "rate_dps": 46},
        {"op": "standoff", "contact": "c", "range_m": 7},
        {"op": "standoff", "contact": "c", "range_m": 41},
        {"op": "orbit", "contact": "c", "radius_m": "15", "rate_dps": 15},
        {"op": "orbit", "contact": "c", "radius_m": True, "rate_dps": 15},
        ["not", "an", "object"],
    ]
    with TestClient(_app(br)) as client:
        for op in bad:
            r = client.post("/api/cmd", json=op)
            assert r.status_code == 400, op
            assert r.json()["ok"] is False
        r = client.post("/api/cmd", content="{not json",
                        headers={"Content-Type": "application/json"})
        assert r.status_code == 400
    assert br.published == []


# ---- ops-bar rendered defaults (W3 codex R4: the H=4 m demo geometry) ----

def test_index_renders_r4_ops_bar_defaults():
    """R4 pin: the served cockpit stages the H=4 m geometry — the orbit
    stepper renders R 15 m and sends 15 m / 8 dps, and Approach/Back-off
    step ±3 m inside the 14-20 m stand-off band (R_min(4)=12, shadow ring
    14; never below the floor), a baseline-less click sending the R4
    demonstration pair 14 m / 18 m."""
    br = FakeBridge()
    with TestClient(_app(br)) as client:
        html = client.get("/").text
    assert 'id="op-rval">R 15 m</span>' in html
    assert "orbitR: 15," in html
    assert "radius_m: S.orbitR, rate_dps: 8" in html
    assert "clamp((S.gap ?? 17) - 3, 14, 20)" in html
    assert "clamp((S.gap ?? 15) + 3, 14, 20)" in html


# ---- /state M3 deep additions (deep-perception plan §5/§6) ----

def _slowlane(stamp=42.0, suspects=None, dets=None):
    return json.dumps({
        "type": "slowlane", "frame_seq": 57, "sim_stamp": stamp,
        "frame_w": 640, "frame_h": 360, "captured_mono": 1234.5,
        "dets": dets if dets is not None else
        [{"cls": "house", "conf": 0.19, "xyxy": [572, 235, 640, 280]}],
        "fp_suspects": suspects or [],
        "fp_checked": suspects is not None,
        "fast_dets": [], "latency_ms": 41.0,
        "health": {"active": True, "note": "default", "hz": 0.3,
                   "vocab": ["building", "house"], "conf": 0.05,
                   "last_error": None, "ticks": 9, "calls": 8, "ok": 7,
                   "dropped_busy": 1, "dropped_unavailable": 0,
                   "dropped_error": 0, "skipped_gate": 1,
                   "skipped_no_frame": 0, "fp_checked": 6}})


def _fp_snap(stamp=42.0):
    """A snapshot whose truck contact box sits inside the slowlane's house
    annotation (the phantom-truck-on-building geometry)."""
    s = json.loads(SNAP)
    s["sim_stamp"] = stamp
    s["contacts"] = [{"name": "vis_truck_0", "cls": "truck", "conf": 0.42,
                      "bbox_xyxy": [575, 238, 638, 279], "range_m": 31.0}]
    return json.dumps(s)


SUSPECT = [{"cls": "truck", "conf": 0.42, "xyxy": [574, 238, 638, 279],
            "ann_cls": "house", "ann_xyxy": [572, 235, 640, 280],
            "overlap": 0.97}]

PINPOINT = json.dumps({
    "type": "pinpoint_mask", "frame_seq": 57, "sim_stamp": 42.0,
    "frame_w": 640, "frame_h": 360, "xyxy": [296, 271, 347, 303],
    "mask": {"rle": "gICAwP8=", "w": 51, "h": 32},
    "centroid": [321.0, 286.8], "area_px": 1398, "score": 0.93,
    "cls": "truck", "color_rgb": [178, 172, 164]})


def _m3_app(br, cam_stamp=42.1):
    return _app(br, FakeCameras(SimpleNamespace(seq=57, sim_stamp=cam_stamp)))


def test_state_serves_fresh_annotations_health_and_fp_flag():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _fp_snap())
    br.set_latest("/pilot/slowlane", _slowlane(suspects=SUSPECT))
    with TestClient(_m3_app(br)) as client:
        d = client.get("/state").json()
    assert d["annotations"] == [
        {"cls": "house", "conf": 0.19, "xyxy": [572, 235, 640, 280],
         "frame_seq": 57, "sim_stamp": 42.0, "age_ms": 100}]
    assert d["contacts"][0]["fp_suspect"] is True
    assert d["slowlane"]["hz"] == 0.3 and d["slowlane"]["ok"] == 7
    assert d["pinpoint_mask"] is None


def test_state_annotations_expire_at_half_a_second_of_frame_age():
    br = FakeBridge()
    br.set_latest("/pilot/detections", _fp_snap())
    br.set_latest("/pilot/slowlane", _slowlane(stamp=41.4, suspects=SUSPECT))
    with TestClient(_m3_app(br)) as client:          # cam 42.1: payload 0.7 s old
        d = client.get("/state").json()
    assert d["annotations"] == []                     # expired, nothing stale
    assert d["contacts"][0]["fp_suspect"] is False    # the advisory clears too
    assert d["slowlane"]["ok"] == 7                   # health is process state


def test_state_annotations_at_the_exact_boundary_survive():
    br = FakeBridge()
    br.set_latest("/pilot/slowlane", _slowlane(stamp=41.5))
    with TestClient(_m3_app(br, cam_stamp=42.0)) as client:   # exactly 0.5 s
        d = client.get("/state").json()
    assert len(d["annotations"]) == 1
    assert d["annotations"][0]["age_ms"] == 500


def test_state_fp_flag_requires_box_and_class_match():
    br = FakeBridge()
    snap = json.loads(_fp_snap())
    snap["contacts"] = [
        {"name": "vis_truck_0", "cls": "truck", "bbox_xyxy": [10, 10, 60, 60]},
        {"name": "vis_car_1", "cls": "car", "bbox_xyxy": [575, 238, 638, 279]}]
    br.set_latest("/pilot/detections", json.dumps(snap))
    br.set_latest("/pilot/slowlane", _slowlane(suspects=SUSPECT))
    with TestClient(_m3_app(br)) as client:
        d = client.get("/state").json()
    by_name = {c["name"]: c for c in d["contacts"]}
    assert by_name["vis_truck_0"]["fp_suspect"] is False   # box doesn't match
    assert by_name["vis_car_1"]["fp_suspect"] is False     # cls doesn't match


def test_state_pinpoint_mask_passthrough_fresh_then_expired():
    br = FakeBridge()
    br.set_latest("/pilot/deep", PINPOINT)
    with TestClient(_m3_app(br)) as client:
        d = client.get("/state").json()
    m = d["pinpoint_mask"]
    assert m["frame_seq"] == 57 and m["age_ms"] == 100
    assert m["mask"] == {"rle": "gICAwP8=", "w": 51, "h": 32}
    assert m["xyxy"] == [296, 271, 347, 303] and m["cls"] == "truck"

    stale = json.loads(PINPOINT)
    stale["sim_stamp"] = 40.0                           # 2.1 s of frame age
    br2 = FakeBridge()
    br2.set_latest("/pilot/deep", json.dumps(stale))
    with TestClient(_m3_app(br2)) as client:
        assert client.get("/state").json()["pinpoint_mask"] is None


def test_state_deep_topics_absent_or_unparseable_are_safe():
    br = FakeBridge()
    with TestClient(_m3_app(br)) as client:
        d = client.get("/state").json()
    assert d["annotations"] == [] and d["pinpoint_mask"] is None
    assert d["slowlane"] is None

    br.set_latest("/pilot/slowlane", "{not json")
    br.set_latest("/pilot/deep", json.dumps({"type": "something_else"}))
    with TestClient(_m3_app(br)) as client:
        d = client.get("/state").json()
    assert d["annotations"] == [] and d["pinpoint_mask"] is None
