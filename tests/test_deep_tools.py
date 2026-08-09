"""M2 deep-perception tools (deep-perception plan §4): look/pinpoint grammar
(header, entries, bearing words, labeled ground_intersection), client-side
prompt/conf caps, NOT_READY no-frame, UNAVAILABLE/BUSY/ERROR mapping, the
single-flight busy flag, pinpoint label resolution from cached look() dets,
x,y validation, advisory wording, the mask-publisher hook, and the BOUND-tool
proofs through make_pilot_options (event loop keeps ticking during a 200 ms
sidecar call; a cancelled call returns ESTOPPED promptly).
"""
import asyncio
import threading
import time
from types import SimpleNamespace

import pytest

from agents.flight import FlightOps, make_pilot_options
from agents.perception import deep_client as dc
from agents.pilot.deep_tools import DEFAULT_CONF, make_deep_tools


# ---------- fakes ----------

def _frame(seq=7, w=640, h=360):
    return SimpleNamespace(seq=seq, sim_stamp=42.0, width=w, height=h,
                           rgb=bytes(w * h * 3))


class FakeWorld:
    def drone_state(self, bridge, i):
        return (10.0, 20.0, 15.0, 0.0)        # e, n, alt, facing north

    def attitude_at(self, t):
        return (0.0, 0.0, 0.0)                # roll, pitch, yaw


class FakePipeline:
    def __init__(self):
        self._snap = SimpleNamespace(completed_monotonic=time.monotonic())

    def latest(self):
        return self._snap


class FakeClient:
    def __init__(self, detect_res=None, segment_res=None, detect_sleep=0.0):
        self.detect_res = detect_res
        self.segment_res = segment_res
        self.detect_sleep = detect_sleep
        self.detect_calls = []
        self.segment_calls = []

    def detect(self, frame, prompts, conf=0.25):
        self.detect_calls.append((list(prompts), conf, frame.seq))
        if self.detect_sleep:
            time.sleep(self.detect_sleep)
        return self.detect_res

    def segment(self, frame, points=None, box=None):
        self.segment_calls.append((points, box, frame.seq))
        return self.segment_res


def _det(cls, conf, xyxy):
    return {"cls": cls, "conf": conf, "xyxy": list(xyxy)}


def _detect_res(dets):
    return dc.DeepResult(dc.OK, {"dets": dets, "latency_ms": 9.1,
                                 "model": "yolo-world-s", "frame_seq": 7,
                                 "sim_stamp": 42.0})


def _segment_res():
    return dc.DeepResult(dc.OK, {
        "xyxy": [10.0, 20.0, 60.0, 80.0],
        "mask": {"rle": "AAEC", "w": 50, "h": 60},
        "centroid": [35.0, 50.0], "area_px": 1234, "score": 0.9,
        "latency_ms": 82.0, "frame_seq": 7, "sim_stamp": 42.0})


def _tools(client, frame=None, world=None, pipeline="fake", publisher=None):
    if frame is None:
        frame = _frame()
    return make_deep_tools(world if world is not None else FakeWorld(), None,
                           FakePipeline() if pipeline == "fake" else pipeline,
                           lambda: frame, client, mask_publisher=publisher)


# ---------- look: grammar ----------

def test_look_header_and_entries():
    client = FakeClient(_detect_res([
        _det("house", 0.19, (450, 200, 630, 300)),
        _det("person", 0.23, (311, 278, 340, 303))]))
    look, _ = _tools(client)
    text = look("house,person")
    header, entries = text.split(": ", 1)
    assert header.startswith("2 advisory deep hit(s) for 'house,person'")
    assert "frame #7" in header and "feed" in header and "9 ms" in header
    e = entries.split(" | ")
    assert e[0].startswith("deep_house_0 house conf 0.19 ahead-right")
    assert e[1].startswith("deep_person_1 person conf 0.23 ahead")
    # both box bottoms visible (y2 300/303 < 358) -> labeled ground_intersection
    assert all("ground_intersection ~" in x and "at E" in x
               and "(from visible box bottom)" in x for x in e)


def test_look_ground_intersection_only_from_a_visible_bottom():
    client = FakeClient(_detect_res([
        _det("car", 0.2, (100, 300, 200, 360))]))     # bottom clipped by frame
    look, _ = _tools(client)
    text = look("car")
    assert "deep_car_0" in text and "ground_intersection" not in text


def test_look_no_range_without_state_or_attitude():
    class NoState(FakeWorld):
        def drone_state(self, bridge, i):
            return None
    client = FakeClient(_detect_res([_det("house", 0.2, (400, 200, 600, 300))]))
    look, _ = _tools(client, world=NoState())
    text = look("house")
    assert "deep_house_0" in text and "ground_intersection" not in text


def test_look_empty_result_is_advisory_and_legible():
    client = FakeClient(_detect_res([]))
    look, _ = _tools(client)
    text = look("zeppelin")
    assert text.startswith("0 advisory deep hit(s) for 'zeppelin'")
    assert "nothing found" in text and "ADVISORY" in text
    assert "`detect` tool stays the mover authority" in text


# ---------- look: prompt parsing / caps (client-side) ----------

def test_prompt_parsing_strips_and_drops_blanks():
    client = FakeClient(_detect_res([]))
    look, _ = _tools(client)
    look("  house , , tree ")
    assert client.detect_calls == [(["house", "tree"], DEFAULT_CONF, 7)]


def test_prompt_caps_enforced_client_side():
    client = FakeClient(_detect_res([]))
    look, _ = _tools(client)
    assert look("").startswith("INVALID_PARAM")
    assert look(",".join(f"c{i}" for i in range(17))).startswith(
        "INVALID_PARAM: too many prompts")
    assert look("x" * 33).startswith("INVALID_PARAM: a prompt exceeds")
    assert client.detect_calls == []          # nothing reached the wire


def test_conf_validation_and_default():
    client = FakeClient(_detect_res([]))
    look, _ = _tools(client)
    assert look("house", conf=0).startswith("INVALID_PARAM: conf")
    assert look("house", conf=1.5).startswith("INVALID_PARAM: conf")
    assert look("house", conf="high").startswith("INVALID_PARAM: conf")
    assert look("house", conf=True).startswith("INVALID_PARAM: conf")
    look("house")
    look("house", conf=0.1)
    assert client.detect_calls == [(["house"], DEFAULT_CONF, 7),
                                   (["house"], 0.1, 7)]


# ---------- look: failure mapping ----------

def test_not_ready_without_a_frame():
    client = FakeClient(_detect_res([]))
    look, pinpoint = make_deep_tools(FakeWorld(), None, None,
                                     lambda: None, client)
    assert look("house") == "NOT_READY: no camera frame yet"
    assert pinpoint(x=10, y=10) == "NOT_READY: no camera frame yet"
    assert client.detect_calls == [] and client.segment_calls == []


@pytest.mark.parametrize("status,detail,prefix,extra", [
    (dc.UNAVAILABLE, "timeout", "UNAVAILABLE:",
     "the fast `detect` tool still works"),
    (dc.BUSY, "sidecar busy", "BUSY:", "retry"),
    (dc.ERROR, "HTTP 500", "ERROR:", "HTTP 500"),
])
def test_typed_statuses_map_to_legible_codes(status, detail, prefix, extra):
    client = FakeClient(dc.DeepResult(status, detail=detail))
    look, _ = _tools(client)
    text = look("house")
    assert text.startswith(prefix) and extra in text


def test_protocol_violation_maps_to_error_text():
    class BadSchemaClient(FakeClient):
        def detect(self, frame, prompts, conf=0.25):
            raise dc.DeepError("response missing 'dets'")
    look, _ = _tools(BadSchemaClient())
    assert look("house").startswith("ERROR: sidecar protocol violation:")


def test_single_flight_busy_flag():
    client = FakeClient(_detect_res([]), detect_sleep=0.3)
    look, _ = _tools(client)
    first = threading.Thread(target=look, args=("house",))
    first.start()
    time.sleep(0.05)                     # first call holds the flag
    try:
        assert look("house").startswith("BUSY: a deep-perception call")
    finally:
        first.join()
    assert look("house").startswith("0 advisory")   # released afterwards


# ---------- pinpoint ----------

def test_pinpoint_explicit_xy_grammar():
    client = FakeClient(segment_res=_segment_res())
    _, pinpoint = _tools(client)
    text = pinpoint(x=100, y=120)
    assert client.segment_calls == [([[100, 120]], None, 7)]
    assert "mask at (100,120) [explicit pixel]" in text
    assert "centroid ahead-left" in text      # centroid (35,50) is left of center
    assert "area 1234px" in text
    assert "tight box [10,20,60,80]" in text
    assert "score 0.90" in text and "82 ms" in text
    assert "UNLABELED (SAM does not identify" in text
    assert "ADVISORY only, never a flight target" in text


def test_pinpoint_label_resolves_from_cached_look_dets():
    client = FakeClient(_detect_res([
        _det("truck", 0.2, (300, 250, 380, 310)),     # centroid (340, 280)
        _det("house", 0.1, (400, 200, 600, 300))]),
        segment_res=_segment_res())
    look, pinpoint = _tools(client)
    look("truck,house")
    text = pinpoint(label="TRUCK")            # case-insensitive
    assert client.segment_calls == [([[340, 280]], None, 7)]
    assert "[look() hit deep_truck_0 on frame #7" in text
    assert "labeled 'truck' from look()" in text
    assert "UNLABELED" not in text


def test_pinpoint_label_prefers_the_highest_conf_hit():
    client = FakeClient(_detect_res([
        _det("car", 0.1, (0, 0, 20, 20)),             # centroid (10, 10)
        _det("car", 0.3, (100, 100, 140, 140))]),     # centroid (120, 120)
        segment_res=_segment_res())
    look, pinpoint = _tools(client)
    look("car")
    pinpoint(label="car")
    assert client.segment_calls == [([[120, 120]], None, 7)]


def test_pinpoint_unknown_label_and_missing_args():
    client = FakeClient(_detect_res([_det("truck", 0.2, (0, 0, 10, 10))]),
                        segment_res=_segment_res())
    look, pinpoint = _tools(client)
    assert pinpoint().startswith("INVALID_PARAM: pinpoint needs")
    assert pinpoint(label="boat").startswith(
        "INVALID_PARAM: no previous look() hit named 'boat'")
    look("truck")
    assert pinpoint(label="boat").startswith("INVALID_PARAM")
    assert client.segment_calls == []


def test_pinpoint_xy_validation():
    client = FakeClient(segment_res=_segment_res())
    _, pinpoint = _tools(client)
    assert pinpoint(x=10).startswith("INVALID_PARAM: pinpoint needs BOTH")
    assert pinpoint(y=10).startswith("INVALID_PARAM: pinpoint needs BOTH")
    assert pinpoint(x=-1, y=10).startswith("INVALID_PARAM: pixel (-1,10)")
    assert pinpoint(x=640, y=10).startswith("INVALID_PARAM: pixel (640,10)")
    assert pinpoint(x=10, y=360).startswith("INVALID_PARAM: pixel (10,360)")
    assert pinpoint(x="a", y=1).startswith("INVALID_PARAM: x/y must be")
    assert client.segment_calls == []


def test_pinpoint_empty_segment_is_legible():
    empty = dc.DeepResult(dc.OK, {
        "xyxy": None, "mask": None, "centroid": None, "area_px": 0,
        "score": 0.0, "latency_ms": 80.0, "frame_seq": 7, "sim_stamp": 42.0})
    client = FakeClient(segment_res=empty)
    _, pinpoint = _tools(client)
    assert pinpoint(x=5, y=5).startswith(
        "no mask at (5,5) — SAM found nothing coherent")


def test_mask_publisher_hook_receives_the_wire_payload():
    published = []
    client = FakeClient(_detect_res([_det("truck", 0.2, (300, 250, 380, 310))]),
                        segment_res=_segment_res())
    look, pinpoint = _tools(client, publisher=published.append)
    pinpoint(x=100, y=120)
    assert len(published) == 1
    p = published[0]
    assert p["type"] == "pinpoint_mask" and p["frame_seq"] == 7
    assert p["sim_stamp"] == 42.0 and p["frame_w"] == 640
    assert p["mask"] == {"rle": "AAEC", "w": 50, "h": 60}
    assert p["xyxy"] == [10.0, 20.0, 60.0, 80.0] and p["area_px"] == 1234
    assert p["cls"] is None                       # no look() seed
    assert p["color_rgb"] == [0, 0, 0]            # sampled at the centroid
    look("truck")
    pinpoint(label="truck")
    assert published[1]["cls"] == "truck"         # class hint from look()


def test_mask_publisher_is_best_effort():
    def boom(_payload):
        raise RuntimeError("ros down")
    client = FakeClient(segment_res=_segment_res())
    _, pinpoint = _tools(client, publisher=boom)
    assert pinpoint(x=1, y=1).startswith("mask at (1,1)")   # never raises


# ---------- bound through make_pilot_options ----------

def _bound(client, **kw):
    look, pinpoint = _tools(client, **kw)
    opts = make_pilot_options(FlightOps(None, None, None, 0, 1),
                              deep_tools=(look, pinpoint),
                              report=lambda m: None)
    srv = opts.mcp_servers["pilot"]["instance"]

    import mcp.types as mcp_types

    async def call(name, arguments):
        req = mcp_types.CallToolRequest(
            method="tools/call",
            params=mcp_types.CallToolRequestParams(name=name,
                                                   arguments=arguments))
        return (await srv.request_handlers[mcp_types.CallToolRequest](req)).root

    return opts, call


def test_deep_tools_register_only_when_supplied():
    bare = make_pilot_options(FlightOps(None, None, None, 0, 1),
                              report=lambda m: None)
    assert "mcp__pilot__look" not in bare.allowed_tools
    assert "mcp__pilot__pinpoint" not in bare.allowed_tools
    opts, _ = _bound(FakeClient(_detect_res([])))
    assert "mcp__pilot__look" in opts.allowed_tools
    assert "mcp__pilot__pinpoint" in opts.allowed_tools


def test_bound_look_does_not_block_the_event_loop():
    """The to_thread wrap (codex B2): a 200 ms sidecar call must not stall
    the loop — a 10 ms ticker keeps ticking, and a second concurrent look is
    rejected BUSY by the single-flight flag."""
    client = FakeClient(_detect_res([_det("house", 0.2, (400, 200, 600, 300))]),
                        detect_sleep=0.2)
    _, call = _bound(client)

    async def go():
        ticks = 0
        stop = False

        async def ticker():
            nonlocal ticks
            while not stop:
                ticks += 1
                await asyncio.sleep(0.01)

        t = asyncio.create_task(ticker())
        first = asyncio.create_task(call("look", {"what": "house"}))
        await asyncio.sleep(0.05)                # first call is in flight
        second = await call("look", {"what": "house"})
        res = await first
        stop = True
        await t
        return res, second, ticks

    res, second, ticks = asyncio.run(go())
    assert not res.isError
    text = res.content[0].text
    assert "advisory deep hit(s)" in text and "deep_house_0" in text
    assert second.content[0].text.startswith("BUSY: a deep-perception call")
    assert ticks >= 5        # ~20 expected at 10 ms over 200 ms; floor for jitter


def test_bound_look_cancellation_returns_estopped_promptly():
    """ESTOPPED semantics: cancelling the await aborts promptly even though
    the worker thread lives on (bounded by the client timeouts)."""
    release = threading.Event()

    class HungClient(FakeClient):
        def detect(self, frame, prompts, conf=0.25):
            release.wait(timeout=10)
            return _detect_res([])

    _, call = _bound(HungClient())

    async def go():
        task = asyncio.create_task(call("look", {"what": "house"}))
        await asyncio.sleep(0.1)                 # the call is stuck in read
        t0 = time.monotonic()
        task.cancel()
        res = await task
        return res, time.monotonic() - t0

    res, dt = asyncio.run(go())
    release.set()
    assert res.isError
    assert res.content[0].text.startswith("ESTOPPED: operator halted look")
    assert dt < 1.0


def test_bound_pinpoint_round_trip():
    client = FakeClient(segment_res=_segment_res())
    _, call = _bound(client)
    res = asyncio.run(call("pinpoint", {"x": 100, "y": 120}))
    assert not res.isError
    assert res.content[0].text.startswith("mask at (100,120)")
