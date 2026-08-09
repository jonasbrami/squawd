"""Deep-perception M3 slowlane tests (plan §5): cadence gating, skip-if-busy
(zero queue), the gate logic matrix, the fp_suspect overlap math on fixed
frame pairs (incl. the ≥0.6 boundary and the exact-frame rule), payload shape
and env parsing. All off-sim: fake client/detector/frame, no ROS, no threads
except the two lifecycle tests (real 50 Hz loop, sub-second).
"""
import threading
import time
from types import SimpleNamespace

import pytest

from agents.perception.deep_client import (BUSY, ERROR, OK, UNAVAILABLE,
                                           DeepResult)
from agents.vision import slowlane as sl
from agents.vision.slowlane import SlowLane


# ---- fakes ----

def _frame(seq=10, stamp=42.0):
    return SimpleNamespace(seq=seq, sim_stamp=stamp, width=640, height=360,
                           rgb=b"\0" * 12)


def _detect_ok(dets, seq=10, stamp=42.0, latency=41.0):
    return DeepResult(OK, {"dets": dets, "latency_ms": latency,
                           "model": "yolo-world-s", "frame_seq": seq,
                           "sim_stamp": stamp})


HOUSE_DET = {"cls": "house", "conf": 0.19, "xyxy": [400.0, 0.0, 540.0, 100.0]}


class FakeClient:
    def __init__(self, result=None, gate_event=None, raises=None):
        self.calls = []
        self.result = result if result is not None else _detect_ok([])
        self.gate_event = gate_event      # threading.Event: block until set
        self.raises = raises              # exception class to raise

    def detect(self, frame, prompts, conf=0.25):
        self.calls.append({"seq": frame.seq, "prompts": list(prompts),
                           "conf": conf})
        if self.gate_event is not None:
            self.gate_event.wait(5)
        if self.raises is not None:
            raise self.raises("boom")
        return self.result


class FakeDetector:
    def __init__(self, result):
        self._r = result

    def detections(self):
        return self._r


def _inference(seq, dets):
    return SimpleNamespace(frame=SimpleNamespace(seq=seq),
                           detections=[SimpleNamespace(cls=c, conf=cf, xyxy=b)
                                       for c, cf, b in dets])


FAST_TRUCK = ("truck", 0.42, (410.0, 10.0, 530.0, 90.0))   # inside HOUSE_DET


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def _lane(client=None, detector=None, gate=None, pubs=None, clock=None,
          **kw):
    clock = clock or FakeClock()
    return SlowLane(_frame, client or FakeClient(), detector=detector,
                    publisher=(pubs.append if pubs is not None else None),
                    gate=gate, monotonic=clock.now, sleep=clock.sleep, **kw)


# ---- gate matrix (plan §5) ----

@pytest.mark.parametrize("force,render,armed,want", [
    # default (no force): nvidia gates off, armed gates off except exempt
    # backends (A/B-exempt: intel)
    (None, "intel", False, True),
    (None, "intel", True, True),          # intel exempt — A/B green (M3 doc)
    (None, "cpu", False, True),
    (None, "cpu", True, False),
    (None, "nvidia", False, False),       # gz shares the GPU (codex F4)
    (None, "nvidia", True, False),        # nvidia reason wins over armed
    (None, "", False, True),
    # force on always wins (even nvidia + armed)
    ("on", "nvidia", True, True),
    ("on", "intel", True, True),
    ("ON ", "intel", False, True),
    # force off always wins
    ("off", "intel", False, False),
    ("off", "intel", True, False),
    ("off", "nvidia", False, False),
    # an unknown value is not a force: the default gate applies
    ("bogus", "intel", False, True),
    ("bogus", "nvidia", False, False),
])
def test_gate_decision_matrix(force, render, armed, want):
    enabled, reason = sl.gate_decision(force, render, armed)
    assert enabled is want
    assert isinstance(reason, str) and reason


def test_gate_armed_exempt_flips_only_that_backend():
    """The A/B flip: intel exempt from the armed gate; nvidia/cpu stay gated."""
    assert sl.gate_decision(None, "intel", True,
                            armed_exempt=("intel",))[0] is True
    assert sl.gate_decision(None, "nvidia", True,
                            armed_exempt=("intel",))[0] is False
    assert sl.gate_decision(None, "cpu", True,
                            armed_exempt=("intel",))[0] is False


def test_shipped_default_gate_matches_the_ab_decision():
    """The M3 A/B gate PASSED for intel (docs/benchmarks/deep-perception-m3.md):
    the shipped default exempts intel from the armed gate; nvidia stays
    gated (gz shares the GPU) and cpu stays conservatively gated."""
    assert sl.ARMED_GATE_EXEMPT == ("intel",)
    assert sl.gate_decision(None, "intel", True)[0] is True
    assert sl.gate_decision(None, "nvidia", True)[0] is False
    assert sl.gate_decision(None, "cpu", True)[0] is False


# ---- overlap math (fixed frame pairs; codex F3: intersection/fast_area) ----

def test_overlap_disjoint_and_contained():
    assert sl.intersection_over_fast((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert sl.intersection_over_fast((10, 10, 20, 20), (0, 0, 100, 100)) == 1.0


def test_overlap_boundary_exactly_0_6():
    """fast 100x100 at origin; ann covering exactly 60% of it -> the ≥0.6
    boundary flags; 59% does not."""
    fast = (0, 0, 100, 100)
    assert sl.intersection_over_fast(fast, (40, 0, 140, 100)) == \
        pytest.approx(0.6)
    assert sl.intersection_over_fast(fast, (41, 0, 141, 100)) == \
        pytest.approx(0.59)


def test_overlap_zero_area_fast_box_is_zero():
    assert sl.intersection_over_fast((5, 5, 5, 9), (0, 0, 100, 100)) == 0.0


def test_fp_suspects_boundary_and_class_filter():
    fast = [{"cls": "truck", "conf": 0.5, "xyxy": (0, 0, 100, 100)}]
    hit = sl.fp_suspects(fast, [{"cls": "house", "conf": 0.3,
                                 "xyxy": (40, 0, 140, 100)}])
    assert len(hit) == 1                              # exactly 0.6 -> flagged
    assert hit[0]["ann_cls"] == "house" and hit[0]["overlap"] == pytest.approx(0.6)
    miss = sl.fp_suspects(fast, [{"cls": "house", "conf": 0.3,
                                  "xyxy": (41, 0, 141, 100)}])
    assert miss == []                                 # 0.59 -> not flagged
    tree = sl.fp_suspects(fast, [{"cls": "tree", "conf": 0.3,
                                  "xyxy": (0, 0, 100, 100)}])
    assert tree == []               # only building/house annotate the advisory
    bld = sl.fp_suspects(fast, [{"cls": "building", "conf": 0.3,
                                 "xyxy": (0, 0, 100, 100)}])
    assert len(bld) == 1 and bld[0]["cls"] == "truck"


# ---- tick_once: gate, frame, drops, payload ----

def test_gated_tick_never_calls_and_publishes_health_only():
    client, pubs = FakeClient(), []
    lane = _lane(client, gate=lambda: (False, "drone armed"), pubs=pubs)
    payload = lane.tick_once()
    assert client.calls == []
    assert payload["dets"] == [] and payload["frame_seq"] is None
    assert payload["health"]["active"] is False
    assert payload["health"]["note"] == "drone armed"
    assert payload["health"]["skipped_gate"] == 1
    assert pubs == [payload]


def test_no_frame_tick_is_skipped():
    client = FakeClient()
    lane = SlowLane(lambda: None, client, gate=lambda: (True, "on"),
                    monotonic=time.monotonic, sleep=time.sleep)
    payload = lane.tick_once()
    assert client.calls == []
    assert payload["health"]["skipped_no_frame"] == 1


def test_ok_tick_publishes_annotations_keyed_by_frame():
    client = FakeClient(_detect_ok([HOUSE_DET], seq=10, stamp=42.0))
    pubs = []
    lane = _lane(client, detector=FakeDetector(None), pubs=pubs)
    payload = lane.tick_once()
    assert client.calls == [{"seq": 10,
                             "prompts": ["building", "house", "tree", "pole",
                                         "tower"], "conf": 0.05}]
    assert payload["type"] == "slowlane"
    assert payload["frame_seq"] == 10 and payload["sim_stamp"] == 42.0
    assert payload["frame_w"] == 640 and payload["frame_h"] == 360
    assert payload["captured_mono"] == pytest.approx(1000.0)
    assert payload["dets"] == [HOUSE_DET]
    assert payload["latency_ms"] == 41.0
    assert payload["health"]["ok"] == 1 and payload["health"]["calls"] == 1
    assert pubs == [payload]


def test_payload_shape_never_births_contacts():
    pubs = []
    lane = _lane(FakeClient(_detect_ok([HOUSE_DET])), pubs=pubs)
    payload = lane.tick_once()
    assert set(payload) == {
        "type", "frame_seq", "sim_stamp", "frame_w", "frame_h",
        "captured_mono", "dets", "fp_suspects", "fp_checked", "fast_dets",
        "latency_ms", "health"}
    assert "contacts" not in payload            # annotations are NOT contacts


@pytest.mark.parametrize("status,counter", [
    (BUSY, "dropped_busy"),
    (UNAVAILABLE, "dropped_unavailable"),
    (ERROR, "dropped_error"),
])
def test_non_ok_results_drop_the_tick(status, counter):
    client = FakeClient(DeepResult(status, detail="nope"))
    lane = _lane(client)
    payload = lane.tick_once()
    assert payload["health"][counter] == 1
    assert payload["health"]["ok"] == 0
    assert payload["health"]["last_error"]
    assert payload["dets"] == []                          # nothing fabricated


def test_protocol_violation_is_a_dropped_error():
    from agents.perception.deep_client import DeepError
    lane = _lane(FakeClient(raises=DeepError))
    payload = lane.tick_once()
    assert payload["health"]["dropped_error"] == 1
    assert "protocol" in payload["health"]["last_error"]


def test_dropped_tick_keeps_last_good_annotations_sticky():
    """A drop overwrites health but NOT the last annotations — the cockpit
    expires them by frame age (≤0.5 s), never by a later failed tick."""
    client = FakeClient(_detect_ok([HOUSE_DET]))
    lane = _lane(client)
    lane.tick_once()
    client.result = DeepResult(BUSY, detail="sidecar busy")
    payload = lane.tick_once()
    assert payload["dets"] == [HOUSE_DET]                 # sticky, expiring
    assert payload["health"]["dropped_busy"] == 1


# ---- exact-frame FP advisory ----

def test_advisory_uses_the_exact_submitted_inference_result():
    client = FakeClient(_detect_ok([HOUSE_DET], seq=10))
    det = FakeDetector(_inference(10, [FAST_TRUCK]))
    pubs = []
    lane = _lane(client, detector=det, pubs=pubs)
    payload = lane.tick_once()
    assert payload["fp_checked"] is True
    assert len(payload["fp_suspects"]) == 1
    s = payload["fp_suspects"][0]
    assert s["cls"] == "truck" and s["ann_cls"] == "house"
    assert s["overlap"] > 0.6
    assert payload["fast_dets"][0]["cls"] == "truck"      # audit trail


def test_advisory_never_uses_a_later_frame():
    """The detector already moved past the submitted seq: NO advisory (codex
    F3 — overlap against the current frame would be geometrically meaningless)."""
    client = FakeClient(_detect_ok([HOUSE_DET], seq=10))
    det = FakeDetector(_inference(11, [FAST_TRUCK]))      # wrong (later) frame
    payload = _lane(client, detector=det).tick_once()
    assert payload["fp_checked"] is False
    assert payload["fp_suspects"] == []
    assert payload["fast_dets"] == []


def test_advisory_times_out_when_the_exact_frame_never_comes():
    clock = FakeClock()
    det = FakeDetector(_inference(9, [FAST_TRUCK]))       # stuck one behind
    lane = _lane(FakeClient(_detect_ok([HOUSE_DET], seq=10)),
                 detector=det, clock=clock)
    payload = lane.tick_once()
    assert payload["fp_checked"] is False and payload["fp_suspects"] == []
    assert clock.t >= 1000.0 + sl.EXACT_FRAME_WAIT_S      # bounded wait


def test_advisory_absent_without_a_fast_lane():
    payload = _lane(FakeClient(_detect_ok([HOUSE_DET])),
                    detector=None).tick_once()
    assert payload["fp_checked"] is False and payload["dets"] == [HOUSE_DET]


# ---- lifecycle: cadence + skip-if-busy (real thread, 50 Hz) ----

def test_thread_ticks_at_the_configured_cadence(monkeypatch):
    monkeypatch.setattr(sl, "MAX_HZ", 100.0)     # test-only: 50 Hz loop
    pubs = []
    lane = _lane(FakeClient(), pubs=pubs, hz=50.0,
                 clock=SimpleNamespace(now=time.monotonic, sleep=time.sleep))
    lane.start()
    time.sleep(0.32)
    lane.stop(timeout=2.0)
    ticks = lane.state()["ticks"]
    assert 8 <= ticks <= 25              # nominal 16 at 50 Hz over 0.32 s
    assert not (lane._thread and lane._thread.is_alive())


def test_skip_if_busy_never_queues(monkeypatch):
    """A call that overruns the period eats the missed ticks: exactly one
    call in flight during the block, and no catch-up burst after it."""
    monkeypatch.setattr(sl, "MAX_HZ", 100.0)     # test-only: 50 Hz loop
    gate = threading.Event()             # cleared: the client blocks
    client = FakeClient(gate_event=gate)
    pubs = []
    lane = _lane(client, pubs=pubs, hz=50.0,
                 clock=SimpleNamespace(now=time.monotonic, sleep=time.sleep))
    lane.start()
    time.sleep(0.25)                     # ~12 periods pass behind the block
    assert len(client.calls) == 1        # one in flight, zero queued
    gate.set()
    time.sleep(0.06)
    after = len(client.calls)
    lane.stop(timeout=2.0)
    assert after <= 4                    # cadence resumed, no 12-tick burst


# ---- env parsing ----

def test_env_overrides_and_clamps(monkeypatch):
    monkeypatch.setenv("DEEP_SLOWLANE_HZ", "50")
    monkeypatch.setenv("DEEP_SLOWLANE_VOCAB", "house")
    monkeypatch.setenv("DEEP_SLOWLANE_CONF", "0.2")
    client = FakeClient()
    lane = _lane(client)
    assert lane._hz == sl.MAX_HZ
    lane.tick_once()
    assert client.calls[0]["prompts"] == ["house"]
    assert client.calls[0]["conf"] == 0.2


def test_bad_env_falls_back_to_defaults(monkeypatch):
    monkeypatch.setenv("DEEP_SLOWLANE_HZ", "fast")
    monkeypatch.setenv("DEEP_SLOWLANE_VOCAB", " , ,")
    monkeypatch.setenv("DEEP_SLOWLANE_CONF", "9")
    client = FakeClient()
    lane = _lane(client)
    lane.tick_once()
    assert lane._hz == sl.DEFAULT_HZ
    assert client.calls[0]["prompts"] == sl.DEFAULT_VOCAB.split(",")
    assert client.calls[0]["conf"] == sl.DEFAULT_CONF


def test_vocab_caps_mirror_the_wire(monkeypatch):
    monkeypatch.setenv("DEEP_SLOWLANE_VOCAB", ",".join(
        [f"p{i}" for i in range(20)] + ["x" * 40]))
    client = FakeClient()
    lane = _lane(client)
    lane.tick_once()
    prompts = client.calls[0]["prompts"]
    assert len(prompts) == sl.MAX_PROMPTS
    assert all(len(p) <= sl.MAX_PROMPT_CHARS for p in prompts)
