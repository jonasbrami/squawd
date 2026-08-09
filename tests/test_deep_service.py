"""Deep sidecar route tests (deep-perception plan M1a): build_app around a
fake registry via Starlette TestClient — health shape, detect/segment happy
paths over a tiny synthetic frame, 401 bearer auth, 400 exact frame-size
validation, 413 oversize body, 422 prompt caps, 429 busy (no queue).
No torch, no GPU, no uvicorn.
"""
import base64
import threading

from starlette.testclient import TestClient

from agents.vision.deep import service
from agents.vision.deep.service import build_app
from agents.vision.types import Detection, rle_decode, rle_encode

TOKEN = "test-token"
AUTH = {"Authorization": f"Bearer {TOKEN}"}

MASK_2X2 = rle_encode([[True, True], [True, True]])


class FakeRegistry:
    """DeepRegistry-shaped fake: a real threading.Lock (the 429 gate rides
    it), canned detect/segment payloads, records the frames it saw."""

    def __init__(self):
        self.lock = threading.Lock()
        self.device = "cpu"
        self.frames = []
        self.calls = []

    def loaded(self):
        return ["fake-world"]

    def vram_mb(self):
        return None

    def detect_locked(self, model, frame, prompts, conf):
        self.frames.append(frame)
        self.calls.append(("detect", model, prompts, conf))
        return [Detection("house", 0.876, (1.0, 2.0, 5.0, 6.0))]

    def segment_locked(self, model, frame, points=None, box=None):
        self.frames.append(frame)
        self.calls.append(("segment", model, points, box))
        return {"xyxy": (1.0, 1.0, 3.0, 3.0), "mask": MASK_2X2,
                "centroid": (1.5, 1.5), "area_px": 4, "score": 0.91}


def frame_payload(w=4, h=3, rgb=None, seq=57, stamp=42.1):
    rgb = bytes(w * h * 3) if rgb is None else rgb
    return {"w": w, "h": h, "rgb_b64": base64.b64encode(rgb).decode("ascii"),
            "seq": seq, "sim_stamp": stamp}


def detect_body(**kw):
    body = {"frame": frame_payload(), "model": "fake-world",
            "prompts": ["house"], "conf": 0.3}
    body.update(kw)
    return body


def app(reg, **kw):
    return TestClient(build_app(reg, token=TOKEN, **kw))


# ---- health ----

def test_health_shape():
    with app(FakeRegistry()) as c:
        d = c.get("/v1/health", headers=AUTH).json()
    assert d == {"ok": True, "device": "cpu", "models_loaded": ["fake-world"],
                 "vram_mb": None}


def test_health_requires_auth():
    with app(FakeRegistry()) as c:
        assert c.get("/v1/health").status_code == 401


# ---- /v1/detect ----

def test_detect_happy_path_over_tiny_frame():
    reg = FakeRegistry()
    with app(reg) as c:
        r = c.post("/v1/detect", json=detect_body(), headers=AUTH)
    assert r.status_code == 200
    d = r.json()
    assert d["dets"] == [{"cls": "house", "conf": 0.876,
                          "xyxy": [1.0, 2.0, 5.0, 6.0]}]
    assert d["model"] == "fake-world"
    assert d["frame_seq"] == 57 and d["sim_stamp"] == 42.1
    assert d["latency_ms"] >= 0.0
    f = reg.frames[0]                                # the exact Frame contract
    assert (f.seq, f.sim_stamp, f.width, f.height) == (57, 42.1, 4, 3)
    assert len(f.rgb) == 4 * 3 * 3
    assert reg.calls[0] == ("detect", "fake-world", ["house"], 0.3)


def test_detect_401_without_and_with_wrong_bearer():
    with app(FakeRegistry()) as c:
        assert c.post("/v1/detect", json=detect_body()).status_code == 401
        r = c.post("/v1/detect", json=detect_body(),
                   headers={"Authorization": "Bearer nope"})
        assert r.status_code == 401
        assert c.post("/v1/segment", json={}, headers={
            "Authorization": "Bearer nope"}).status_code == 401


def test_detect_400_on_wrong_rgb_length():
    with app(FakeRegistry()) as c:
        r = c.post("/v1/detect", headers=AUTH, json=detect_body(
            frame=frame_payload(rgb=bytes(4 * 3 * 3 - 1))))
        assert r.status_code == 400
        assert "w*h*3" in r.json()["error"]


def test_detect_400_on_malformed_frame_and_body():
    with app(FakeRegistry()) as c:
        assert c.post("/v1/detect", headers=AUTH, json=detect_body(
            frame=frame_payload(w=0))).status_code == 400
        assert c.post("/v1/detect", headers=AUTH, json=detect_body(
            frame={**frame_payload(), "rgb_b64": "!!not b64!!"}
        )).status_code == 400
        assert c.post("/v1/detect", headers=AUTH,
                      json={"model": "m"}).status_code == 400
        assert c.post("/v1/detect", headers=AUTH, json=detect_body(
            model="")).status_code == 400
        assert c.post("/v1/detect", headers=AUTH, json=detect_body(
            conf=1.5)).status_code == 400
        r = c.post("/v1/detect", content="{not json",
                   headers={**AUTH, "Content-Type": "application/json"})
        assert r.status_code == 400


def test_body_over_cap_is_413():
    assert service.MAX_BODY_BYTES == 8 << 20         # the ~8 MB pin (R5)
    reg = FakeRegistry()
    with app(reg, max_body=1024) as c:               # same check, small cap
        r = c.post("/v1/detect", headers=AUTH, json=detect_body(
            frame=frame_payload(w=64, h=64)))
        assert r.status_code == 413
    assert reg.calls == []


def test_detect_422_on_prompt_overflow():
    reg = FakeRegistry()
    with app(reg) as c:
        r = c.post("/v1/detect", headers=AUTH, json=detect_body(
            prompts=[f"p{i}" for i in range(17)]))
        assert r.status_code == 422
        r = c.post("/v1/detect", headers=AUTH,
                   json=detect_body(prompts=["x" * 33]))
        assert r.status_code == 422
        r = c.post("/v1/detect", headers=AUTH, json=detect_body(prompts=[]))
        assert r.status_code == 422
    assert reg.calls == []                           # rejected before inference


def test_detect_429_when_lock_held():
    """Single-request concurrency (codex R4): a second caller while the ONE
    inference lock is held gets 429, never a queue."""
    reg = FakeRegistry()
    holder = threading.Thread(target=reg.lock.acquire)   # held by another thread
    holder.start()
    holder.join()
    try:
        with app(reg) as c:
            r = c.post("/v1/detect", json=detect_body(), headers=AUTH)
        assert r.status_code == 429
    finally:
        reg.lock.release()
    assert reg.calls == []


# ---- /v1/segment ----

def test_segment_happy_path_points_and_box():
    reg = FakeRegistry()
    with app(reg) as c:
        r = c.post("/v1/segment", headers=AUTH, json={
            "frame": frame_payload(), "points": [[2, 2]]})
        assert r.status_code == 200
        d = r.json()
        assert d["xyxy"] == [1.0, 1.0, 3.0, 3.0]
        assert d["mask"]["w"] == 2 and d["mask"]["h"] == 2
        rows = rle_decode(base64.b64decode(d["mask"]["rle"]), 2, 2)
        assert rows == [[True, True], [True, True]]  # box-local round-trip
        assert d["centroid"] == [1.5, 1.5]
        assert d["area_px"] == 4 and d["score"] == 0.91
        assert d["frame_seq"] == 57 and d["sim_stamp"] == 42.1
        r = c.post("/v1/segment", headers=AUTH, json={
            "frame": frame_payload(), "box": [0, 0, 3, 3], "model": "sam-x"})
        assert r.status_code == 200
    assert reg.calls[0] == ("segment", "sam2.1-t", [[2, 2]], None)
    assert reg.calls[1] == ("segment", "sam-x", None, [0, 0, 3, 3])


def test_segment_400_on_prompt_shape():
    with app(FakeRegistry()) as c:
        base = {"frame": frame_payload()}
        both = {**base, "points": [[1, 1]], "box": [0, 0, 2, 2]}
        assert c.post("/v1/segment", json=both, headers=AUTH) \
            .status_code == 400                      # both -> 400
        assert c.post("/v1/segment", json=base, headers=AUTH) \
            .status_code == 400                      # neither -> 400
        bad_pts = {**base, "points": [[1]]}
        assert c.post("/v1/segment", json=bad_pts, headers=AUTH) \
            .status_code == 400
        bad_box = {**base, "box": [2, 2, 0, 0]}
        assert c.post("/v1/segment", json=bad_box, headers=AUTH) \
            .status_code == 400                      # x1<x2 required


def test_segment_429_when_lock_held():
    reg = FakeRegistry()
    holder = threading.Thread(target=reg.lock.acquire)
    holder.start()
    holder.join()
    try:
        with app(reg) as c:
            r = c.post("/v1/segment", headers=AUTH, json={
                "frame": frame_payload(), "points": [[1, 1]]})
        assert r.status_code == 429
    finally:
        reg.lock.release()
