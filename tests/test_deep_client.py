"""DeepClient tests (deep-perception plan M1a): a local stdlib http.server
fixture pins the wire contract — happy-path parse with bearer + frame
identity, strict schema violations -> DeepError, connect-refused ->
UNAVAILABLE, 429 -> BUSY, read-timeout -> UNAVAILABLE, other HTTP -> ERROR.
Operational failures NEVER raise; DeepError is for protocol/caller misuse.
"""
import base64
import http.server
import json
import socket
import threading
import time
from contextlib import contextmanager

import pytest

from agents.core.contact import Frame
from agents.perception import deep_client
from agents.perception.deep_client import (BUSY, ERROR, OK, UNAVAILABLE,
                                           DeepClient, DeepError)

TOKEN = "sekret"


def frame(w=4, h=3):
    return Frame(57, 42.1, w, h, bytes(range(w * h * 3)))


@contextmanager
def serve(handler):
    srv = http.server.HTTPServer(("127.0.0.1", 0), handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        yield f"http://127.0.0.1:{srv.server_port}", srv
    finally:
        srv.shutdown()
        srv.server_close()
        t.join()


def handler(routes):
    """routes: (method, path) -> (status, body-dict|raw-bytes|("sleep", s))."""
    class H(http.server.BaseHTTPRequestHandler):
        def _dispatch(self, method):
            body = {}
            if method == "POST":
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
            self.server.last_request = {
                "path": self.path, "auth": self.headers.get("Authorization"),
                "body": body}
            status, payload = routes[(method, self.path.split("?")[0])]
            if isinstance(payload, tuple) and payload[0] == "sleep":
                time.sleep(payload[1])
                status, payload = 200, {}
            raw = payload if isinstance(payload, bytes) \
                else json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            try:
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError):
                pass                            # client timed out mid-read

        do_GET = lambda self: self._dispatch("GET")     # noqa: E731
        do_POST = lambda self: self._dispatch("POST")   # noqa: E731

        def log_message(self, *a):
            pass
    return H


DETECT_OK = {"dets": [{"cls": "house", "conf": 0.87, "xyxy": [1, 2, 5, 6]}],
             "latency_ms": 12.3, "model": "yolo-world-s", "frame_seq": 57,
             "sim_stamp": 42.1}
SEGMENT_OK = {"xyxy": [1.0, 1.0, 3.0, 3.0],
              "mask": {"rle": base64.b64encode(b"\x00\x04").decode(), "w": 2,
                       "h": 2},
              "centroid": [1.5, 1.5], "area_px": 4, "score": 0.9,
              "latency_ms": 30.0, "frame_seq": 57, "sim_stamp": 42.1}
HEALTH_OK = {"ok": True, "device": "cuda", "models_loaded": ["yolov8s-worldv2"],
             "vram_mb": 512}


def client(url, **kw):
    return DeepClient(url, token=TOKEN, **kw)


# ---- happy paths ----

def test_detect_happy_path_sends_bearer_and_frame_identity():
    with serve(handler({("POST", "/v1/detect"): (200, DETECT_OK)})) as (url, srv):
        r = client(url).detect(frame(), ["house"], conf=0.3)
        req = srv.last_request
    assert r.ok and r.status == OK
    assert r.data["dets"][0]["cls"] == "house"
    assert req["auth"] == f"Bearer {TOKEN}"
    assert req["body"]["model"] == "yolo-world-s"        # the default name
    assert req["body"]["prompts"] == ["house"]
    assert req["body"]["conf"] == 0.3
    f = req["body"]["frame"]
    assert (f["seq"], f["sim_stamp"], f["w"], f["h"]) == (57, 42.1, 4, 3)
    assert base64.b64decode(f["rgb_b64"]) == bytes(range(36))


def test_segment_happy_path_points_and_box():
    routes = {("POST", "/v1/segment"): (200, SEGMENT_OK)}
    with serve(handler(routes)) as (url, srv):
        r = client(url).segment(frame(), points=[(2, 2)])
        assert r.ok and r.data["area_px"] == 4
        assert srv.last_request["body"]["points"] == [[2, 2]]
        assert "box" not in srv.last_request["body"]
        assert "model" not in srv.last_request["body"]   # service default
        r = client(url).segment(frame(), box=[0, 0, 3, 3], model="sam-x")
        assert r.ok
        assert srv.last_request["body"]["box"] == [0, 0, 3, 3]
        assert srv.last_request["body"]["model"] == "sam-x"


def test_health_happy_path():
    with serve(handler({("GET", "/v1/health"): (200, HEALTH_OK)})) as (url, _):
        r = client(url).health()
    assert r.ok and r.data["models_loaded"] == ["yolov8s-worldv2"]


# ---- strict schema validation -> DeepError ----

def test_detect_schema_violations_raise_deep_error():
    bad = [
        {**DETECT_OK, "dets": [{"cls": "house", "conf": 0.9}]},   # no xyxy
        {**DETECT_OK, "dets": [{"cls": "h", "conf": 0.9,
                                "xyxy": [1, 2, 3]}]},             # 3 numbers
        {k: v for k, v in DETECT_OK.items() if k != "latency_ms"},
        {**DETECT_OK, "frame_seq": "57"},
        {**DETECT_OK, "dets": [{"cls": "h", "conf": True,
                                "xyxy": [1, 2, 3, 4]}]},          # bool conf
    ]
    for payload in bad:
        with serve(handler({("POST", "/v1/detect"): (200, payload)})) as (url, _):
            with pytest.raises(DeepError):
                client(url).detect(frame(), ["x"])


def test_segment_schema_violations_raise_deep_error():
    bad = [
        {**SEGMENT_OK, "mask": {"rle": "!!junk!!", "w": 2, "h": 2}},
        {**SEGMENT_OK, "mask": None},                # null mask, set xyxy
        {**SEGMENT_OK, "area_px": "4"},
    ]
    for payload in bad:
        with serve(handler({("POST", "/v1/segment"): (200, payload)})) as (url, _):
            with pytest.raises(DeepError):
                client(url).segment(frame(), points=[(1, 1)])


def test_non_json_200_raises_deep_error():
    routes = {("POST", "/v1/detect"): (200, b"<html>proxy error</html>")}
    with serve(handler(routes)) as (url, _):
        with pytest.raises(DeepError, match="non-JSON"):
            client(url).detect(frame(), ["x"])


def test_caller_misuse_raises_deep_error():
    c = client("http://127.0.0.1:1")
    with pytest.raises(DeepError, match="exactly one"):
        c.segment(frame())
    with pytest.raises(DeepError, match="exactly one"):
        c.segment(frame(), points=[(1, 1)], box=[0, 0, 2, 2])
    with pytest.raises(DeepError, match="list of strings"):
        c.detect(frame(), "house")
    with pytest.raises(DeepError, match="http://"):
        client("ftp://x").health()


# ---- operational failures -> typed results, never raised ----

def test_connect_refused_is_unavailable():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()                                        # nothing listening
    r = client(f"http://127.0.0.1:{port}").health()
    assert r.status == UNAVAILABLE and not r.ok and r.detail


def test_429_is_busy():
    routes = {("POST", "/v1/detect"): (429, {"ok": False, "error": "busy"})}
    with serve(handler(routes)) as (url, _):
        r = client(url).detect(frame(), ["x"])
    assert r.status == BUSY and r.data is None


def test_other_http_errors_are_error_not_exception():
    routes = {("POST", "/v1/detect"): (401, {"ok": False}),
              ("GET", "/v1/health"): (500, {"ok": False})}
    with serve(handler(routes)) as (url, _):
        r = client(url).detect(frame(), ["x"])
        assert r.status == ERROR and "401" in r.detail
        assert client(url).health().status == ERROR


def test_read_timeout_maps_to_unavailable():
    routes = {("POST", "/v1/segment"): (200, ("sleep", 0.5))}
    with serve(handler(routes)) as (url, _):
        t0 = time.monotonic()
        r = client(url, read_timeout_segment=0.1).segment(frame(),
                                                          points=[(1, 1)])
        dt = time.monotonic() - t0
    assert r.status == UNAVAILABLE and r.detail == "timeout"
    assert dt < 0.5                                  # failed fast, no hang


def test_default_timeouts_match_the_plan():
    c = DeepClient("http://127.0.0.1:1", token="t")
    assert (c._ct, c._rt_detect, c._rt_segment) == (1.0, 4.0, 6.0)


def test_env_defaults(monkeypatch):
    monkeypatch.setenv("DEEP_PERCEPTION_URL", "http://example:8100/")
    monkeypatch.setenv("DEEP_TOKEN", "env-token")
    c = DeepClient()
    assert c.base_url == "http://example:8100" and c.token == "env-token"
    monkeypatch.delenv("DEEP_PERCEPTION_URL")
    assert DeepClient().base_url == "http://host.docker.internal:8100"
    assert deep_client.CONNECT_TIMEOUT_S == 1.0
