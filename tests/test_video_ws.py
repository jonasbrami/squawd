"""Protocol test for the cockpit video WebSocket (ICD §8.2).

Exercises the REAL ws_cam handler (server.build_app) against a real VideoHub +
fake cameras: a one-off text announce {"seq","sim_stamp","codec"} precedes the
first binary access unit, whose header is [seq:u32 BE, sim_stamp:f64 BE] —
the stamps the overlay match keys on.
"""
import json
import struct

from PIL import Image
from starlette.testclient import TestClient

from agents.core.contact import Frame
from agents.observatory.server import build_app
from agents.observatory.video import VideoHub


def _rgb(w, h, val):
    return Image.new("RGB", (w, h), (val, val, val)).tobytes()


class _FakeString:
    def __init__(self):
        self.data = ""


class _FakeBridge:
    def __init__(self):
        self.published = []

    def subscribe(self, topic, msg_type, qos=None, callback=None):
        pass

    def latest(self, topic):
        return None

    def publish(self, topic, msg_type, msg, qos=None):
        self.published.append((topic, msg.data))


class _FakeCameras:
    def __init__(self):
        self._f = Frame(1, 12.5, 640, 360, _rgb(640, 360, 42))

    def snapshot(self, i):
        return self._f


def test_ws_cam_announces_then_sends_stamped_access_units():
    cams = _FakeCameras()
    hub = VideoHub(cams, 0, maxpx=320, interval=0.01)
    app = build_app(_FakeBridge(), cams, hub,
                    msg_type=_FakeString, cmd_qos=object(), chat_qos=object())
    with TestClient(app) as client:
        with client.websocket_connect("/ws_cam") as ws:
            cfg = json.loads(ws.receive_text())           # announce first
            assert cfg["seq"] == 1 and cfg["sim_stamp"] == 12.5
            assert cfg["codec"].startswith("avc1.42")
            data = ws.receive_bytes()                     # then a binary AU
            seq, stamp = struct.unpack(">Id", data[:12])
            assert (seq, stamp) == (1, 12.5)              # the frame's stamps
            assert len(data) > 12                         # carries NAL bytes
