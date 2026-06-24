"""Protocol test for the observatory video WebSocket.

Mirrors server.ws_cams against a real VideoHub + fake cameras (server.py itself
imports ROS/gz, unavailable off-sim). Asserts the wire format: a one-off text
config {"d","codec"} precedes a drone's first binary frame, whose header is
[drone id, flags] with bit0 = keyframe.
"""
import json

from PIL import Image
from starlette.applications import Starlette
from starlette.routing import WebSocketRoute
from starlette.testclient import TestClient

from agents.observatory.video import VideoHub


def _rgb(w, h, val):
    return Image.new("RGB", (w, h), (val, val, val)).tobytes()


class _FakeCameras:
    def __init__(self, n):
        self._seq = {i: 1 for i in range(n)}   # one frame ready per drone

    def seq(self, i):
        return self._seq[i]

    def raw(self, i):
        return (640, 360, _rgb(640, 360, 10 + i))


def _make_app(hub):
    async def ws_cams(websocket):
        await websocket.accept()
        q = hub.subscribe()
        announced = set()
        try:
            while True:
                i, is_key, codec, data = await q.get()
                if codec and i not in announced:
                    await websocket.send_text(json.dumps({"d": i, "codec": codec}))
                    announced.add(i)
                await websocket.send_bytes(bytes([i, 1 if is_key else 0]) + data)
        except Exception:
            pass
        finally:
            hub.unsubscribe(q)

    return Starlette(routes=[WebSocketRoute("/ws", ws_cams)])   # hub self-starts on subscribe


def test_ws_sends_codec_config_then_keyframe():
    hub = VideoHub(_FakeCameras(1), 1, maxpx=320, interval=0.01)
    with TestClient(_make_app(hub)) as client:
        with client.websocket_connect("/ws") as ws:
            cfg = json.loads(ws.receive_text())           # text config first
            assert cfg["d"] == 0
            assert cfg["codec"].startswith("avc1.42")
            data = ws.receive_bytes()                      # then a binary frame
            assert data[0] == 0                            # drone id
            assert data[1] & 1 == 1                        # keyframe flag set
            assert len(data) > 2                           # carries NAL bytes
