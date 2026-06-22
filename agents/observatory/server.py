"""Swarm Observatory — web UI. Scales to N drones (SWARM_N).

- Per-drone camera tiles: read Gazebo camera topics directly via gz-transport
  (system gz, no ros_gz), JPEG-encode (Pillow), serve as MJPEG streams.
- Map + status: drone positions from ROS2 /px4_<i>/fmu (RosBridge).
- Swarm chat feed: /swarm/chat (ROS2).
- Commander input: POST /command -> publishes /swarm/user_input for the
  Commander agent to act on.
Pure consumer of the sim; the only thing it publishes is your typed commands.
"""
import asyncio
import io
import os
import threading

import uvicorn
from starlette.applications import Starlette
from starlette.responses import JSONResponse, FileResponse, StreamingResponse, Response
from starlette.routing import Route, Mount, WebSocketRoute
from starlette.staticfiles import StaticFiles

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image as PILImage

from agents.common.bus import RosBridge, CHAT_QOS

N = int(os.environ.get("SWARM_N", "3"))
HERE = os.path.dirname(__file__)
# World name must match the generated world (make_city_world.py names it 'city',
# and PX4 runs with PX4_GZ_WORLD=city). Override with GZ_WORLD if you change it.
GZ_WORLD = os.environ.get("GZ_WORLD", "city")
CAM_TOPIC = ("/world/" + GZ_WORLD + "/model/x500_depth_{i}/link/OakD-Lite/base_link"
             "/sensor/IMX214/image")

_chat_lock = threading.Lock()
_chat: list[str] = []
_frame_lock = threading.Lock()
_frames: dict[int, bytes] = {}


def _on_chat(m) -> None:
    with _chat_lock:
        _chat.append(m.data)


# --- ROS2 side: chat + positions ---
bridge = RosBridge(node_name="observatory")
bridge.subscribe("/swarm/chat", String, CHAT_QOS, _on_chat)
for _i in range(N):
    bridge.subscribe(f"/px4_{_i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
bridge.start()

# --- gz side: per-drone cameras (read system gz directly) ---
_gz = GzNode()


def _make_cam_cb(i: int):
    def cb(msg) -> None:
        try:
            img = PILImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
            buf = io.BytesIO()
            img.save(buf, "JPEG", quality=55)
            with _frame_lock:
                _frames[i] = buf.getvalue()
        except Exception:
            pass
    return cb


for _i in range(N):
    _gz.subscribe(GzImage, CAM_TOPIC.format(i=_i), _make_cam_cb(_i))


async def index(request):
    return FileResponse(os.path.join(HERE, "static", "index.html"))


async def state(request):
    drones = []
    for i in range(N):
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        with _frame_lock:
            has_cam = i in _frames
        drones.append({
            "id": i,
            "north": round(p.x, 1) if p else None,
            "east": round(p.y, 1) if p else None,
            "alt": round(-p.z, 1) if p else None,
            "cam": has_cam,
        })
    with _chat_lock:
        chat = list(_chat)
    return JSONResponse({"n": N, "drones": drones, "chat": chat})


async def cam(request):
    i = int(request.path_params["id"])

    async def gen():
        while True:
            with _frame_lock:
                f = _frames.get(i)
            if f:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
            await asyncio.sleep(0.1)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


async def frame(request):
    """One latest JPEG (short-lived request). The frontend polls this per tile so we
    don't hold N forever-open MJPEG streams — browsers cap ~6 connections per host,
    which would leave tiles 7..N permanently black. Polling cycles through that budget."""
    i = int(request.path_params["id"])
    with _frame_lock:
        f = _frames.get(i)
    if not f:
        return Response(status_code=204)
    return Response(f, media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


async def ws_cams(websocket):
    """Push every drone's latest camera frame over ONE WebSocket. Each message is
    binary: byte 0 = drone id, rest = JPEG. One connection for all N tiles, so the
    browser's ~6-per-host cap is irrelevant. We send a drone's frame only when a NEW
    one has arrived (identity check: the gz callback stores a fresh bytes object per
    frame), so a hovering/idle drone costs nothing."""
    await websocket.accept()
    last: dict[int, bytes] = {}
    try:
        while True:
            for i in range(N):
                with _frame_lock:
                    f = _frames.get(i)
                if f is not None and last.get(i) is not f:
                    last[i] = f
                    await websocket.send_bytes(bytes([i]) + f)
            await asyncio.sleep(0.08)
    except Exception:
        pass  # client disconnected


async def command(request):
    body = await request.json()
    text = (body.get("text") or "").strip()
    if text:
        print(f"[command] received: {text!r}", flush=True)
        # Echo into the local chat view only (NOT onto /swarm/chat, which the drones
        # listen to) so the UI confirms the message landed without bypassing the
        # commander.
        with _chat_lock:
            _chat.append(f"you: {text}")
        m = String()
        m.data = text
        bridge.publish("/swarm/user_input", String, m, CHAT_QOS)
    return JSONResponse({"ok": True})


app = Starlette(routes=[
    Route("/", index),
    Route("/state", state),
    Route("/cam/{id:int}", cam),
    Route("/frame/{id:int}", frame),
    WebSocketRoute("/ws", ws_cams),
    Route("/command", command, methods=["POST"]),
    Mount("/static", app=StaticFiles(directory=os.path.join(HERE, "static"))),
])

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
