"""Swarm Observatory — web UI. Scales to N drones (SWARM_N).

- Per-drone camera tiles: read Gazebo camera topics via core.GzCameras, H.264
  encoded by VideoHub and streamed over one WebSocket for the browser's WebCodecs
  decoder; /frame/{id} stays as a JPEG fallback for browsers without WebCodecs.
- Map + status: drone positions from ROS2 /px4_<i>/fmu (RosBridge).
- Swarm chat feed: /swarm/chat via a TopicLog.
- Commander input: POST /command -> publishes /swarm/user_input for the
  Commander agent to act on.
Pure consumer of the sim; the only thing it publishes is your typed commands.
"""
import asyncio
import json
import os

import uvicorn
from starlette.applications import Starlette
from starlette.responses import JSONResponse, FileResponse, Response
from starlette.routing import Route, Mount, WebSocketRoute
from starlette.staticfiles import StaticFiles

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus

from agents.core.bus import RosBridge, CHAT_QOS
from agents.core.store import TopicLog
from agents.core.camera import GzCameras
from agents.observatory.video import VideoHub
from agents.observatory import metrics

N = int(os.environ.get("SWARM_N", "3"))
HERE = os.path.dirname(__file__)

# ROS2 side: chat + positions.
bridge = RosBridge(node_name="observatory")
chat = TopicLog(bridge, "/swarm/chat", String, CHAT_QOS)
for _i in range(N):
    base = f"/px4_{_i}/fmu/out"
    bridge.subscribe(f"{base}/vehicle_local_position", VehicleLocalPosition)
    bridge.subscribe(f"{base}/vehicle_status", VehicleStatus)
    bridge.subscribe(f"{base}/battery_status", BatteryStatus)
    # Latched String channels (CHAT_QOS) so the observatory replays the last
    # task/report on connect. The drones publish these; we only read them.
    bridge.subscribe(f"/swarm/cmd/drone_{_i}", String, CHAT_QOS)
    bridge.subscribe(f"/swarm/report/drone_{_i}", String, CHAT_QOS)
bridge.start()

# gz side: per-drone cameras (system gz, read directly) + H.264 encode/fan-out.
cameras = GzCameras(N)
hub = VideoHub(cameras, N)


async def index(request):
    return FileResponse(os.path.join(HERE, "static", "index.html"))


def _text(msg):
    """Latest String channel payload, or None if nothing heard yet."""
    return msg.data if msg is not None else None


async def state(request):
    drones = []
    for i in range(N):
        base = f"/px4_{i}/fmu/out"
        drones.append(metrics.build_drone_state(
            i,
            bridge.latest(f"{base}/vehicle_local_position"),
            bridge.latest(f"{base}/vehicle_status"),
            bridge.latest(f"{base}/battery_status"),
            _text(bridge.latest(f"/swarm/cmd/drone_{i}")),
            _text(bridge.latest(f"/swarm/report/drone_{i}")),
            cameras.has(i),
        ))
    return JSONResponse({"n": N, "drones": drones, "chat": chat.all()})


async def frame(request):
    """One latest JPEG (short-lived request). The WebCodecs-less fallback path in
    the frontend polls this per tile so older browsers still see the cameras."""
    i = int(request.path_params["id"])
    f = cameras.jpeg(i)
    if not f:
        return Response(status_code=204)
    return Response(f, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


async def ws_cams(websocket):
    """Stream every drone's H.264 over ONE WebSocket (browser ~6-per-host cap is
    irrelevant). VideoHub encodes each new frame once and fans it out here. Per
    drone we send a one-off text config {"d", "codec"} before its first keyframe,
    then binary frames: byte0 = drone id, byte1 = flags (bit0 = keyframe), rest =
    Annex-B NAL units for the browser's VideoDecoder."""
    await websocket.accept()
    q = hub.subscribe()
    announced: set[int] = set()
    try:
        while True:
            i, is_key, codec, data = await q.get()
            if codec and i not in announced:
                await websocket.send_text(json.dumps({"d": i, "codec": codec}))
                announced.add(i)
            await websocket.send_bytes(bytes([i, 1 if is_key else 0]) + data)
    except Exception:
        pass  # client disconnected
    finally:
        hub.unsubscribe(q)


async def command(request):
    body = await request.json()
    text = (body.get("text") or "").strip()
    if text:
        print(f"[command] received: {text!r}", flush=True)
        # Echo into the local chat view only (NOT onto /swarm/chat, which the drones
        # listen to) so the UI confirms the message landed without bypassing the
        # commander.
        chat.append(f"you: {text}")
        m = String()
        m.data = text
        bridge.publish("/swarm/user_input", String, m, CHAT_QOS)
    return JSONResponse({"ok": True})


app = Starlette(routes=[
    Route("/", index),
    Route("/state", state),
    Route("/frame/{id:int}", frame),
    WebSocketRoute("/ws", ws_cams),
    Route("/command", command, methods=["POST"]),
    Mount("/static", app=StaticFiles(directory=os.path.join(HERE, "static"))),
])

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
