"""Swarm Observatory — web UI. Scales to N drones (SWARM_N).

- Per-drone camera tiles: read Gazebo camera topics via core.GzCameras, served
  as MJPEG / single frames / one WebSocket of JPEGs.
- Map + status: drone positions from ROS2 /px4_<i>/fmu (RosBridge).
- Swarm chat feed: /swarm/chat via a TopicLog.
- Commander input: POST /command -> publishes /swarm/user_input for the
  Commander agent to act on.
Pure consumer of the sim; the only thing it publishes is your typed commands.
"""
import asyncio
import os

import uvicorn
from starlette.applications import Starlette
from starlette.responses import JSONResponse, FileResponse, StreamingResponse, Response
from starlette.routing import Route, Mount, WebSocketRoute
from starlette.staticfiles import StaticFiles

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

from agents.core.bus import RosBridge, CHAT_QOS
from agents.core.store import TopicLog
from agents.core.camera import GzCameras

N = int(os.environ.get("SWARM_N", "3"))
HERE = os.path.dirname(__file__)

# ROS2 side: chat + positions.
bridge = RosBridge(node_name="observatory")
chat = TopicLog(bridge, "/swarm/chat", String, CHAT_QOS)
for _i in range(N):
    bridge.subscribe(f"/px4_{_i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
bridge.start()

# gz side: per-drone cameras (system gz, read directly).
cameras = GzCameras(N)


async def index(request):
    return FileResponse(os.path.join(HERE, "static", "index.html"))


async def state(request):
    drones = []
    for i in range(N):
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        drones.append({
            "id": i,
            "north": round(p.x, 1) if p else None,
            "east": round(p.y, 1) if p else None,
            "alt": round(-p.z, 1) if p else None,
            "cam": cameras.has(i),
        })
    return JSONResponse({"n": N, "drones": drones, "chat": chat.all()})


async def cam(request):
    i = int(request.path_params["id"])

    async def gen():
        while True:
            f = cameras.jpeg(i)
            if f:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
            await asyncio.sleep(0.1)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


async def frame(request):
    """One latest JPEG (short-lived request). The frontend polls this per tile so we
    don't hold N forever-open MJPEG streams — browsers cap ~6 connections per host."""
    i = int(request.path_params["id"])
    f = cameras.jpeg(i)
    if not f:
        return Response(status_code=204)
    return Response(f, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


async def ws_cams(websocket):
    """Push every drone's latest camera frame over ONE WebSocket. Each message is
    binary: byte 0 = drone id, rest = JPEG. One connection for all N tiles, so the
    browser's ~6-per-host cap is irrelevant. We encode+send a drone's frame only
    when a NEW one has arrived (seq changed), so a hovering/idle drone costs nothing."""
    await websocket.accept()
    last: dict[int, int] = {}
    try:
        while True:
            for i in range(N):
                seq = cameras.seq(i)
                if seq and last.get(i) != seq:
                    f = cameras.jpeg(i)
                    if f is not None:
                        last[i] = seq
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
        chat.append(f"you: {text}")
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
