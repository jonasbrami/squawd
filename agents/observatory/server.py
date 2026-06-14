"""Swarm Observatory — read-only web UI. Starlette + HTTP polling (no websockets).

Subscribes the bus (drone positions + /swarm/chat) and serves a browser map +
live chat feed. Pure consumer; never publishes.
"""
import os
import threading

import uvicorn
from starlette.applications import Starlette
from starlette.responses import JSONResponse, FileResponse
from starlette.routing import Route, Mount
from starlette.staticfiles import StaticFiles

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

from agents.common.bus import RosBridge, CHAT_QOS

N = int(os.environ.get("SWARM_N", "3"))
HERE = os.path.dirname(__file__)

_chat_lock = threading.Lock()
_chat: list[str] = []


def _on_chat(m) -> None:
    with _chat_lock:
        _chat.append(m.data)


bridge = RosBridge(node_name="observatory")
bridge.subscribe("/swarm/chat", String, CHAT_QOS, _on_chat)
for _i in range(N):
    bridge.subscribe(f"/px4_{_i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
bridge.start()


async def index(request):
    return FileResponse(os.path.join(HERE, "static", "index.html"))


async def state(request):
    drones = []
    for i in range(N):
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        if p is not None:
            drones.append({"id": i, "north": round(p.x, 1), "east": round(p.y, 1),
                           "alt": round(-p.z, 1)})
        else:
            drones.append({"id": i, "north": None, "east": None, "alt": None})
    with _chat_lock:
        chat = list(_chat)
    return JSONResponse({"drones": drones, "chat": chat})


app = Starlette(routes=[
    Route("/", index),
    Route("/state", state),
    Mount("/static", app=StaticFiles(directory=os.path.join(HERE, "static"))),
])

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
