"""Single-drone cockpit server (M4, ICD §8).

Endpoints (ICD §8.2):
- GET  /state         pose, attitude, flight mode, battery, detector/beam/track
                      health (attitude+battery ride the new §8.1 topics)
- WS   /ws_cam        one text announce {"seq","sim_stamp","codec"}, then
                      binary [seq:u32, stamp:f64, H.264 AU] per access unit
- WS   /ws_detections verbatim relay of the pilot's /pilot/detections
- POST /command       {text} -> /pilot/user_input (CMD_QOS)
- POST /estop         {action: "hold"|"land"} -> /pilot/estop (CMD_QOS); the
                      pilot's own arbiter (M1) does the cancel + emergency act
- GET  /chat?since=n  /pilot/chat TopicLog lines

NO local detector, no vision/flight/pilot imports (ICD §0.1): frames arrive
via core.GzCameras, fusion state via the /pilot/detections topic only.
ROS/gz imports are lazy (main()) so build_app and every handler are
unit-testable off-sim with fakes — the telemetry/estop/pipeline pattern.
"""
import asyncio
import json
import os
import struct

from starlette.applications import Starlette
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Mount, Route, WebSocketRoute
from starlette.staticfiles import StaticFiles

from agents.core.store import TopicLog
from agents.observatory import metrics
from agents.observatory.video import VideoHub

HERE = os.path.dirname(__file__)
I = 0                                   # the one drone

PX4_BASE = f"/px4_{I}/fmu/out"
T_POSE = PX4_BASE + "/vehicle_local_position"
T_ATT = PX4_BASE + "/vehicle_attitude"
T_STATUS = PX4_BASE + "/vehicle_status"
T_BATT = PX4_BASE + "/battery_status"
T_DETECTIONS = "/pilot/detections"
T_USER_INPUT = "/pilot/user_input"
T_ESTOP = "/pilot/estop"
T_CHAT = "/pilot/chat"

AU_HEADER = struct.Struct(">Id")        # [seq:u32 BE, sim_stamp:f64 BE] (ICD §8.2)


def _snap_json(msg):
    """Latest /pilot/detections String payload parsed to a dict, or None."""
    if msg is None:
        return None
    try:
        return json.loads(msg.data)
    except Exception:
        return None


def build_app(bridge, cameras, hub, *, msg_type, cmd_qos, chat_qos):
    """Assemble the Starlette app around duck-typed bridge/cameras/hub.

    bridge  -> RosBridge API (subscribe/latest/publish); subscriptions for the
               /state px4 topics and /pilot/detections are the CALLER's job
               (main() holds the px4 msg types; tests inject fakes).
    cameras -> GzCameras API (snapshot(i))
    hub     -> VideoHub (or any object with subscribe/unsubscribe)
    """
    chat = TopicLog(bridge, T_CHAT, msg_type, chat_qos)

    async def index(request):
        return FileResponse(os.path.join(HERE, "static", "index.html"))

    async def state(request):
        snap = _snap_json(bridge.latest(T_DETECTIONS))
        f = cameras.snapshot(I)
        att_msg = bridge.latest(T_ATT)
        att = metrics.rpy_from_quat(getattr(att_msg, "q", None)) \
            if att_msg is not None else None
        return JSONResponse(metrics.build_state(
            bridge.latest(T_POSE), bridge.latest(T_STATUS),
            bridge.latest(T_BATT),
            att=att,
            cam_seq=f.seq if f else 0,
            cam_stamp=f.sim_stamp if f else None,
            snapshot=snap))

    async def ws_cam(websocket):
        """One camera channel (ICD §8.2): the text announce goes out once with
        the first (key)frame's seq/sim_stamp/codec; every binary access unit
        then carries its own [seq, stamp] header for exact overlay matching."""
        await websocket.accept()
        q = hub.subscribe()
        announced = False
        try:
            while True:
                seq, stamp, _is_key, codec, data = await q.get()
                if codec and not announced:
                    await websocket.send_text(json.dumps(
                        {"seq": seq, "sim_stamp": stamp, "codec": codec}))
                    announced = True
                await websocket.send_bytes(AU_HEADER.pack(seq, stamp) + data)
        except Exception:
            pass                            # client disconnected
        finally:
            hub.unsubscribe(q)

    async def ws_detections(websocket):
        """Verbatim relay of /pilot/detections: the latched latest snapshot
        goes out immediately on connect (STATE_QOS depth 1), then each new
        publication as it lands (~detector rate; the UI does the parsing)."""
        await websocket.accept()
        last = None
        try:
            while True:
                msg = bridge.latest(T_DETECTIONS)
                if msg is not None and msg is not last:
                    last = msg
                    await websocket.send_text(msg.data)
                await asyncio.sleep(0.05)
        except Exception:
            pass                            # client disconnected

    async def command(request):
        body = await request.json()
        text = (body.get("text") or "").strip()
        if text:
            print(f"[command] received: {text!r}", flush=True)
            # Local echo only (never republished): the pilot's own reports
            # land on /pilot/chat via CHAT_QOS.
            chat.append(f"you: {text}")
            m = msg_type()
            m.data = text
            bridge.publish(T_USER_INPUT, msg_type, m, cmd_qos)
        return JSONResponse({"ok": True})

    async def estop(request):
        body = await request.json()
        action = (body.get("action") or "hold").strip().lower()
        if action not in ("hold", "land"):
            return JSONResponse(
                {"ok": False, "error": "action must be 'hold' or 'land'"},
                status_code=400)
        print(f"[estop] {action}", flush=True)
        m = msg_type()
        m.data = action
        bridge.publish(T_ESTOP, msg_type, m, cmd_qos)
        return JSONResponse({"ok": True, "action": action})

    async def chat_feed(request):
        try:
            since = int(request.query_params.get("since", "0"))
        except ValueError:
            since = 0
        lines, n = chat.since(max(0, since))
        return JSONResponse({"lines": lines, "next": n})

    return Starlette(routes=[
        Route("/", index),
        Route("/state", state),
        Route("/chat", chat_feed),
        Route("/command", command, methods=["POST"]),
        Route("/estop", estop, methods=["POST"]),
        WebSocketRoute("/ws_cam", ws_cam),
        WebSocketRoute("/ws_detections", ws_detections),
        Mount("/static", app=StaticFiles(directory=os.path.join(HERE, "static"))),
    ])


def main() -> None:
    """Real on-sim assembly: ROS bridge + gz cameras + uvicorn. Importing ROS
    here (not at module scope) keeps the handlers unit-testable off-sim."""
    import uvicorn
    from std_msgs.msg import String
    from px4_msgs.msg import (BatteryStatus, VehicleAttitude,
                              VehicleLocalPosition, VehicleStatus)
    from agents.core.bus import CHAT_QOS, CMD_QOS, STATE_QOS, RosBridge
    from agents.core.camera import GzCameras

    bridge = RosBridge(node_name="cockpit")
    bridge.subscribe(T_POSE, VehicleLocalPosition)       # PX4_QOS (default)
    bridge.subscribe(T_ATT, VehicleAttitude)
    bridge.subscribe(T_STATUS, VehicleStatus)
    bridge.subscribe(T_BATT, BatteryStatus)
    bridge.subscribe(T_DETECTIONS, String, STATE_QOS)
    cameras = GzCameras(1)
    hub = VideoHub(cameras, I)
    app = build_app(bridge, cameras, hub,
                    msg_type=String, cmd_qos=CMD_QOS, chat_qos=CHAT_QOS)
    bridge.start()
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")


if __name__ == "__main__":
    main()
