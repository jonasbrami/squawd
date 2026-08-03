"""Single-drone cockpit server (M4, ICD §8).

Endpoints (ICD §8.2):
- GET  /state         pose, attitude, flight mode, battery, detector/beam/track
                      health (attitude+battery ride the new §8.1 topics)
- WS   /ws_cam        one text announce {"seq","sim_stamp","codec"}, then
                      binary [seq:u32, stamp:f64, H.264 AU] per access unit
- WS   /ws_detections verbatim relay of the pilot's /pilot/detections
- POST /command       {text} -> /pilot/user_input (CMD_QOS)
- POST /api/lock      {x, y} frame px -> SERVER-side hit-test (overlay.hit_test,
                      W0.3) -> {"op":"lock","contact":name} on /pilot/cmd
                      (CMD_QOS) for the pilot's W0.4 arbiter; 409 + reason
                      (stale|ambiguous|miss) when the click can't be honored
- POST /api/cmd       raw locked-object op (design v0.3 §5 schema: orbit /
                      standoff / stop / resume; per-op fields + numeric bounds
                      checked) published VERBATIM on /pilot/cmd (CMD_QOS) —
                      400 on anything malformed (W3b)
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
from agents.observatory import metrics, overlay
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
T_CMD = "/pilot/cmd"
T_CHAT = "/pilot/chat"

AU_HEADER = struct.Struct(">Id")        # [seq:u32 BE, sim_stamp:f64 BE] (ICD §8.2)

# Locked-object op schema (design v0.3 §5) — the cockpit<->pilot contract on
# /pilot/cmd, shared with the flight workstream's W0.4 arbiter. The server
# validates and relays VERBATIM; it never reshapes a payload.
OP_REQUIRED = {
    "lock": ("contact",),
    "orbit": ("contact", "radius_m", "rate_dps"),
    "standoff": ("contact", "range_m"),
    "stop": (),
    "resume": ("contact",),
}
OP_BOUNDS = {"radius_m": (8, 40), "rate_dps": (2, 45), "range_m": (8, 40)}


def validate_op(body) -> str | None:
    """Gate a raw /pilot/cmd op payload against the v0.3 §5 schema.

    None when the op is well-formed (op known, its required fields present,
    numeric args in bounds); otherwise the legible 400 reason. Extra fields
    pass through untouched — validation gates, never reshapes.
    """
    if not isinstance(body, dict):
        return "body must be a JSON object"
    op = body.get("op")
    if op not in OP_REQUIRED:
        return f"unknown op {op!r} (expected one of {sorted(OP_REQUIRED)})"
    for field in OP_REQUIRED[op]:
        if field not in body:
            return f"op {op!r} requires {field!r}"
    if "contact" in OP_REQUIRED[op]:
        contact = body["contact"]
        if not isinstance(contact, str) or not contact.strip():
            return "contact must be a non-empty string"
    for field, (lo, hi) in OP_BOUNDS.items():
        if field in OP_REQUIRED[op]:
            v = body[field]
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                return f"{field} must be a number"
            if not lo <= v <= hi:
                return f"{field} must be within [{lo}, {hi}] (got {v})"
    return None


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

    async def lock(request):
        """Click-to-lock (W0.3): server-side hit-test of frame-pixel coords
        against the latest snapshot's contacts[].bbox_xyxy, gated on the
        server's newest camera frame stamp (the same 0.5 s rule the overlay
        draws by). A unique hit publishes the lock op on /pilot/cmd; anything
        else is a 409 with the legible reason (never a guessed contact)."""
        body = await request.json()
        try:
            x, y = float(body["x"]), float(body["y"])
        except (KeyError, TypeError, ValueError):
            return JSONResponse(
                {"ok": False, "error": "body must carry numeric x and y "
                 "(frame pixels)"}, status_code=400)
        snap = _snap_json(bridge.latest(T_DETECTIONS))
        f = cameras.snapshot(I)
        res = overlay.hit_test(snap, x, y, f.sim_stamp if f else None)
        if res["contact"] is None:
            print(f"[lock] rejected: {res['reason']}", flush=True)
            return JSONResponse({"ok": False, "reason": res["reason"]},
                                status_code=409)
        print(f"[lock] {res['contact']}", flush=True)
        m = msg_type()
        m.data = json.dumps({"op": "lock", "contact": res["contact"]})
        bridge.publish(T_CMD, msg_type, m, cmd_qos)
        return JSONResponse({"ok": True, **res})

    async def cmd(request):
        """Locked-object operations (W3b, design v0.3 §5): the ops bar's raw
        op payload, schema-validated and published VERBATIM on /pilot/cmd —
        the pilot's W0.4 arbiter owns execution. 400 on anything malformed."""
        try:
            body = await request.json()
        except Exception:
            return JSONResponse({"ok": False, "error": "body must be JSON"},
                                status_code=400)
        err = validate_op(body)
        if err:
            print(f"[cmd] rejected: {err}", flush=True)
            return JSONResponse({"ok": False, "error": err}, status_code=400)
        print(f"[cmd] {json.dumps(body)}", flush=True)
        m = msg_type()
        m.data = json.dumps(body)
        bridge.publish(T_CMD, msg_type, m, cmd_qos)
        return JSONResponse({"ok": True, "op": body["op"]})

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
        Route("/api/lock", lock, methods=["POST"]),
        Route("/api/cmd", cmd, methods=["POST"]),
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
