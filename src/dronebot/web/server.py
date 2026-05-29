# src/dronebot/web/server.py
"""FastAPI cockpit backend. Runs the shared dronebot stack inside the app
lifespan (single asyncio loop) and exposes chat, telemetry, and camera.
"""
from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from dronebot.config import load_config
from dronebot.stack import build_stack, start_stack, stop_stack
from dronebot.web.framing import mjpeg_part, telemetry_frame

_ABORT_WORDS = {"stop", "abort", "emergency", "land now"}
_STATIC = Path(__file__).parent / "static"


@asynccontextmanager
async def lifespan(app: FastAPI):
    stack = build_stack(load_config())
    await start_stack(stack)
    await stack.agent.__aenter__()
    app.state.stack = stack
    try:
        yield
    finally:
        await stack.agent.__aexit__(None, None, None)
        await stop_stack(stack)


app = FastAPI(lifespan=lifespan)


def _text_of(message) -> str:
    content = getattr(message, "content", None) or []
    return " ".join(getattr(b, "text", "") for b in content if getattr(b, "text", ""))


@app.websocket("/chat")
async def chat(ws: WebSocket) -> None:
    await ws.accept()
    stack = app.state.stack
    try:
        while True:
            user = (await ws.receive_text()).strip()
            if not user:
                continue
            if user.lower() in _ABORT_WORDS:
                stack.log.record("abort", {"trigger": user})
                try:
                    await stack.agent.interrupt()
                    result = await stack.executor.hold()
                    await ws.send_text(f"[ABORT] {result.message}")
                except Exception as exc:  # abort must always respond
                    stack.log.record("abort_error", {"error": str(exc)})
                    await ws.send_text(f"[ABORT] hold command errored: {exc}")
                continue
            stack.log.record("utterance", {"text": user})
            async for message in stack.agent.ask(user):
                text = _text_of(message)
                if text:
                    await ws.send_text(text)
            await ws.send_text("\n")
    except WebSocketDisconnect:
        return


@app.websocket("/telemetry")
async def telemetry(ws: WebSocket) -> None:
    await ws.accept()
    stack = app.state.stack
    period = 1.0 / max(stack.config.telemetry_rate_hz, 0.5)
    try:
        while True:
            frame = telemetry_frame(stack.state, stack.perception_store)
            frame["geofence_radius_m"] = stack.config.limits.geofence_radius_m
            await ws.send_text(json.dumps(frame))
            await asyncio.sleep(period)
    except WebSocketDisconnect:
        return


@app.get("/camera")
async def camera():
    stack = app.state.stack

    async def gen():
        while True:
            snap = stack.perception_store.latest()
            if snap is not None and snap.jpeg_frame is not None:
                yield mjpeg_part(snap.jpeg_frame)
            await asyncio.sleep(0.1)

    return StreamingResponse(
        gen(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


_STATIC.mkdir(parents=True, exist_ok=True)
app.mount("/", StaticFiles(directory=str(_STATIC), html=True), name="static")
