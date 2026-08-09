"""vision/deep/service.py — deep-perception sidecar (deep-perception plan §1).

Host-GPU HTTP service mirroring observatory/server.py's shape:
build_app(registry, token=...) is fake-testable; heavy imports (uvicorn)
stay lazy in main() and the registry's own torch/ultralytics imports are
lazy in its load path — this module imports clean in the plain venv.

Endpoints (all bearer-authed with DEEP_TOKEN, codex R5):
- GET  /v1/health  -> {ok, device, models_loaded, vram_mb}
- POST /v1/detect  {frame:{w,h,rgb_b64,seq,sim_stamp}, model, prompts[], conf}
                   -> {dets:[{cls,conf,xyxy}], latency_ms, model, frame_seq,
                       sim_stamp}
- POST /v1/segment {frame:{...}, points:[[x,y]]|box:[x1,y1,x2,y2], model?}
                   -> {xyxy, mask:{rle,w,h}|null, centroid, area_px, score,
                       latency_ms, frame_seq, sim_stamp}

Binding rules (codex R5): exact frame-size validation (w*h*3 bytes after
b64decode -> 400), body capped at MAX_BODY_BYTES (-> 413), prompt caps
(-> 422), and single-request concurrency: the registry's ONE inference lock
is taken NON-blocking and inference runs via asyncio.to_thread, so a second
caller gets 429 immediately instead of a queue. Binds the discovered docker0
gateway address, never 0.0.0.0.
"""
import asyncio
import base64
import binascii
import hmac
import json
import re
import subprocess
import time

from starlette.applications import Starlette
from starlette.responses import JSONResponse
from starlette.routing import Route

from agents.core.contact import Frame
from agents.vision.deep.registry import (PromptError, canonical_prompts)
from agents.vision.types import BackendError

PORT = 8100
MAX_BODY_BYTES = 8 << 20         # ~8 MB; a 1280x720 RGB frame b64s to ~3.7 MB

_NUM = (int, float)


def _is_num(v) -> bool:
    return isinstance(v, _NUM) and not isinstance(v, bool)


def build_app(registry, *, token: str, max_body: int = MAX_BODY_BYTES):
    """Starlette app around a duck-typed registry: .lock (threading.Lock),
    detect_locked/segment_locked, .device, loaded(), vram_mb(). Tests inject
    a fake; main() builds the real DeepRegistry."""

    def err(status: int, msg: str) -> JSONResponse:
        return JSONResponse({"ok": False, "error": msg}, status_code=status)

    def authed(request) -> bool:
        return hmac.compare_digest(request.headers.get("authorization", ""),
                                   f"Bearer {token}")

    async def read_body(request):
        """-> (body, None) | (None, response): bounded + JSON (codex R5)."""
        try:
            if int(request.headers.get("content-length") or 0) > max_body:
                return None, err(413, f"body over {max_body} bytes")
        except ValueError:
            pass
        raw = await request.body()
        if len(raw) > max_body:
            return None, err(413, f"body over {max_body} bytes")
        try:
            return json.loads(raw), None
        except Exception:
            return None, err(400, "body must be JSON")

    def parse_frame(body):
        """-> (Frame, None) | (None, response): EXACT w*h*3 RGB validation."""
        f = body.get("frame") if isinstance(body, dict) else None
        if not isinstance(f, dict):
            return None, err(400, "frame must be an object")
        w, h, seq, stamp = f.get("w"), f.get("h"), f.get("seq"), \
            f.get("sim_stamp")
        if not all(isinstance(v, int) and not isinstance(v, bool)
                   for v in (w, h, seq)) or not _is_num(stamp):
            return None, err(400, "frame needs int w/h/seq + number sim_stamp")
        try:
            rgb = base64.b64decode(f.get("rgb_b64", ""), validate=True)
        except (TypeError, binascii.Error):
            return None, err(400, "rgb_b64 must be base64")
        if w <= 0 or h <= 0 or w * h * 3 > max_body:
            return None, err(400, "bad frame dimensions")
        if len(rgb) != w * h * 3:
            return None, err(400, f"rgb must be exactly w*h*3={w * h * 3} "
                           f"bytes, got {len(rgb)}")
        return Frame(seq, float(stamp), w, h, rgb), None

    def parse_prompt(body):
        """-> (model, points, box, None) | (None,)*3 + response."""
        points, box = body.get("points"), body.get("box")
        model = body.get("model") or "sam2.1-t"
        if not isinstance(model, str):
            return None, None, None, err(400, "model must be a string")
        if (points is None) == (box is None):
            return None, None, None, err(
                400, "segment needs exactly one of points or box")
        if points is not None:
            if not isinstance(points, list) or not points or not all(
                    isinstance(p, (list, tuple)) and len(p) == 2
                    and all(_is_num(c) for c in p) for p in points):
                return None, None, None, err(
                    400, "points must be [[x, y], ...] numbers")
            return model, points, None, None
        if not isinstance(box, (list, tuple)) or len(box) != 4 \
                or not all(_is_num(v) for v in box) \
                or not (box[0] < box[2] and box[1] < box[3]):
            return None, None, None, err(
                400, "box must be [x1, y1, x2, y2] with x1<x2, y1<y2")
        return model, None, list(box), None

    async def health(request):
        if not authed(request):
            return err(401, "unauthorized")
        return JSONResponse({"ok": True, "device": registry.device,
                             "models_loaded": registry.loaded(),
                             "vram_mb": registry.vram_mb()})

    async def detect(request):
        if not authed(request):
            return err(401, "unauthorized")
        body, e = await read_body(request)
        if e:
            return e
        frame, e = parse_frame(body)
        if e:
            return e
        model, conf = body.get("model"), body.get("conf", 0.25)
        if not isinstance(model, str) or not model:
            return err(400, "model must be a non-empty string")
        if not _is_num(conf) or not 0.0 <= conf <= 1.0:
            return err(400, "conf must be a number in [0, 1]")
        try:
            canonical_prompts(body.get("prompts"))
        except PromptError as ex:
            return err(422, str(ex))
        if not registry.lock.acquire(blocking=False):
            return err(429, "busy")                  # no queue (codex R4)
        try:
            t0 = time.monotonic()
            dets = await asyncio.to_thread(registry.detect_locked, model,
                                           frame, body["prompts"], float(conf))
            latency_ms = (time.monotonic() - t0) * 1e3
        except PromptError as ex:
            return err(422, str(ex))
        except BackendError as ex:
            return err(500, str(ex))
        finally:
            registry.lock.release()
        return JSONResponse({
            "dets": [{"cls": d.cls, "conf": round(d.conf, 3),
                      "xyxy": [round(v, 1) for v in d.xyxy]} for d in dets],
            "latency_ms": round(latency_ms, 1), "model": model,
            "frame_seq": frame.seq, "sim_stamp": frame.sim_stamp})

    async def segment(request):
        if not authed(request):
            return err(401, "unauthorized")
        body, e = await read_body(request)
        if e:
            return e
        frame, e = parse_frame(body)
        if e:
            return e
        model, points, box, e = parse_prompt(body)
        if e:
            return e
        if not registry.lock.acquire(blocking=False):
            return err(429, "busy")
        try:
            t0 = time.monotonic()
            seg = await asyncio.to_thread(registry.segment_locked, model,
                                          frame, points, box)
            latency_ms = (time.monotonic() - t0) * 1e3
        except PromptError as ex:
            return err(422, str(ex))
        except BackendError as ex:
            return err(500, str(ex))
        finally:
            registry.lock.release()
        resp = {"xyxy": None, "mask": None, "centroid": None,
                "area_px": seg["area_px"], "score": round(seg["score"], 3),
                "latency_ms": round(latency_ms, 1),
                "frame_seq": frame.seq, "sim_stamp": frame.sim_stamp}
        if seg["mask"] is not None:                  # box-local RLE (F8)
            x1, y1, x2, y2 = seg["xyxy"]
            resp.update(
                xyxy=[round(v, 1) for v in seg["xyxy"]],
                mask={"rle": base64.b64encode(seg["mask"]).decode("ascii"),
                      "w": max(1, int(x2) - int(x1)),
                      "h": max(1, int(y2) - int(y1))},
                centroid=[round(v, 1) for v in seg["centroid"]])
        return JSONResponse(resp)

    return Starlette(routes=[
        Route("/v1/health", health),
        Route("/v1/detect", detect, methods=["POST"]),
        Route("/v1/segment", segment, methods=["POST"]),
    ])


def gateway_addr() -> str:
    """The host-side docker0 bridge address to bind (codex R5: the container
    reaches it as host.docker.internal; binding 0.0.0.0 would expose authed
    GPU work to the whole LAN). `ip route` src parse; fallback 127.0.0.1."""
    try:
        out = subprocess.run(["ip", "-4", "route", "show", "dev", "docker0"],
                             capture_output=True, text=True, timeout=2).stdout
        m = re.search(r"\bsrc (\d+\.\d+\.\d+\.\d+)", out)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "127.0.0.1"


def main() -> None:
    """Real assembly: DEEP_TOKEN + DeepRegistry + uvicorn on the docker0
    gateway. uvicorn is lazy here so build_app stays importable off-GPU."""
    import os
    token = os.environ.get("DEEP_TOKEN")
    if not token:
        raise SystemExit(
            "DEEP_TOKEN env required (scripts/deep_perception.sh sets it)")
    import uvicorn                                  # lazy: the `deep` extra
    from agents.vision.deep.registry import DeepRegistry

    registry = DeepRegistry(
        models_dir=os.environ.get("DEEP_MODELS_DIR", "models"),
        device=os.environ.get("DEEP_DEVICE", "cuda"))
    host = gateway_addr()
    print(f"[deep] binding {host}:{PORT} (docker0 gateway, bearer auth)",
          flush=True)
    uvicorn.run(build_app(registry, token=token), host=host, port=PORT,
                log_level="warning")


if __name__ == "__main__":
    main()
