"""perception/deep_client.py — stdlib-only client for the deep sidecar
(deep-perception plan §3). Runs INSIDE the pilot container: http.client only,
no new image dependencies. Frame identity (seq/sim_stamp) rides every call so
the cockpit can join results to the exact submitted frame (codex R3).

Failure semantics (codex B2): 1 s connect cap, per-op read caps (detect 4 s,
segment 6 s). Operational failures come back as typed DeepResult statuses —
UNAVAILABLE (unreachable/timeout), BUSY (429), ERROR (other HTTP) — and never
raise into the pilot. DeepError raises only for protocol violations (malformed
200 payloads) and caller misuse (programming errors).

`frame` args are duck-typed agents.core.contact.Frame (seq/sim_stamp/width/
height/rgb) — imported by NAME only: ICD §0.1 keeps agents/perception
self-contained (the import-rules gate), and every field used is public.
"""
import base64
import http.client
import json
import os
import socket
from dataclasses import dataclass
from urllib.parse import urlsplit

CONNECT_TIMEOUT_S = 1.0
READ_TIMEOUT_DETECT_S = 4.0
READ_TIMEOUT_SEGMENT_S = 6.0

OK, UNAVAILABLE, BUSY, ERROR = "OK", "UNAVAILABLE", "BUSY", "ERROR"

_NUM = (int, float)


class DeepError(Exception):
    """Sidecar protocol violation (schema mismatch) or caller misuse."""


@dataclass(frozen=True)
class DeepResult:
    """One sidecar call's outcome. data is the schema-validated payload on OK,
    None otherwise; detail carries the legible failure reason."""
    status: str                  # OK | UNAVAILABLE | BUSY | ERROR
    data: dict | None = None
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status == OK


def _num(v) -> bool:
    return isinstance(v, _NUM) and not isinstance(v, bool)


def _need(d: dict, key: str, *types):
    if not isinstance(d, dict) or key not in d:
        raise DeepError(f"response missing {key!r}")
    v = d[key]
    if not isinstance(v, types) or (isinstance(v, bool) and bool not in types):
        raise DeepError(f"response {key!r} has wrong type: {type(v).__name__}")
    return v


def _nums(v, n: int, what: str):
    if not isinstance(v, list) or len(v) != n or not all(_num(x) for x in v):
        raise DeepError(f"{what} must be {n} numbers")
    return v


def _validate_detect(d: dict) -> None:
    """Strict /v1/detect schema (plan §1); any deviation -> DeepError."""
    for det in _need(d, "dets", list):
        _need(det, "cls", str)
        if not _num(_need(det, "conf", *_NUM)):
            raise DeepError("det.conf must be a number")
        _nums(_need(det, "xyxy", list), 4, "det.xyxy")
    if not _num(_need(d, "latency_ms", *_NUM)):
        raise DeepError("latency_ms must be a number")
    _need(d, "model", str)
    _need(d, "frame_seq", int)
    if not _num(_need(d, "sim_stamp", *_NUM)):
        raise DeepError("sim_stamp must be a number")


def _validate_segment(d: dict) -> None:
    """Strict /v1/segment schema; empty-segment is all-null fields together."""
    xyxy = _need(d, "xyxy", list, type(None))
    mask = _need(d, "mask", dict, type(None))
    centroid = _need(d, "centroid", list, type(None))
    if (xyxy is None) != (mask is None) or (xyxy is None) != (centroid is None):
        raise DeepError("xyxy/mask/centroid must be all null or all set")
    if xyxy is not None:
        _nums(xyxy, 4, "xyxy")
        _nums(centroid, 2, "centroid")
        _need(mask, "rle", str)
        if not isinstance(_need(mask, "w", int), int) \
                or not isinstance(_need(mask, "h", int), int):
            raise DeepError("mask w/h must be ints")
        try:
            base64.b64decode(mask["rle"], validate=True)
        except Exception as e:
            raise DeepError(f"mask.rle not base64: {e}")
    _need(d, "area_px", int)
    if not _num(_need(d, "score", *_NUM)) \
            or not _num(_need(d, "latency_ms", *_NUM)) \
            or not _num(_need(d, "sim_stamp", *_NUM)):
        raise DeepError("score/latency_ms/sim_stamp must be numbers")
    _need(d, "frame_seq", int)


def _validate_health(d: dict) -> None:
    _need(d, "ok", bool)
    _need(d, "device", str)
    models = _need(d, "models_loaded", list)
    if not all(isinstance(m, str) for m in models):
        raise DeepError("models_loaded must be strings")
    vram = _need(d, "vram_mb", *_NUM, type(None))
    if vram is not None and not _num(vram):
        raise DeepError("vram_mb must be a number or null")


class DeepClient:
    """One sidecar endpoint. base_url/token default to the DEEP_PERCEPTION_URL
    / DEEP_TOKEN env (run_single_demo.sh passes both into the container)."""

    def __init__(self, base_url: str | None = None, token: str | None = None,
                 *, connect_timeout: float = CONNECT_TIMEOUT_S,
                 read_timeout_detect: float = READ_TIMEOUT_DETECT_S,
                 read_timeout_segment: float = READ_TIMEOUT_SEGMENT_S) -> None:
        self.base_url = (base_url
                         or os.environ.get("DEEP_PERCEPTION_URL")
                         or "http://host.docker.internal:8100").rstrip("/")
        self.token = token if token is not None \
            else os.environ.get("DEEP_TOKEN", "")
        self._ct = connect_timeout
        self._rt_detect = read_timeout_detect
        self._rt_segment = read_timeout_segment

    # -- wire --

    @staticmethod
    def _frame_json(frame) -> dict:
        return {"w": frame.width, "h": frame.height,
                "rgb_b64": base64.b64encode(frame.rgb).decode("ascii"),
                "seq": frame.seq, "sim_stamp": frame.sim_stamp}

    def _call(self, method: str, path: str, payload: dict | None,
              read_timeout: float):
        """-> (body: dict, None) | (None, DeepResult non-OK). Never raises for
        operational failures; DeepError only on a malformed 200."""
        u = urlsplit(self.base_url)
        if u.scheme != "http" or not u.hostname:
            raise DeepError(f"base_url must be http://host[:port], got "
                            f"{self.base_url!r}")
        conn = http.client.HTTPConnection(u.hostname, u.port or 80,
                                          timeout=self._ct)
        try:
            conn.request(method, path,
                         body=json.dumps(payload) if payload is not None
                         else None,
                         headers={"Authorization": f"Bearer {self.token}",
                                  "Content-Type": "application/json"})
            conn.sock.settimeout(read_timeout)   # connect cap -> read cap
            resp = conn.getresponse()
            raw = resp.read()
        except (socket.timeout, TimeoutError):
            return None, DeepResult(UNAVAILABLE, detail="timeout")
        except (OSError, http.client.HTTPException) as e:
            return None, DeepResult(UNAVAILABLE, detail=str(e))
        finally:
            conn.close()
        if resp.status == 429:
            return None, DeepResult(BUSY, detail="sidecar busy")
        if resp.status != 200:
            return None, DeepResult(
                ERROR, detail=f"HTTP {resp.status}: {raw[:200]!r}")
        try:
            return json.loads(raw), None
        except Exception as e:
            raise DeepError(f"non-JSON 200 response: {e}")

    # -- API --

    def health(self) -> DeepResult:
        body, fail = self._call("GET", "/v1/health", None, self._ct)
        if fail:
            return fail
        _validate_health(body)
        return DeepResult(OK, body)

    def detect(self, frame, prompts, conf: float = 0.25,
               model: str = "yolo-world-s") -> DeepResult:
        if not isinstance(prompts, (list, tuple)) \
                or not all(isinstance(p, str) for p in prompts):
            raise DeepError("prompts must be a list of strings")
        body, fail = self._call("POST", "/v1/detect", {
            "frame": self._frame_json(frame), "model": model,
            "prompts": list(prompts), "conf": float(conf)}, self._rt_detect)
        if fail:
            return fail
        _validate_detect(body)
        return DeepResult(OK, body)

    def segment(self, frame, points=None, box=None,
                model: str | None = None) -> DeepResult:
        if (points is None) == (box is None):
            raise DeepError("segment needs exactly one of points= or box=")
        payload = {"frame": self._frame_json(frame)}
        if points is not None:
            payload["points"] = [list(p) for p in points]
        else:
            payload["box"] = list(box)
        if model:
            payload["model"] = model
        body, fail = self._call("POST", "/v1/segment", payload,
                                self._rt_segment)
        if fail:
            return fail
        _validate_segment(body)
        return DeepResult(OK, body)
