"""GzCameras: the single owner of the per-drone camera feed.

Reads each drone's onboard RGB frame straight off gz-transport (system gz, no
ros_gz) and encodes to JPEG on demand. Both consumers go through here:
- the observatory wants raw JPEG bytes for its MJPEG/WebSocket tiles;
- a drone agent's `look` tool wants a downscaled base64 JPEG for the VLM.

Each stored frame carries a monotonically increasing seq so a streaming
consumer can send/encode only when a NEW frame has arrived (idle drones cost
nothing).
"""
import base64
import io
import os
import threading

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image as PILImage

# World name must match the generated world (make_city_world.py names it 'city'
# and PX4 runs with PX4_GZ_WORLD=city). Override with GZ_WORLD if you change it.
GZ_WORLD = os.environ.get("GZ_WORLD", "city")
CAM_TOPIC = ("/world/" + GZ_WORLD + "/model/x500_depth_{i}/link/OakD-Lite/base_link"
             "/sensor/IMX214/image")


def _encode_jpeg(w: int, h: int, data: bytes, quality: int, max_px: int | None) -> bytes | None:
    try:
        img = PILImage.frombytes("RGB", (w, h), data)
        if max_px and max(w, h) > max_px:
            scale = max_px / max(w, h)
            img = img.resize((int(w * scale), int(h * scale)))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=quality)
        return buf.getvalue()
    except Exception:
        return None


class GzCameras:
    """Holds the latest RGB frame per drone, read off gz-transport (own Node)."""

    def __init__(self, n: int, world: str | None = None) -> None:
        topic = CAM_TOPIC if world is None else (
            f"/world/{world}/model/x500_depth_{{i}}/link/OakD-Lite/base_link/sensor/IMX214/image")
        self._node = GzNode()
        self._lock = threading.Lock()
        self._frames: dict[int, tuple] = {}     # i -> (seq, w, h, raw_rgb_bytes)
        self._cbs = []                           # keep callbacks alive for gz
        for i in range(n):
            cb = self._make_cb(i)
            self._cbs.append(cb)
            self._node.subscribe(GzImage, topic.format(i=i), cb)

    def _make_cb(self, i: int):
        def cb(msg):
            with self._lock:
                prev = self._frames.get(i)
                seq = (prev[0] + 1) if prev else 1
                self._frames[i] = (seq, msg.width, msg.height, bytes(msg.data))
        return cb

    def seq(self, i: int) -> int:
        """Frame counter for drone i (0 if none yet); changes when a new frame arrives."""
        with self._lock:
            f = self._frames.get(i)
        return f[0] if f else 0

    def has(self, i: int) -> bool:
        return self.seq(i) > 0

    def raw(self, i: int) -> tuple[int, int, bytes] | None:
        """Latest raw (width, height, rgb_bytes) for drone i, or None. The
        observatory's H.264 encoder needs raw RGB; the VLM still uses jpeg()."""
        with self._lock:
            f = self._frames.get(i)
        if not f:
            return None
        _, w, h, data = f
        return (w, h, data)

    def jpeg(self, i: int, quality: int = 55, max_px: int | None = None) -> bytes | None:
        with self._lock:
            f = self._frames.get(i)
        if not f:
            return None
        _, w, h, data = f
        return _encode_jpeg(w, h, data, quality, max_px)

    def jpeg_b64(self, i: int, quality: int = 50, max_px: int = 768) -> str | None:
        raw = self.jpeg(i, quality=quality, max_px=max_px)
        return None if raw is None else base64.b64encode(raw).decode("ascii")
