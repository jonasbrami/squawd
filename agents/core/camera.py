"""GzCameras: the single owner of the per-drone camera feed.

Reads each drone's onboard RGB frame straight off gz-transport (system gz, no
ros_gz) and encodes to JPEG on demand. Both consumers go through here:
- the observatory wants raw JPEG bytes for its MJPEG/WebSocket tiles;
- the detector wants the atomic Frame snapshot (C1) for inference.

Each stored frame carries a monotonically increasing seq AND the gz header's
sim stamp, so a streaming consumer can send/encode only when a NEW frame has
arrived (idle drones cost nothing) and vision can join frames to sim time.
"""
import io
import os

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image as PILImage

from agents.core.contact import Frame, LatestFrame

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
        self._frames: dict[int, LatestFrame] = {}
        self._cbs = []                           # keep callbacks alive for gz
        for i in range(n):
            self._frames[i] = LatestFrame()
            cb = self._make_cb(i)
            self._cbs.append(cb)
            self._node.subscribe(GzImage, topic.format(i=i), cb)

    def _make_cb(self, i: int):
        holder = self._frames[i]

        def cb(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            holder.set(stamp, msg.width, msg.height, bytes(msg.data))
        return cb

    def seq(self, i: int) -> int:
        """Frame counter for drone i (0 if none yet); changes when a new frame arrives."""
        return self._frames[i].seq()

    def has(self, i: int) -> bool:
        return self.seq(i) > 0

    def snapshot(self, i: int) -> Frame | None:
        """THE consumer API (C1): one atomic (seq, sim_stamp, w, h, rgb) view —
        fields never mixed across generations. Detector, VideoHub, accuracy
        tooling all use this."""
        return self._frames[i].get()

    def raw(self, i: int) -> tuple[int, int, bytes] | None:
        """Legacy accessor: latest raw (width, height, rgb_bytes) or None."""
        f = self._frames[i].get()
        return None if f is None else (f.width, f.height, f.rgb)

    def stamp(self, i: int) -> float:
        """Sim-time (s) of the latest frame, 0.0 before any."""
        f = self._frames[i].get()
        return f.sim_stamp if f else 0.0

    def jpeg(self, i: int, quality: int = 55, max_px: int | None = None) -> bytes | None:
        f = self._frames[i].get()
        if f is None:
            return None
        return _encode_jpeg(f.width, f.height, f.rgb, quality, max_px)

    # jpeg_b64 was deleted (ICD v2): its only consumer was the `look` tool,
    # removed in design v4.2. jpeg() stays for off-line accuracy tooling;
    # the cockpit streams H.264 only (ICD §8.2 — no JPEG fallback endpoint).
