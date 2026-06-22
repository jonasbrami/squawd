"""Environment awareness for the drone agents — no SLAM, no depth sensor, no GPU.

Two cheap, VLM-friendly senses, both from data already in the running sim:

- look(i):  the drone's live onboard RGB frame, returned to the agent as an image
            (read straight off gz-transport, exactly like the observatory does).
- scan(i):  a short TEXT readout of the nearest buildings + other drones with
            distance and WORLD-frame bearing, computed in pure Python from the
            ground-truth building boxes (city_boxes.json, written by the world
            generator) and the drone's live ROS telemetry.

Frames: gz world is ENU (+x East, +y North, +z Up). PX4 VehicleLocalPosition is
NED (x=North, y=East, z=Down). Drone i spawns at world (x=0, y=i*spacing, z) with
yaw=0, so axes only swap (no rotation):
    world_East  = spawn_x      + p.y(east)
    world_North = i*spacing    + p.x(north)
We report world-frame bearings (N/E/S/W); we do NOT claim "ahead"/"blocking"
because the subscribed telemetry carries no heading.
"""
import base64
import io
import json
import math
import os
import threading

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image as PILImage

GZ_WORLD = os.environ.get("GZ_WORLD", "city")
CAM_TOPIC = ("/world/" + GZ_WORLD + "/model/x500_depth_{i}/link/OakD-Lite/base_link"
             "/sensor/IMX214/image")
_DEFAULT_BOXES = "/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/city_boxes.json"

_boxes_cache: dict | None = None


def load_boxes(path: str | None = None) -> dict:
    """Load (and cache) the ground-truth world layout written by make_city_world.py."""
    global _boxes_cache
    if _boxes_cache is None:
        path = path or os.environ.get("CITY_BOXES", _DEFAULT_BOXES)
        try:
            with open(path) as f:
                _boxes_cache = json.load(f)
        except Exception:
            _boxes_cache = {"spawn_x": 0.0, "spawn_spacing": 3.0, "spawn_z": 0.5,
                            "buildings": []}
    return _boxes_cache


def drone_world_xy(bridge, i: int):
    """(east, north, alt) of drone i in the gz world frame, or None if no valid fix."""
    cfg = load_boxes()
    p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
    if p is None or not getattr(p, "xy_valid", True):
        return None
    east = cfg.get("spawn_x", 0.0) + p.y
    north = cfg.get("spawn_spacing", 3.0) * i + p.x
    return (east, north, -p.z)


def bearing_word(d_east: float, d_north: float) -> str:
    """8-point compass for a world-frame delta (0deg = North, 90deg = East)."""
    ang = math.degrees(math.atan2(d_east, d_north)) % 360.0
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int((ang + 22.5) // 45) % 8]


def scan_text(bridge, i: int, n_drones: int, k: int = 4) -> str:
    """Nearest k buildings + other drones, with distance and world-frame bearing."""
    cfg = load_boxes()
    me = drone_world_xy(bridge, i)
    if me is None:
        return f"drone_{i}: position not yet available"
    mx, my, alt = me
    parts = []
    ranked = []
    for b in cfg.get("buildings", []):
        dx, dy = b["x"] - mx, b["y"] - my
        radius = 0.5 * math.hypot(b["w"], b["d"])        # approx footprint half-extent
        edge = max(0.0, math.hypot(dx, dy) - radius)     # distance to the wall, not centre
        ranked.append((edge, b, dx, dy))
    ranked.sort(key=lambda t: t[0])
    for edge, b, dx, dy in ranked[:k]:
        parts.append(f"{b['name']} {edge:.0f}m {bearing_word(dx, dy)} (h={b['h']:.0f}m)")
    for j in range(n_drones):
        if j == i:
            continue
        oj = drone_world_xy(bridge, j)
        if oj is None:
            continue
        dx, dy = oj[0] - mx, oj[1] - my
        parts.append(f"drone_{j} {math.hypot(dx, dy):.0f}m {bearing_word(dx, dy)}")
    head = f"drone_{i} at world (E{mx:.0f}, N{my:.0f}), alt {alt:.0f}m. Nearby:"
    return head + (" " + " | ".join(parts) if parts else " nothing close")


def situation_text(bridge, n_drones: int) -> str:
    """Commander-level overview: each drone's position + its single nearest building."""
    cfg = load_boxes()
    lines = []
    for i in range(n_drones):
        me = drone_world_xy(bridge, i)
        if me is None:
            lines.append(f"drone_{i}: (no telemetry)")
            continue
        mx, my, alt = me
        nearest = ""
        if cfg.get("buildings"):
            b = min(cfg["buildings"], key=lambda b: math.hypot(b["x"] - mx, b["y"] - my))
            dx, dy = b["x"] - mx, b["y"] - my
            nearest = f"; nearest {b['name']} {math.hypot(dx, dy):.0f}m {bearing_word(dx, dy)} (h={b['h']:.0f}m)"
        lines.append(f"drone_{i}: world E{mx:.0f} N{my:.0f} alt {alt:.0f}m{nearest}")
    return "\n".join(lines)


class GzLook:
    """Holds the latest RGB frame per drone, read off gz-transport (own Node)."""

    def __init__(self, n: int):
        self._node = GzNode()
        self._lock = threading.Lock()
        self._frames: dict[int, tuple] = {}     # i -> (w, h, raw_rgb_bytes)
        self._cbs = []                           # keep callbacks alive for gz
        for i in range(n):
            cb = self._make_cb(i)
            self._cbs.append(cb)
            self._node.subscribe(GzImage, CAM_TOPIC.format(i=i), cb)

    def _make_cb(self, i: int):
        def cb(msg):
            with self._lock:
                self._frames[i] = (msg.width, msg.height, bytes(msg.data))
        return cb

    def latest_jpeg(self, i: int, quality: int = 50, max_px: int = 768) -> str | None:
        with self._lock:
            frame = self._frames.get(i)
        if not frame:
            return None
        w, h, data = frame
        try:
            img = PILImage.frombytes("RGB", (w, h), data)
            if max(w, h) > max_px:
                scale = max_px / max(w, h)
                img = img.resize((int(w * scale), int(h * scale)))
            buf = io.BytesIO()
            img.save(buf, "JPEG", quality=quality)
            return base64.b64encode(buf.getvalue()).decode("ascii")
        except Exception:
            return None
