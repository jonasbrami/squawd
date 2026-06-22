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


# Camera is body-fixed, pointing forward (along heading) with ~69deg horizontal
# FOV (hfov 1.204 rad). Something is "in view" if its bearing relative to the
# drone's heading is within ~half the FOV.
FOV_HALF_DEG = 35.0


def drone_state(bridge, i: int):
    """(east, north, alt, heading_rad) of drone i in the gz world, or None."""
    cfg = load_boxes()
    p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
    if p is None or not getattr(p, "xy_valid", True):
        return None
    east = cfg.get("spawn_x", 0.0) + p.y
    north = cfg.get("spawn_spacing", 3.0) * i + p.x
    return (east, north, -p.z, float(getattr(p, "heading", 0.0)))


def drone_world_xy(bridge, i: int):
    """(east, north, alt) of drone i, or None if no valid fix."""
    st = drone_state(bridge, i)
    return None if st is None else (st[0], st[1], st[2])


def bearing_word(d_east: float, d_north: float) -> str:
    """8-point compass for a world-frame delta (0deg = North, 90deg = East)."""
    ang = math.degrees(math.atan2(d_east, d_north)) % 360.0
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int((ang + 22.5) // 45) % 8]


def heading_word(heading_rad: float) -> str:
    """Compass direction the drone (and its camera) faces."""
    return bearing_word(math.sin(heading_rad), math.cos(heading_rad))


def rel_bearing(d_east: float, d_north: float, heading_rad: float):
    """Bearing of a target relative to where the drone faces.
    Returns (label, in_view, rel_deg). rel_deg: 0=straight ahead, +=right, -=left."""
    world = math.degrees(math.atan2(d_east, d_north))       # 0=N, 90=E
    rel = ((world - math.degrees(heading_rad)) + 180) % 360 - 180
    a = abs(rel)
    if a <= 22.5:
        word = "ahead"
    elif a <= 67.5:
        word = "ahead-right" if rel > 0 else "ahead-left"
    elif a <= 112.5:
        word = "right" if rel > 0 else "left"
    elif a <= 157.5:
        word = "behind-right" if rel > 0 else "behind-left"
    else:
        word = "behind"
    return word, a <= FOV_HALF_DEG, rel


def scan_text(bridge, i: int, n_drones: int, k: int = 4) -> str:
    """Nearest k buildings + other drones, with distance and bearing RELATIVE to
    where the drone faces, flagging what's in the camera's view."""
    cfg = load_boxes()
    st = drone_state(bridge, i)
    if st is None:
        return f"drone_{i}: position not yet available"
    mx, my, alt, hd = st
    parts = []
    ranked = []
    for b in cfg.get("buildings", []):
        dx, dy = b["x"] - mx, b["y"] - my
        radius = 0.5 * math.hypot(b["w"], b["d"])        # approx footprint half-extent
        edge = max(0.0, math.hypot(dx, dy) - radius)     # distance to the wall, not centre
        ranked.append((edge, b, dx, dy))
    ranked.sort(key=lambda t: t[0])
    for edge, b, dx, dy in ranked[:k]:
        word, inview, _ = rel_bearing(dx, dy, hd)
        tag = " [IN VIEW]" if inview else ""
        parts.append(f"{b['name']} {edge:.0f}m {word}{tag} (h={b['h']:.0f}m)")
    for j in range(n_drones):
        if j == i:
            continue
        oj = drone_world_xy(bridge, j)
        if oj is None:
            continue
        dx, dy = oj[0] - mx, oj[1] - my
        word, inview, _ = rel_bearing(dx, dy, hd)
        tag = " [IN VIEW]" if inview else ""
        parts.append(f"drone_{j} {math.hypot(dx, dy):.0f}m {word}{tag}")
    head = (f"drone_{i} at world (E{mx:.0f}, N{my:.0f}), alt {alt:.0f}m, facing "
            f"{heading_word(hd)}. Your camera shows what's 'ahead' / [IN VIEW]; "
            f"turn (face) to bring other things into view. Nearby:")
    return head + (" " + " | ".join(parts) if parts else " nothing close")


def resolve_xy(name: str, bridge, n_drones: int):
    """World (east, north) of a named target: 'drone_<j>' or a building name. None if unknown."""
    name = name.strip().lower()
    if name.startswith("drone_"):
        try:
            j = int(name.split("_", 1)[1])
        except ValueError:
            return None
        if 0 <= j < n_drones:
            xy = drone_world_xy(bridge, j)
            return None if xy is None else (xy[0], xy[1])
        return None
    for b in load_boxes().get("buildings", []):
        if b["name"].lower() == name:
            return (b["x"], b["y"])
    return None


def yaw_deg_to(east_from: float, north_from: float, east_to: float, north_to: float) -> float:
    """Heading (deg, NED: 0=N, +clockwise toward E) to face a world point."""
    return math.degrees(math.atan2(east_to - east_from, north_to - north_from))


def situation_text(bridge, n_drones: int) -> str:
    """Commander-level overview: each drone's position + its single nearest building."""
    cfg = load_boxes()
    lines = []
    for i in range(n_drones):
        st = drone_state(bridge, i)
        if st is None:
            lines.append(f"drone_{i}: (no telemetry)")
            continue
        mx, my, alt, hd = st
        facing = heading_word(hd)
        nearest = ""
        if cfg.get("buildings"):
            b = min(cfg["buildings"], key=lambda b: math.hypot(b["x"] - mx, b["y"] - my))
            dx, dy = b["x"] - mx, b["y"] - my
            nearest = f"; nearest {b['name']} {math.hypot(dx, dy):.0f}m {bearing_word(dx, dy)} (h={b['h']:.0f}m)"
        lines.append(f"drone_{i}: world E{mx:.0f} N{my:.0f} alt {alt:.0f}m facing {facing}{nearest}")
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
