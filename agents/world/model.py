"""World: the ground-truth city layout the agents reason about.

Wraps `city_boxes.json` (written by sim/worlds/make_city_world.py): the exact
building boxes + the drones' spawn layout. Also maps a drone's local PX4 NED
telemetry into the gz world frame, so perception can compute bearing/distance in
pure Python (no extra Gazebo subscriptions) from the same ground truth that
built the world.

Frames: gz world is ENU (+x East, +y North, +z Up). PX4 VehicleLocalPosition is
NED (x=North, y=East, z=Down). Drone i spawns at world (x=0, y=i*spacing, z) with
yaw=0, so axes only swap (no rotation):
    world_East  = spawn_x   + p.y(east)
    world_North = i*spacing + p.x(north)
"""
import json
import math
import os
import threading
from collections import deque

_FALLBACK = {"spawn_x": 0.0, "spawn_spacing": 3.0, "spawn_z": 0.5, "buildings": []}
_BUF_S = 4.0          # W1 ring depth, seconds of pose/attitude history


def _ang_lerp(a: float, b: float, t: float) -> float:
    """Shortest-angle interpolation (never across ±π the long way)."""
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + d * t


def _default_boxes_path() -> str:
    """Per-world sidecar: <world>_boxes.json. 'city' has buildings (written by
    make_city_world.py); 'baylands' has none, so its file is absent and World
    falls back to empty buildings — scan then reports only nearby drones."""
    world = os.environ.get("GZ_WORLD") or os.environ.get("PX4_GZ_WORLD") or "baylands"
    return f"/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/{world}_boxes.json"


class World:
    def __init__(self, path: str | None = None) -> None:
        path = path or os.environ.get("CITY_BOXES") or _default_boxes_path()
        try:
            with open(path) as f:
                self._cfg = json.load(f)
        except Exception:
            self._cfg = dict(_FALLBACK)
        self._buf_lock = threading.Lock()
        self._pose_buf: deque = deque()        # (t, e, n, alt, heading)
        self._att_buf: deque = deque()         # (t, roll, pitch, yaw)

    @property
    def buildings(self) -> list[dict]:
        return self._cfg.get("buildings", [])

    @property
    def movers(self) -> list[dict]:
        """Scripted-mover specs (dynamic worlds only): name/kind/shape/z/traj.
        Live positions come from core.GzPoses, not from here — this is the
        authoring-time ground truth (trajectory params) the oracle cross-checks."""
        return self._cfg.get("movers", [])

    @property
    def spawn_x(self) -> float:
        return self._cfg.get("spawn_x", 0.0)

    @property
    def spawn_spacing(self) -> float:
        return self._cfg.get("spawn_spacing", 3.0)

    def ned_to_enu(self, i: int, x: float, y: float, z: float) -> tuple[float, float, float]:
        """PX4 local NED -> gz world ENU for drone i (axis swap only, yaw=0
        spawn). THE one conversion — drone_state and Px4StateRecorder share it."""
        return (self.spawn_x + y, self.spawn_spacing * i + x, -z)

    def drone_state(self, bridge, i: int):
        """(east, north, alt, heading_rad) of drone i in the gz world, or None."""
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        if p is None or not getattr(p, "xy_valid", True):
            return None
        east, north, alt = self.ned_to_enu(i, p.x, p.y, p.z)
        return (east, north, alt, float(getattr(p, "heading", 0.0)))

    def world_xy(self, bridge, i: int):
        """(east, north, alt) of drone i, or None if no valid fix."""
        st = self.drone_state(bridge, i)
        return None if st is None else (st[0], st[1], st[2])

    def resolve_xy(self, name: str):
        """World (east, north) of a named building, or None if unknown."""
        name = name.strip().lower()
        for b in self.buildings:
            if b["name"].lower() == name:
                return (b["x"], b["y"])
        return None

    # ---- W1: timestamped pose/attitude buffers (fed ONLY by Px4StateRecorder) ----

    def note_pose(self, t: float, e: float, n: float, alt: float,
                  heading: float) -> None:
        with self._buf_lock:
            if self._pose_buf and t < self._pose_buf[0][0] - 1.0:
                # future-dated poison at the head (a boot-transient garbage
                # stamp, see Px4StateRecorder._sim_t): it would defeat
                # _interp's coverage test forever — flush and start clean.
                self._pose_buf.clear()
            self._pose_buf.append((t, e, n, alt, heading))
            while self._pose_buf and self._pose_buf[0][0] < t - _BUF_S:
                self._pose_buf.popleft()

    def note_attitude(self, t: float, roll: float, pitch: float, yaw: float) -> None:
        with self._buf_lock:
            if self._att_buf and t < self._att_buf[0][0] - 1.0:
                self._att_buf.clear()         # future-dated poison (note_pose)
            self._att_buf.append((t, roll, pitch, yaw))
            while self._att_buf and self._att_buf[0][0] < t - _BUF_S:
                self._att_buf.popleft()

    def _interp(self, buf, t: float, angular_idx: frozenset):
        """Linear interpolation at t; channels in angular_idx (tuple positions)
        use shortest-angle. None outside coverage — never extrapolates."""
        with self._buf_lock:
            items = list(buf)
        if not items or t < items[0][0] or t > items[-1][0]:
            return None
        n_ch = len(items[0])
        for k in range(1, len(items)):
            t0, t1 = items[k - 1][0], items[k][0]
            if t0 <= t <= t1:
                f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                out = []
                for c in range(1, n_ch):
                    a, b = items[k - 1][c], items[k][c]
                    if c in angular_idx:
                        out.append(_ang_lerp(a, b, f))
                    else:
                        out.append(a + (b - a) * f)
                return tuple(out)
        return None

    def pose_at(self, t: float) -> tuple[float, float, float, float] | None:
        """(east, north, alt, heading_rad) interpolated at sim-time t, or None."""
        return self._interp(self._pose_buf, t, frozenset({4}))   # heading only

    def attitude_at(self, t: float) -> tuple[float, float, float] | None:
        """(roll, pitch, yaw) interpolated at sim-time t, or None."""
        return self._interp(self._att_buf, t, frozenset({1, 2, 3}))
