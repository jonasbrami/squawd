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
import os

_DEFAULT_BOXES = "/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/city_boxes.json"
_FALLBACK = {"spawn_x": 0.0, "spawn_spacing": 3.0, "spawn_z": 0.5, "buildings": []}


class World:
    def __init__(self, path: str | None = None) -> None:
        path = path or os.environ.get("CITY_BOXES", _DEFAULT_BOXES)
        try:
            with open(path) as f:
                self._cfg = json.load(f)
        except Exception:
            self._cfg = dict(_FALLBACK)

    @property
    def buildings(self) -> list[dict]:
        return self._cfg.get("buildings", [])

    @property
    def spawn_x(self) -> float:
        return self._cfg.get("spawn_x", 0.0)

    @property
    def spawn_spacing(self) -> float:
        return self._cfg.get("spawn_spacing", 3.0)

    def drone_state(self, bridge, i: int):
        """(east, north, alt, heading_rad) of drone i in the gz world, or None."""
        p = bridge.latest(f"/px4_{i}/fmu/out/vehicle_local_position")
        if p is None or not getattr(p, "xy_valid", True):
            return None
        east = self.spawn_x + p.y
        north = self.spawn_spacing * i + p.x
        return (east, north, -p.z, float(getattr(p, "heading", 0.0)))

    def world_xy(self, bridge, i: int):
        """(east, north, alt) of drone i, or None if no valid fix."""
        st = self.drone_state(bridge, i)
        return None if st is None else (st[0], st[1], st[2])

    def resolve_xy(self, name: str, bridge, n_drones: int):
        """World (east, north) of a named target: 'drone_<j>' or a building name.
        None if unknown."""
        name = name.strip().lower()
        if name.startswith("drone_"):
            try:
                j = int(name.split("_", 1)[1])
            except ValueError:
                return None
            if 0 <= j < n_drones:
                xy = self.world_xy(bridge, j)
                return None if xy is None else (xy[0], xy[1])
            return None
        for b in self.buildings:
            if b["name"].lower() == name:
                return (b["x"], b["y"])
        return None
