"""World: loading city_boxes.json, frame mapping, and target resolution."""
import json
from types import SimpleNamespace

from agents.world import World


def _world(tmp_path):
    boxes = {"spawn_x": 0.0, "spawn_spacing": 3.0, "spawn_z": 0.5,
             "buildings": [{"name": "bldg_0", "x": 50.0, "y": 20.0, "w": 6, "d": 6, "h": 12}]}
    p = tmp_path / "city_boxes.json"
    p.write_text(json.dumps(boxes))
    return World(str(p))


class FakeBridge:
    """Returns a fixed PX4 VehicleLocalPosition-like message for every drone."""
    def __init__(self, x, y, z, heading=0.0):
        self._p = SimpleNamespace(x=x, y=y, z=z, heading=heading, xy_valid=True)

    def latest(self, topic):
        return self._p


def test_missing_file_falls_back_to_empty(tmp_path):
    w = World(str(tmp_path / "nope.json"))
    assert w.buildings == [] and w.spawn_spacing == 3.0


def test_resolve_building_and_unknown(tmp_path):
    w = _world(tmp_path)
    assert w.resolve_xy("bldg_0", FakeBridge(0, 0, -10), 3) == (50.0, 20.0)
    assert w.resolve_xy("bldg_404", FakeBridge(0, 0, -10), 3) is None


def test_drone_state_maps_ned_to_world_enu(tmp_path):
    w = _world(tmp_path)
    # drone_1 spawns at world north = 1*spacing = 3; PX4 NED x=north, y=east, z=down
    east, north, alt, _ = w.drone_state(FakeBridge(x=5.0, y=2.0, z=-10.0), 1)
    assert (east, north, alt) == (2.0, 8.0, 10.0)


def test_resolve_drone_target(tmp_path):
    w = _world(tmp_path)
    assert w.resolve_xy("drone_2", FakeBridge(x=4.0, y=1.0, z=-10.0), 3) == (1.0, 10.0)
    assert w.resolve_xy("drone_9", FakeBridge(x=0, y=0, z=-1), 3) is None
