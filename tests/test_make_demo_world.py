"""Demo-world generator (W1b): SDF output + sidecar + the mesh-visual mover
contract (Fuel mesh parsed from the cache when present, colored-box fallback
with a loud warning when not — host CI has no fuel cache), heading_align on
every mover, scaled direct-mesh pines, and spawn clearance."""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

from sim.worlds.make_demo_world import (CAMERAS, LANDMARKS, MESH_LANDMARKS,
                                        MOVERS, TREES)

REPO = Path(__file__).resolve().parents[1]

MINIMAL_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="ground_plane"><static>true</static>
      <link name="link"><visual name="v"><geometry><plane><size>100 100</size></plane></geometry></visual></link>
    </model>
  </world>
</sdf>
"""

FAKE_MODEL_SDF = """<?xml version="1.0" ?>
<sdf version="1.6"><model name="{name}"><static>true</static><link name="link">
  <visual name="visual"><pose>0 0 0 0 0 {yaw}</pose>
    <geometry><mesh><scale>{s} {s} {s}</scale><uri>{uri}</uri></mesh></geometry>
  </visual>
</link></model></sdf>
"""

# TruckDelivery's real model.sdf: no <pose>, no <scale> on the visual.
FAKE_MODEL_SDF_BARE = """<?xml version="1.0" ?>
<sdf version="1.6"><model name="{name}"><static>true</static><link name="link">
  <visual name="visual">
    <geometry><mesh><uri>{uri}</uri></mesh></geometry>
  </visual>
</link></model></sdf>
"""


def _run_generator(src: Path, dst: Path, cache: Path) -> subprocess.CompletedProcess:
    env = dict(os.environ, DEMO_FUEL_MODELS=str(cache))
    return subprocess.run(
        [sys.executable, str(REPO / "sim/worlds/make_demo_world.py"),
         str(src), str(dst)], check=True, capture_output=True, text=True,
        env=env)


def _fake_car(cache: Path, dirname: str, ver: str, mesh: str, scale: float,
              yaw: float) -> None:
    d = cache / dirname / ver
    (d / "meshes").mkdir(parents=True)
    (d / "meshes" / mesh).write_text("fake")
    (d / "model.sdf").write_text(FAKE_MODEL_SDF.format(
        name=dirname, yaw=yaw, s=scale, uri=f"meshes/{mesh}"))


@pytest.fixture()
def generated(tmp_path):
    """No cache: box-visual + house-include fallback path (host CI shape)."""
    src = tmp_path / "default.sdf"
    src.write_text(MINIMAL_SDF)
    dst = tmp_path / "demo.sdf"
    cache = tmp_path / "fuelcache"          # intentionally absent
    res = _run_generator(src, dst, cache)
    return (dst.read_text(), res,
            json.loads((tmp_path / "demo_boxes.json").read_text()))


@pytest.fixture()
def generated_cached(tmp_path):
    """Fake fuel cache: mesh-visual movers + mesh house/tree path."""
    src = tmp_path / "default.sdf"
    src.write_text(MINIMAL_SDF)
    dst = tmp_path / "demo.sdf"
    cache = tmp_path / "fuelcache"
    _fake_car(cache, "hatchback red", "2", "hatchback.obj", 0.0254,
              1.57079632679)
    _fake_car(cache, "suv", "4", "suv.obj", 0.06, -1.57079632679)
    d = cache / "truckdelivery" / "2"       # bare visual: no pose/scale
    (d / "meshes").mkdir(parents=True)
    (d / "meshes" / "truck.obj").write_text("fake")
    (d / "model.sdf").write_text(FAKE_MODEL_SDF_BARE.format(
        name="truckdelivery", uri="meshes/truck.obj"))
    d = cache / "walking person" / "3"      # frozen preferred over animated
    (d / "meshes").mkdir(parents=True)
    (d / "meshes" / "walking.dae").write_text("fake")
    (d / "meshes" / "walking_frozen.dae").write_text("fake")
    for dae, ver in (("house 1", "3"), ("gas station", "3"), ("house 2", "3")):
        d = cache / dae / ver / "meshes"
        d.mkdir(parents=True)
        (d / f"{dae.replace(' ', '_')}.dae").write_text("fake")
    d = cache / "pine tree" / "2"           # dae + both submesh textures
    (d / "meshes").mkdir(parents=True)
    (d / "meshes" / "pine_tree.dae").write_text("fake")
    (d / "materials" / "textures").mkdir(parents=True)
    (d / "materials" / "textures" / "branch_2_diffuse.png").write_text("fake")
    (d / "materials" / "textures" / "bark_diffuse.png").write_text("fake")
    res = _run_generator(src, dst, cache)
    return (dst.read_text(), res,
            json.loads((tmp_path / "demo_boxes.json").read_text()))


def test_world_renamed_ground_widened_and_cast_present(generated):
    sdf, _, _ = generated
    assert '<world name="demo"' in sdf
    assert "<size>500 500</size>" in sdf
    for name in ([m["name"] for m in MOVERS] + [lm["name"] for lm in MESH_LANDMARKS]
                 + [t["name"] for t in TREES] + [lm["name"] for lm in LANDMARKS]
                 + [c["name"] for c in CAMERAS]):
        assert f'<model name="{name}"' in sdf or f"<name>{name}</name>" in sdf
    assert sdf.count("PythonSystemLoader") == len(MOVERS)   # movers only
    assert len(MOVERS) <= 8                       # design W1b: keep it readable


def test_sidecar_schema(generated):
    _, _, sidecar = generated
    assert sidecar["spawn_x"] == 0.0 and sidecar["spawn_spacing"] == 3.0
    names = [m["name"] for m in sidecar["movers"]]
    assert names == ["car_1", "car_2", "car_3", "walker_1", "walker_2"]
    for m in sidecar["movers"]:             # mover_system + World.movers schema
        assert set(m) >= {"name", "kind", "z", "shape", "traj"}
        assert set(m["shape"]) == {"w", "d", "h"}
        assert m["heading_align"] is True   # W1b: yaw chases velocity
    bnames = [b["name"] for b in sidecar["buildings"]]
    assert bnames == ([lm["name"] for lm in MESH_LANDMARKS]
                      + [t["name"] for t in TREES]
                      + [lm["name"] for lm in LANDMARKS])
    for b in sidecar["buildings"]:          # scan/goto/track consumer schema
        assert set(b) >= {"name", "x", "y", "w", "d", "h"}
    assert [c["name"] for c in sidecar["cameras"]] == [c["name"] for c in CAMERAS]


def test_initial_pose_matches_trajectory_t0(generated):
    from agents.world.trajectory import pos_xy, vel_xy
    sdf, _, _ = generated
    for m in MOVERS:
        x0, y0 = pos_xy(m["traj"], 0.0)
        vx, vy = vel_xy(m["traj"], 0.0)
        yaw0 = math.atan2(vy, vx)
        assert (f"<pose>{x0:.2f} {y0:.2f} {m['z']:.2f} 0 0 {yaw0:.4f}</pose>"
                in sdf)


def test_movers_use_fuel_mesh_visuals_when_cached(generated_cached):
    sdf, res, _ = generated_cached
    assert "WARNING" not in res.stderr
    car1 = sdf.split('<model name="car_1">')[1].split("</model>")[0]
    assert "hatchback red/2/meshes/hatchback.obj" in car1
    assert "<scale>0.0254 0.0254 0.0254</scale>" in car1
    assert "file://" in car1                  # absolute cache path, not include
    car2 = sdf.split('<model name="car_2">')[1].split("</model>")[0]
    assert "suv/4/meshes/suv.obj" in car2
    assert "<scale>0.06 0.06 0.06</scale>" in car2


def test_truck_visual_defaults_when_sdf_has_no_pose_or_scale(generated_cached):
    """TruckDelivery's real model.sdf ships neither <pose> nor <scale> on the
    visual — the generator must default to identity rpy / scale 1 (the obj is
    metres, nose +x), not fall back to a box."""
    sdf, _, _ = generated_cached
    car3 = sdf.split('<model name="car_3">')[1].split("</model>")[0]
    assert "truckdelivery/2/meshes/truck.obj" in car3
    assert "<scale>1.0 1.0 1.0</scale>" in car3
    assert "<pose>0 0 0 0.0000 0.0000 0.0000</pose>" in car3
    assert car3.count("<box>") == 1    # the collision; the visual is the mesh


def test_walkers_use_frozen_mesh_with_nose_yaw_offset(generated_cached):
    """Walkers are STATIC mesh visuals (no actors): walking_frozen.dae
    preferred, visual yaw +pi/2 (the dae faces -y at identity; mover heading
    convention is nose=+x for heading_align)."""
    sdf, _, _ = generated_cached
    for w in ("walker_1", "walker_2"):
        blk = sdf.split(f'<model name="{w}">')[1].split("</model>")[0]
        assert "walking person/3/meshes/walking_frozen.dae" in blk
        assert f"<pose>0 0 0 0.0000 0.0000 {math.pi / 2:.4f}</pose>" in blk
        assert blk.count("<box>") == 1   # the collision; visual is the mesh
        assert "<actor>" not in sdf


def test_pines_are_scaled_direct_mesh_not_include(generated_cached):
    """W1b: pines scale x2.4 (~12 m street trees) as direct-mesh submesh
    visuals with the Fuel pbr-albedo textures (include has no scale)."""
    sdf, res, _ = generated_cached
    assert "WARNING" not in res.stderr
    assert "models/Pine Tree</uri>" not in sdf      # no plain include anymore
    for t in TREES:
        blk = sdf.split(f'<model name="{t["name"]}">')[1].split("</model>")[0]
        assert "pine tree/2/meshes/pine_tree.dae" in blk
        assert "<scale>2.4 2.4 2.4</scale>" in blk
        assert "<name>Branch</name>" in blk and "<name>Bark</name>" in blk
        assert "branch_2_diffuse.png" in blk and "bark_diffuse.png" in blk
        assert "<script>" not in blk                # the black-render class
        assert "<collision" in blk                  # slim trunk box


def test_mesh_landmarks_are_mesh_with_plain_material_not_include(generated_cached):
    sdf, _, _ = generated_cached
    assert "models/House 1</uri>" not in sdf  # the include renders black headless
    assert "models/House 2</uri>" not in sdf
    assert "models/Gas Station</uri>" not in sdf
    house = sdf.split('<model name="house_1">')[1].split("</model>")[0]
    assert "house_1.dae" in house and "<scale>1.5 1.5 1.5</scale>" in house
    assert "<ambient>0.55 0.5 0.45 1</ambient>" in house   # plain material
    assert "<collision" in house                            # drone can't clip through
    house2 = sdf.split('<model name="house_2">')[1].split("</model>")[0]
    assert "house_2.dae" in house2 and "<scale>1.5 1.5 1.5</scale>" in house2
    assert "<ambient>0.5 0.46 0.42 1</ambient>" in house2
    assert "<collision" in house2
    gas = sdf.split('<model name="gas_station">')[1].split("</model>")[0]
    assert "gas_station.dae" in gas and "<scale>1.0 1.0 1.0</scale>" in gas
    assert "<ambient>0.6 0.56 0.5 1</ambient>" in gas
    assert "<collision" in gas


def test_oaks_are_fuel_includes(generated_cached):
    """The oak shares the pine's script+pbr structure that renders correctly
    headless as an include — and at 10.6x8.5x6.6 m needs no scaling."""
    sdf, _, _ = generated_cached
    assert sdf.count("models/Oak Tree</uri>") == 2


def test_gas_station_sidecar_center_is_yaw_rotated(generated_cached):
    """center_offset is LOCAL-frame; the sidecar x/y must be the world center
    (yaw pi maps local (0,-9.18) -> world (0,+9.18) on pose (88,42)).
    Numbers are the transform-aware measurements (W1b correction — the W1a
    raw-bounds pins (50.68 / 17.5,25.5,7.7) sat off the rendered mesh)."""
    _, _, sidecar = generated_cached
    gas = next(b for b in sidecar["buildings"] if b["name"] == "gas_station")
    assert gas["x"] == 88.0 and gas["y"] == 51.18
    assert (gas["w"], gas["d"], gas["h"]) == (20.6, 30.0, 9.0)


def test_fallback_box_visuals_warn_without_cache(generated):
    sdf, res, _ = generated
    assert "WARNING" in res.stderr
    car1 = sdf.split('<model name="car_1">')[1].split("</model>")[0]
    assert "<box><size>4.00 2.14 1.57</size></box>" in car1
    assert "<diffuse>0.75 0.1 0.08 1</diffuse>" in car1    # the red fallback
    assert "models/House 1</uri>" in sdf                   # include fallback
    assert "models/House 2</uri>" in sdf
    assert "models/Gas Station</uri>" in sdf
    assert "models/Pine Tree</uri>" in sdf                 # unscaled fallback
    walker = sdf.split('<model name="walker_1">')[1].split("</model>")[0]
    assert "<box>" in walker                               # person-sized box


def test_movers_stay_clear_of_spawn_and_each_other():
    """Drones spawn at (0, i*3): every mover keeps >=20 m from the origin
    corridor over a 10-minute horizon; no two movers ever overlap (>=3 m —
    walkers share the street scene with the cars by design); the two original
    cars keep their 20 m berth."""
    from agents.world.trajectory import pos_xy
    for m in MOVERS:
        best = min((p[0] ** 2 + p[1] ** 2) ** 0.5
                   for p in (pos_xy(m["traj"], k) for k in range(0, 601)))
        assert best >= 20.0, f"{m['name']} comes within {best:.0f}m of spawn"
    import itertools
    for a, b in itertools.combinations(MOVERS, 2):
        best = min((lambda u, v: ((u[0] - v[0]) ** 2 + (u[1] - v[1]) ** 2) ** 0.5)(
            pos_xy(a["traj"], k), pos_xy(b["traj"], k)) for k in range(0, 601))
        assert best >= 3.0, f"{a['name']}~{b['name']} come within {best:.1f}m"
    a, b = MOVERS[0], MOVERS[1]
    best = min((lambda u, v: ((u[0] - v[0]) ** 2 + (u[1] - v[1]) ** 2) ** 0.5)(
        pos_xy(a["traj"], k), pos_xy(b["traj"], k)) for k in range(0, 601))
    assert best >= 20.0, f"cars come within {best:.0f}m of each other"


def test_house_sits_clear_of_car1_loop():
    """The house must not intersect car_1's street loop (visual sanity)."""
    house = MESH_LANDMARKS[0]
    hx, hy, _ = house["pose"]
    ox, oy = house["center_offset"]
    cx, cy = hx + ox, hy + oy
    w, d, _ = house["dims"]
    pts = MOVERS[0]["traj"]["pts"]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    assert (cx + w / 2 < min(xs) or cx - w / 2 > max(xs)
            or cy + d / 2 < min(ys) or cy - d / 2 > max(ys))
