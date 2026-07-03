"""Dynamic-world generator: SDF output + sidecar + region-separation guarantee."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

from sim.worlds.make_dynamic_world import MOVERS, check_min_separation

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


@pytest.fixture()
def generated(tmp_path):
    src = tmp_path / "default.sdf"
    src.write_text(MINIMAL_SDF)
    dst = tmp_path / "dynamic.sdf"
    subprocess.run([sys.executable, str(REPO / "sim/worlds/make_dynamic_world.py"),
                    str(src), str(dst)], check=True, capture_output=True)
    return dst.read_text(), json.loads((tmp_path / "dynamic_boxes.json").read_text())


def test_world_renamed_and_all_movers_present(generated):
    sdf, _ = generated
    assert '<world name="dynamic"' in sdf
    for m in MOVERS:
        assert f'<model name="{m["name"]}"' in sdf
    assert sdf.count("PythonSystemLoader") == len(MOVERS)
    assert "<static>false</static>" in sdf
    assert "<gravity>false</gravity>" in sdf


def test_sidecar_has_spawn_movers_and_empty_buildings(generated):
    _, sidecar = generated
    assert sidecar["spawn_x"] == 0.0 and sidecar["buildings"] == []
    assert [m["name"] for m in sidecar["movers"]] == [m["name"] for m in MOVERS]
    for m in sidecar["movers"]:
        assert m["kind"] in ("target", "obstacle")
        assert m["traj"]["type"] in ("line", "waypoint_loop", "circle")


def test_initial_pose_matches_trajectory_t0(generated):
    from agents.world.trajectory import pos_xy
    sdf, _ = generated
    for m in MOVERS:
        x0, y0 = pos_xy(m["traj"], 0.0)
        assert f"<pose>{x0:.2f} {y0:.2f} {m['z']:.2f} 0 0 0</pose>" in sdf


def test_layout_keeps_regions_separated():
    check_min_separation(MOVERS, min_m=40.0)


def test_separation_check_catches_collisions():
    close = [
        {"name": "a", "traj": {"type": "line", "p0": [0, 0], "p1": [100, 0],
                               "speed_mps": 5.0}},
        {"name": "b", "traj": {"type": "line", "p0": [100, 0], "p1": [0, 0],
                               "speed_mps": 5.0}},
    ]
    with pytest.raises(SystemExit, match="come within"):
        check_min_separation(close)
