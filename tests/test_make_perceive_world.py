"""Perceive-world generator (M5): SDF output + sidecar + the distinct-visual
contract (decoys differ in color AND shape from the true target — the blob
can't separate same-orange decoys) and the p2 crossing stressor."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

from sim.worlds.make_perceive_world import COLOR, MOVERS

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
    dst = tmp_path / "perceive.sdf"
    subprocess.run([sys.executable, str(REPO / "sim/worlds/make_perceive_world.py"),
                    str(src), str(dst)], check=True, capture_output=True)
    return dst.read_text(), json.loads((tmp_path / "perceive_boxes.json").read_text())


def test_world_renamed_and_all_movers_present(generated):
    sdf, _ = generated
    assert '<world name="perceive"' in sdf
    for m in MOVERS:
        assert f'<model name="{m["name"]}"' in sdf
    assert sdf.count("PythonSystemLoader") == len(MOVERS)


def test_sidecar_carries_movers_for_oracle_and_gzposes(generated):
    _, sidecar = generated
    assert sidecar["spawn_x"] == 0.0 and sidecar["buildings"] == []
    names = [m["name"] for m in sidecar["movers"]]
    assert names == ["mov_true", "mov_decoy_red", "mov_decoy_blue"]


def test_initial_pose_matches_trajectory_t0(generated):
    from agents.world.trajectory import pos_xy
    sdf, _ = generated
    for m in MOVERS:
        x0, y0 = pos_xy(m["traj"], 0.0)
        assert f"<pose>{x0:.2f} {y0:.2f} {m['z']:.2f} 0 0 0</pose>" in sdf


def test_decoys_are_visually_distinct_and_ground_bound():
    # distinct COLOR per kind (the blob's orange exists exactly once) ...
    assert len({tuple(c) for c in COLOR.values()}) == len(COLOR)
    shapes = {(m["shape"]["w"], m["shape"]["d"], m["shape"]["h"]) for m in MOVERS}
    assert len(shapes) == len(MOVERS)          # ... and distinct SHAPE
    for m in MOVERS:                           # ground-mover variants only (v1)
        assert m["z"] <= 1.5, f"{m['name']} must stay on the support plane"


def test_decoy_trajectories_cross_the_true_target():
    """The p2 stressor must exist by construction: each decoy comes within 25 m
    of the true target at some point in a 10-minute horizon (same-plaza
    crossings), while the spawn area stays mover-free (>= 60 m from origin)."""
    from agents.world.trajectory import pos_xy
    true = next(m for m in MOVERS if m["name"] == "mov_true")
    for decoy in (m for m in MOVERS if m["name"] != "mov_true"):
        best = min(
            (lambda a, b: ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)(
                pos_xy(true["traj"], k), pos_xy(decoy["traj"], k))
            for k in range(0, 601))
        assert best <= 25.0, f"{decoy['name']} never crosses mov_true (min {best:.0f}m)"
    for m in MOVERS:
        assert min((p[0] ** 2 + p[1] ** 2) ** 0.5
                   for p in (pos_xy(m["traj"], k) for k in range(0, 601))) >= 60.0
