"""Demo-capture-world generator + demo_dataset labeler (W2.5b): SDF/sidecar
contract, capture-camera pose math, the exact-pinhole labeler (rotation by
the truth quaternion — codex R5), aspect/band/clip binning, the 70/15/15
time-block split, and the quota binder. Host-only (no gz): mirrors
test_make_demo_world.py's fixture pattern; the labeler math is pure.
"""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

from sim.worlds.make_demo_capture_world import (
    CAPTURE_CAMERAS, PROFILES, WORLD, _cam, _oncam)

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

sys.path.insert(0, str(REPO / "scripts"))
import demo_dataset as dd  # noqa: E402


@pytest.fixture()
def generated(tmp_path):
    src = tmp_path / "default.sdf"
    src.write_text(MINIMAL_SDF)
    dst = tmp_path / "demo_capture.sdf"
    cache = tmp_path / "fuelcache"          # intentionally absent
    env = dict(os.environ, DEMO_FUEL_MODELS=str(cache))
    res = subprocess.run(
        [sys.executable,
         str(REPO / "sim/worlds/make_demo_capture_world.py"),
         str(src), str(dst), "car_1"], check=True, capture_output=True,
        text=True, env=env)
    return (dst.read_text(), res,
            json.loads((tmp_path / "demo_capture_boxes.json").read_text()))


# ---- generator contract ----------------------------------------------------

def test_world_renamed_and_profile_cameras_present(generated):
    sdf, _, sidecar = generated
    assert f'<world name="{WORLD}"' in sdf
    assert "<size>500 500</size>" in sdf
    for name in [m["name"] for m in sidecar["movers"]]:
        assert f'<model name="{name}"' in sdf
    cams = CAPTURE_CAMERAS["car_1"]
    for c in cams:
        assert f'<model name="{c["name"]}">' in sdf
    # no evidence cams from the demo world, only the profile's capture cams
    assert sdf.count('sensor name="IMX214"') == len(cams)
    assert sidecar["profile"] == "car_1"
    assert [c["name"] for c in sidecar["cameras"]] == [c["name"] for c in cams]
    assert sdf.count("PythonSystemLoader") == len(sidecar["movers"])


def test_all_profiles_generate(tmp_path):
    src = tmp_path / "default.sdf"
    src.write_text(MINIMAL_SDF)
    cache = tmp_path / "fuelcache"
    env = dict(os.environ, DEMO_FUEL_MODELS=str(cache))
    for profile in PROFILES:
        dst = tmp_path / f"cap_{profile}.sdf"
        subprocess.run(
            [sys.executable,
             str(REPO / "sim/worlds/make_demo_capture_world.py"),
             str(src), str(dst), profile], check=True, capture_output=True,
            text=True, env=env)
        sidecar = json.loads(
            (tmp_path / f"cap_{profile}_boxes.json").read_text())
        assert sidecar["profile"] == profile
        got = [(c["name"], c["target"], tuple(c["pose"]))
               for c in sidecar["cameras"]]
        want = [(c["name"], c["target"], c["pose"])
                for c in CAPTURE_CAMERAS[profile]]
        assert got == want


def test_cam_spec_pose_aims_back_at_anchor():
    c = _cam("x", "car_1", (50.0, -30.0), 180.0, 10.0, 4.0, 0.0)
    x, y, z, roll, pitch, yaw = c["pose"]
    assert (x, y, z) == (50.0, -40.0, 4.0)         # 10 m south of the anchor
    assert roll == 0.0 and pitch == 0.0
    # looking north at the anchor: SDF yaw +90 deg (0=E, ccw+)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-3)
    on = _oncam("y", "car_1", 50.0, -30.0, 4.0, 180.0)
    assert on["pose"] == (50.0, -30.0, 4.0, 0.0, 0.0,
                          round(math.pi, 4))


# ---- exact camera model -----------------------------------------------------

def _ref_project(cam_pose, px, py, pz):
    """Reference pinhole in the w0_assets_eval.project convention (yaw/pitch
    form): pitch-blind-free, F*(lat/f) — the eval-validated math."""
    cx, cy, cz, _, pitch, yaw = cam_pose
    fpx = 320.0 / math.tan(1.204 / 2.0)
    dx, dy = px - cx, py - cy
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
    dz = pz - cz
    f = fwd * math.cos(pitch) - dz * math.sin(pitch)
    h = fwd * math.sin(pitch) + dz * math.cos(pitch)
    if f <= 0.05:
        return None
    return (320.0 - fpx * lat / f, 180.0 - fpx * h / f)


def test_cam_projection_matches_eval_reference():
    # pitched static cam (alt 12, 30 deg down, yawed): exact agreement with
    # the W0.1 eval harness's full-3D projection on a grid of world points
    pose = (50.0, -40.0, 12.0, 0.0, math.radians(30.0), math.radians(70.0))
    cam = dd.Cam(*pose[0:3], pose[4], pose[5])
    for px in (45.0, 50.0, 55.0, 60.0):
        for py in (-30.0, -25.0, -20.0):
            for pz in (0.0, 1.0):
                got = cam.project(px, py, pz)
                want = _ref_project(pose, px, py, pz)
                if want is None:
                    assert got is None
                    continue
                assert got[0] == pytest.approx(want[0], abs=0.6)
                assert got[1] == pytest.approx(want[1], abs=0.6)


def test_cam_right_vector_not_mirrored():
    # facing north (yaw +90 deg), a point EAST of boresight must land on the
    # RIGHT half of the image (u > cx) — the up x f sign bug mirrored this
    cam = dd.Cam(50.0, -40.0, 4.0, 0.0, math.pi / 2)
    u, v = cam.project(53.0, -30.0, 1.0)
    assert u > dd.CX


# ---- labeling: rotation, bins, clips ---------------------------------------

def _yaw_quat(yaw_e):
    return (math.cos(yaw_e / 2), 0.0, 0.0, math.sin(yaw_e / 2))


def test_label_corners_rotate_with_truth_yaw():
    """codex R5: the cuboid corners rotate by the truth quaternion — a car
    heading NORTH (yaw 90) viewed from the south is end-on (2.14 m wide),
    where vision_dataset's axis-aligned box would stay 4.0 m wide."""
    cam = dd.Cam(50.0, -50.0, 4.0, 0.0, math.pi / 2)     # looking north
    shape = {"w": 4.0, "d": 2.14, "h": 1.57}
    pos = (50.0, -30.0, 0.0)
    straight = dd.label_mover(cam, pos, _yaw_quat(0.0), shape)[0]
    yawed = dd.label_mover(cam, pos, _yaw_quat(math.pi / 2), shape)[0]
    w_straight = max(p[0] for p in straight) - min(p[0] for p in straight)
    w_yawed = max(p[0] for p in yawed) - min(p[0] for p in yawed)
    assert w_yawed < w_straight * 0.7       # end-on extent, not axis box


def test_label_clip_classification():
    shape = {"w": 4.0, "d": 2.14, "h": 1.57}
    # level alt-4 cam 20 m north of the target: fully inside -> clean
    cam = dd.Cam(50.0, -50.0, 4.0, 0.0, math.pi / 2)
    _, clip = dd.label_mover(cam, (50.0, -30.0, 0.0), _yaw_quat(0.0), shape)
    assert clip == "clean"
    # 9.5 m north: the bottom of the car drops below the 21.07 deg frame
    # floor -> bottom-clipped (the pursuit frame-floor case)
    cam = dd.Cam(50.0, -39.5, 4.0, 0.0, math.pi / 2)
    _, clip = dd.label_mover(cam, (50.0, -30.0, 0.0), _yaw_quat(0.0), shape)
    assert clip == "bottom"
    # far right at the frame edge -> horizontally edge-clipped
    cam = dd.Cam(50.0, -50.0, 4.0, 0.0, math.pi / 2)
    _, clip = dd.label_mover(cam, (63.0, -31.0, 0.0), _yaw_quat(0.0), shape)
    assert clip == "edge"
    # fully below the floor: no label at all (not a mislabeled frame)
    cam = dd.Cam(50.0, -36.0, 14.0, 0.0, math.pi / 2)
    assert dd.label_mover(cam, (50.0, -30.0, 0.0), _yaw_quat(0.0),
                          shape) is None


def test_aspect_and_band_bins():
    assert dd.aspect_of(0.0, 10.0) == "front"
    assert dd.aspect_of(44.9, 10.0) == "front"
    assert dd.aspect_of(90.0, 10.0) == "side"
    assert dd.aspect_of(125.0, 10.0) == "rear-quarter"
    assert dd.aspect_of(179.0, 10.0) == "rear"
    assert dd.aspect_of(0.0, 40.0) == "oblique"     # depression overrides
    assert dd.aspect_of(120.0, 40.0) == "oblique"
    assert dd.aspect_of(0.0, 60.0) == "top-down"
    assert dd.band_of(10.0, dd.BANDS_VEH) == "10-15"
    assert dd.band_of(21.99, dd.BANDS_VEH) == "15-22"
    assert dd.band_of(30.0, dd.BANDS_VEH) is None
    assert dd.band_of(9.99, dd.BANDS_PER) == "6-10"
    assert dd.band_of(5.0, dd.BANDS_PER) is None


def test_split_blocks_are_70_15_15_and_deterministic():
    assert dd.split_of(0.0) == "train"
    assert dd.split_of(279.9) == "train"          # 14th 20 s block edge
    assert dd.split_of(280.0) == "val"
    assert dd.split_of(340.0) == "test"
    assert dd.split_of(400.0) == "train"          # wraps every 20 blocks
    counts = {"train": 0, "val": 0, "test": 0}
    for k in range(2000):                          # 2000 x 1 s
        counts[dd.split_of(float(k))] += 1
    assert counts["train"] == 1400
    assert counts["val"] == 300
    assert counts["test"] == 300


def test_coco_class_map_and_names():
    assert len(dd.COCO_NAMES) == 80
    assert dd.COCO_NAMES[0] == "person"
    assert dd.COCO_NAMES[2] == "car"
    assert dd.COCO_NAMES[7] == "truck"
    assert dd.COCO_MAP == {"car_1": 2, "car_2": 2, "car_3": 7,
                           "walker_1": 0, "walker_2": 0}


# ---- quota binder -----------------------------------------------------------

def _lab(cell, clip="clean", alt=4):
    return {"cell": cell, "clip": clip, "alt": alt}


def test_binder_fills_cell_quotas_then_stops():
    b = dd.Binder()
    cell = ("car_1", "front", "10-15")
    for _ in range(dd.VEH_QUOTA["clean"]):
        assert b.needs(_lab(cell, "clean"))
        b.record(_lab(cell, "clean"))
    assert not b.needs(_lab(cell, "clean"))       # clean quota full
    assert b.needs(_lab(cell, "bottom"))          # other clip still open
    assert not b.filled("car_1")                  # other cells open


def test_binder_completion_by_profile():
    b = dd.Binder()
    for asp in dd.ASPECTS:
        for band in ("10-15", "15-22", "22-30"):
            if ("car_1", asp, band) in dd.IMPOSSIBLE_CELLS:
                continue
            for clip in dd.CELL_HARD:
                for _ in range(dd.VEH_QUOTA[clip]):
                    b.record(_lab(("car_1", asp, band), clip))
    assert not b.filled("car_1")                  # edge pool still open
    b.edge_frames = dd.EDGE_POOL["car_1"]
    assert b.filled("car_1")
    assert not b.filled("walkers")                # person cells untouched


def test_binder_alt_soft_cap():
    # multi-alt cell, even replay shares: plain 60% cap per altitude
    b = dd.Binder(feas={"car_1|front|10-15": {4: 30, 6: 30}})
    cell = ("car_1", "front", "10-15")
    cap = math.ceil(sum(dd.VEH_QUOTA.values()) * dd.ALT_CAP_FRAC)
    for _ in range(dd.VEH_QUOTA["clean"]):
        b.record(_lab(cell, "clean", alt=4))
    for _ in range(cap - dd.VEH_QUOTA["clean"]):
        b.record(_lab(cell, "bottom", alt=4))   # alt 4 now at the 60% cap
    assert not b.needs(_lab(cell, "bottom", alt=4))   # alt saturated
    assert b.needs(_lab(cell, "bottom", alt=6))       # another alt still ok


def test_binder_alt_soft_cap_single_alt_cell():
    # effectively single-alt cell (steep/pitched-cam cells): the cap floors
    # at the alt's replay share (~the full quota) so the cell still fills
    # (car_2 boot 2-5 stall: byproduct-inflated altcounts idled open cells).
    b = dd.Binder(feas={"car_1|front|10-15": {14: 100}})
    cell = ("car_1", "front", "10-15")
    old_cap = math.ceil(sum(dd.VEH_QUOTA.values()) * dd.ALT_CAP_FRAC)
    for _ in range(dd.VEH_QUOTA["clean"]):
        b.record(_lab(cell, "clean", alt=14))
    for _ in range(old_cap - dd.VEH_QUOTA["clean"]):
        b.record(_lab(cell, "bottom", alt=14))
    assert b.needs(_lab(cell, "bottom", alt=14))   # 60% cap would stall here


def test_binder_alt_budget_not_spent_by_overquota_clips():
    # a saved frame records EVERY visible label; clips already at quota
    # (byproduct overshoot) must not consume the cell's alt budget, else
    # still-open clips of the same cell deadlock at the cap
    # (person|rear-quarter|6-10: bottom 33/10 ate alt4's cap 48 with clean
    # 12/24 stranded, walkers boot stall 2026-08-02)
    b = dd.Binder(feas={"person|rear-quarter|6-10": {4: 100}})
    cell = ("person", "rear-quarter", "6-10")
    for _ in range(12):
        b.record(_lab(cell, "clean", alt=4))
    for _ in range(40):
        b.record(_lab(cell, "bottom", alt=4))   # byproduct overshoot
    assert b.needs(_lab(cell, "clean", alt=4))   # alt budget intact


def test_planner_smoke_runs():
    # short window: exercises the full analytic-truth label path; deficits
    # are expected at 5 s (returns 1), sufficiency is a boot-time check
    assert dd.plan("car_1", 5.0, dt=0.5) in (0, 1)
