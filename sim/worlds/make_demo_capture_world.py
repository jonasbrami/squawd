"""Demo CAPTURE world (W2.5b): the exact W1b demo cast (make_demo_world —
same movers, landmarks, trees, IMX214 sensor block) plus a scripted lattice
of STATIC capture cameras for the demo-domain detector fine-tune dataset
(codex R5, docs/benchmarks/w3-detector-codex-r5.md). The demo world itself
stays untouched — this is the opt-in capture variant.

No PX4, no drone: boots as a plain gz server (`gz sim -s -r`), movers driven
by the same mover_system plugin (velocity-drive + heading_align), frames +
ground truth harvested off gz-transport by scripts/demo_dataset.py.

Camera lattice: two spec forms —
  _cam(...)      off-road vantage: positioned by bearing/range around an
                 ANCHOR it aims back at (perp cams, pitched oblique/top cams,
                 negative scene cams); yaw_off_deg swings the aim past the
                 anchor to park targets near a horizontal frame edge
  _oncam(...)    ON-ROAD axis cam: sits on a mover's route with an explicit
                 yaw along the travel direction; the mover approaches head-on
                 (front), passes under (brief top-down), recedes (rear) —
                 one cam sweeps the azimuth aspects through every slant band.
                 Cameras have no collision; the kinematic movers pass through.
pitch_deg is down-positive (make_demo_world convention: SDF pitch+ tilts the
+X optical axis DOWN). "target" names the primary subject (mover name) or
"neg:<scene>" for hard-negative vantages (roofs — the W0.1 "chair" trap —
street furniture, empty ground). scripts/demo_dataset.py --plan tallies the
(aspect x band x clip) coverage of this lattice against the R5 cell quotas
BEFORE any gz boot; the lattice below is the product of that loop.

Lattice roles per vehicle (the level-cam frame floor is the 21.07 deg
half-vfov; a frame is bottom-clipped only while the target TOP is still
inside — so high level cams are blind to close targets and the steep cells
need pitched cams):
  on-road axis cams (alts 4/6/10) -> front/rear sweeps in all bands;
    bottom-clips for slant < ~2.6*alt, clean beyond; the alt-10 leg cam
    covers band-22-30 azimuth clips (slant 23.8-27.8) + cleans (27.8-30)
  perp cams (9.5/12 alt 4, 15 alt 6, 24 alt 10) -> side aspects one band
    each (clipped abeam, clean off-abeam); the yaw_off 25 deg cam supplies
    the horizontal edge-clips; along-leg offsets give front/rear-quarter
  elevation fan near one leg (alt 8 p20, alt 14 p12/p30, alt 14 p65/p50) ->
    oblique + top-down aspects; the pitch sets the frame floor so each
    (band x clean/bottom) pair lands a window
NOTE (spec gap): top-down x band-22-30 is geometrically impossible at the
R5 altitudes (slant 22-30 m at dep >= 55 deg needs alt >= 18 m) — those
three cells (one per vehicle) are dropped, reported as a deviation.
Profiles: one boot per vehicle + walkers + negatives (render load: <=13
cams per boot keeps the Intel iGPU at real-time).

Usage: python make_demo_capture_world.py <px4_default.sdf> <out.sdf> [profile]
Sidecar: <out_stem>_boxes.json (movers + buildings + the profile's cameras
with their capture metadata — the dataset script reads it).
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from sim.worlds.make_demo_world import (   # noqa: E402
    LANDMARKS, MESH_LANDMARKS, MOVERS, TREES, buildings, camera_sdf,
    include_sdf, mesh_landmark_sdf, mover_sdf, tree_sdf, _dae_mesh,
    _tree_version)

WORLD = "demo_capture"
PROFILES = ("car_1", "car_2", "car_3", "walkers", "negatives")


def _cam(name: str, target: str, anchor: tuple[float, float], az_deg: float,
         horiz: float, alt: float, pitch_deg: float,
         yaw_off_deg: float = 0.0) -> dict:
    """Off-road capture camera: az_deg is the bearing of the camera POSITION
    from the anchor (0=N, cw+); the camera aims back at the anchor (plus
    yaw_off_deg to shoot past it — horizontal edge-clip transits)."""
    az = math.radians(az_deg)
    x = anchor[0] + horiz * math.sin(az)
    y = anchor[1] + horiz * math.cos(az)
    back = (az_deg + 180.0) % 360.0                 # bearing cam -> anchor
    yaw = math.radians((90.0 - back - yaw_off_deg))  # SDF: 0=E, ccw+
    return {"name": name, "target": target, "anchor": list(anchor),
            "az_deg": az_deg, "horiz": horiz, "alt": alt,
            "pitch_deg": pitch_deg,
            "pose": (round(x, 2), round(y, 2), alt, 0.0,
                     round(math.radians(pitch_deg), 4), round(yaw, 4))}


def _oncam(name: str, target: str, x: float, y: float, alt: float,
           yaw_deg: float, pitch_deg: float = 0.0) -> dict:
    """On-road axis cam at (x, y), yawed along the travel direction (SDF
    degrees: 0=E, 90=N)."""
    return {"name": name, "target": target, "anchor": [x, y], "az_deg": None,
            "horiz": 0.0, "alt": alt, "pitch_deg": pitch_deg,
            "pose": (x, y, alt, 0.0, round(math.radians(pitch_deg), 4),
                     round(math.radians(yaw_deg), 4))}


# Route reference points (make_demo_world.MOVERS trajectories):
#   car_1  40x60 loop x30-70 / y-30..30 (4.0 m/s, 50 s lap), legs E/N/W/S
#   car_2  circle c=(-45,35) r=20 (3.5 m/s, 36 s lap), ccw
#   car_3  35x40 loop x85-120 / y-40..0 (3.0 m/s, 43 s lap)
#   walker_1 back-and-forth x32-68 @ y=-36 (1.3 m/s, 55 s ping-pong)
#   walker_2 rectangle x-71..-19 / y9..61 (1.2 m/s, 173 s lap)
CAPTURE_CAMERAS: dict[str, list[dict]] = {
    "car_1": [
        _oncam("c1_on_s4f", "car_1", 50.0, -30.0, 4.0, 180.0),
        _oncam("c1_on_e6f", "car_1", 70.0, 0.0, 6.0, 270.0),
        _oncam("c1_on_s6r", "car_1", 50.0, -30.0, 6.0, 0.0),
        _oncam("c1_on_n10f", "car_1", 41.0, 30.0, 10.0, 0.0),
        _cam("c1_ne10r", "car_1", (70.0, 30.0), 36.9, 10.0, 10.0, 0.0),
        _oncam("c1_on_w4r", "car_1", 30.0, 0.0, 4.0, 270.0),
        _cam("c1_perp_s9", "car_1", (50.0, -30.0), 180.0, 9.5, 4.0, 0.0),
        _cam("c1_perp_s12", "car_1", (50.0, -30.0), 180.0, 12.0, 4.0, 0.0),
        _cam("c1_perp_s12y", "car_1", (50.0, -30.0), 180.0, 12.0, 4.0, 0.0,
             25.0),
        _cam("c1_perp_e15", "car_1", (70.0, 0.0), 90.0, 15.0, 6.0, 0.0),
        _cam("c1_perp_n24", "car_1", (50.0, 30.0), 0.0, 24.0, 10.0, 0.0),
        _cam("c1_obl8", "car_1", (50.0, -30.0), 140.0, 12.0, 8.0, 20.0),
        _cam("c1_oblA14", "car_1", (50.0, -30.0), 20.0, 20.0, 14.0, 12.0),
        _cam("c1_oblB14", "car_1", (50.0, -30.0), 160.0, 13.0, 14.0, 30.0),
        _cam("c1_topB14", "car_1", (50.0, -30.0), 200.0, 3.0, 14.0, 65.0),
    ],
    "car_2": [
        _oncam("c2_on_s4f", "car_2", -45.0, 15.0, 4.0, 180.0),
        _oncam("c2_on_e6f", "car_2", -25.0, 35.0, 6.0, 270.0),
        _oncam("c2_on_se6r", "car_2", -31.0, 21.0, 6.0, 45.0),
        _oncam("c2_on_n10f", "car_2", -45.0, 55.0, 10.0, 0.0),
        _oncam("c2_on_nw10r", "car_2", -59.0, 49.0, 10.0, 225.0),
        _oncam("c2_on_w4r", "car_2", -65.0, 35.0, 4.0, 270.0),
        _oncam("c2_on_sw4r", "car_2", -59.0, 21.0, 4.0, 135.0),
        _cam("c2_perp_s9", "car_2", (-45.0, 15.0), 180.0, 9.5, 4.0, 0.0),
        _cam("c2_perp_s12", "car_2", (-45.0, 15.0), 180.0, 12.0, 4.0, 0.0),
        _cam("c2_perp_s12y", "car_2", (-45.0, 15.0), 180.0, 12.0, 4.0, 0.0,
             25.0),
        _cam("c2_perp_e15", "car_2", (-25.0, 35.0), 90.0, 15.5, 6.0, 0.0),
        _cam("c2_perp_e18", "car_2", (-25.0, 35.0), 90.0, 18.0, 6.0, 0.0),
        _cam("c2_perp_n24", "car_2", (-45.0, 55.0), 0.0, 24.0, 10.0, 0.0),
        _cam("c2_perp_n25", "car_2", (-45.0, 55.0), 0.0, 25.0, 6.0, 0.0),
        _cam("c2_tan_s10", "car_2", (-45.0, 15.0), 270.0, 30.0, 10.0, 0.0),
        _cam("c2_tan_s4", "car_2", (-45.0, 15.0), 270.0, 13.5, 4.0, 0.0),
        _cam("c2_obl8", "car_2", (-45.0, 15.0), 140.0, 12.0, 8.0, 20.0),
        _cam("c2_oblA14", "car_2", (-45.0, 15.0), 20.0, 20.0, 14.0, 12.0),
        _cam("c2_oblB14", "car_2", (-45.0, 15.0), 160.0, 13.0, 14.0, 30.0),
        _cam("c2_topB14", "car_2", (-45.0, 15.0), 200.0, 3.0, 14.0, 65.0),
    ],
    "car_3": [
        _oncam("c3_on_s4f", "car_3", 102.0, -40.0, 4.0, 180.0),
        _oncam("c3_on_e6f", "car_3", 120.0, -20.0, 6.0, 270.0),
        _oncam("c3_on_s6r", "car_3", 102.0, -40.0, 6.0, 0.0),
        _oncam("c3_on_n10f", "car_3", 90.0, 0.0, 10.0, 0.0),
        _cam("c3_ne10r", "car_3", (120.0, 0.0), 36.9, 10.0, 10.0, 0.0),
        _cam("c3_ne8r", "car_3", (101.0, 0.0), 72.2, 26.2, 8.0, 0.0),
        _oncam("c3_on_w4r", "car_3", 85.0, -20.0, 4.0, 270.0),
        _cam("c3_perp_s9", "car_3", (102.0, -40.0), 180.0, 9.5, 4.0, 0.0),
        _cam("c3_perp_s12", "car_3", (102.0, -40.0), 180.0, 12.0, 4.0, 0.0),
        _cam("c3_perp_s12y", "car_3", (102.0, -40.0), 180.0, 12.0, 4.0, 0.0,
             25.0),
        _cam("c3_perp_e15", "car_3", (120.0, -20.0), 90.0, 15.0, 6.0, 0.0),
        _cam("c3_perp_n24", "car_3", (102.0, 0.0), 0.0, 24.0, 10.0, 0.0),
        _cam("c3_obl8", "car_3", (102.0, -40.0), 140.0, 12.0, 8.0, 20.0),
        _cam("c3_oblA14", "car_3", (102.0, -40.0), 20.0, 20.0, 14.0, 12.0),
        _cam("c3_oblB14", "car_3", (102.0, -40.0), 160.0, 13.0, 14.0, 30.0),
        _cam("c3_topB14", "car_3", (102.0, -40.0), 200.0, 3.0, 14.0, 65.0),
    ],
    "walkers": [
        # --- walker_1 (sidewalk x32-68 @ y=-36, ping-pong E/W, 55 s lap) ---
        _oncam("w1_on4", "walker_1", 50.0, -36.0, 4.0, 0.0),
        _oncam("w1_on6", "walker_1", 58.0, -36.0, 6.0, 180.0),
        _oncam("w1_on4p10", "walker_1", 50.0, -36.0, 4.0, 0.0, 10.0),
        _cam("w1_perp_s7", "walker_1", (50.0, -36.0), 180.0, 7.0, 4.0, 0.0),
        _cam("w1_perp_s7p10", "walker_1", (50.0, -36.0), 180.0, 7.0, 4.0, 10.0),
        _cam("w1_perp_s10y", "walker_1", (50.0, -36.0), 180.0, 10.0, 4.0,
             0.0, 25.0),
        _cam("w1_perp_n16", "walker_1", (50.0, -36.0), 0.0, 16.0, 6.0, 0.0),
        _cam("w1_perp_s15", "walker_1", (50.0, -36.0), 180.0, 15.0, 6.0, 0.0),
        _cam("w1_flat6", "walker_1", (55.0, -36.0), 180.0, 5.0, 6.0, 0.0),
        _cam("w1_obl6", "walker_1", (44.0, -36.0), 58.6, 9.0, 6.0, 30.0),
        _cam("w1_obl6p10", "walker_1", (44.0, -36.0), 58.6, 9.0, 6.0, 10.0),
        _cam("w1_obl4", "walker_1", (50.0, -36.0), 160.0, 5.5, 4.0, 20.0),
        _cam("w1_topA8", "walker_1", (50.0, -36.0), 160.0, 2.0, 8.0, 50.0),
        _cam("w1_topB14", "walker_1", (44.0, -36.0), 180.0, 3.0, 14.0, 65.0),
        _cam("w1_oblB14", "walker_1", (50.0, -36.0), 20.0, 8.0, 14.0, 30.0),
        _cam("w1_topC14", "walker_1", (50.0, -36.0), 180.0, 5.65, 14.0, 45.0),
        _cam("w1_topD14", "walker_1", (50.0, -36.0), 180.0, 4.1, 14.0, 50.0),
        # --- walker_2 (plaza rectangle, 173 s lap) ---
        _oncam("w2_on4", "walker_2", -45.0, 9.0, 4.0, 0.0),
        _cam("w2_oblB14", "walker_2", (-45.0, 35.0), 180.0, 8.0, 14.0, 30.0),
    ],
    "negatives": [
        # hard negatives (codex R5: roofs incl. the W0.1 "chair" trap,
        # street furniture, empty ground, partial/empty frames)
        _cam("neg_house1_roof", "neg:house_1", (49.6, -48.4), 210.0, 16.0,
             10.0, 25.0),
        _cam("neg_house1_obl", "neg:house_1", (49.6, -48.4), 30.0, 22.0,
             6.0, 10.0),
        _cam("neg_house2_roof", "neg:house_2", (19.2, -63.2), 150.0, 14.0,
             9.0, 28.0),
        _cam("neg_gas_canopy", "neg:gas_station", (88.0, 51.2), 250.0, 20.0,
             11.0, 22.0),
        _cam("neg_pines", "neg:trees", (24.0, 10.0), 120.0, 13.0, 4.0, 5.0),
        _cam("neg_oak_lamp", "neg:trees", (108.0, 12.0), 200.0, 12.0, 4.0,
             8.0),
        _cam("neg_road", "neg:road", (50.0, -18.0), 270.0, 9.0, 12.0, 45.0),
        _cam("neg_ground", "neg:ground", (-45.0, 35.0), 40.0, 10.0, 12.0,
             38.0),
    ],
}


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    profile = sys.argv[3] if len(sys.argv) > 3 else "car_1"
    if profile not in PROFILES:
        raise SystemExit(f"profile must be one of {PROFILES}")
    with open(src) as f:
        sdf = f.read()
    sdf = re.sub(r'<world\s+name="[^"]*"', f'<world name="{WORLD}"', sdf,
                 count=1)
    sdf = sdf.replace("<size>100 100</size>", "<size>500 500</size>")

    blocks = [mover_sdf(m) for m in MOVERS]
    for lm in MESH_LANDMARKS:
        mesh = _dae_mesh(lm)
        if mesh:
            blocks.append(mesh_landmark_sdf(lm, mesh))
        else:
            hx, hy, hz = lm["pose"]
            print(f"WARNING: {lm['name']} mesh not in fuel cache, using plain "
                  f"include (renders black headless)", file=sys.stderr)
            blocks.append(include_sdf(
                {"name": lm["name"], "fuel": lm["fuel"],
                 "pose": (hx, hy, hz, 0.0, 0.0, lm["yaw"])}))
    for t in TREES:
        ver = _tree_version(t)
        if ver:
            blocks.append(tree_sdf(t, ver))
        else:
            x, y, z = t["pose"]
            print(f"WARNING: {t['name']} mesh/textures not in fuel cache, "
                  f"using plain include (unscaled ~5 m tree)", file=sys.stderr)
            blocks.append(include_sdf(
                {"name": t["name"], "fuel": t["fuel"],
                 "pose": (x, y, z, 0.0, 0.0, 0.0)}))
    blocks.extend(include_sdf(lm) for lm in LANDMARKS)

    cams = CAPTURE_CAMERAS[profile]
    blocks.extend(camera_sdf({"name": c["name"], "pose": c["pose"]})
                  for c in cams)

    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")
    sdf = sdf[:idx] + "".join(blocks) + "\n  " + sdf[idx:]
    with open(dst, "w") as f:
        f.write(sdf)

    import json
    sidecar = os.path.splitext(dst)[0] + "_boxes.json"
    with open(sidecar, "w") as f:
        json.dump({"world": WORLD, "profile": profile,
                   "buildings": buildings(), "movers": MOVERS,
                   "cameras": cams}, f, indent=2)
    print(f"wrote {dst} (+{len(blocks)} blocks, profile={profile}: "
          f"{len(cams)} capture cams) and {sidecar}")


if __name__ == "__main__":
    main()
