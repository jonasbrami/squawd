"""Assets world = default.sdf (flat) + the W0.1 validation cast: the demo
world's Fuel objects rendered at KNOWN engagement geometries, watched by
STATIC cameras that replicate the x500_depth onboard camera (design
2026-07-28 §2 item 1 — the COCO-on-Fuel domain-gap gate, BEFORE the demo
world is built).

Camera replica (sim/models/x500_depth/model.sdf + OakD-Lite/model.sdf):
  IMX214 sensor: horizontal_fov 1.204 rad, 640x360 (swarm_sim.sh CAM_W/H),
  far clip 100 m, NO downward tilt (rpy 0 0 0 on the airframe). Each static
  camera below keeps that sensor block verbatim; only the model pose differs.

Geometry — linear along +X, one row per range, one camera pair per row
(nothing between a camera and its row => no self-occlusion; other rows sit
behind the camera or far in the background):
  cam10_low/high at x=0   -> row at x=10  (10 m),  cast front-facing (yaw pi)
  cam25_low/high at x=15  -> row at x=40  (25 m),  cast side-facing (yaw pi/2)
  cam40_low/high at x=55  -> row at x=95  (40 m),  cast front-facing (yaw pi)
  cam_house at x=-15 yaw pi -> House 1 at x=-40 (~30 m, negative check)
low  = z=3,  pitch 0     (drone in 3 m hold — the EXACT production geometry)
high = z=12, pitch 0.55  (drone at 12 m: with the natural untilted camera the
                          10/25 m rows are BELOW the frame — only >=40 m is
                          visible; so the near high cameras are pitched down
                          31.5 deg to emulate a look-down engagement, while
                          cam40_high keeps pitch 0 as the faithful 12 m case)
Per-row Y from bearings -28,-14,0,+14,+28 deg off the camera axis (inside
the +/-34.5 deg half-hfov at every range).

Cast per row (order = bearing order): Hatchback, SUV, TruckDelivery,
TinyRobot, Walking person. The walker renders as a STATIC model whose visual
is the Fuel "Walking person" walking.dae mesh (frozen bind pose) — NOT an
<actor>: actors stall the headless llvmpipe render thread outright (found
2026-08-01 — even with <library_animations> stripped (26 MB -> 4 MB) the sim
pegged one core for 3.5+ min and never published /clock; includes-only and
camera-only worlds are healthy). The mesh path is resolved from the
DOWNLOADED fuel cache (swarm_sim.sh `assets` branch downloads first); if
absent the walker is skipped with a warning so the rest of the gate still
runs. A frozen pose is also what the demo design plans for W1b.

Usage: python make_assets_world.py <px4_default.sdf> <out_assets.sdf>
"""
import glob
import json
import math
import os
import re
import sys

# --- sensor replica (OakD-Lite IMX214, post swarm_sim.sh 640x360 patch) ----
HFOV = 1.204
CAM_W, CAM_H = 640, 360
SENSOR = f"""<sensor name="IMX214" type="camera">
          <camera>
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{CAM_W}</width><height>{CAM_H}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>5</update_rate>
        </sensor>"""

CAMERAS = [
    {"name": "cam10_low",  "row": "10m", "pose": (0.0, 0.0, 3.0, 0.0, 0.0, 0.0)},
    {"name": "cam25_low",  "row": "25m", "pose": (15.0, 0.0, 3.0, 0.0, 0.0, 0.0)},
    {"name": "cam40_low",  "row": "40m", "pose": (55.0, 0.0, 3.0, 0.0, 0.0, 0.0)},
    {"name": "cam10_high", "row": "10m", "pose": (0.0, 0.0, 12.0, 0.0, 0.55, 0.0)},
    {"name": "cam25_high", "row": "25m", "pose": (15.0, 0.0, 12.0, 0.0, 0.55, 0.0)},
    # faithful 12 m natural-tilt case: only the 40 m row is in frame
    {"name": "cam40_high", "row": "40m", "pose": (55.0, 0.0, 12.0, 0.0, 0.0, 0.0)},
    {"name": "cam_house",  "row": None,  "pose": (-15.0, 0.0, 5.0, 0.0, 0.0, math.pi)},
]

# dims = [length x, width y, height] in m, MEASURED from the Fuel meshes
# (2026-08-01: obj/dae vertex bounds x model.sdf scale — the SUV is really
# 2.6 m wide; the TruckDelivery van is only 1.8 m tall). Used only for the
# box-size sanity band, never for hit tests.
# expect = COCO classes that count as a hit. Vehicles use the design's
# admission group (§4: car/truck/bus are all trackable vehicles — an SUV
# labeled "truck" is a detection, not a failure; the label histogram in the
# report keeps the confusion visible). TinyRobot has no COCO class: the gate
# records what it is detected AS, if anything.
CAST = [
    {"key": "hatchback", "fuel": "Hatchback",     "expect": ["car", "truck", "bus"],
     "dims": [4.0, 2.14, 1.57]},
    {"key": "suv",       "fuel": "SUV",           "expect": ["car", "truck", "bus"],
     "dims": [5.02, 2.60, 2.16]},
    {"key": "truck",     "fuel": "TruckDelivery", "expect": ["truck", "car", "bus"],
     "dims": [5.12, 1.80, 1.79]},
    {"key": "tinyrobot", "fuel": "TinyRobot",     "expect": [],
     "dims": [0.41, 0.41, 0.30]},
    {"key": "walker",    "fuel": "Walking person", "expect": ["person"],
     "dims": [0.55, 0.88, 1.87], "actor": True},
]

BEARINGS_DEG = (-28.0, -14.0, 0.0, 14.0, 28.0)
ROWS = [
    {"tag": "10m", "cam_x": 0.0,  "row_x": 10.0, "yaw": math.pi},
    {"tag": "25m", "cam_x": 15.0, "row_x": 40.0, "yaw": math.pi / 2},
    {"tag": "40m", "cam_x": 55.0, "row_x": 95.0, "yaw": math.pi},
]

# House 1's Fuel model.sdf uses a custom Ogre material SCRIPT, which gz-sim
# does not support headless -> the include renders BLACK (found 2026-08-01).
# So the world places the same mesh as a static visual with a plain material,
# at the Fuel model's own scale (1.5): the dae declares inches, ogre honors
# the <unit> conversion -> rendered house ~14.6 x 12.9 x 7.7 m, mesh center
# offset (-5.6, +0.4) from the pose. (At 15 m that fills the frame, which is
# how the black include presented; cam_house therefore stands 30 m out.)
HOUSE = {"key": "house", "fuel": "House 1", "pose": (-40.0, 0.0, 0.0),
         "yaw": 0.0, "dims": [14.6, 12.9, 7.7], "center_offset": (-5.63, 0.375),
         "scale": 1.5}


def objects() -> list[dict]:
    """The full placement list (eval imports this — single source of truth)."""
    out = []
    for row in ROWS:
        dist = row["row_x"] - row["cam_x"]
        for cast, bdeg in zip(CAST, BEARINGS_DEG):
            y = dist * math.tan(math.radians(bdeg))
            out.append({
                "name": f"{cast['key']}_{row['tag']}",
                "key": cast["key"], "range": row["tag"],
                "pose": (row["row_x"], y, 0.0, 0.0, 0.0, row["yaw"]),
                "expect": cast["expect"], "actor": cast.get("actor", False),
                "fuel": cast["fuel"],
                # measured dims; the eval derives the apparent extent from
                # yaw + view azimuth (3/4 perspective at edge bearings)
                "length": cast["dims"][0], "width": cast["dims"][1],
                "height": cast["dims"][2],
            })
    return out


OBJECTS = objects()


def _walker_mesh() -> str | None:
    pat = os.path.expanduser(
        "~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/walking person/"
        "*/meshes/*.dae")
    meshes = sorted(glob.glob(pat))
    # swarm_sim.sh strips <library_animations> into walking_frozen.dae (the
    # 26 MB keyframe payload is what stalls the headless render thread) —
    # prefer it, then any walk-named mesh, then anything.
    for want in ("frozen", "walk"):
        hit = [m for m in meshes if want in os.path.basename(m).lower()]
        if hit:
            return hit[0]
    return (meshes or [None])[0]


def _house_mesh() -> str | None:
    meshes = sorted(glob.glob(os.path.expanduser(
        "~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/house 1/"
        "*/meshes/house_1.dae")))
    return (meshes or [None])[0]


def house_model_sdf(mesh: str) -> str:
    """Plain-material static house (the Fuel include renders black headless —
    custom Ogre material script unsupported; see HOUSE comment)."""
    hx, hy, hz = HOUSE["pose"]
    s = HOUSE["scale"]
    return f"""
    <model name="house_1">
      <static>true</static>
      <pose>{hx:.2f} {hy:.2f} {hz:.2f} 0 0 {HOUSE['yaw']:.4f}</pose>
      <link name="link">
        <visual name="v">
          <geometry><mesh><uri>file://{mesh}</uri>
            <scale>{s} {s} {s}</scale></mesh></geometry>
          <material>
            <ambient>0.55 0.5 0.45 1</ambient>
            <diffuse>0.55 0.5 0.45 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def include_sdf(o: dict) -> str:
    x, y, z, rr, pp, yy = o["pose"]
    return f"""
    <include>
      <name>{o['name']}</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/{o['fuel']}</uri>
      <pose>{x:.2f} {y:.2f} {z:.2f} {rr:.4f} {pp:.4f} {yy:.4f}</pose>
    </include>"""


def walker_model_sdf(o: dict, mesh: str) -> str:
    """Frozen-pose person: static model + mesh visual (actors stall the
    headless render thread — see module docstring)."""
    x, y, z, rr, pp, yy = o["pose"]
    return f"""
    <model name="{o['name']}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.2f} {rr:.4f} {pp:.4f} {yy:.4f}</pose>
      <link name="link">
        <visual name="v">
          <geometry><mesh><uri>file://{mesh}</uri></mesh></geometry>
        </visual>
      </link>
    </model>"""


def camera_sdf(c: dict) -> str:
    x, y, z, rr, pp, yy = c["pose"]
    return f"""
    <model name="{c['name']}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.2f} {rr:.4f} {pp:.4f} {yy:.4f}</pose>
      <link name="link">
        <inertial><mass>0.1</mass></inertial>
        {SENSOR}
      </link>
    </model>"""


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as f:
        sdf = f.read()
    # World name MUST match PX4_GZ_WORLD (PX4 gz_bridge + launch polling).
    sdf = re.sub(r'<world\s+name="[^"]*"', '<world name="assets"', sdf, count=1)
    # Row40 sits at x=95 — widen the ground visual so everything stays on canvas.
    sdf = sdf.replace("<size>100 100</size>", "<size>500 500</size>")

    blocks = []
    mesh = _walker_mesh()
    for o in OBJECTS:
        if o["actor"]:
            if mesh:
                blocks.append(walker_model_sdf(o, mesh))
            else:
                print(f"WARNING: walker mesh not in fuel cache, skipping "
                      f"{o['name']} (person class untested)", file=sys.stderr)
        else:
            blocks.append(include_sdf(o))
    house_mesh = _house_mesh()
    if house_mesh:
        blocks.append(house_model_sdf(house_mesh))
    else:
        hx, hy, hz = HOUSE["pose"]
        print("WARNING: house mesh not in fuel cache, using plain include "
              "(renders black headless)", file=sys.stderr)
        blocks.append(include_sdf({"name": "house_1", "fuel": HOUSE["fuel"],
                                   "pose": (hx, hy, hz, 0.0, 0.0,
                                            HOUSE["yaw"])}))
    blocks.extend(camera_sdf(c) for c in CAMERAS)

    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")
    sdf = sdf[:idx] + "".join(blocks) + "\n  " + sdf[idx:]
    with open(dst, "w") as f:
        f.write(sdf)

    sidecar = os.path.join(os.path.dirname(dst) or ".", "assets_boxes.json")
    with open(sidecar, "w") as f:
        json.dump({"cameras": CAMERAS, "objects": OBJECTS, "house": HOUSE,
                   "walker_mesh": mesh, "house_mesh": house_mesh}, f, indent=2)
    print(f"wrote {dst} (+{len(blocks)} blocks, walker_mesh={mesh}, "
          f"house_mesh={house_mesh}) and {sidecar}")


if __name__ == "__main__":
    main()
