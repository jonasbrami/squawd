"""Demo world = default.sdf (flat) + the W1b demo cast (design 2026-07-28 §3):
FIVE mesh-visual movers on the proven velocity-drive mover pattern
(sim/plugins/mover_system.py) plus static Fuel landmarks arranged as a small
neighborhood, and static evidence cameras replicating the x500_depth IMX214.

  car_1     RED Hatchback (Fuel "Hatchback red", plain "Hatchback" fallback)
            on a 40x60 m rectangular street loop (waypoint_loop, 4 m/s)
  car_2     SUV on a r=20 m circle (3.5 m/s) around the NW plaza
  car_3     TruckDelivery van on a slower 35x40 m delivery route (3 m/s) E
  walker_1/2 "Walking person" as STATIC MESH VISUALS (walking_frozen.dae —
            <actor> wedges headless gz outright, W0.1) pacing sidewalks at
            1.2-1.3 m/s: S of the street loop / around the plaza
  house_1 / house_2 / gas_station as MESH + PLAIN MATERIAL (their Fuel
            includes render BLACK headless — custom Ogre material scripts,
            the W0.1 House-1 lesson) + box collision from measured mesh bounds
  2x pine_tree as SCALED direct-mesh visuals (x2.4 -> ~12 m street trees; the
            include renders fine but only ~5 m and <include> has no scale) —
            same submesh+pbr-albedo material structure as the Fuel include,
            textures by absolute file:// path
  2x oak_tree / 2x lamp_post as Fuel includes (oak shares the pine's
            script+pbr structure that renders correctly headless)

Movers are one-link gravity-off models (velocity-drive; pose commands break
PX4's EKF — mover_system.py:100-131) whose VISUAL is the Fuel car mesh parsed
from the downloaded cache (scale + visual pose read from the cached model.sdf,
so the red variant needs no hardcoding; missing pose/scale default to identity
— TruckDelivery ships neither). The collision is a plain box lifted 5 cm off
the ground so it never contacts the ground plane. Every mover sets
"heading_align": the plugin drives yaw to the velocity direction through a
clamped proportional yaw-RATE (W1a froze yaw at the t0 heading — cars slid
sideways on every other leg). The walker's mesh faces -y at identity
(verified on the W0.1 assets frames: yaw pi = profile walking +y) so its
visual carries a +pi/2 yaw offset to put its nose on link +x. When the cache
is absent (host unit tests) the mover falls back to a colored box visual
with a loud warning — the world stays functional.

Landmark dims/center_offsets are TRANSFORM-AWARE mesh measurements (2026-08-01:
dae vertex bounds composed with the visual-scene node transforms — rotate /
scale / translate — x unit x model.sdf scale; ogre applies the node transforms
when rendering, verified against the W1a cam_loop frame where house_1's
rendered center sits 5.5 m EAST of the raw-bounds value W1a shipped. house_1's
and gas_station's W1a box/sidecar numbers were raw-bounds and are corrected
here). z = 0: bases rest on the ground plane.

Cameras: the validated IMX214 sensor geometry, with far clip 300
(the 100 m production clip cuts the 130 m overview sightline; these are
evidence vantages, not the production replica).

Usage: python make_demo_world.py <px4_default.sdf> <out_demo.sdf>
"""
import glob
import json
import math
import os
import re
import sys

SPAWN_X = 0.0
SPAWN_SPACING = 3.0
SPAWN_Z = 0.5

# Fuel model cache (dir names are lowercase); overridable for tests.
FUEL_MODELS = os.environ.get(
    "DEMO_FUEL_MODELS",
    os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models"))

# dims = [length x, width y, height] MEASURED from the Fuel meshes (W0.1,
# 2026-08-01) — used for the collision box + the sidecar shape, never for
# hit tests. z = 0: the wheels rest on the ground plane.
# heading_align: the mover plugin drives visual yaw to the velocity direction
# (W1b — see module docstring).
MOVERS = [
    {"name": "car_1", "kind": "car", "z": 0.0, "heading_align": True,
     "fuel": ["Hatchback red", "Hatchback"],   # red first, plain fallback
     "fallback_color": [0.75, 0.10, 0.08],
     "shape": {"w": 4.0, "d": 2.14, "h": 1.57},
     "traj": {"type": "waypoint_loop",
              "pts": [[30.0, -30.0], [70.0, -30.0], [70.0, 30.0], [30.0, 30.0]],
              "speed_mps": 4.0}},
    {"name": "car_2", "kind": "car", "z": 0.0, "heading_align": True,
     "fuel": ["SUV"],
     "fallback_color": [0.55, 0.58, 0.62],
     "shape": {"w": 5.02, "d": 2.60, "h": 2.16},
     "traj": {"type": "circle", "center": [-45.0, 35.0], "radius_m": 20.0,
              "speed_mps": 3.5}},
    # slower delivery route on the east side (TruckDelivery ships no visual
    # pose/scale in its model.sdf — raw obj in metres, nose +x)
    {"name": "car_3", "kind": "car", "z": 0.0, "heading_align": True,
     "fuel": ["TruckDelivery"],
     "fallback_color": [0.85, 0.85, 0.80],
     "shape": {"w": 5.12, "d": 1.80, "h": 1.79},
     "traj": {"type": "waypoint_loop",
              "pts": [[85.0, -40.0], [120.0, -40.0], [120.0, 0.0], [85.0, 0.0]],
              "speed_mps": 3.0}},
    # walkers: STATIC mesh visuals (frozen stride — NO actors, W0.1) on slow
    # sidewalk loops; the mesh faces -y at identity, the +pi/2 visual yaw puts
    # its nose on link +x so heading_align faces it along its walk.
    {"name": "walker_1", "kind": "walker", "z": 0.0, "heading_align": True,
     "fuel": ["Walking person"], "walker": True,
     "fallback_color": [0.45, 0.47, 0.55],
     "shape": {"w": 0.6, "d": 0.5, "h": 1.87},
     "traj": {"type": "waypoint_loop",
              "pts": [[32.0, -36.0], [68.0, -36.0]],
              "speed_mps": 1.3}},
    {"name": "walker_2", "kind": "walker", "z": 0.0, "heading_align": True,
     "fuel": ["Walking person"], "walker": True,
     "fallback_color": [0.45, 0.47, 0.55],
     "shape": {"w": 0.6, "d": 0.5, "h": 1.87},
     "traj": {"type": "waypoint_loop",
              "pts": [[-71.0, 9.0], [-19.0, 9.0], [-19.0, 61.0], [-71.0, 61.0]],
              "speed_mps": 1.2}},
]

# Mesh+plain-material landmarks (see module docstring). dims are
# TRANSFORM-AWARE mesh measurements (dae bounds composed with the visual-scene
# node transforms x <unit> x model.sdf scale — house_1's Rz(-180)/z-translate
# and gas_station's 1.174 node scale included; W1a shipped raw-bounds numbers
# that put house_1's collision box 5.5 m west of its render).
# center_offset = mesh-center offset from the pose in the LOCAL (link) frame;
# buildings() rotates it by yaw for the world-frame sidecar entry.
MESH_LANDMARKS = [
    {"name": "house_1", "fuel": "House 1", "mesh": "house_1.dae",
     "pose": (50.0, -48.0, 0.0), "yaw": 0.0, "dims": [16.5, 12.9, 7.7],
     "center_offset": (-0.40, -0.37), "scale": 1.5,
     "color": (0.55, 0.5, 0.45)},
    {"name": "gas_station", "fuel": "Gas Station", "mesh": "gas_station.dae",
     "pose": (88.0, 42.0, 0.0), "yaw": math.pi, "dims": [20.6, 30.0, 9.0],
     "center_offset": (0.0, -9.18), "scale": 1.0,
     "color": (0.6, 0.56, 0.5)},
    # House 2: same black-include class as House 1 (script material) -> same
    # mesh+plain-material pattern. Second residential lot, S of the loop.
    {"name": "house_2", "fuel": "House 2", "mesh": "house_2.dae",
     "pose": (18.0, -64.0, 0.0), "yaw": 0.0, "dims": [12.7, 9.8, 7.2],
     "center_offset": (1.21, 0.78), "scale": 1.5,
     "color": (0.5, 0.46, 0.42)},
]

# Scaled street trees (W1b): the pine include renders correctly but only ~5 m
# tall and <include> has no scale — direct-mesh visuals at x2.4 (~12 m),
# replicating the Fuel model.sdf's submesh + pbr-albedo material structure
# (the script block is the unsupported black-render class, dropped) with the
# textures by absolute file:// path. Collision is a slim trunk box (drones may
# brush the canopy; the include's full-mesh collision is canopy-wide).
# textures maps submesh name -> texture file under materials/textures/.
TREES = [
    {"name": "pine_tree_1", "fuel": "Pine Tree", "mesh": "pine_tree.dae",
     "textures": {"Branch": "branch_2_diffuse.png", "Bark": "bark_diffuse.png"},
     "pose": (24.0, 10.0, 0.0), "scale": 2.4, "dims": [7.6, 7.6, 12.2]},
    {"name": "pine_tree_2", "fuel": "Pine Tree", "mesh": "pine_tree.dae",
     "textures": {"Branch": "branch_2_diffuse.png", "Bark": "bark_diffuse.png"},
     "pose": (-68.0, 52.0, 0.0), "scale": 2.4, "dims": [7.6, 7.6, 12.2]},
]

# Static Fuel includes (pine/lamp verified rendering headless on the first W1a
# boot; the oak shares the pine's script+pbr structure — same render path).
# dims = approximate footprint [w, d, h] for the buildings sidecar (proximity
# text + roof clamps — not hit tests); oak measured transform-aware 2026-08-01.
LANDMARKS = [
    {"name": "oak_tree_1", "fuel": "Oak Tree", "dims": [10.6, 8.5, 6.6],
     "pose": (108.0, 12.0, 0.0, 0.0, 0.0, 0.0)},
    {"name": "oak_tree_2", "fuel": "Oak Tree", "dims": [10.6, 8.5, 6.6],
     "pose": (2.0, -70.0, 0.0, 0.0, 0.0, 0.0)},
    {"name": "lamp_post_1", "fuel": "Lamp Post", "dims": [0.6, 0.6, 6.0],
     "pose": (27.0, -34.0, 0.0, 0.0, 0.0, 0.0)},
    {"name": "lamp_post_2", "fuel": "Lamp Post", "dims": [0.6, 0.6, 6.0],
     "pose": (-25.0, 55.0, 0.0, 0.0, 0.0, 0.0)},
]

# --- sensor replica (OakD-Lite IMX214, post swarm_sim.sh 640x360 patch) ----
# far clip 300: evidence vantages see 130+ m (the production 100 m clip would
# cut the overview's far edge). hfov/resolution stay the production values.
HFOV = 1.204
CAM_W, CAM_H = 640, 360
SENSOR = f"""<sensor name="IMX214" type="camera">
          <camera>
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{CAM_W}</width><height>{CAM_H}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>300</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>5</update_rate>
        </sensor>"""

# Evidence vantages (see module docstring; bearings verified by construction):
CAMERAS = [
    # whole car_1 neighborhood from the SW: loop + house + gas station + trees
    {"name": "cam_overview", "pose": (-10.0, -60.0, 45.0, 0.0, 0.49, 0.785)},
    # oblique over the street loop, house + gas station behind the far legs
    {"name": "cam_loop",     "pose": (50.0, -90.0, 50.0, 0.0, 0.65, 1.5708)},
    # 3 m production geometry looking east down the south leg (car + house)
    {"name": "cam_street",   "pose": (12.0, -30.0, 3.0, 0.0, 0.0, 0.0)},
    # the NW plaza circle (car_2) + walker_2's perimeter + lamp_post_2
    {"name": "cam_plaza",    "pose": (-75.0, 8.0, 25.0, 0.0, 0.55, 0.734)},
    # gas station corner closeup
    {"name": "cam_corner",   "pose": (70.0, 20.0, 6.0, 0.0, 0.10, 0.886)},
    # walker_1's sidewalk down its length from the W, car_1's south leg behind
    # (W1b reposition: the first vantage (50,-50) sits INSIDE house_1's
    # transform-corrected render)
    {"name": "cam_walker",   "pose": (24.0, -38.0, 3.0, 0.0, 0.05, 0.15)},
    # car_3's delivery route from the S, oak_tree_1 + gas station behind
    {"name": "cam_truck",    "pose": (102.0, -55.0, 12.0, 0.0, 0.35, 1.5708)},
    # house_2 + oak_tree_2 from the SE
    {"name": "cam_house2",   "pose": (45.0, -85.0, 8.0, 0.0, 0.25, 2.44)},
]


def _latest_version_dir(fuel_name: str) -> str | None:
    vers = sorted(glob.glob(os.path.join(FUEL_MODELS, fuel_name.lower(), "*/")))
    return (vers or [None])[-1]


def _fuel_visual(fuel_names: list[str]) -> dict | None:
    """Mesh visual spec from the cached Fuel model.sdf: absolute mesh path,
    scale, and the visual pose rpy (car meshes ship yaw-rotated and in
    non-metre units; pose/scale default to identity when the sdf omits them —
    TruckDelivery ships neither). Tries each name in order; None when
    uncached/parseless."""
    for fuel in fuel_names:
        ver = _latest_version_dir(fuel)
        if not ver:
            continue
        try:
            sdf = open(os.path.join(ver, "model.sdf"), encoding="utf-8",
                       errors="ignore").read()
            vis = re.search(r"<visual\b.*?</visual>", sdf, re.S).group(0)
            pose = re.search(r"<pose>([^<]*)</pose>", vis)
            rpy = ([float(v) for v in pose.group(1).split()[3:6]]
                   if pose else [0.0, 0.0, 0.0])
            scale_m = re.search(r"<scale>([^<]*)</scale>", vis)
            scale = float(scale_m.group(1).split()[0]) if scale_m else 1.0
            base = os.path.basename(re.search(
                r"<uri>([^<]*)</uri>", vis).group(1))
            mesh = os.path.join(ver, "meshes", base)
            if os.path.isfile(mesh):
                return {"fuel": fuel, "mesh": os.path.abspath(mesh),
                        "scale": scale, "rpy": rpy}
        except (OSError, AttributeError, ValueError, IndexError):
            continue
    return None


def _walker_visual() -> dict | None:
    """The frozen-stride person mesh (visual pose rpy 0 0 +pi/2: walking.dae
    faces -y at identity — verified on the W0.1 assets frames — and the mover
    heading convention is nose=+x). Prefers walking_frozen.dae (swarm_sim.sh
    strips the keyframe payload — actors/animation stall the headless render
    thread), then any walk-named dae. None when uncached."""
    ver = _latest_version_dir("Walking person")
    if not ver:
        return None
    meshes = sorted(glob.glob(os.path.join(ver, "meshes", "*.dae")))
    for want in ("frozen", "walk"):
        hit = [m for m in meshes if want in os.path.basename(m).lower()]
        if hit:
            return {"fuel": "Walking person", "mesh": os.path.abspath(hit[0]),
                    "scale": 1.0, "rpy": [0.0, 0.0, math.pi / 2]}
    return None


def _dae_mesh(lm: dict) -> str | None:
    ver = _latest_version_dir(lm["fuel"])
    if not ver:
        return None
    meshes = sorted(glob.glob(os.path.join(ver, "meshes", lm["mesh"])))
    return (meshes or [None])[0]


def mover_sdf(m: dict) -> str:
    from agents.world.trajectory import pos_xy, vel_xy
    x0, y0 = pos_xy(m["traj"], 0.0)
    vx, vy = vel_xy(m["traj"], 0.0)
    yaw0 = math.atan2(vy, vx)           # spawn heading; heading_align keeps
                                        # the yaw chasing the velocity in-sim
    s = m["shape"]
    vis = _walker_visual() if m.get("walker") else _fuel_visual(m["fuel"])
    if vis:
        rr, pp, yy = vis["rpy"]
        visual = f"""<visual name="v">
          <pose>0 0 0 {rr:.4f} {pp:.4f} {yy:.4f}</pose>
          <geometry><mesh><uri>file://{vis['mesh']}</uri>
            <scale>{vis['scale']} {vis['scale']} {vis['scale']}</scale></mesh></geometry>
        </visual>"""
    else:
        print(f"WARNING: no fuel mesh for {m['name']} ({m['fuel']}) — "
              f"falling back to a box visual", file=sys.stderr)
        r, g, b = m["fallback_color"]
        visual = f"""<visual name="v">
          <pose>0 0 {s['h'] / 2:.2f} 0 0 0</pose>
          <geometry><box><size>{s['w']:.2f} {s['d']:.2f} {s['h']:.2f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>"""
    # collision box floats 5 cm up: no contact with the ground plane.
    return f"""
    <model name="{m['name']}">
      <static>false</static>
      <pose>{x0:.2f} {y0:.2f} {m['z']:.2f} 0 0 {yaw0:.4f}</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass></inertial>
        <collision name="c"><pose>0 0 {s['h'] / 2 + 0.05:.2f} 0 0 0</pose><geometry><box><size>{s['w']:.2f} {s['d']:.2f} {s['h']:.2f}</size></box></geometry></collision>
        {visual}
      </link>
      <plugin filename="gz-sim-python-system-loader-system"
              name="gz::sim::systems::PythonSystemLoader">
        <module_name>mover_system</module_name>
      </plugin>
    </model>"""


def mesh_landmark_sdf(lm: dict, mesh: str) -> str:
    """Plain-material static mesh landmark + box collision (the Fuel include
    renders black headless — custom Ogre material script; see MESH_LANDMARKS)."""
    hx, hy, hz = lm["pose"]
    s = lm["scale"]
    w, d, h = lm["dims"]
    ox, oy = lm["center_offset"]
    r, g, b = lm["color"]
    return f"""
    <model name="{lm['name']}">
      <static>true</static>
      <pose>{hx:.2f} {hy:.2f} {hz:.2f} 0 0 {lm['yaw']:.4f}</pose>
      <link name="link">
        <collision name="c"><pose>{ox:.2f} {oy:.2f} {h / 2:.2f} 0 0 0</pose><geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry></collision>
        <visual name="v">
          <geometry><mesh><uri>file://{mesh}</uri>
            <scale>{s} {s} {s}</scale></mesh></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def tree_sdf(t: dict, ver: str) -> str:
    """Scaled direct-mesh tree: one visual per submesh with the Fuel model's
    pbr-albedo material (the unsupported script block dropped), textures by
    absolute file:// path, plus a slim trunk collision (see TREES comment)."""
    x, y, z = t["pose"]
    s = t["scale"]
    visuals = []
    for submesh, tex in t["textures"].items():
        png = os.path.abspath(os.path.join(ver, "materials", "textures", tex))
        dae = os.path.abspath(os.path.join(ver, "meshes", t["mesh"]))
        visuals.append(f"""<visual name="{submesh.lower()}">
          <geometry><mesh><uri>file://{dae}</uri>
            <scale>{s} {s} {s}</scale>
            <submesh><name>{submesh}</name></submesh></mesh></geometry>
          <material>
            <double_sided>true</double_sided>
            <diffuse>1.0 1.0 1.0 1.0</diffuse>
            <pbr><metal><albedo_map>file://{png}</albedo_map></metal></pbr>
          </material>
        </visual>""")
    w, d, h = t["dims"]
    return f"""
    <model name="{t['name']}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>0 0 {h / 2:.2f} 0 0 0</pose><geometry><box><size>0.8 0.8 {h:.2f}</size></box></geometry></collision>
        {chr(10).join(visuals)}
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


def _tree_version(t: dict) -> str | None:
    """Cache version dir for the direct-mesh tree path; needs the dae AND every
    texture (else the include fallback renders fine, just unscaled)."""
    ver = _latest_version_dir(t["fuel"])
    if not ver:
        return None
    paths = [os.path.join(ver, "meshes", t["mesh"])]
    paths += [os.path.join(ver, "materials", "textures", tex)
              for tex in t["textures"].values()]
    return ver if all(os.path.isfile(p) for p in paths) else None


def buildings() -> list[dict]:
    """Landmark boxes for the sidecar (agents/world/model.py consumers: scan
    proximity text, goto/track roof clamps — name/x/y/w/d/h schema)."""
    out = []
    for lm in MESH_LANDMARKS:
        hx, hy, _ = lm["pose"]
        ox, oy = lm["center_offset"]            # local frame — rotate by yaw
        c, s = math.cos(lm["yaw"]), math.sin(lm["yaw"])
        w, d, h = lm["dims"]
        out.append({"name": lm["name"],
                    "x": round(hx + c * ox - s * oy, 2),
                    "y": round(hy + s * ox + c * oy, 2),
                    "w": w, "d": d, "h": h})
    for lm in TREES + LANDMARKS:
        x, y = lm["pose"][0], lm["pose"][1]
        w, d, h = lm["dims"]
        out.append({"name": lm["name"], "x": x, "y": y, "w": w, "d": d, "h": h})
    return out


def main() -> None:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as f:
        sdf = f.read()
    # World name MUST match PX4_GZ_WORLD (PX4 gz_bridge + launch polling).
    sdf = re.sub(r'<world\s+name="[^"]*"', '<world name="demo"', sdf, count=1)
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
    blocks.extend(camera_sdf(c) for c in CAMERAS)

    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")
    sdf = sdf[:idx] + "".join(blocks) + "\n  " + sdf[idx:]
    with open(dst, "w") as f:
        f.write(sdf)

    sidecar = os.path.join(os.path.dirname(dst) or ".", "demo_boxes.json")
    with open(sidecar, "w") as f:
        json.dump({"spawn_x": SPAWN_X, "spawn_spacing": SPAWN_SPACING,
                   "spawn_z": SPAWN_Z, "buildings": buildings(),
                   "movers": MOVERS, "cameras": CAMERAS}, f, indent=2)
    print(f"wrote {dst} (+{len(blocks)} blocks) and {sidecar}")


if __name__ == "__main__":
    main()
