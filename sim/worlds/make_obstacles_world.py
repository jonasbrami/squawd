"""Prototype: obstacles world = default.sdf + 6 static box buildings.

Boxes sit east 35-110 / north -55..55 (spawn corridor at east=0 stays clear;
drones spawn at world (0, i*3, 0.5), yaw=0 — must match swarm_sim.sh
PX4_GZ_MODEL_POSE="0,${y},0.5").

Writes <out>.sdf + <out_dir>/obstacles_boxes.json, consumed by
agents/world/model.py and the eval sampler/oracle.

Usage: python make_obstacles_world.py <px4_default.sdf> <out_obstacles.sdf>
"""
import json
import os
import re
import sys

SPAWN_X = 0.0
SPAWN_SPACING = 3.0
SPAWN_Z = 0.5

# Hand-picked deterministic layout: 10-20 m footprints, 15-30 m tall, all >=35 m
# east of the spawn corridor. obs_0/obs_4 make an east-bound slalom lane along
# north=0; obs_1/obs_3 gate the NE quadrant; obs_2/obs_5 populate the SE.
BOXES = [
    {"name": "obs_0", "x": 45.0,  "y": 0.0,   "w": 14.0, "d": 14.0, "h": 20.0},
    {"name": "obs_1", "x": 40.0,  "y": 35.0,  "w": 12.0, "d": 18.0, "h": 25.0},
    {"name": "obs_2", "x": 70.0,  "y": -30.0, "w": 16.0, "d": 12.0, "h": 15.0},
    {"name": "obs_3", "x": 80.0,  "y": 30.0,  "w": 20.0, "d": 14.0, "h": 30.0},
    {"name": "obs_4", "x": 110.0, "y": 0.0,   "w": 15.0, "d": 15.0, "h": 18.0},
    {"name": "obs_5", "x": 55.0,  "y": -55.0, "w": 12.0, "d": 12.0, "h": 22.0},
]
GREY = [0.55, 0.55, 0.58]


def box_sdf(b: dict) -> str:
    x, y, w, d, h = b["x"], b["y"], b["w"], b["d"], b["h"]
    r, g, bl = GREY
    return f"""
    <model name="{b['name']}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {h / 2:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {bl} 1</ambient>
            <diffuse>{r} {g} {bl} 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as f:
        sdf = f.read()
    # World name MUST match PX4_GZ_WORLD: PX4's gz_bridge subscribes to
    # /world/<PX4_GZ_WORLD>/... and swarm_sim.sh polls /world/<name>/create.
    sdf = re.sub(r'<world\s+name="[^"]*"', '<world name="obstacles"', sdf, count=1)
    # Widen the ground-plane VISUAL (100x100 -> 300x300) so boxes past east 50
    # don't float over grey void on camera; the collision plane is a
    # mathematically infinite plane in the physics engine either way.
    sdf = sdf.replace("<size>100 100</size>", "<size>300 300</size>")
    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")
    sdf = sdf[:idx] + "".join(box_sdf(b) for b in BOXES) + "\n  " + sdf[idx:]
    with open(dst, "w") as f:
        f.write(sdf)

    sidecar = os.path.join(os.path.dirname(dst) or ".", "obstacles_boxes.json")
    with open(sidecar, "w") as f:
        json.dump({"spawn_x": SPAWN_X, "spawn_spacing": SPAWN_SPACING,
                   "spawn_z": SPAWN_Z,
                   "buildings": [dict(b, color=GREY) for b in BOXES]}, f)
    print(f"wrote {dst} (+{len(BOXES)} boxes) and {sidecar}")


if __name__ == "__main__":
    main()
