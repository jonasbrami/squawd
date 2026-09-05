"""Generate a 'city' Gazebo world from PX4's default.sdf by injecting buildings.

Keeps default.sdf's required plugins/scene/sun/ground (so PX4 + sensors work) and
adds a deterministic scatter of box "buildings" so the drone and camera have
something to see. Buildings avoid the spawn corridor near the origin.

Also writes a sidecar `city_boxes.json` next to the world: the exact building
boxes and spawn layout, so the pilot can compute obstacle/proximity
awareness in pure Python (no extra Gazebo subscriptions) from the same ground
truth that built the world. See agents/perception/perception.py.

Usage: python make_city_world.py <px4_default.sdf> <out_city.sdf>
       -> writes <out_city.sdf> and <out_dir>/city_boxes.json
"""
import json
import os
import random
import re
import sys

# The drone spawns at world (x=0, y=0, z=SPAWN_Z), yaw=0. The pilot reads this
# geometry to map local NED telemetry into the world frame.
SPAWN_X = 0.0
SPAWN_SPACING = 3.0
SPAWN_Z = 0.5


def building_boxes(seed: int = 7) -> list[dict]:
    """Deterministic list of building boxes in the gz world (ENU: +x East, +y North)."""
    rng = random.Random(seed)
    boxes = []
    n = 0
    # sparse grid of plots with jitter; skip the spawn/flight corridor near origin.
    # Kept light (~15 buildings) so camera rendering and flight stay real-time.
    for gx in range(-80, 81, 40):
        for gy in range(-30, 91, 40):
            x = gx + rng.uniform(-6, 6)
            y = gy + rng.uniform(-6, 6)
            if abs(x) < 14 and -6 < y < 40:      # keep the spawn corridor clear
                continue
            if rng.random() < 0.15:               # some empty lots
                continue
            w = rng.uniform(4, 10)
            d = rng.uniform(4, 10)
            h = rng.uniform(6, 30)
            g = rng.uniform(0.35, 0.7)            # greyscale, slight tint
            r, gg, b = g, g * rng.uniform(0.9, 1.0), g * rng.uniform(0.9, 1.05)
            boxes.append({"name": f"bldg_{n}", "x": round(x, 2), "y": round(y, 2),
                          "w": round(w, 2), "d": round(d, 2), "h": round(h, 2),
                          "color": [round(r, 2), round(gg, 2), round(b, 2)]})
            n += 1
    return boxes


def boxes_to_sdf(boxes: list[dict]) -> str:
    out = []
    for box in boxes:
        x, y, w, d, h = box["x"], box["y"], box["w"], box["d"], box["h"]
        r, gg, b = box["color"]
        out.append(f"""
    <model name="{box['name']}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {h/2:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry>
          <material>
            <ambient>{r:.2f} {gg:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {gg:.2f} {b:.2f} 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>""")
    return "".join(out)


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as f:
        sdf = f.read()
    # Rename the world to match this file's basename ('city'). PX4 starts gz with
    # -w $PX4_GZ_WORLD ('city') and calls /world/city/create; if the SDF's internal
    # name stayed 'default' that service never exists and the gz-launching instance
    # dies on a spawn timeout. Keep them consistent. (Sensor + camera topics then
    # live under /world/city/... — the observatory reads the same name.)
    sdf = re.sub(r'<world\s+name="[^"]*"', '<world name="city"', sdf, count=1)
    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")

    boxes = building_boxes()
    city = sdf[:idx] + boxes_to_sdf(boxes) + "\n" + sdf[idx:]
    with open(dst, "w") as f:
        f.write(city)

    # Sidecar ground truth for the agents (same source as the world geometry).
    sidecar = os.path.join(os.path.dirname(dst), "city_boxes.json")
    with open(sidecar, "w") as f:
        json.dump({"spawn_x": SPAWN_X, "spawn_spacing": SPAWN_SPACING,
                   "spawn_z": SPAWN_Z, "buildings": boxes}, f)
    print(f"wrote {dst} (+{len(boxes)} buildings) and {sidecar}")


if __name__ == "__main__":
    main()
