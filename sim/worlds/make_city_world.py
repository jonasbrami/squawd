"""Generate a 'city' Gazebo world from PX4's default.sdf by injecting buildings.

Keeps default.sdf's required plugins/scene/sun/ground (so PX4 + sensors work) and
adds a deterministic scatter of box "buildings" so the drones + their cameras have
something to see. Buildings avoid the spawn corridor near the origin.

Usage: python make_city_world.py <px4_default.sdf> <out_city.sdf>
"""
import random
import sys


def buildings(seed: int = 7) -> str:
    rng = random.Random(seed)
    out = []
    n = 0
    # sparse grid of plots with jitter; skip the spawn/flight corridor near origin.
    # Kept light (~15 buildings) so 3 camera renders + flight stay real-time.
    for gx in range(-80, 81, 40):
        for gy in range(-30, 91, 40):
            x = gx + rng.uniform(-6, 6)
            y = gy + rng.uniform(-6, 6)
            if abs(x) < 14 and -6 < y < 14:      # keep spawn area clear
                continue
            if rng.random() < 0.15:               # some empty lots
                continue
            w = rng.uniform(4, 10)
            d = rng.uniform(4, 10)
            h = rng.uniform(6, 30)
            g = rng.uniform(0.35, 0.7)            # greyscale, slight tint
            r, gg, b = g, g * rng.uniform(0.9, 1.0), g * rng.uniform(0.9, 1.05)
            out.append(f"""
    <model name="bldg_{n}">
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
            n += 1
    return "".join(out)


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as f:
        sdf = f.read()
    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")
    city = sdf[:idx] + buildings() + "\n" + sdf[idx:]
    with open(dst, "w") as f:
        f.write(city)
    marker = '<model name="bldg_'
    print(f"wrote {dst} (+{city.count(marker)} buildings)")


if __name__ == "__main__":
    main()
