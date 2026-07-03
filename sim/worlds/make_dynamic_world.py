"""Dynamic world = default.sdf (flat) + scripted kinematic movers.

Each mover is a non-static box driven by sim/plugins/mover_system.py (a
PythonSystemLoader system evaluating agents/world/trajectory.py per physics
step), so its position is an exact, repeatable function of sim time. Task
regions are separated by quadrant so one world hosts the whole L1-L5 dynamic
ladder; drones spawn at world (0, i*3, 0.5) (must match swarm_sim.sh
PX4_GZ_MODEL_POSE).

Writes <out>.sdf + <out_dir>/dynamic_boxes.json — the same sidecar file
agents/world/model.py already loads by world name (buildings kept, empty)
plus the new "movers" array that mover_system.py (via MOVERS_JSON) and the
evals sampler/oracle read.

Usage: python make_dynamic_world.py <px4_default.sdf> <out_dynamic.sdf>
"""
import json
import os
import re
import sys

SPAWN_X = 0.0
SPAWN_SPACING = 3.0
SPAWN_Z = 0.5

# One region per ladder rung, mutually >=40 m apart at every point in time
# (verified by check_min_separation below at generation time):
#   mov_0  L1 courier      N edge, W<->E shuttle at 4 m/s
#   mov_1  L2 survey rover ground vehicle circling the SE plaza at 1.5 m/s
#   mov_2  L3 gate sweeper 4 m box patrolling the x=110 fence line (obstacle)
#   mov_3  L4 transit      constant-velocity one-way crosser, unknown course
#   mov_4  L5 intruder     one-way inbound run at the SW tower
MOVERS = [
    {"name": "mov_0", "kind": "target", "z": 10.0,
     "shape": {"w": 0.8, "d": 0.8, "h": 0.4},
     "traj": {"type": "line", "p0": [-40.0, 100.0], "p1": [140.0, 100.0],
              "speed_mps": 4.0}},
    {"name": "mov_1", "kind": "target", "z": 1.2,
     "shape": {"w": 1.8, "d": 1.0, "h": 1.2},
     "traj": {"type": "circle", "center": [70.0, -100.0], "radius_m": 35.0,
              "speed_mps": 3.5}},
    {"name": "mov_2", "kind": "obstacle", "z": 8.0,
     "shape": {"w": 4.0, "d": 4.0, "h": 4.0},
     "traj": {"type": "line", "p0": [110.0, -10.0], "p1": [110.0, 30.0],
              "speed_mps": 4.0}},
    {"name": "mov_3", "kind": "target", "z": 12.0,
     "shape": {"w": 0.8, "d": 0.8, "h": 0.4},
     "traj": {"type": "line", "p0": [-240.0, -40.0], "p1": [0.0, 200.0],
              "speed_mps": 6.0, "mode": "once"}},
    {"name": "mov_4", "kind": "target", "z": 10.0,
     "shape": {"w": 0.8, "d": 0.8, "h": 0.4},
     "traj": {"type": "line", "p0": [-240.0, -240.0], "p1": [-80.0, -80.0],
              "speed_mps": 3.5, "mode": "once"}},
]
COLOR = {"target": [0.9, 0.45, 0.1], "obstacle": [0.35, 0.4, 0.6]}


def mover_sdf(m: dict) -> str:
    from agents.world.trajectory import pos_xy
    x0, y0 = pos_xy(m["traj"], 0.0)
    s = m["shape"]
    r, g, b = COLOR[m["kind"]]
    return f"""
    <model name="{m['name']}">
      <static>false</static>
      <pose>{x0:.2f} {y0:.2f} {m['z']:.2f} 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass></inertial>
        <collision name="c"><geometry><box><size>{s['w']:.2f} {s['d']:.2f} {s['h']:.2f}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{s['w']:.2f} {s['d']:.2f} {s['h']:.2f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>
      </link>
      <plugin filename="gz-sim-python-system-loader-system"
              name="gz::sim::systems::PythonSystemLoader">
        <module_name>mover_system</module_name>
      </plugin>
    </model>"""


def check_min_separation(movers: list[dict], min_m: float = 40.0,
                         dt: float = 1.0, horizon_s: float = 600.0) -> None:
    """Task regions must stay separated so scan near one region never
    conflates another region's mover; sampled over a 10-minute horizon."""
    from agents.world.trajectory import pos_xy
    steps = int(horizon_s / dt)
    for i in range(len(movers)):
        for j in range(i + 1, len(movers)):
            for k in range(steps + 1):
                t = k * dt
                a = pos_xy(movers[i]["traj"], t)
                b = pos_xy(movers[j]["traj"], t)
                d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
                if d < min_m:
                    raise SystemExit(
                        f"{movers[i]['name']} and {movers[j]['name']} come "
                        f"within {d:.1f}m at t={t:.0f}s (min {min_m}m)")


def main() -> None:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    src, dst = sys.argv[1], sys.argv[2]
    check_min_separation(MOVERS)
    with open(src) as f:
        sdf = f.read()
    # World name MUST match PX4_GZ_WORLD (PX4 gz_bridge + launch polling).
    sdf = re.sub(r'<world\s+name="[^"]*"', '<world name="dynamic"', sdf, count=1)
    # Movers roam to +/-200 m — widen the ground visual so they stay on canvas.
    sdf = sdf.replace("<size>100 100</size>", "<size>500 500</size>")
    idx = sdf.rfind("</world>")
    if idx == -1:
        raise SystemExit("no </world> in source SDF")
    sdf = sdf[:idx] + "".join(mover_sdf(m) for m in MOVERS) + "\n  " + sdf[idx:]
    with open(dst, "w") as f:
        f.write(sdf)

    sidecar = os.path.join(os.path.dirname(dst) or ".", "dynamic_boxes.json")
    with open(sidecar, "w") as f:
        json.dump({"spawn_x": SPAWN_X, "spawn_spacing": SPAWN_SPACING,
                   "spawn_z": SPAWN_Z, "buildings": [], "movers": MOVERS}, f)
    print(f"wrote {dst} (+{len(MOVERS)} movers) and {sidecar}")


if __name__ == "__main__":
    main()
