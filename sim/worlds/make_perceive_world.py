"""Perceive world = default.sdf (flat) + scripted GROUND movers: ONE true
target plus visually DISTINCT decoys (design §3.8 — the blob backend is
single-class and cannot separate same-orange decoys, so every decoy here
differs in COLOR and SHAPE, review Codex-Mj4).

  mov_true        orange survey rover (the blob's "target" color), circles the
                  SE plaza — same kinematics as dynamic mov_1 (proven
                  detectable/shadowable at M2/M3a)
  mov_decoy_red   RED tall hauler, tighter/faster circle on the same center —
                  angular-rate difference makes it CROSS the rover's bearing
                  fan repeatedly (the ID-switch stressor for p2)
  mov_decoy_blue  blue-grey low sled (the obstacle color), bounce shuttle
                  straight through the plaza — crosses the rover's ring twice
                  per lap

All movers are GROUND vehicles (z so the box base rests at ~0.6 m, mov_1's
measured convention): the v1 vision-ladder variants never leave the support
plane. The oracle reads names from the sidecar, never hardcoded (§3.4/§3.8).

Usage: python make_perceive_world.py <px4_default.sdf> <out_perceive.sdf>
"""
import os
import sys

MOVERS = [
    {"name": "mov_true", "kind": "target", "z": 1.2,
     "shape": {"w": 1.8, "d": 1.0, "h": 1.2},
     "traj": {"type": "circle", "center": [70.0, -100.0], "radius_m": 35.0,
              "speed_mps": 3.5}},
    {"name": "mov_decoy_red", "kind": "decoy_red", "z": 1.5,
     "shape": {"w": 1.2, "d": 1.2, "h": 1.8},
     "traj": {"type": "circle", "center": [70.0, -100.0], "radius_m": 22.0,
              "speed_mps": 4.5}},
    {"name": "mov_decoy_blue", "kind": "decoy_blue", "z": 1.0,
     "shape": {"w": 2.5, "d": 0.8, "h": 0.8},
     "traj": {"type": "line", "p0": [30.0, -135.0], "p1": [110.0, -65.0],
              "speed_mps": 5.0}},
]
# Distinct visual evidence per mover — the blob's HSV gate fires ONLY on the
# target orange; red and blue-grey fall outside it by construction.
COLOR = {"target": [0.9, 0.45, 0.1], "decoy_red": [0.75, 0.12, 0.08],
         "decoy_blue": [0.35, 0.4, 0.6]}


def main() -> None:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from sim.worlds.make_dynamic_world import write_world
    src, dst = sys.argv[1], sys.argv[2]
    write_world(src, dst, "perceive", MOVERS, COLOR)


if __name__ == "__main__":
    main()
