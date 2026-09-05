"""Spawn visual-only props into a running dynamic world so eval footage reads.

Everything here is static, collision-free set dressing for the FILM container
only — the eval world stays untouched. Gives the overhead cinecam a map grid
and task landmarks (courier route ribbon, tower + 50 m perimeter ring, fence
walls with the real gap, depot/plaza markers).

usage (inside container): python3 film_dressing.py --world dynamic
"""
import argparse


def box(name, x, y, z, sx, sy, sz, rgba, yaw=0.0):
    r, g, b, a = rgba
    return f"""<?xml version="1.0"?><sdf version="1.9">
<model name="{name}"><static>true</static>
<pose>{x} {y} {z} 0 0 {yaw}</pose>
<link name="l"><visual name="v">
<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
<transparency>{1.0 - a}</transparency>
</visual></link></model></sdf>"""


def cylinder(name, x, y, z, radius, length, rgba):
    r, g, b, a = rgba
    return f"""<?xml version="1.0"?><sdf version="1.9">
<model name="{name}"><static>true</static>
<pose>{x} {y} {z} 0 0 0</pose>
<link name="l"><visual name="v">
<geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
</visual></link></model></sdf>"""


def props():
    GRID = (0.45, 0.45, 0.48, 1.0)
    ps = []
    # 50 m map grid over the playfield (thin flat bars, slightly above ground)
    for i, v in enumerate(range(-250, 251, 50)):
        ps.append(box(f"grid_ew_{i}", 0, v, 0.03, 520, 0.8, 0.02, GRID))
        ps.append(box(f"grid_ns_{i}", v, 0, 0.03, 0.8, 520, 0.02, GRID))
    # d1: courier route ribbon along N100
    ps.append(box("route_d1", 50, 100, 0.06, 184, 1.6, 0.02, (1.0, 0.55, 0.1, 1.0)))
    # d5: tower pad + 50 m perimeter ring of posts
    ps.append(cylinder("tower_pad", -80, -80, 0.05, 8, 0.1, (0.85, 0.1, 0.1, 1.0)))
    ps.append(box("tower_mast", -80, -80, 6, 2, 2, 12, (0.85, 0.1, 0.1, 1.0)))
    import math
    for k in range(16):
        a = 2 * math.pi * k / 16
        ps.append(box(f"perim_{k}", -80 + 50 * math.cos(a), -80 + 50 * math.sin(a),
                      2.0, 1.2, 1.2, 4.0, (0.9, 0.25, 0.1, 1.0)))
    # d3: the fence walls (visual only!) with the real N[-10,30] gap + depot pad
    ps.append(box("fence_s", 110, -130, 4, 1.0, 240, 8, (0.5, 0.15, 0.15, 0.85)))
    ps.append(box("fence_n", 110, 140, 4, 1.0, 220, 8, (0.5, 0.15, 0.15, 0.85)))
    ps.append(cylinder("depot_pad", 170, -5, 0.05, 7, 0.1, (0.1, 0.7, 0.2, 1.0)))
    # d2: plaza circle outline (24 kerb posts on r=35 around (70,-100))
    for k in range(24):
        a = 2 * math.pi * k / 24
        ps.append(box(f"plaza_{k}", 70 + 35 * math.cos(a), -100 + 35 * math.sin(a),
                      0.5, 0.8, 0.8, 1.0, (0.2, 0.35, 0.75, 1.0)))
    # spawn pad
    ps.append(cylinder("home_pad", 0, 0, 0.05, 5, 0.1, (0.95, 0.85, 0.2, 1.0)))
    return ps


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="dynamic")
    args = ap.parse_args()
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory

    node = Node()
    ok_n = 0
    for sdf in props():
        req = EntityFactory()
        req.sdf = sdf
        ok, resp = node.request(f"/world/{args.world}/create", req,
                                EntityFactory, Boolean, 3000)
        ok_n += bool(ok and resp.data)
    print(f"spawned {ok_n} props")


if __name__ == "__main__":
    main()
