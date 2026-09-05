"""Environment awareness for the drone agents — pure trig + text, no I/O.

Turns a World (ground-truth buildings + spawn layout) plus live ROS telemetry
into VLM-friendly readouts:
- scan_text(i):  nearest buildings + other drones with distance and bearing
                 RELATIVE to where the drone faces, flagging what's in view.
- situation_text(): commander-level overview of every drone + its nearest building.

The live camera image (`look`) is served separately by core.GzCameras.

Camera is body-fixed, pointing forward (along heading) with ~69deg horizontal
FOV (hfov 1.204 rad). Something is "in view" if its bearing relative to the
drone's heading is within ~half the FOV.
"""
import math

FOV_HALF_DEG = 35.0
# Movers beyond this range are silently absent from scan: search tasks need a
# bounded sensor (unbounded scan makes "find the moving target" a no-op) and
# distant task-regions' movers would otherwise be pure prompt noise.
MOVER_SCAN_RANGE_M = 150.0


def bearing_word(d_east: float, d_north: float) -> str:
    """8-point compass for a world-frame delta (0deg = North, 90deg = East)."""
    ang = math.degrees(math.atan2(d_east, d_north)) % 360.0
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int((ang + 22.5) // 45) % 8]


def heading_word(heading_rad: float) -> str:
    """Compass direction the drone (and its camera) faces."""
    return bearing_word(math.sin(heading_rad), math.cos(heading_rad))


def rel_bearing(d_east: float, d_north: float, heading_rad: float):
    """Bearing of a target relative to where the drone faces.
    Returns (label, in_view, rel_deg). rel_deg: 0=straight ahead, +=right, -=left."""
    world = math.degrees(math.atan2(d_east, d_north))       # 0=N, 90=E
    rel = ((world - math.degrees(heading_rad)) + 180) % 360 - 180
    a = abs(rel)
    if a <= 22.5:
        word = "ahead"
    elif a <= 67.5:
        word = "ahead-right" if rel > 0 else "ahead-left"
    elif a <= 112.5:
        word = "right" if rel > 0 else "left"
    elif a <= 157.5:
        word = "behind-right" if rel > 0 else "behind-left"
    else:
        word = "behind"
    return word, a <= FOV_HALF_DEG, rel


def yaw_deg_to(east_from: float, north_from: float, east_to: float, north_to: float) -> float:
    """Heading (deg, NED: 0=N, +clockwise toward E) to face a world point."""
    return math.degrees(math.atan2(east_to - east_from, north_to - north_from))


def scan_text(world, bridge, i: int, n_drones: int, k: int = 8,
              mover_poses: dict | None = None,
              bearing_only: list | None = None) -> str:
    """Nearest k buildings + other drones, with distance and bearing RELATIVE to
    where the drone faces, flagging what's in the camera's view.

    k=8 covers every building in current worlds: with k=4 a drone planned a
    perfect route around the four buildings it was told about and flew into the
    fifth (obs_4 was #5-nearest from spawn). If a world ever exceeds k, the
    truncation must be announced in the text, never silent."""
    st = world.drone_state(bridge, i)
    if st is None:
        return f"drone_{i}: position not yet available"
    mx, my, alt, hd = st
    parts = []
    ranked = []
    for b in world.buildings:
        dx, dy = b["x"] - mx, b["y"] - my
        radius = 0.5 * math.hypot(b["w"], b["d"])        # approx footprint half-extent
        edge = max(0.0, math.hypot(dx, dy) - radius)     # distance to the wall, not centre
        ranked.append((edge, b, dx, dy))
    ranked.sort(key=lambda t: t[0])
    for edge, b, dx, dy in ranked[:k]:
        word, inview, _ = rel_bearing(dx, dy, hd)
        tag = " [IN VIEW]" if inview else ""
        # World-frame centre + footprint, not just an edge distance: a clearance
        # route around a rectangle is unplannable without knowing where its walls
        # are (a drone flew a sensible detour straight into a corner it couldn't see).
        parts.append(f"{b['name']} {edge:.0f}m {word}{tag} "
                     f"(centre E{b['x']:.0f} N{b['y']:.0f}, {b['w']:.0f}x{b['d']:.0f}m, "
                     f"h={b['h']:.0f}m)")
    for name in sorted(mover_poses or {}):
        # Live contacts: position + bearing ONLY — no velocity, no trajectory.
        # Deriving a contact's course by differencing two scans over a known
        # interval is a deliberate capability probe (see dynamic-scenarios spec);
        # names stay semantically neutral so the script never leaks.
        cx, cy, cz = mover_poses[name]
        dx, dy = cx - mx, cy - my
        dist = math.hypot(dx, dy)
        if dist > MOVER_SCAN_RANGE_M:
            continue
        word, inview, _ = rel_bearing(dx, dy, hd)
        tag = " [IN VIEW]" if inview else ""
        parts.append(f"contact {name} {dist:.0f}m {word}{tag} "
                     f"(at E{cx:.0f} N{cy:.0f}, alt {cz:.0f}m)")
    for name in sorted(bearing_only or []):
        # Vision contacts seen but not yet ranged: honest "alt unk" — never
        # a fabricated position (M3a scan contract).
        parts.append(f"contact {name} bearing only, alt unk")
    for j in range(n_drones):
        if j == i:
            continue
        oj = world.world_xy(bridge, j)
        if oj is None:
            continue
        dx, dy = oj[0] - mx, oj[1] - my
        word, inview, _ = rel_bearing(dx, dy, hd)
        tag = " [IN VIEW]" if inview else ""
        parts.append(f"drone_{j} {math.hypot(dx, dy):.0f}m {word}{tag}")
    head = (f"drone_{i} at world (E{mx:.0f}, N{my:.0f}), alt {alt:.0f}m, facing "
            f"{heading_word(hd)}. Your camera shows what's 'ahead' / [IN VIEW]; "
            f"turn (face) to bring other things into view. Nearby:")
    return head + (" " + " | ".join(parts) if parts else " nothing close")


def situation_text(world, bridge, n_drones: int) -> str:
    """Commander-level overview: each drone's position + its single nearest building."""
    lines = []
    for i in range(n_drones):
        st = world.drone_state(bridge, i)
        if st is None:
            lines.append(f"drone_{i}: (no telemetry)")
            continue
        mx, my, alt, hd = st
        facing = heading_word(hd)
        nearest = ""
        if world.buildings:
            b = min(world.buildings, key=lambda b: math.hypot(b["x"] - mx, b["y"] - my))
            dx, dy = b["x"] - mx, b["y"] - my
            nearest = f"; nearest {b['name']} {math.hypot(dx, dy):.0f}m {bearing_word(dx, dy)} (h={b['h']:.0f}m)"
        lines.append(f"drone_{i}: world E{mx:.0f} N{my:.0f} alt {alt:.0f}m facing {facing}{nearest}")
    return "\n".join(lines)
