"""Image-space -> world-space projection (ICD §4.2). Pure functions, no I/O —
unit-testable with plain values.

Camera model: body-fixed forward camera, hfov 1.204 rad (~69deg), 640x360 in
the current sim patch. Bearings use the pinhole-intrinsics form (the same angle
parameterization as MAVLink LANDING_TARGET): ax = atan((u-cx)/fx).
Footpoint = box BOTTOM-center row — projecting a centroid overestimates range
at oblique angles.
"""
import math

HFOV_DEG = 69.0        # camera SDF hfov 1.204 rad (verified in M2 against SDF)
HFOV_RAD = math.radians(HFOV_DEG)


def vfov_deg(img_w: int, img_h: int, hfov_deg: float = HFOV_DEG) -> float:
    """Vertical FOV derived from hfov + aspect: 2*atan(tan(hfov/2) * h/w)."""
    return math.degrees(2.0 * math.atan(
        math.tan(math.radians(hfov_deg) / 2.0) * img_h / img_w))


def pixel_to_angles(u: float, v: float, img_w: int, img_h: int,
                    hfov_deg: float = HFOV_DEG) -> tuple[float, float]:
    """(angle_x, angle_y) RADIANS of a pixel: 0 = boresight, +x = right,
    +y = DOWN from boresight (depression-positive). Pinhole intrinsics in
    BOTH axes: fx = (w/2)/tan(hfov/2), fy from the derived vfov."""
    fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    vf = vfov_deg(img_w, img_h, hfov_deg)
    fy = (img_h / 2.0) / math.tan(math.radians(vf) / 2.0)
    ax = math.atan((u - img_w / 2.0) / fx)
    ay = math.atan((v - img_h / 2.0) / fy)
    return ax, ay


def ray_support_range(angle_x: float, angle_y: float, *, roll: float,
                      pitch: float, alt: float,
                      support_z: float = 0.0) -> float | None:
    """Slant range where the camera ray meets the plane z = support_z, or None
    when the ray does not converge (depression <= ~1deg): the ray through
    (angle_x, angle_y) rotated by vehicle roll/pitch, from altitude `alt`.

    v1 approximation: camera boresight forward, so the ground depression is
    dep = ay·cos(roll) − pitch + ax·sin(roll) (PX4 pitch is nose-UP positive;
    the roll term is first-order — at ax=20°, roll=12° the roll-less form
    misplaces the depression ~4°, and that garbage passes the 6° envelope
    check from inside; the ay+pitch sign bug inverted the pitch term entirely
    until 2026-07-21). A forward-ray's ground intersection is slant =
    (alt - support_z) / sin(dep), and we additionally require the ray to
    point forward (angle_x within hfov)."""
    dep = angle_y * math.cos(roll) - pitch + angle_x * math.sin(roll)
    if dep < math.radians(1.0):
        return None
    drop = alt - support_z
    if drop <= 0.0:
        return None
    return drop / math.sin(dep)


def contact_world(me_e: float, me_n: float, heading_rad: float,
                  angle_x: float, slant_range: float) -> tuple[float, float]:
    """World (e, n) of a contact: world bearing = heading + angle_x (rad),
    polar -> cartesian about the drone's pose."""
    b = heading_rad + angle_x
    return (me_e + slant_range * math.sin(b),
            me_n + slant_range * math.cos(b))


def erode_box(xyxy: tuple[float, float, float, float],
              frac: float = 0.22) -> tuple[float, float, float, float]:
    """Shrink a box by `frac` of w/h on each side (beam-footprint margin,
    design §3.10's eroded-box association fallback)."""
    x1, y1, x2, y2 = xyxy
    dx = (x2 - x1) * frac
    dy = (y2 - y1) * frac
    return (x1 + dx, y1 + dy, x2 - dx, y2 - dy)


def footprint_in_region(foot_c: tuple[float, float], foot_r_px: float,
                        region_xyxy: tuple[float, float, float, float]) -> bool:
    """True iff the beam footprint DISC (center px, radius px) lies entirely
    inside a mask/box region."""
    cx, cy = foot_c
    x1, y1, x2, y2 = region_xyxy
    return (cx - foot_r_px >= x1 and cx + foot_r_px <= x2
            and cy - foot_r_px >= y1 and cy + foot_r_px <= y2)
