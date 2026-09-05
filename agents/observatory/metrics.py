"""Pure helpers for the single-drone cockpit /state (M4, ICD §8.2).

No ROS imports here on purpose: the pure logic is unit-tested without a
sourced ROS2 environment (see tests/). server.py feeds these functions the
latest px4 messages (duck-typed) and the parsed /pilot/detections snapshot,
and ships the result as JSON on /state.
"""
import math

from agents.observatory import overlay

# PX4 VehicleStatus constants (PX4-Autopilot/msg/VehicleStatus.msg).
ARMING_STATE_ARMED = 2

NAV_STATE_NAMES = {
    0: "MANUAL",
    3: "MISSION",
    4: "HOLD",      # AUTO_LOITER
    14: "OFFBOARD",
    17: "TAKEOFF",  # AUTO_TAKEOFF
    18: "LAND",     # AUTO_LAND
}


def is_armed(arming_state):
    if arming_state is None:
        return None
    return arming_state == ARMING_STATE_ARMED


def mode_name(nav_state):
    return NAV_STATE_NAMES.get(nav_state, f"#{nav_state}")


def heading_deg(heading_rad):
    """PX4 heading is -pi..pi. Return 0..360 degrees (int), or None."""
    if heading_rad is None:
        return None
    return round(math.degrees(heading_rad) % 360.0)


def rpy_from_quat(q):
    """PX4 VehicleAttitude.q (w,x,y,z) -> (roll, pitch, yaw) in degrees
    (yaw wrapped 0..360), or None when no attitude has arrived."""
    if q is None or len(q) < 4:
        return None
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (round(math.degrees(roll), 1), round(math.degrees(pitch), 1),
            round(math.degrees(yaw) % 360.0, 1))


def _r(v, n=1):
    return round(v, n) if v is not None else None


def build_state(pos, status, batt, *, att=None, cam_seq=0, cam_stamp=None,
                snapshot=None, contacts=None, annotations=None,
                pinpoint_mask=None, slowlane=None):
    """Assemble the single-drone /state dict (any input may be None).

    pos      -> VehicleLocalPosition (x, y, z, vx, vy, vz, heading)
    status   -> VehicleStatus (arming_state, nav_state)
    batt     -> BatteryStatus (remaining[-1..1], voltage_v, warning)
    att      -> (roll, pitch, yaw) degrees from rpy_from_quat, or None
    cam_seq  -> latest camera frame seq (0 = no frame yet)
    cam_stamp-> latest camera frame sim_stamp (None = no frame yet)
    snapshot -> parsed /pilot/detections PerceptionSnapshot v1 dict, or None
    contacts -> contact views to serve INSTEAD of snapshot's (M3: the
                fp_suspect-marked copy from overlay.mark_fp_suspects)
    annotations   -> M3 slowlane view from overlay.annotations_for ([] when
                absent/expired — never stale advisory state)
    pinpoint_mask -> M3 /pilot/deep passthrough from overlay.pinpoint_mask_for
    slowlane -> the slowlane payload's health dict (process state, not
                frame-gated), or None
    """
    return {
        "north": _r(pos.x) if pos else None,
        "east": _r(pos.y) if pos else None,
        "alt": _r(-pos.z) if pos else None,
        "speed": _r(math.hypot(pos.vx, pos.vy)) if pos else None,
        "vspeed": _r(-pos.vz) if pos else None,
        "heading": heading_deg(pos.heading) if pos else None,
        "roll": att[0] if att else None,
        "pitch": att[1] if att else None,
        "yaw": att[2] if att else None,
        "armed": is_armed(status.arming_state) if status else None,
        "mode": mode_name(status.nav_state) if status else None,
        "batt_pct": round(batt.remaining * 100) if batt and batt.remaining is not None and batt.remaining >= 0 else None,
        "voltage": _r(batt.voltage_v) if batt and batt.voltage_v is not None and batt.voltage_v > 0 else None,
        "warn": batt.warning if batt else None,
        "cam": cam_seq > 0,
        "cam_seq": cam_seq,
        "cam_stamp": cam_stamp,
        "sim_stamp": snapshot.get("sim_stamp") if snapshot else None,
        "detector": snapshot.get("detector") if snapshot else None,
        "beam": snapshot.get("beam") if snapshot else None,
        "track": snapshot.get("track") if snapshot else None,
        "contacts": (contacts if contacts is not None
                     else (snapshot.get("contacts") if snapshot else None)),
        "annotations": annotations or [],
        "pinpoint_mask": pinpoint_mask,
        "slowlane": slowlane,
        "banner": overlay.hud_banner(snapshot),
    }
