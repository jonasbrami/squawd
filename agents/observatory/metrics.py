"""Pure helpers for the observatory's per-drone metrics.

No ROS imports here on purpose: the swarm's pure logic is unit-tested without a
sourced ROS2 environment (see tests/). server.py feeds these functions the latest
px4 messages (duck-typed) and ships the result as JSON on /state.
"""
import math

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


def _r(v, n=1):
    return round(v, n) if v is not None else None


def build_drone_state(i, pos, status, batt, task, report, has_cam):
    """Assemble one drone's /state dict from the latest messages (any may be None).

    pos    -> VehicleLocalPosition (x, y, z, vx, vy, vz, heading)
    status -> VehicleStatus (arming_state, nav_state)
    batt   -> BatteryStatus (remaining[-1..1], voltage_v, warning)
    task   -> last /swarm/cmd/drone_<i> text, or None
    report -> last /swarm/report/drone_<i> text, or None
    has_cam-> whether a camera topic exists for this drone
    """
    return {
        "id": i,
        "north": _r(pos.x) if pos else None,
        "east": _r(pos.y) if pos else None,
        "alt": _r(-pos.z) if pos else None,
        "speed": _r(math.hypot(pos.vx, pos.vy)) if pos else None,
        "vspeed": _r(-pos.vz) if pos else None,
        "heading": heading_deg(pos.heading) if pos else None,
        "armed": is_armed(status.arming_state) if status else None,
        "mode": mode_name(status.nav_state) if status else None,
        "batt_pct": round(batt.remaining * 100) if batt and batt.remaining is not None and batt.remaining >= 0 else None,
        "voltage": _r(batt.voltage_v) if batt and batt.voltage_v is not None and batt.voltage_v > 0 else None,
        "warn": batt.warning if batt else None,
        "task": task,
        "report": report,
        "cam": has_cam,
    }
