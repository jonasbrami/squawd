import pytest
from dronebot.control.geo import GeoPoint, offset_point
from dronebot.control.safety import (
    SafetyGuard, SafetyLimits, SafetyError, DroneSnapshot,
)

HOME = GeoPoint(47.3977, 8.5456, 500.0)
LIMITS = SafetyLimits(
    max_altitude_m=30.0,
    geofence_radius_m=100.0,
    max_goto_distance_m=60.0,
    min_takeoff_altitude_m=2.0,
    max_takeoff_altitude_m=20.0,
)


def flying_snapshot(**over):
    base = dict(
        is_connected=True, is_armed=True, in_air=True, has_position=True,
        flight_mode="HOLD", home=HOME, position=HOME,
    )
    base.update(over)
    return DroneSnapshot(**base)


def test_arm_requires_connection():
    guard = SafetyGuard(LIMITS)
    with pytest.raises(SafetyError):
        guard.check_arm(flying_snapshot(is_connected=False))


def test_takeoff_rejected_when_not_armed():
    guard = SafetyGuard(LIMITS)
    snap = flying_snapshot(in_air=False, is_armed=False)
    with pytest.raises(SafetyError):
        guard.check_takeoff(10.0, snap)


def test_takeoff_altitude_above_cap_rejected():
    guard = SafetyGuard(LIMITS)
    snap = flying_snapshot(in_air=False)
    with pytest.raises(SafetyError):
        guard.check_takeoff(999.0, snap)


def test_takeoff_ok_within_limits():
    guard = SafetyGuard(LIMITS)
    snap = flying_snapshot(in_air=False)
    guard.check_takeoff(10.0, snap)  # no raise


def test_goto_requires_position_fix():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 10.0)
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot(has_position=False))


def test_goto_outside_geofence_rejected():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 200.0, 0.0, 10.0)  # 200m > 100m radius
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot())


def test_goto_exceeding_per_command_distance_rejected():
    guard = SafetyGuard(LIMITS)
    # within geofence but >60m from current position
    far = offset_point(HOME, 80.0, 0.0, 10.0)
    snap = flying_snapshot(position=HOME)
    with pytest.raises(SafetyError):
        guard.check_goto(far, snap)


def test_goto_above_altitude_cap_rejected():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 50.0)  # 50m > 30m cap
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot())


def test_goto_during_failsafe_rejected():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 10.0)
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot(flight_mode="RETURN_TO_LAUNCH"))


def test_goto_ok_within_all_limits():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 10.0, 10.0)
    guard.check_goto(target, flying_snapshot())  # no raise


def test_goto_rejected_when_disarmed():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 10.0)
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot(is_armed=False))
