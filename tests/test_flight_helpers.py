import math

from mavsdk.mission import MissionItem

from agents.flight.ops import _mission_item


def test_mission_item_defaults_are_nan_and_enums_none():
    it = _mission_item()
    assert math.isnan(it.latitude_deg)
    assert math.isnan(it.longitude_deg)
    assert math.isnan(it.relative_altitude_m)
    assert math.isnan(it.speed_m_s)
    assert it.is_fly_through is True
    assert it.camera_action == MissionItem.CameraAction.NONE
    assert it.vehicle_action == MissionItem.VehicleAction.NONE


def test_mission_item_overrides_apply():
    it = _mission_item(latitude_deg=1.5, longitude_deg=2.5,
                       relative_altitude_m=15.0, speed_m_s=5.0,
                       is_fly_through=False)
    assert it.latitude_deg == 1.5
    assert it.longitude_deg == 2.5
    assert it.relative_altitude_m == 15.0
    assert it.speed_m_s == 5.0
    assert it.is_fly_through is False
    # untouched fields keep their nan default
    assert math.isnan(it.yaw_deg)
