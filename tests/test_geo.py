import math
from agents.core.geo import GeoPoint, offset_point

# Zurich-ish reference; values are convention checks, not site-specific.
ORIGIN = GeoPoint(latitude_deg=47.3977, longitude_deg=8.5456, absolute_altitude_m=500.0)


def test_north_offset_increases_latitude():
    # 1 degree of latitude ~= 111319.49 m
    p = offset_point(ORIGIN, north_m=111319.49, east_m=0.0, up_m=0.0)
    assert math.isclose(p.latitude_deg, ORIGIN.latitude_deg + 1.0, abs_tol=1e-3)
    assert math.isclose(p.longitude_deg, ORIGIN.longitude_deg, abs_tol=1e-9)


def test_south_offset_decreases_latitude():
    p = offset_point(ORIGIN, north_m=-50.0, east_m=0.0, up_m=0.0)
    assert p.latitude_deg < ORIGIN.latitude_deg


def test_east_offset_increases_longitude():
    p = offset_point(ORIGIN, north_m=0.0, east_m=100.0, up_m=0.0)
    assert p.longitude_deg > ORIGIN.longitude_deg


def test_up_increases_absolute_altitude():
    p = offset_point(ORIGIN, north_m=0.0, east_m=0.0, up_m=10.0)
    assert math.isclose(p.absolute_altitude_m, 510.0, abs_tol=1e-9)
