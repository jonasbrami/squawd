# src/dronebot/control/geo.py
"""Geodetic offset math. Frame convention (NED-style inputs):
  north_m: + toward geographic north
  east_m:  + toward geographic east
  up_m:    + increases altitude (i.e. -Down)

Local-tangent-plane (equirectangular) approximation about the reference
point. Sub-meter accurate for offsets up to several hundred meters (the v1
envelope). NOT valid near the poles or across the antimeridian.

Altitude note: `absolute_altitude_m` is AMSL; callers building a goto
target offset the drone's current absolute altitude, and the safety layer
compares (target_abs_alt - home_abs_alt) against the altitude cap.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

_WGS84_A = 6378137.0  # Earth equatorial radius (m)


@dataclass(frozen=True)
class GeoPoint:
    latitude_deg: float
    longitude_deg: float
    absolute_altitude_m: float


def offset_point(origin: GeoPoint, north_m: float, east_m: float, up_m: float) -> GeoPoint:
    lat_rad = math.radians(origin.latitude_deg)
    dlat_deg = math.degrees(north_m / _WGS84_A)
    dlon_deg = math.degrees(east_m / (_WGS84_A * math.cos(lat_rad)))
    return GeoPoint(
        latitude_deg=origin.latitude_deg + dlat_deg,
        longitude_deg=origin.longitude_deg + dlon_deg,
        absolute_altitude_m=origin.absolute_altitude_m + up_m,
    )


def horizontal_distance_m(a: GeoPoint, b: GeoPoint) -> float:
    lat_rad = math.radians((a.latitude_deg + b.latitude_deg) / 2.0)
    dn = math.radians(b.latitude_deg - a.latitude_deg) * _WGS84_A
    de = math.radians(b.longitude_deg - a.longitude_deg) * _WGS84_A * math.cos(lat_rad)
    return math.hypot(dn, de)
