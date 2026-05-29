# src/dronebot/web/framing.py
"""Pure helpers for the web layer. No I/O — unit-testable without the sim."""
from __future__ import annotations

from dronebot.control.state import StateStore
from dronebot.perception.store import PerceptionStore

_BOUNDARY = b"frame"


def mjpeg_part(jpeg: bytes) -> bytes:
    """Frame one JPEG as a multipart/x-mixed-replace part."""
    return (
        b"--" + _BOUNDARY + b"\r\n"
        + b"Content-Type: image/jpeg\r\n"
        + b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
        + jpeg + b"\r\n"
    )


def telemetry_frame(state: StateStore, perception: PerceptionStore) -> dict:
    """Serialize the authoritative state + surroundings for the cockpit."""
    snap = state.snapshot()
    pos = snap.position
    home = snap.home
    rel_alt = None
    if pos is not None and home is not None:
        rel_alt = pos.absolute_altitude_m - home.absolute_altitude_m
    return {
        "connected": snap.is_connected,
        "armed": snap.is_armed,
        "in_air": snap.in_air,
        "flight_mode": snap.flight_mode,
        "battery": state.battery_remaining,
        "rel_alt": rel_alt,
        "position": None if pos is None else {
            "lat": pos.latitude_deg,
            "lon": pos.longitude_deg,
            "abs_alt": pos.absolute_altitude_m,
        },
        "home": None if home is None else {
            "lat": home.latitude_deg,
            "lon": home.longitude_deg,
            "abs_alt": home.absolute_altitude_m,
        },
        "surroundings": perception.surroundings_summary(),
    }
