# src/dronebot/config.py
"""The single sim-vs-real seam. All limits fail closed (conservative
defaults if unset). sim->hardware should be a config + connection change.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from dronebot.control.safety import SafetyLimits


@dataclass(frozen=True)
class Config:
    connection_url: str
    model: str
    limits: SafetyLimits
    telemetry_rate_hz: float
    # If set ("host:port"), connect to an EXTERNAL mavsdk_server over gRPC
    # instead of the in-process bundled one. Required when the agent forks
    # subprocesses (the Claude CLI), which kills an in-process server.
    mavsdk_server_address: str | None


def _envf(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"invalid value for {name}: {raw!r}") from exc


def load_config() -> Config:
    return Config(
        connection_url=os.environ.get("DRONEBOT_CONNECTION_URL", "udpin://0.0.0.0:14540"),
        model=os.environ.get("DRONEBOT_MODEL", "claude-opus-4-8"),
        telemetry_rate_hz=_envf("DRONEBOT_TELEMETRY_RATE_HZ", 4.0),
        mavsdk_server_address=os.environ.get("DRONEBOT_MAVSDK_SERVER") or None,
        limits=SafetyLimits(
            max_altitude_m=_envf("DRONEBOT_MAX_ALTITUDE_M", 30.0),
            geofence_radius_m=_envf("DRONEBOT_GEOFENCE_RADIUS_M", 100.0),
            max_goto_distance_m=_envf("DRONEBOT_MAX_GOTO_DISTANCE_M", 60.0),
            min_takeoff_altitude_m=_envf("DRONEBOT_MIN_TAKEOFF_ALT_M", 2.0),
            max_takeoff_altitude_m=_envf("DRONEBOT_MAX_TAKEOFF_ALT_M", 20.0),
        ),
    )
