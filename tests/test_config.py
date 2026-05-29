import pytest
from dronebot.config import load_config, Config
from dronebot.control.safety import SafetyLimits


def test_defaults_are_conservative(monkeypatch):
    monkeypatch.delenv("DRONEBOT_MAX_ALTITUDE_M", raising=False)
    cfg = load_config()
    assert isinstance(cfg, Config)
    assert isinstance(cfg.limits, SafetyLimits)
    # Fail-closed: defaults are tight.
    assert cfg.limits.max_altitude_m <= 30.0
    assert cfg.limits.geofence_radius_m <= 100.0


def test_env_overrides_limit(monkeypatch):
    monkeypatch.setenv("DRONEBOT_MAX_ALTITUDE_M", "15")
    cfg = load_config()
    assert cfg.limits.max_altitude_m == 15.0


def test_connection_url_default(monkeypatch):
    monkeypatch.delenv("DRONEBOT_CONNECTION_URL", raising=False)
    cfg = load_config()
    assert cfg.connection_url == "udp://:14540"


def test_invalid_numeric_env_raises_clear_error(monkeypatch):
    monkeypatch.setenv("DRONEBOT_MAX_ALTITUDE_M", "thirty")
    with pytest.raises(ValueError, match="DRONEBOT_MAX_ALTITUDE_M"):
        load_config()
