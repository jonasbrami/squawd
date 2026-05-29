# tests/test_telemetry.py
import asyncio
from dronebot.control.state import StateStore
from dronebot.control.telemetry import update_position_stream


class _FakePosition:
    def __init__(self, lat, lon, abs_alt):
        self.latitude_deg = lat
        self.longitude_deg = lon
        self.absolute_altitude_m = abs_alt
        self.relative_altitude_m = abs_alt - 500.0


async def _one_position():
    yield _FakePosition(47.0, 8.0, 510.0)


async def test_position_stream_updates_store():
    store = StateStore()
    await update_position_stream(_one_position(), store)
    assert store.position is not None
    assert store.position.absolute_altitude_m == 510.0
