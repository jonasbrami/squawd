import asyncio
import pytest

from evals.reset import home_xy, check_home, soft_reset


class FakeWorld:
    spawn_x = 0.0
    spawn_spacing = 3.0

    def __init__(self, xy):
        self._xy = xy  # dict[i] -> (e, n, alt)

    def world_xy(self, bridge, i):
        return self._xy.get(i)


def test_home_xy():
    assert home_xy(FakeWorld({}), 2) == (0.0, 6.0)


def test_check_home_pass():
    w = FakeWorld({0: (0.2, 0.0, 0.1), 1: (0.0, 3.1, 0.1)})
    assert check_home(w, None, 2, tol_m=5.0).ok


def test_check_home_fail_names_drone():
    w = FakeWorld({0: (0.0, 0.0, 0.1), 1: (50.0, 3.0, 0.1)})
    r = check_home(w, None, 2, tol_m=5.0)
    assert not r.ok and "drone_1" in r.reason


def test_check_home_fail_on_missing_fix():
    r = check_home(FakeWorld({0: None}), None, 1, tol_m=5.0)
    assert not r.ok


class FakeSystem:
    class _Action:
        async def return_to_launch(self):
            return None
    def __init__(self):
        self.action = self._Action()


def test_soft_reset_times_out_when_never_home():
    # Drone parked 50 m from home, never converges -> soft_reset returns ok=False
    # after the (short) deadline, without raising.
    w = FakeWorld({0: (50.0, 0.0, 0.1)})
    res = asyncio.run(soft_reset([FakeSystem()], w, None, 1,
                                 tol_m=5.0, timeout_s=0.05, poll_interval_s=0.01))
    assert res.ok is False
    assert "drone_0" in res.reason


def test_soft_reset_reports_rtl_failure():
    # An RTL that raises is aggregated into an infra-fail ResetResult, not propagated.
    class BadSystem:
        class _Action:
            async def return_to_launch(self):
                raise RuntimeError("link lost")
        def __init__(self):
            self.action = self._Action()

    w = FakeWorld({0: (0.0, 0.0, 0.1)})
    res = asyncio.run(soft_reset([BadSystem()], w, None, 1, timeout_s=0.05, poll_interval_s=0.01))
    assert res.ok is False
    assert "RTL command failed" in res.reason


def test_check_home_fails_when_still_airborne():
    """XY at home but 12m up = the leaky-reset state the 2D-only gate waved through:
    the next cell's take_off then starts from altitude, not the ground."""
    from evals.reset import check_home

    class W:
        spawn_x = 0.0
        spawn_spacing = 2.0
        def world_xy(self, bridge, i):
            return (0.0, 0.0, 12.0)

    r = check_home(W(), None, 1, tol_m=5.0)
    assert not r.ok and "airborne" in r.reason


def test_check_home_passes_when_landed_at_home():
    from evals.reset import check_home

    class W:
        spawn_x = 0.0
        spawn_spacing = 2.0
        def world_xy(self, bridge, i):
            return (0.5, -0.5, 0.3)

    assert check_home(W(), None, 1, tol_m=5.0).ok
