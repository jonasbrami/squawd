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



class _FerryWorld:
    """States: landed_away -> (takeoff) airborne_away -> (goto home) airborne_home
    -> (land) home. RTL is deliberately a trap: PX4 re-set home AT ARMING, so RTL
    lands the drone back at the ferry spot."""
    spawn_x = 0.0
    spawn_spacing = 2.0

    def __init__(self, start="landed_away"):
        self.state = start

    def world_xy(self, bridge, i):
        return {"landed_away": (-100.0, 0.0, 0.2),
                "landed_away_drifted": (-100.0, 0.0, 2.9),
                "airborne_away": (-100.0, 0.0, 10.0),
                "airborne_home": (0.0, 0.0, 10.0),
                "home": (0.0, 0.0, 0.2)}[self.state]


class _FerryAction:
    def __init__(self, world):
        self.world = world
        self.calls = []

    async def set_takeoff_altitude(self, a):
        self.calls.append("set_takeoff_altitude")

    async def arm(self):
        self.calls.append("arm")

    async def takeoff(self):
        self.calls.append("takeoff")
        self.world.state = "airborne_away"

    async def goto_location(self, lat, lon, alt, yaw):
        self.calls.append("goto_location")
        self.world.state = "airborne_home"

    async def land(self):
        self.calls.append("land")
        if self.world.state == "airborne_home":
            self.world.state = "home"

    async def return_to_launch(self):
        self.calls.append("rtl")
        # PX4 home was re-set AT ARMING (the ferry spot): RTL from ANY airborne
        # state flies back THERE — including from over world home (observed live:
        # the RTL wave caught the ferried drone mid-descent and teleported it
        # back to the w2 checkpoint it was stranded at).
        if self.world.state in ("airborne_away", "airborne_home"):
            self.world.state = "landed_away"


class _FerryTelemetry:
    def __init__(self, world):
        self.world = world

    async def armed(self):
        yield self.world.state in ("airborne_away", "airborne_home")

    async def position(self):
        class P:
            latitude_deg = 47.0
            longitude_deg = 8.0
            absolute_altitude_m = 500.0
        while True:
            yield P()


class _FerrySystem:
    def __init__(self, world):
        self.action = _FerryAction(world)
        self.telemetry = _FerryTelemetry(world)


def test_soft_reset_ferries_home_a_drone_landed_away():
    """A cell can legitimately END with the drone landed away from home. RTL is
    doubly useless there: a no-op while disarmed, and after re-arming it returns
    to the RE-ARM spot (PX4 sets home at arming — observed live, 99.9m from home
    after every ferry). The ferry must fly to WORLD home itself and land."""
    w = _FerryWorld()
    s = _FerrySystem(w)
    r = asyncio.run(soft_reset([s], w, None, 1, timeout_s=5.0, poll_interval_s=0.01))
    assert r.ok, r.reason
    for step in ("takeoff", "goto_location", "land"):
        assert step in s.action.calls
    assert s.action.calls.index("takeoff") < s.action.calls.index("goto_location")


def test_soft_reset_ferry_triggers_on_disarmed_even_with_drifted_altitude():
    """A parked drone's EKF altitude drifts (observed ~2m after 40min) — the ferry
    must key on the DISARMED state, not a grounded-altitude threshold."""
    w = _FerryWorld(start="landed_away_drifted")
    s = _FerrySystem(w)
    r = asyncio.run(soft_reset([s], w, None, 1, timeout_s=5.0, poll_interval_s=0.01))
    assert r.ok, r.reason
    assert "goto_location" in s.action.calls


def test_soft_reset_never_rtls_a_ferried_drone():
    """The RTL wave must EXCLUDE ferried drones: they re-armed away from home, so
    their PX4 home is the stranding point — RTL would undo the ferry (observed
    live at N=2: ferry 'succeeded', RTL flew the drone straight back out)."""
    w = _FerryWorld()
    s = _FerrySystem(w)
    r = asyncio.run(soft_reset([s], w, None, 1, timeout_s=5.0, poll_interval_s=0.01))
    assert r.ok, r.reason
    assert "rtl" not in s.action.calls
    assert w.state == "home"
