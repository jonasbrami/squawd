"""M3a ops-layer tests: O2 LOST semantics, O3 velocity dispatch, O4
contact-aware resolution, O5 face heading-wait, scan 'alt unk', and the
pipeline starvation regression (Fable-B2)."""
import asyncio
import math
import time
from types import SimpleNamespace

import pytest

from agents.flight.ops import FlightOps
from agents.flight.track import TargetEstimator
from agents.perception.perception import scan_text


# ---------- fakes ----------

class FakeContacts:
    def __init__(self, poses=None, sim_t=0.0, velocities=None, views=None,
                 lost_s=2.0):
        self._poses = poses or {}
        self.sim_t = sim_t
        self._vels = velocities or {}
        self._views = views or []
        self.config = SimpleNamespace(lost_s=lost_s)

    def poses(self):
        return dict(self._poses)

    def sim_time(self):
        return self.sim_t

    def velocities(self):
        return dict(self._vels)

    def all_views(self):
        return list(self._views)

    def set(self, name, pos):
        if pos is None:
            self._poses.pop(name, None)
        else:
            self._poses[name] = pos


class FakeOffboard:
    def __init__(self):
        self.streamed = []
        self.started = False
        self.stopped = False

    async def set_position_velocity_ned(self, pos, vel):
        self.streamed.append((pos, vel))

    async def start(self):
        self.started = True

    async def stop(self):
        self.stopped = True


class FakeAction:
    def __init__(self):
        self.goto_calls = []
        self.held = False

    async def goto_location(self, lat, lon, alt, yaw):
        self.goto_calls.append(yaw)

    async def hold(self):
        self.held = True


class _AsyncIter:
    def __init__(self, values):
        self._values = list(values)
        self._i = 0

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self._i >= len(self._values):
            raise StopAsyncIteration
        v = self._values[self._i]
        self._i += 1
        return v


class FakeTelemetry:
    def __init__(self, headings):
        self._headings = headings

    def position(self):
        return _AsyncIter([SimpleNamespace(latitude_deg=47.0, longitude_deg=8.0,
                                           absolute_altitude_m=410.0)])

    def heading(self):
        return _AsyncIter(self._headings)


class FakeDrone:
    def __init__(self, headings=()):
        self.offboard = FakeOffboard()
        self.action = FakeAction()
        self.telemetry = FakeTelemetry(headings)
        self.mission = SimpleNamespace(pause_mission=self._pause)

    async def _pause(self):
        pass


class FakeWorld:
    buildings = []

    def world_xy(self, bridge, i):
        return (0.0, 0.0, 6.0)

    def drone_state(self, bridge, i):
        return (0.0, 0.0, 6.0, 0.0)


class FakeBridge:
    def latest(self, topic):
        return SimpleNamespace(x=0.0, y=0.0, z=-6.0)


def _ops(contacts, headings=()):
    return FlightOps(FakeDrone(headings), FakeWorld(), FakeBridge(), 0, 1,
                     contacts=contacts)


# ---------- O3: velocity dispatch ----------

def test_o3_feed_direct_bypasses_the_ema():
    est = TargetEstimator()
    est.feed_direct(3.5, -1.0)
    assert est.ready
    assert est.ve == pytest.approx(3.5)
    assert est.vn == pytest.approx(-1.0)
    # and it is not dragged by a subsequent EMA update toward the fd velocity
    est.update(1.0, 0.0, 0.0)
    assert est.ve == pytest.approx(3.5)


def test_o3_track_dispatches_feed_direct_when_velocities_available():
    contacts = FakeContacts(poses={"vis_t": (50.0, 0.0)},
                            velocities={"vis_t": (2.0, 0.0)})
    ops = _ops(contacts)
    called = []
    orig = TargetEstimator.feed_direct
    TargetEstimator.feed_direct = lambda self, ve, vn: (
        called.append((ve, vn)), orig(self, ve, vn))[1]
    try:
        r = asyncio.run(ops.track("vis_t", duration_s=1.0))
    finally:
        TargetEstimator.feed_direct = orig
    assert called and called[0] == (2.0, 0.0)
    assert "shadowed" in r


def test_o3_track_falls_back_to_ema_with_empty_velocities():
    contacts = FakeContacts(poses={"mov_1": (50.0, 0.0)}, velocities={})
    ops = _ops(contacts)
    r = asyncio.run(ops.track("mov_1", duration_s=1.0))
    assert "shadowed" in r            # ran the EMA path without crashing


# ---------- O2: LOST semantics ----------

def _dropout_contacts(drop_at, back_at=None, sim_per_tick=0.1):
    """poses() drops the contact between drop_at and back_at (sim seconds)."""
    c = FakeContacts(poses={"vis_t": (50.0, 0.0)})
    orig_poses = c.poses

    def poses():
        c.sim_t += sim_per_tick
        if drop_at <= c.sim_t < (back_at if back_at is not None else 1e9):
            return {}
        return orig_poses()
    c.poses = poses
    return c


def test_o2_structured_lost_after_dropout_beyond_lost_s():
    contacts = _dropout_contacts(drop_at=0.5, back_at=None)
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_t", duration_s=30.0))
    assert r.startswith("drone_0 LOST:"), r
    assert "holding" in r
    assert ops.drone.offboard.stopped          # offboard left cleanly, no flyaway


def test_o2_recovers_when_dropout_is_shorter_than_lost_s():
    contacts = _dropout_contacts(drop_at=0.5, back_at=1.5)
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_t", duration_s=2.5))
    assert "LOST" not in r, r
    assert "shadowed" in r


# ---------- O4: contact-aware resolution ----------

def test_o4_bearing_only_contact_enters_acquisition():
    """O6: a bearing-only contact is DESIGNATED and acquired (yaw onto the
    bearing, altitude bias), not refused — and when no beam lock lands inside
    the budget, track returns a legible NOT_ACQUIRED result."""
    view = SimpleNamespace(name="vis_target_0", position_src="none",
                           bearing_deg=45.0, elevation_deg=0.0)
    calls = []

    class AcqContacts(FakeContacts):
        def designate(self, name, **kw):
            calls.append(("designate", name))

        def clear_designation(self):
            calls.append(("clear",))

        def observation(self, name):
            return view

    contacts = AcqContacts(poses={}, views=[view])
    ops = _ops(contacts)
    ops_drone = ops.drone
    r = asyncio.run(ops.track("vis_target_0", duration_s=1.0, acquire_budget_s=3.0))
    assert ("designate", "vis_target_0") in calls
    assert "could NOT acquire" in r
    # acquisition aims via offboard yaw streaming (a same-position goto is
    # ignored by PX4 — offboard is the working mechanism, M3b)
    assert ops_drone.offboard.streamed


def test_o4_unknown_contact_lists_both_kinds():
    view = SimpleNamespace(name="vis_target_1", position_src="none")
    contacts = FakeContacts(poses={"mov_2": (1.0, 2.0)}, views=[view])
    ops = _ops(contacts)
    with pytest.raises(ValueError, match="bearing only"):
        asyncio.run(ops.track("nope"))


# ---------- O5: face heading-wait ----------

def test_o5_face_waits_until_heading_lands():
    headings = [SimpleNamespace(heading_deg=h) for h in (10.0, 60.0, 88.0, 90.0)]
    ops = _ops(FakeContacts(), headings=headings)
    r = asyncio.run(ops.face("east"))
    assert "facing east" in r, r
    assert ops.drone.action.goto_calls and abs(ops.drone.action.goto_calls[-1] - 90.0) < 1e-6


# ---------- scan: alt unk ----------

class _ScanWorld:
    buildings = []

    def drone_state(self, bridge, i):
        return (0.0, 0.0, 6.0, 0.0)


def test_scan_renders_alt_unk_for_bearing_only_contacts():
    txt = scan_text(_ScanWorld(), None, 0, 1,
                    mover_poses={"mov_1": (10.0, 0.0, 1.2)},
                    bearing_only=["vis_target_3"])
    assert "mov_1" in txt and "alt 1m" in txt
    assert "vis_target_3" in txt and "alt unk" in txt


# ---------- pipeline starvation (Fable-B2 regression) ----------

def test_pipeline_keeps_publishing_while_the_agent_turn_is_held_open():
    """The pipeline ticks on its own task; Detector.wait_next blocks on a
    threading.Condition and MUST be driven off-loop (to_thread) — before the
    fix, a quiet 0.5 s wait stalled the whole agent loop."""
    import threading
    from agents.core.contact import Frame
    from agents.vision.pipeline import VisionPipeline
    from agents.vision.types import InferenceResult

    class SpinDetector:
        """Realistic wait_next: blocks on a Condition until a result lands or
        the timeout lapses (the blocking the regression is about)."""

        def __init__(self):
            self._seq = 0
            self._cond = threading.Condition()

        def healthy(self):
            return True

        def latency_ms(self):
            return 1.0

        def wait_next(self, after_seq, timeout):
            with self._cond:
            # simulate ~20 Hz production: 50 ms per result
                self._cond.wait(timeout=min(timeout, 0.05))
            self._seq += 1
            f = Frame(self._seq, 100.0 + self._seq, 4, 4, bytes(4 * 4 * 3))
            return InferenceResult(f, [], time.monotonic(), self._seq, None)

    async def run():
        pipe = VisionPipeline(SpinDetector(), contacts=None, bridge=None)
        pipe.start()
        first = pipe.latest().frame_seq if pipe.latest() else 0
        t0 = time.monotonic()
        # hold "the agent's turn" open: the pipeline must keep ticking anyway,
        # and the held turn must complete on time (the loop is not starved)
        await asyncio.sleep(0.6)
        held_for = time.monotonic() - t0
        pipe.stop()
        last = pipe.latest().frame_seq if pipe.latest() else 0
        assert last > first + 3, f"pipeline starved: {first} -> {last}"
        assert held_for < 0.9, f"agent loop starved: sleep(0.6) took {held_for:.2f}s"

    asyncio.run(run())


def test_o4_successful_acquisition_does_not_raise_unknown():
    """v15 splice-bug regression: after a SUCCESSFUL _acquire (ToF lock ->
    positioned), track() must proceed to the pursuit, NOT fall through to
    the unknown-contact ValueError (the suite only covered the failure
    paths, which is how the bug shipped)."""
    view = SimpleNamespace(name="vis_target_0", position_src="none",
                           bearing_deg=45.0, elevation_deg=0.0, age_s=0.0,
                           foot_px=None)

    class AcqContacts(FakeContacts):
        def designate(self, name, **kw):
            # the beam locks: the contact becomes positioned + RANGE_LOCKED
            self._poses[name] = (10.0, 20.0, 1.2)

        def track_state(self, name):
            return "RANGE_LOCKED"

        def observation(self, name):
            return view

    contacts = AcqContacts(poses={}, views=[view])
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_target_0", duration_s=1.0,
                              acquire_budget_s=3.0))
    assert "unknown moving contact" not in r
