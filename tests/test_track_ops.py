"""M3a ops-layer tests: O2 LOST semantics, O3 velocity dispatch, O4
contact-aware resolution, O5 face heading-wait, scan 'alt unk', the
pipeline starvation regression (Fable-B2), and the d2 truth-shadow
regression (direct reference vs the M3b shaped lane). W3a: the locked-object
orbit/stand-off controller (control_ref geometry, θ init, keep-out clamp,
the direct-lane FF stream, and the truth-style orbit smoke)."""
import asyncio
import math
import random
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
    return FlightOps(FakeDrone(headings), FakeWorld(), FakeBridge(),
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


# ---------- M6 commitment safeguards: alt default + envelope guard ----------

def test_track_alt_defaults_to_current_altitude():
    """alt omitted -> the pursuit holds the drone's CURRENT altitude (the
    FakeWorld hovers at 6.0m; bridge local z=-6.0 -> off_d=0 -> down_m=-alt)
    instead of the old silent fixed 12m default."""
    contacts = FakeContacts(poses={"mov_1": (50.0, 0.0)})
    ops = _ops(contacts)
    r = asyncio.run(ops.track("mov_1", duration_s=1.0))
    assert "shadowed" in r
    assert ops.drone.offboard.streamed[0][0].down_m == pytest.approx(-6.0)


def test_track_explicit_alt_still_honored():
    contacts = FakeContacts(poses={"mov_1": (50.0, 0.0)})
    ops = _ops(contacts)
    asyncio.run(ops.track("mov_1", alt=9.0, duration_s=1.0))
    assert ops.drone.offboard.streamed[0][0].down_m == pytest.approx(-9.0)


def test_track_tool_rejects_alt_above_the_envelope_ceiling():
    """An EXPLICIT pursuit alt past the envelope ceiling is refused at the
    tool boundary (INVALID_PARAM — the same shape as the take_off/set_speed
    envelope rejections) and never reaches FlightOps.track; an omitted alt
    sails through (FlightOps then holds the current altitude)."""
    import mcp.types as mcp_types

    from agents.flight.envelope import Envelope
    from agents.flight.tools import make_pilot_options

    class GuardOps:
        envelope = Envelope()
        calls = []

        async def track(self, *args, **kw):
            self.calls.append(args[2] if len(args) > 2 else kw.get("alt"))
            return f"tracked alt={self.calls[-1]}"

    opts = make_pilot_options(GuardOps(), report=lambda m: None)
    srv = opts.mcp_servers["pilot"]["instance"]

    async def call(arguments):
        req = mcp_types.CallToolRequest(
            method="tools/call",
            params=mcp_types.CallToolRequestParams(name="track",
                                                   arguments=arguments))
        return (await srv.request_handlers[mcp_types.CallToolRequest](req)).root

    async def go():
        return (await call({"target": "mov_1", "alt": 90.0}),
                await call({"target": "mov_1"}))

    rejected, accepted = asyncio.run(go())
    assert rejected.isError
    assert rejected.content[0].text.startswith(
        "INVALID_PARAM: alt 90m exceeds ceiling 80m")
    assert not accepted.isError
    assert "alt=None" in accepted.content[0].text
    assert GuardOps.calls == [None]        # the rejection never reached FlightOps


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
    txt = scan_text(_ScanWorld(), None,
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
            return InferenceResult(f, [], time.monotonic(), None)

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


# ---------- W3: designation on the click path (geom contacts) ----------

def test_track_designates_positioned_contact():
    """W3 integration pin: the /api/lock click path tracks an ALREADY-
    positioned (geom) contact — before the fix nothing designated it, so
    _feed_tof idled on `designated is None`, the ToF beam never fused, and
    the cockpit's track banner/beam chip stayed IDLE for the whole pursuit
    (design §5: the click perception path runs through designate())."""
    calls = []

    class ClickContacts(FakeContacts):
        def designate(self, name, **kw):
            calls.append(("designate", name))

    contacts = ClickContacts(poses={"vis_car_1": (50.0, 0.0)})
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_car_1", duration_s=1.0))
    assert "shadowed" in r
    assert ("designate", "vis_car_1") in calls


def test_track_readoption_redesignates():
    """W3 integration pin: the EKF rebirths the tracked contact under a new
    id mid-pursuit; the adoption must move the DESIGNATION onto the new id
    or ToF fusion (and the cockpit banner) dies on the first churn."""
    calls = []

    class ChurnContacts(FakeContacts):
        def designate(self, name, **kw):
            calls.append(("designate", name))

    contacts = ChurnContacts(poses={"vis_car_1": (50.0, 0.0)})
    orig_poses = contacts.poses

    def poses():                      # rebirth vis_car_1 -> vis_car_2 at t=.5
        contacts.sim_t += 0.1
        if contacts.sim_t >= 0.5:
            return {"vis_car_2": (50.5, 0.0)}
        return orig_poses()
    contacts.poses = poses

    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_car_1", duration_s=2.0))
    assert "LOST" not in r, r         # adopted, not lost
    assert ("designate", "vis_car_1") in calls
    assert ("designate", "vis_car_2") in calls


def test_track_applies_pursuit_tuning():
    """W3 integration pin: track() must apply the pursuit tuning itself —
    only the eval harnesses called tune_pursuit_params, so the live
    pilot/operator path pursued with stock MPC_TILTMAX_AIR and pitched the
    contact out of the ±21° vfov on the first dash (LOST in ~2 s live)."""
    param_calls = []

    class FakeParam:
        async def set_param_float(self, name, value):
            param_calls.append((name, value))

    contacts = FakeContacts(poses={"vis_car_1": (50.0, 0.0)})
    ops = _ops(contacts)
    ops.drone.param = FakeParam()
    r = asyncio.run(ops.track("vis_car_1", duration_s=1.0))
    assert "shadowed" in r
    assert ("MPC_TILTMAX_AIR", 12.0) in param_calls
    assert ("MPC_XY_VEL_MAX", 6.0) in param_calls


# ---------- d2 regression: truth-fed shadow lane (direct reference) ----------

class SimClock:
    """Sim-time base for the truth-shadow fixture: one jittered dt per
    sim_time() call (one per control tick), so the mover, the estimator and
    the point-mass drone all live in the same SIM time while the loop paces
    on wall-clock — RTF < 1 in miniature."""

    def __init__(self, seed=7):
        self.t = 0.0
        self.last_dt = 0.0
        self._rng = random.Random(seed)

    def advance(self):
        self.last_dt = self._rng.uniform(0.06, 0.18)
        self.t += self.last_dt


class TruthContacts:
    """GzPoses-shaped: poses + sim_time + empty velocities, and crucially NO
    observation()/health() — the observation-less (truth-fed) lane. The
    mover runs a 35 m circle at 3.5 m/s; poses() drops the contact past
    seen_until so the run exits via the usual LOST path once the analysis
    window is covered (the harness's loop exit, not the assertion)."""

    OMEGA = 0.1                        # 3.5 m/s on a 35 m circle

    def __init__(self, clock, name="mov_1", radius=35.0, seen_until=76.5):
        self.clock = clock
        self.name = name
        self.radius = radius
        self.seen_until = seen_until

    def target(self, t):
        return (self.radius * math.cos(self.OMEGA * t),
                self.radius * math.sin(self.OMEGA * t))

    def poses(self):
        if self.clock.t > self.seen_until:
            return {}
        return {self.name: self.target(self.clock.t)}

    def sim_time(self):
        t = self.clock.t
        self.clock.advance()
        return t

    def velocities(self):
        return {}


class TaggingOffboard(FakeOffboard):
    """streamed entries as (started, pos, vel) so pursuit-phase setpoints
    are distinguishable from the pre-start priming ones."""

    async def set_position_velocity_ned(self, pos, vel):
        self.streamed.append((self.started, pos, vel))


class TruthOffboard(FakeOffboard):
    """Records pursuit-phase setpoints and steps the drone as a point mass
    through PX4's cascade law v = v_ff + KP*(p_sp - p), speed-capped — the
    PD shape track.py's docstring describes — in SIM time. The fixture
    zeroes the local<->world offset, so streamed NED == world ENU."""

    KP = 0.95                          # PX4 default MPC_XY_P
    VMAX = 12.0                        # track.MAX_SPEED_MPS

    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.drone = [0.0, 0.0]        # world (e, n)
        self.setpoints = []            # (t_read, pos, vel) per pursuit tick
        self.hist = []                 # (t, e, n) post-step per pursuit tick

    async def set_position_velocity_ned(self, pos, vel):
        await super().set_position_velocity_ned(pos, vel)
        if not self.started:
            return                     # priming setpoints: hover in place
        t_read = self.clock.t - self.clock.last_dt
        self.setpoints.append((t_read, pos, vel))
        v_e = vel.east_m_s + self.KP * (pos.east_m - self.drone[0])
        v_n = vel.north_m_s + self.KP * (pos.north_m - self.drone[1])
        sp = math.hypot(v_e, v_n)
        if sp > self.VMAX:
            v_e, v_n = v_e * self.VMAX / sp, v_n * self.VMAX / sp
        self.drone[0] += v_e * self.clock.last_dt
        self.drone[1] += v_n * self.clock.last_dt
        self.hist.append((self.clock.t, self.drone[0], self.drone[1]))


class TruthWorld:
    buildings = []

    def __init__(self, offboard):
        self._ob = offboard

    def world_xy(self, bridge, i):
        return (self._ob.drone[0], self._ob.drone[1], 12.0)


class TruthBridge:
    def latest(self, topic):
        return SimpleNamespace(x=0.0, y=0.0, z=-12.0, vx=0.0, vy=0.0)


def test_truth_shadow_preserves_direct_reference_and_altitude(monkeypatch):
    """d2 regression (truth-fed d2_shadow): an observation-LESS contact feed
    must stream control_ref's DIRECT reference — target+standoff with
    velocity feedforward at the COMMANDED alt — not the accel-limited _shp
    carrot initialized at the drone (a ~7-10 s lag on a 3.5 m/s mover) and
    not the beam-geometry descent toward 0.18*gap+1.1 (3.8 m at the 15 m
    gate). A 75 s SIM window of jittered ticks through the point-mass PX4
    must re-meet the original >=45 s contiguous <=15 m dwell gate."""
    from agents.flight import track as trk
    # CTRL_HZ paces only the wall-clock loop sleep on this lane (no shaper
    # integration runs here): 1000 Hz compresses 75 s of sim into ~1 s wall.
    monkeypatch.setattr(trk, "CTRL_HZ", 1000.0)
    so_e, so_n = 1.5, -2.0
    clock = SimClock()
    contacts = TruthContacts(clock)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="shadow", alt=12.0,
                              duration_s=10.0, within_m=15.0,
                              standoff_east=so_e, standoff_north=so_n))
    # the harness exit: the contact drops past 76.5 sim s and the loop takes
    # the usual LOST path out — the assertions are on the recorded window.
    assert r.startswith("drone_0 LOST:"), r
    assert offboard.setpoints, "no pursuit setpoints streamed"

    # (a) EVERY seen-phase pursuit setpoint is exactly target+standoff...
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert len(seen) > 400             # ~75 s of ~10 Hz jittered ticks
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        assert pos.east_m == pytest.approx(te + so_e, abs=1e-6)
        assert pos.north_m == pytest.approx(tn + so_n, abs=1e-6)
        # (b) ...at the COMMANDED alt (never the 3.8 m beam-geometry floor)
        assert pos.down_m == pytest.approx(-12.0, abs=1e-6)
        # velocity feedforward of the 3.5 m/s mover (after EMA warmup)
        if t_read >= 1.0:
            assert math.hypot(vel.east_m_s, vel.north_m_s) == pytest.approx(
                3.5, abs=0.1)
    # ...NOT the _shp carrot initialized at the drone: the FIRST pursuit
    # setpoint already sits ~35 m away, on the mover.
    first = seen[0][1]
    assert math.hypot(first.east_m, first.north_m) > 20.0

    # (c) the point-mass drone re-meets the dwell gate over the 75 s window
    window = [h for h in offboard.hist if h[0] <= 75.0]
    assert window and window[-1][0] >= 74.5   # the run covered the window
    best, run_start = 0.0, None
    for t, e, n in window:
        te, tn = contacts.target(t)
        if math.hypot(te - e, tn - n) <= 15.0:
            run_start = t if run_start is None else run_start
            best = max(best, t - run_start)
        else:
            run_start = None
    assert best >= 45.0


def test_beam_capable_shadow_keeps_shaper_and_altitude_profile():
    """The camera-fed M3b lane is byte-identical: a provider WITH an
    observation() method keeps the shaped-velocity servo (the streamed point
    starts AT the drone and creeps out accel-limited) and the beam-geometry
    altitude profile alt_ref = min(alt, max(2.3, 0.18*gap+1.1)) — here 7.4 m
    at a 35 m gap, NOT the commanded 12 m of the direct lane."""
    class BeamContacts(FakeContacts):
        def observation(self, name):
            return None                # the gate is callability, not payload

    contacts = BeamContacts(poses={"vis_t": (35.0, 0.0)})
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_t", mode="shadow", alt=12.0,
                              duration_s=1.0))
    assert "shadowed" in r
    pursuit = [p for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    # shaper engaged: the streamed point starts at/near the drone (0,0),
    # ~35 m SHORT of the direct target reference
    first = pursuit[0]
    assert math.hypot(first.east_m, first.north_m) < 0.5
    assert math.hypot(35.0 - first.east_m, first.north_m) > 30.0
    # altitude profile engaged: 0.18*35+1.1 = 7.4 m, not the commanded 12 m
    for p in pursuit:
        assert p.down_m == pytest.approx(-7.4, abs=1e-6)


def test_intercept_lane_keeps_shaper_without_observation():
    """The direct-reference bypass is gated on mode == 'shadow': a truth-fed
    INTERCEPT must still run the shaped servo (first streamed point at the
    drone) with the altitude profile — only shadow restores the July 6
    law."""
    contacts = FakeContacts(poses={"mov_1": (35.0, 0.0)})   # no observation()
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="intercept", alt=12.0,
                              duration_s=1.0))
    assert "did NOT close" in r
    pursuit = [p for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    first = pursuit[0]
    assert math.hypot(first.east_m, first.north_m) < 0.5
    assert first.down_m == pytest.approx(-7.4, abs=1e-6)


def test_truth_shadow_summary_stats_stay_populated():
    """The direct lane still feeds the TrackLog: the shadow summary keeps
    reporting gap min/mean and best contiguous dwell (the LLM-checkable
    surface) — here a static 10 m target inside the 15 m within gate."""
    contacts = FakeContacts(poses={"mov_1": (10.0, 0.0)})   # no observation()
    ops = _ops(contacts)                 # drone parked at the origin
    r = asyncio.run(ops.track("mov_1", duration_s=1.0, within_m=15.0))
    assert "shadowed" in r
    assert "gap min 10.0m / mean 10.0m" in r
    assert "best contiguous ≤15m:" in r


# ---------- W3a: orbit / stand-off controller ----------

def test_orbit_control_ref_holds_radius_and_ff_over_a_full_revolution():
    """(a) one full θ sweep against a MOVING target: every reference sits at
    exactly radius from the target and the feedforward is the reference's
    true velocity — tangential ω×r plus the target's own."""
    from agents.flight import track as trk
    est = TargetEstimator()
    est.feed_direct(3.5, 0.0)                  # target translating +E at 3.5
    orb = trk.OrbitPhase(15.0, 15.0)
    omega = math.radians(15.0)
    me = (100.0, 100.0)                        # parked far away: init only
    n = int(360.0 / 15.0 * trk.CTRL_HZ) + 1    # one full revolution of ticks
    for k in range(n):
        tgt = (10.0 + 3.5 * k / trk.CTRL_HZ, 20.0)
        ref_e, ref_n, ff_e, ff_n = trk.control_ref(
            "orbit", me[0], me[1], tgt[0], tgt[1], est, 12.0, orbit=orb)
        re_, rn_ = ref_e - tgt[0], ref_n - tgt[1]
        assert math.hypot(re_, rn_) == pytest.approx(15.0, abs=1e-9)
        # tangential = ω × r (perpendicular to the radius), plus est velocity
        assert ff_e == pytest.approx(-omega * rn_ + 3.5, abs=1e-9)
        assert ff_n == pytest.approx(omega * re_, abs=1e-9)
    swept = (n - 1) * omega / trk.CTRL_HZ      # the first tick initializes θ
    assert swept == pytest.approx(2.0 * math.pi, abs=omega / trk.CTRL_HZ)


def test_orbit_control_ref_theta_init_and_direction_sign():
    """(b) θ initializes from the drone's CURRENT relative bearing (no phase
    jump): due east of the target -> θ0 = 0, the first ref sits due east at
    radius; rate sign picks the direction of the sweep."""
    from agents.flight import track as trk
    est = TargetEstimator()
    orb = trk.OrbitPhase(15.0, 15.0)
    ref = trk.control_ref("orbit", 20.0, 0.0, 0.0, 0.0, est, 12.0, orbit=orb)
    assert ref[:2] == pytest.approx((15.0, 0.0), abs=1e-9)
    assert orb.theta == pytest.approx(0.0, abs=1e-12)
    # positive rate: counterclockwise — the next ref rotates toward +N
    ref = trk.control_ref("orbit", 20.0, 0.0, 0.0, 0.0, est, 12.0, orbit=orb)
    assert ref[1] > 0.0
    # negative rate: clockwise — toward -N
    orb = trk.OrbitPhase(15.0, -15.0)
    trk.control_ref("orbit", 20.0, 0.0, 0.0, 0.0, est, 12.0, orbit=orb)
    ref = trk.control_ref("orbit", 20.0, 0.0, 0.0, 0.0, est, 12.0, orbit=orb)
    assert ref[1] < 0.0


def test_shadow_range_m_is_a_radial_hold_reevaluated_per_tick():
    """(c, geometry) stand-off: ref = tgt + range_m·(me−tgt)/|me−tgt| on the
    LIVE drone->target ray every tick (NOT the ω=0 degenerate orbit), with
    the target velocity as feedforward."""
    from agents.flight import track as trk
    est = TargetEstimator()
    est.feed_direct(1.0, 2.0)
    ref = trk.control_ref("shadow", 30.0, 0.0, 0.0, 0.0, est, 12.0,
                          range_m=12.0)
    assert ref == pytest.approx((12.0, 0.0, 1.0, 2.0), abs=1e-9)
    # the drone moves to due north -> the stand-off direction follows per tick
    ref = trk.control_ref("shadow", 0.0, 40.0, 0.0, 0.0, est, 12.0,
                          range_m=12.0)
    assert ref[:2] == pytest.approx((0.0, 12.0), abs=1e-9)
    # target off the origin: the ray is target-relative
    ref = trk.control_ref("shadow", 110.0, 100.0, 100.0, 100.0, est, 12.0,
                          range_m=20.0)
    assert ref[:2] == pytest.approx((120.0, 100.0), abs=1e-9)
    # degenerate (drone ON the target): falls back to the plain standoff
    ref = trk.control_ref("shadow", 5.0, 5.0, 5.0, 5.0, est, 12.0,
                          range_m=12.0, standoff_e=1.0)
    assert ref[:2] == pytest.approx((6.0, 5.0), abs=1e-9)


def test_orbit_first_setpoint_takes_the_current_relative_bearing():
    """(b, ops level) parked 50 m due west of the contact, the FIRST streamed
    reference is on the near side of the circle — bearing error ~0, no phase
    jump — and the run reports with the orbit verb."""
    contacts = FakeContacts(poses={"mov_1": (50.0, 0.0)})
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="orbit", alt=6.0, duration_s=1.0,
                              radius_m=15.0, rate_dps=15.0))
    assert "orbited" in r
    pursuit = [p for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    first = pursuit[0]
    assert first.east_m == pytest.approx(35.0, abs=1e-6)   # 50 - 15, near side
    assert first.north_m == pytest.approx(0.0, abs=1e-6)


def test_orbit_and_standoff_clamp_to_the_keep_out_margin():
    """(d) radius_m/range_m below the keep-out margin (the 7 m bubble + 1 m)
    floor at track.MIN_ORBIT_RADIUS_M: the streamed reference never enters
    the bubble even if the caller asks for it."""
    from agents.flight import track as trk
    for mode, kw in (("orbit", {"radius_m": 5.0, "rate_dps": 15.0}),
                     ("shadow", {"range_m": 5.0})):
        contacts = FakeContacts(poses={"mov_1": (50.0, 0.0)})
        drone = FakeDrone()
        drone.offboard = TaggingOffboard()
        ops = FlightOps(drone, FakeWorld(), FakeBridge(),
                        contacts=contacts)
        asyncio.run(ops.track("mov_1", mode=mode, alt=6.0, duration_s=1.0,
                              **kw))
        pursuit = [p for started, p, v in drone.offboard.streamed if started]
        assert pursuit
        for p in pursuit:
            assert math.hypot(50.0 - p.east_m, p.north_m) >= \
                trk.MIN_ORBIT_RADIUS_M - 1e-6


class FixedDtClock(SimClock):
    """Exact dt = 1/hz per sim_time() call. With CTRL_HZ monkeypatched to the
    same hz, the controller's per-tick θ advance matches sim time exactly —
    a faithful compressed miniature of the orbit (the jittered d2 clock
    would desync θ from the streamed feedforward). dt must exceed the
    estimator's 1e-3 stale-stamp skip, so hz < 1000."""

    def __init__(self, hz):
        super().__init__()
        self._dt = 1.0 / hz

    def advance(self):
        self.last_dt = self._dt
        self.t += self._dt


class QuickLostTruthContacts(TruthContacts):
    """TruthContacts with a short lost_s so the post-window LOST drain costs
    ~0.5 s wall instead of the 2.0 s default."""

    config = SimpleNamespace(lost_s=0.5)


def test_truth_orbit_streams_sane_setpoints(monkeypatch):
    """(f) full ops.track(mode="orbit") smoke on the truth-style fakes: >=5 s
    of sim time without exceptions, every reference exactly at radius from
    the MOVING target with the analytic tangential+target feedforward on the
    direct lane, at the commanded alt — and the point-mass PX4 cascade
    converges onto the circle and actually sweeps around the target."""
    from agents.flight import track as trk
    hz = 300.0                                 # dt > 1e-3; leaves scheduler headroom
    monkeypatch.setattr(trk, "CTRL_HZ", hz)
    radius, rate = 12.0, 30.0                  # tangential 6.3 m/s, < VMAX
    omega = math.radians(rate)
    clock = FixedDtClock(hz)
    contacts = QuickLostTruthContacts(clock, seen_until=5.2)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="orbit", alt=12.0,
                              duration_s=30.0, radius_m=radius,
                              rate_dps=rate))
    assert r.startswith("drone_0 LOST:"), r    # the harness's loop exit
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert seen and seen[-1][0] - seen[0][0] >= 5.0    # >=5 s sim streamed
    # θ(t) = π + ω·t: the drone starts at the origin, due WEST of the target
    # at (35, 0); dt == 1/CTRL_HZ makes the per-tick advance exact in sim time.
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        # every reference exactly on the circle around the moving target...
        assert math.hypot(pos.east_m - te, pos.north_m - tn) == \
            pytest.approx(radius, abs=1e-6)
        # ...at the COMMANDED alt (never the shaper's 2.3 m floor)...
        assert pos.down_m == pytest.approx(-12.0, abs=1e-6)
        # ...with the analytic feedforward actually streamed (after warmup)
        if t_read >= 0.3:
            th = math.pi + omega * t_read
            vte, vtn = -3.5 * math.sin(0.1 * t_read), 3.5 * math.cos(0.1 * t_read)
            assert vel.east_m_s == pytest.approx(
                -radius * omega * math.sin(th) + vte, abs=0.05)
            assert vel.north_m_s == pytest.approx(
                radius * omega * math.cos(th) + vtn, abs=0.05)
    # the point-mass cascade converges onto the circle: gap ~= radius after
    # settle, never inside the keep-out margin
    settled = [h for h in offboard.hist if 2.0 <= h[0] <= 5.0]
    assert settled
    for t, e, n in settled:
        te, tn = contacts.target(t)
        gap = math.hypot(e - te, n - tn)
        assert 8.5 < gap < 15.5
    # and genuinely ORBITS: the drone's bearing from the target sweeps
    sweep, prev = 0.0, None
    for t, e, n in settled:
        te, tn = contacts.target(t)
        b = math.atan2(n - tn, e - te)
        if prev is not None:
            sweep += (b - prev + math.pi) % (2 * math.pi) - math.pi
        prev = b
    assert math.degrees(abs(sweep)) > 60.0


def test_truth_standoff_range_m_holds_radial_distance(monkeypatch):
    """(c, ops level) standoff op: shadow + range_m streams every reference at
    EXACTLY range_m from the moving target, on the live drone->target ray
    (re-evaluated per tick — the radial hold)."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 1000.0)   # shadow: no θ, jitter is safe
    clock = SimClock(seed=3)
    contacts = QuickLostTruthContacts(clock, seen_until=3.0)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="shadow", alt=12.0,
                              duration_s=20.0, range_m=12.0))
    assert r.startswith("drone_0 LOST:"), r
    assert "shadowed" not in r                  # LOST line replaces the summary
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert len(seen) > 15                       # ~3 s of jittered ~8 Hz ticks
    # setpoint k used the drone position stepped after setpoint k-1 (hist);
    # pair each ref with the drone position its radial direction came from.
    for (t_read, pos, vel), (t_prev, me_e, me_n) in zip(seen[1:],
                                                        offboard.hist):
        te, tn = contacts.target(t_read)
        re_, rn_ = pos.east_m - te, pos.north_m - tn
        assert math.hypot(re_, rn_) == pytest.approx(12.0, abs=1e-6)
        de, dn = me_e - te, me_n - tn
        cross = re_ * dn - rn_ * de
        assert abs(cross) / (12.0 * math.hypot(de, dn) + 1e-9) < 0.02


# ---------- W3 codex §3: designated readoption relaxation (COCO profile) ----------

def _super_cfg(lost_s=5.0, rebind_window_s=5.0):
    """The COCO tracker profile as the provider's config: the superclass
    assoc_keys map is what opts a provider INTO the relaxed readoption law
    (the legacy law runs byte-identical without it)."""
    return SimpleNamespace(lost_s=lost_s, gate_m=5.0,
                           rebind_window_s=rebind_window_s,
                           assoc_keys={"car": "vehicle", "truck": "vehicle",
                                       "bus": "vehicle"})


def _rebirth_contacts(reborn, *, drop_at=1.0, back_at=3.5, sim_per_tick=0.5,
                      cfg=None):
    """vis_car_1 at (50,0) drops at drop_at and is reborn as `reborn` (a
    poses dict) at back_at, sim seconds; sim advances sim_per_tick per
    poses() call (same clock idiom as _dropout_contacts)."""
    c = FakeContacts(poses={"vis_car_1": (50.0, 0.0)})
    orig = dict(c._poses)

    def poses():
        c.sim_t += sim_per_tick
        if c.sim_t < drop_at:
            return dict(orig)
        if c.sim_t < back_at:
            return {}
        return dict(reborn)
    c.poses = poses
    if cfg is not None:
        c.config = cfg
    return c


def test_designated_readoption_accepts_vehicle_reclass_and_duplicate_cluster():
    """The W3 integration churn, fixed at the ops layer: the locked car drops
    and rebirths ~3 s later as a TRUCK with a duplicate double-birth (one
    physical vehicle, two ids 0.6 m apart). The association KEY matches, the
    dupes form a <=2 m cluster, so the pursuit adopts the nearest and keeps
    tracking — re-designating onto the adopted id, no LOST."""
    calls = []
    contacts = _rebirth_contacts(
        {"vis_truck_9": (50.5, 0.0), "vis_car_10": (51.0, 0.4)},
        cfg=_super_cfg())
    contacts.designate = lambda name, **kw: calls.append(name)
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_car_1", duration_s=2.0))
    assert "LOST" not in r, r
    assert calls == ["vis_car_1", "vis_truck_9"]


def test_readoption_refuses_two_separated_plausible_vehicles():
    """Two SAME-KEY rebirths at similar ranges but >2 m apart (genuinely two
    vehicles — or a split birth): NOT a duplicate cluster and the runner-up
    is <2 m farther, so adoption refuses ambiguity and the op ends in the
    structured LOST (hold/reacquire, never a guessed identity)."""
    contacts = _rebirth_contacts(
        {"vis_truck_9": (51.5, 0.0), "vis_car_10": (50.0, 1.8)},
        cfg=_super_cfg(lost_s=2.0))
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_car_1", duration_s=10.0))
    assert r.startswith("drone_0 LOST:"), r


def test_readoption_expires():
    """A rebirth past the provider's rebind window is a NEW object, not the
    designated one: readoption refuses (dt_s > rebind_window_s) even with a
    single same-key candidate in gate, and the op exits LOST."""
    contacts = _rebirth_contacts(
        {"vis_truck_9": (50.5, 0.0)}, back_at=4.0,
        cfg=_super_cfg(lost_s=6.0, rebind_window_s=2.0))
    ops = _ops(contacts)
    r = asyncio.run(ops.track("vis_car_1", duration_s=10.0))
    assert r.startswith("drone_0 LOST:"), r


# ---------- W3 codex §4: hold_altitude opt-out on the camera-fed lane ----------

def test_coco_vehicle_shadow_holds_commanded_altitude():
    """hold_altitude=True (the /pilot/cmd operator layer) skips the M3b
    beam-geometry altitude profile — 7.4 m at this 35 m gap — and holds the
    COMMANDED 12 m on every streamed pursuit setpoint. W3 codex R3: the held
    camera-fed shadow now takes the DIRECT lane, so the FIRST pursuit
    reference already sits on the R_min(12)+2 = 38 m lock ring ((35,0) target
    -> ref (-3,0) behind the parked drone), not at the drone (the shaper
    carrot). The mover default keeps the shaper + profile (pinned by
    test_beam_capable_shadow_keeps_shaper_and_altitude_profile)."""
    class BeamContacts(FakeContacts):
        def observation(self, name):
            return None                # the gate is callability, not payload

    contacts = BeamContacts(poses={"vis_car_1": (35.0, 0.0)})
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=12.0,
                              duration_s=1.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [p for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    first = pursuit[0]
    # direct lane: the first reference is on the 38 m ring, NOT the
    # drone-initialized shaper carrot
    assert math.hypot(35.0 - first.east_m, first.north_m) == \
        pytest.approx(38.0, abs=1e-6)
    assert math.hypot(first.east_m, first.north_m) > 2.0
    for p in pursuit:
        assert p.down_m == pytest.approx(-12.0, abs=1e-6)  # NOT the 7.4 m sag


# ---------- W3 codex R2: the hold-altitude radial floor ----------

def test_demo_shadow_default_has_two_metre_transient_reserve(monkeypatch):
    """hold_altitude=True with NO explicit range_m: the shadow defaults to
    range_m = R_min(alt)+2 = 20 m at a 6 m hold — the R2 radial floor plus
    the W3 codex R3 corner-transient reserve (R_min is a steady-state law;
    the mover's 90deg corners transiently cut inside it — w3-run3.md), still
    instead of closing onto the target and parking it in the level camera's
    blind cone. Every direct-lane reference sits at EXACTLY 20 m from the
    mover; an EXPLICIT standoff still floors at R_min itself (pinned by
    test_r2_standoff_below_the_floor_clamps_to_r_min)."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 1000.0)   # shadow: no θ, jitter is safe
    clock = SimClock(seed=3)
    contacts = QuickLostTruthContacts(clock, seen_until=3.0)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="shadow", alt=6.0,
                              duration_s=20.0, hold_altitude=True))
    assert r.startswith("drone_0 LOST:"), r
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert len(seen) > 15
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        assert math.hypot(pos.east_m - te, pos.north_m - tn) == \
            pytest.approx(20.0, abs=1e-6)
        assert pos.down_m == pytest.approx(-6.0, abs=1e-6)  # commanded hold


def test_r2_standoff_below_the_floor_clamps_to_r_min(monkeypatch):
    """An EXPLICIT standoff under the floor is raised: range_m=12 at a 6 m
    hold (the validation's own step-3 value — 23.9deg depression, inside the
    blind cone) streams references at R_min=18 m, not at the requested 12."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 1000.0)
    clock = SimClock(seed=3)
    contacts = QuickLostTruthContacts(clock, seen_until=3.0)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="shadow", alt=6.0,
                              duration_s=20.0, range_m=12.0,
                              hold_altitude=True))
    assert r.startswith("drone_0 LOST:"), r
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert len(seen) > 15
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        assert math.hypot(pos.east_m - te, pos.north_m - tn) == \
            pytest.approx(18.0, abs=1e-6)


def test_r2_orbit_radius_clamps_before_the_phase_is_built(monkeypatch):
    """hold_altitude orbit: radius_m under the floor (15 m at a 6 m hold —
    22.6deg depression to the ground) is clamped to R_min=18 BEFORE the
    OrbitPhase is constructed: every streamed reference sits at 18 m, never
    at the requested 15."""
    from agents.flight import track as trk
    hz = 900.0                                 # dt = 1/900 > 1e-3 (EMA alive)
    monkeypatch.setattr(trk, "CTRL_HZ", hz)
    clock = FixedDtClock(hz)
    contacts = QuickLostTruthContacts(clock, seen_until=2.2)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="orbit", alt=6.0,
                              duration_s=30.0, radius_m=15.0, rate_dps=8.0,
                              hold_altitude=True))
    assert r.startswith("drone_0 LOST:"), r
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert seen and seen[-1][0] - seen[0][0] >= 2.0
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        assert math.hypot(pos.east_m - te, pos.north_m - tn) == \
            pytest.approx(18.0, abs=1e-6)
        assert pos.down_m == pytest.approx(-6.0, abs=1e-6)


def test_r2_default_path_keeps_the_keep_out_only(monkeypatch):
    """hold_altitude=False (the mover/LLM path) is byte-identical: a 6 m
    shadow without range_m streams the reference ON the target (no radial
    floor default), and a 12 m orbit requested at a 6 m alt keeps 12 m —
    only the static 8 m keep-out applies."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 1000.0)
    clock = SimClock(seed=5)
    contacts = QuickLostTruthContacts(clock, seen_until=3.0)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="shadow", alt=6.0,
                              duration_s=20.0))
    assert r.startswith("drone_0 LOST:"), r
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert len(seen) > 15
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        # range_m stayed None: the reference is the target itself
        assert pos.east_m == pytest.approx(te, abs=1e-6)
        assert pos.north_m == pytest.approx(tn, abs=1e-6)
    # orbit without the flag: the requested 12 m survives at a 6 m alt
    hz = 900.0
    monkeypatch.setattr(trk, "CTRL_HZ", hz)
    clock = FixedDtClock(hz)
    contacts = QuickLostTruthContacts(clock, seen_until=2.2)
    offboard = TruthOffboard(clock)
    drone = FakeDrone()
    drone.offboard = offboard
    ops = FlightOps(drone, TruthWorld(offboard), TruthBridge(),
                    contacts=contacts)
    r = asyncio.run(ops.track("mov_1", mode="orbit", alt=6.0,
                              duration_s=30.0, radius_m=12.0, rate_dps=8.0))
    assert r.startswith("drone_0 LOST:"), r
    seen = [s for s in offboard.setpoints if s[0] <= contacts.seen_until]
    assert seen
    for t_read, pos, vel in seen:
        te, tn = contacts.target(t_read)
        assert math.hypot(pos.east_m - te, pos.north_m - tn) == \
            pytest.approx(12.0, abs=1e-6)


# ---------- W3 codex R3: the demo corner transient ----------

class CornerBeamContacts(FakeContacts):
    """Parked-drone corner miniature (W3 codex R3): the mover runs +E at
    4 m/s along y=30, then turns 90deg SOUTH at corner_t — the leg passes
    18 m east of the parked drone, so the live gap transits INSIDE
    R_guard = R_min(6)+1 = 19 m for ~2 s around the crossing (the w3-run3
    corner cut, compressed and made deterministic). observation() EXISTS
    (the camera-fed gate) but carries no bearing payload, so the
    predicted-lead yaw fallback is what streams; velocities() feeds the
    exact leg velocity (no EMA warmup). sim advances sim_per_tick per
    poses() call (the _dropout_contacts clock idiom); ticks records
    (sim_t, target) per call for the per-tick assertions."""

    def __init__(self, corner_t=5.0, speed=4.0, corner=(18.0, 30.0),
                 sim_per_tick=0.1):
        super().__init__(poses={})
        self.corner_t = corner_t
        self.speed = speed
        self.corner = corner
        self.sim_per_tick = sim_per_tick
        self.ticks = []                        # (sim_t, (te, tn)) per call

    def target(self, t):
        if t < self.corner_t:
            return (self.corner[0] - self.speed * (self.corner_t - t),
                    self.corner[1])
        return (self.corner[0],
                self.corner[1] - self.speed * (t - self.corner_t))

    def target_vel(self, t):
        return ((self.speed, 0.0) if t < self.corner_t
                else (0.0, -self.speed))

    def poses(self):
        self.sim_t += self.sim_per_tick
        tgt = self.target(self.sim_t)
        self.ticks.append((self.sim_t, tgt))
        return {"vis_car_1": tgt}

    def velocities(self):
        return {"vis_car_1": self.target_vel(self.sim_t)}

    def observation(self, name):
        return None                # the gate is callability, not payload


def test_hold_altitude_camera_shadow_uses_direct_reference_through_right_angle_corner(
        monkeypatch):
    """R3 pin: a CAMERA-FED shadow with hold_altitude=True (the demo
    pursuit) leaves the 1 m/s^2 shaper for the DIRECT lane — through the
    mover's 90deg corner every streamed reference is control_ref's
    target-relative point exactly on the R_min(6)+2 = 20 m lock ring at the
    commanded hold alt, and the FIRST pursuit setpoint already sits ~10 m
    from the parked drone on the ring, NOT at the drone (the shaper carrot
    whose accel-limited lag cut 15.1 m inside the ring through this corner
    in w3-run3.md). The mover default (hold_altitude=False) keeps the
    shaper — pinned by
    test_beam_capable_shadow_keeps_shaper_and_altitude_profile."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)   # 10x the 0.1 s sim ticks
    contacts = CornerBeamContacts()
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=2.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert len(pursuit) > 100
    assert contacts.ticks[-1][0] > contacts.corner_t + 2.0  # ran the corner
    for (sim_t, tgt), (pos, vel) in zip(contacts.ticks[1:], pursuit):
        assert math.hypot(pos.east_m - tgt[0], pos.north_m - tgt[1]) == \
            pytest.approx(20.0, abs=1e-6)      # the R_min(6)+2 lock ring
        assert pos.down_m == pytest.approx(-6.0, abs=1e-6)
    first = pursuit[0][0]
    assert math.hypot(first.east_m, first.north_m) > 5.0   # NOT the carrot


def test_demo_radial_barrier_commands_outward_relative_velocity(monkeypatch):
    """R3 corner interlock: while the mover's southbound leg transits
    inside R_guard = R_min(6)+1 = 19 m of the parked drone, EVERY sub-guard
    tick's streamed feedforward is the mover velocity PLUS the outward
    radial escape min(2.0, 0.8*(R_guard-g))*(me-tgt)/g (no sub-guard tick
    without the escape term); outside the guard the feedforward is the
    plain mover velocity."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)
    guard = 19.0                               # R_min(6)=18 + 1
    contacts = CornerBeamContacts()
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=2.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert len(pursuit) > 100
    inside = 0
    for (sim_t, tgt), (pos, vel) in zip(contacts.ticks[1:], pursuit):
        g = math.hypot(tgt[0], tgt[1])         # the drone is parked at 0,0
        vte, vtn = contacts.target_vel(sim_t)
        if g < guard - 1e-9:
            inside += 1
            esc = min(2.0, 0.8 * (guard - g))
            ue, un = -tgt[0] / g, -tgt[1] / g
            assert vel.east_m_s == pytest.approx(vte + esc * ue, abs=1e-6)
            assert vel.north_m_s == pytest.approx(vtn + esc * un, abs=1e-6)
        else:
            assert vel.east_m_s == pytest.approx(vte, abs=1e-6)
            assert vel.north_m_s == pytest.approx(vtn, abs=1e-6)
    assert inside >= 5           # the leg genuinely transits the guard


def test_direct_lane_prefers_measured_bearing_then_falls_back_to_lead():
    """R3 yaw on the direct lane: with a FRESH observation the streamed yaw
    is the measured camera bearing (the shaper lane's precedence — the
    level camera centers what the detector actually sees); with none, the
    0.4 s predicted-lead yaw — here a static target due east of the parked
    drone: atan2(50, 0) = 90deg."""
    class BearingContacts(FakeContacts):
        def __init__(self, *a, obs=None, **kw):
            super().__init__(*a, **kw)
            self._obs = obs

        def observation(self, name):
            return self._obs

    obs = SimpleNamespace(bearing_deg=123.0)
    contacts = BearingContacts(poses={"vis_car_1": (50.0, 0.0)}, obs=obs)
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=1.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [p for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    for p in pursuit:
        assert p.yaw_deg == pytest.approx(123.0, abs=1e-6)

    contacts = BearingContacts(poses={"vis_car_1": (50.0, 0.0)}, obs=None)
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=1.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [p for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    for p in pursuit:
        assert p.yaw_deg == pytest.approx(90.0, abs=1e-6)


# ---------- W3 codex R8: the image-edge barrier ----------

class EdgeBeamContacts(CornerBeamContacts):
    """Image-edge miniature (W3 codex R8): the R3 corner fixture with a
    payload-carrying observation() — the designated contact's bbox bottom
    row y2 on the 640x360 frame (floor row 360) plus the view age. y2=280
    -> q=0 (no effect); y2=320 -> q=0.5; y2>=340 -> q=1.0 (clamped); a
    stale view (age_s >= 0.3) -> q=0."""

    def __init__(self, *a, y2=320.0, age_s=0.0, **kw):
        super().__init__(*a, **kw)
        self.y2 = y2
        self.age_s = age_s

    def observation(self, name):
        return SimpleNamespace(
            bearing_deg=None, range_src="geom",
            bbox_xyxy=(100.0, 240.0, 220.0, self.y2), age_s=self.age_s)


def test_demo_image_edge_barrier_expands_radius_and_backs_off(monkeypatch):
    """R8 pin: on the demo direct lane a FRESH bbox ramps
    q=clamp((y2-300)/40) as the box bottom nears the 360-row frame
    floor, the guard grows R_vis = R_guard+4q (19 -> 21 -> 23 m at a 6 m
    hold), the radial reference projects out from the 20 m ring to R_vis,
    and every sub-R_vis tick adds the outward visibility feedforward
    min(3, R_vis-gap) (stacked on the R3 escape inside the guard) BEFORE
    the 6 m/s cap — y2=280/320/345 give q=0/0.5/1.0 (monotonic expansion,
    none at q=0) and the streamed vector never exceeds the cap."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)
    guard = 19.0                               # R_min(6)=18 + 1
    ring = 20.0                                # R_min(6) + 2
    for y2, want_q in ((280.0, 0.0), (320.0, 0.5), (345.0, 1.0)):
        contacts = EdgeBeamContacts(y2=y2)
        drone = FakeDrone()
        drone.offboard = TaggingOffboard()
        ops = FlightOps(drone, FakeWorld(), FakeBridge(),
                        contacts=contacts)
        r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                                  duration_s=2.0, hold_altitude=True))
        assert "shadowed" in r
        pursuit = [(p, v) for started, p, v in drone.offboard.streamed
                   if started]
        assert len(pursuit) > 100
        r_vis = guard + 4.0 * want_q           # the 300/40/4 law
        bands = {"plain": 0, "vis": 0, "both": 0}
        for (sim_t, tgt), (pos, vel) in zip(contacts.ticks[1:], pursuit):
            g = math.hypot(tgt[0], tgt[1])     # the drone is parked at 0,0
            vte, vtn = contacts.target_vel(sim_t)
            ue, un = -tgt[0] / g, -tgt[1] / g
            fve, fvn = vte, vtn
            if g < r_vis:
                if want_q > 0.0:
                    vis = min(3.0, r_vis - g)  # the min(3, ...) law
                    fve, fvn = fve + vis * ue, fvn + vis * un
                    if g < guard - 1e-9:
                        bands["both"] += 1
                    else:
                        bands["vis"] += 1
                if g < guard - 1e-9:
                    esc = min(2.0, 0.8 * (guard - g))
                    fve, fvn = fve + esc * ue, fvn + esc * un
                fv = math.hypot(fve, fvn)
                if fv > 6.0:
                    fve, fvn = fve * 6.0 / fv, fvn * 6.0 / fv
            else:
                bands["plain"] += 1
            assert vel.east_m_s == pytest.approx(fve, abs=1e-6)
            assert vel.north_m_s == pytest.approx(fvn, abs=1e-6)
            assert math.hypot(vel.east_m_s, vel.north_m_s) <= 6.0 + 1e-9
            # the ring projects out to R_vis (monotonic 20 -> 21 -> 23)
            assert math.hypot(pos.east_m - tgt[0],
                              pos.north_m - tgt[1]) == \
                pytest.approx(max(ring, r_vis), abs=1e-6)
            assert pos.down_m == pytest.approx(-6.0, abs=1e-6)
        assert bands["plain"] >= 5
        if want_q > 0.0:
            assert bands["vis"] >= 5     # the visibility-only band engages
            assert bands["both"] >= 5    # and stacks on the R3 escape


def test_demo_image_edge_barrier_ff_never_exceeds_the_cap(monkeypatch):
    """R8 cap pin: a HEAD-ON approach (the mover runs straight at the
    parked drone along the x-axis) stacks the 4 m/s target velocity AND
    the outward visibility + corner terms on one axis — up to 7.8 m/s of
    uncapped demand — yet every streamed vector is the 6 m/s cap exactly,
    direction preserved."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)
    guard = 19.0
    contacts = EdgeBeamContacts(corner=(-10.0, 0.0), y2=345.0)   # q=1.0
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=2.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert len(pursuit) > 100
    r_vis = guard + 4.0                        # q=1.0
    capped = 0
    for (sim_t, tgt), (pos, vel) in zip(contacts.ticks[1:], pursuit):
        g = math.hypot(tgt[0], tgt[1])
        vte, vtn = contacts.target_vel(sim_t)
        ue, un = -tgt[0] / g, -tgt[1] / g
        fve, fvn = vte, vtn
        if g < r_vis:
            vis = min(3.0, r_vis - g)
            fve, fvn = fve + vis * ue, fvn + vis * un
            if g < guard - 1e-9:
                esc = min(2.0, 0.8 * (guard - g))
                fve, fvn = fve + esc * ue, fvn + esc * un
            fv = math.hypot(fve, fvn)
            if fv > 6.0:
                capped += 1
                fve, fvn = fve * 6.0 / fv, fvn * 6.0 / fv
        assert vel.east_m_s == pytest.approx(fve, abs=1e-6)
        assert vel.north_m_s == pytest.approx(fvn, abs=1e-6)
        assert math.hypot(vel.east_m_s, vel.north_m_s) <= 6.0 + 1e-9
    assert capped >= 5                         # the cap genuinely binds


def test_image_edge_barrier_ignores_stale_views_and_the_mover_default(
        monkeypatch):
    """R8 gates: a STALE view (age_s=0.5 >= 0.3) keeps q=0 even with the
    box bottom at row 345 — the 20 m ring and the plain R3 escape law, no
    visibility term. And hold_altitude=False (the mover/LLM path) never
    leaves the M3b shaper: the streamed point starts AT the drone and
    creeps out accel-limited — no outward visibility term on any tick."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)
    guard = 19.0
    contacts = EdgeBeamContacts(y2=345.0, age_s=0.5)   # stale -> q=0
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=2.0, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert len(pursuit) > 100
    for (sim_t, tgt), (pos, vel) in zip(contacts.ticks[1:], pursuit):
        g = math.hypot(tgt[0], tgt[1])
        vte, vtn = contacts.target_vel(sim_t)
        ue, un = -tgt[0] / g, -tgt[1] / g
        fve, fvn = vte, vtn
        if g < guard - 1e-9:                   # the plain R3 law, no vis
            esc = min(2.0, 0.8 * (guard - g))
            fve, fvn = fve + esc * ue, fvn + esc * un
            fv = math.hypot(fve, fvn)
            if fv > 6.0:
                fve, fvn = fve * 6.0 / fv, fvn * 6.0 / fv
        assert vel.east_m_s == pytest.approx(fve, abs=1e-6)
        assert vel.north_m_s == pytest.approx(fvn, abs=1e-6)
        assert math.hypot(pos.east_m - tgt[0], pos.north_m - tgt[1]) == \
            pytest.approx(20.0, abs=1e-6)      # the ring never expands
    # the mover default: the shaper lane has no barrier at all
    contacts = EdgeBeamContacts(y2=345.0)
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=6.0,
                              duration_s=2.0))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    first_p, first_v = pursuit[0]
    assert math.hypot(first_p.east_m, first_p.north_m) < 0.5   # the carrot
    assert math.hypot(first_v.east_m_s, first_v.north_m_s) < 0.1


# ---------- W3 codex R4: the coast steers on the predicted bearing ----------

class CoastBeamContacts(FakeContacts):
    """Coast miniature (W3 codex R4): the contact is measured at `meas`
    (bearing atan2 — the STALE bearing the pre-R4 coast froze on), then the
    detector goes silent and the EKF predicts it along `vel` — poses() serves
    the prediction and health() reads COASTING for the whole op (the track
    outlives the drought inside lost_s, so the loop never takes the LOST
    exit). observation() carries the full view: the stale bearing, the
    predicted e/n + velocity, and the coast age (1 s already elapsed at op
    start, so age > coast_s from the first tick). sim advances sim_per_tick
    per poses() call (the _dropout_contacts clock idiom); ticks records
    (sim_t, predicted) per call for the per-tick assertions."""

    def __init__(self, meas=(50.0, 0.0), vel=(0.0, 3.0), coast_age0=1.0,
                 sim_per_tick=0.1, range_src="geom"):
        super().__init__(poses={})
        self.meas = meas
        self.vel = vel
        self.coast_age0 = coast_age0
        self.sim_per_tick = sim_per_tick
        self.range_src = range_src
        self.ticks = []                        # (sim_t, (e, n)) per call

    def age(self):
        return self.coast_age0 + self.sim_t

    def predicted(self):
        a = self.age()
        return (self.meas[0] + self.vel[0] * a,
                self.meas[1] + self.vel[1] * a)

    def poses(self):
        self.sim_t += self.sim_per_tick
        p = self.predicted()
        self.ticks.append((self.sim_t, p))
        return {"vis_car_1": p}

    def velocities(self):
        return {"vis_car_1": self.vel}

    def health(self, name):
        return "COASTING"

    def observation(self, name):
        p = self.predicted()
        return SimpleNamespace(
            bearing_deg=math.degrees(math.atan2(self.meas[0], self.meas[1])),
            range_src=self.range_src, e=p[0], n=p[1],
            ve=self.vel[0], vn=self.vel[1], age_s=self.age())


class MovedCoastWorld(FakeWorld):
    """Starts at the engagement origin, then reports an advanced vehicle."""

    def __init__(self):
        self.reads = 0

    def world_xy(self, bridge, i):
        self.reads += 1
        return (0.0, 0.0, 6.0) if self.reads == 1 else (12.0, -7.0, 6.0)


@pytest.mark.parametrize("range_src, want_ve, want_vn", [
    ("geom", 0.0, 0.0),
    ("tof", 0.0, 2.0),
])
def test_coast_latches_current_vehicle_position_not_engagement_start(
        monkeypatch, range_src, want_ve, want_vn):
    """A coast after pursuit must not command a return to the start point."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)
    contacts = CoastBeamContacts(range_src=range_src)
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, MovedCoastWorld(), FakeBridge(),
                    contacts=contacts)

    result = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=4.0,
                                   duration_s=0.2, hold_altitude=True))

    assert "shadowed" in result
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert pursuit
    for pos, vel in pursuit:
        assert pos.east_m == pytest.approx(12.0, abs=1e-6)
        assert pos.north_m == pytest.approx(-7.0, abs=1e-6)
        assert vel.east_m_s == pytest.approx(want_ve, abs=1e-6)
        assert vel.north_m_s == pytest.approx(want_vn, abs=1e-6)


def test_demo_coast_yaw_follows_predicted_contact_without_translating(
        monkeypatch):
    """R4 fix (demo/hold_altitude path): through a coast the EKF keeps
    predicting the contact while bearing_deg is frozen on the stale
    measurement — the held shadow must steer yaw on the PREDICTED position's
    bearing (here the mover walks north off the 90deg measured bearing at
    3 m/s, so the setpoint yaw falls from ~86.6deg toward the 83.2deg cap
    value) while position and velocity setpoints stay HELD at the shaped
    point (the parked drone) — the pursuit never translates toward the
    ghost. Layer-5 pre-emption: the prediction horizon is capped at 2 s, so
    once the coast age passes 2 s the yaw freezes at the horizon bearing
    even though the EKF ghost keeps walking."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)   # 10x the 0.1 s sim ticks
    contacts = CoastBeamContacts()
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=4.0,
                              duration_s=0.6, hold_altitude=True))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert len(pursuit) > 40
    for (sim_t, _pred), (pos, vel) in zip(contacts.ticks[1:], pursuit):
        h = min(contacts.coast_age0 + sim_t, 2.0)
        want = math.degrees(math.atan2(
            contacts.meas[0] + contacts.vel[0] * h,
            contacts.meas[1] + contacts.vel[1] * h))
        assert pos.yaw_deg == pytest.approx(want, abs=1e-6)
        # position/velocity HELD: no translation toward the prediction
        assert pos.east_m == pytest.approx(0.0, abs=1e-6)
        assert pos.north_m == pytest.approx(0.0, abs=1e-6)
        assert pos.down_m == pytest.approx(-4.0, abs=1e-6)
        assert vel.east_m_s == pytest.approx(0.0, abs=1e-6)
        assert vel.north_m_s == pytest.approx(0.0, abs=1e-6)
    yaws = [p.yaw_deg for p, _v in pursuit]
    assert min(yaws) < 84.0            # left the stale 90deg measured bearing
    capped = [p.yaw_deg for (sim_t, _p), (p, _v)
              in zip(contacts.ticks[1:], pursuit)
              if contacts.coast_age0 + sim_t > 2.0]
    assert capped and max(capped) - min(capped) < 1e-6   # the 2 s horizon cap


def test_mover_default_coast_yaw_keeps_stale_measured_bearing(monkeypatch):
    """R4 scope pin: hold_altitude=False (the mover/LLM path) is
    byte-identical — the SAME coast streams the STALE measured bearing
    (atan2(50, 0) = 90deg) on every setpoint, never the predicted-contact
    bearing the demo path follows."""
    from agents.flight import track as trk
    monkeypatch.setattr(trk, "CTRL_HZ", 100.0)
    contacts = CoastBeamContacts()
    drone = FakeDrone()
    drone.offboard = TaggingOffboard()
    ops = FlightOps(drone, FakeWorld(), FakeBridge(), contacts=contacts)
    r = asyncio.run(ops.track("vis_car_1", mode="shadow", alt=4.0,
                              duration_s=0.6))
    assert "shadowed" in r
    pursuit = [(p, v) for started, p, v in drone.offboard.streamed if started]
    assert len(pursuit) > 40
    for p, _v in pursuit:
        assert p.yaw_deg == pytest.approx(90.0, abs=1e-6)
