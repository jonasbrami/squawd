"""Px4StateRecorder: clock alignment from a sim-time reference, NED→ENU shared
conversion, honest silence before alignment, quaternion → rpy."""
import math

from agents.core.telemetry import Px4StateRecorder, _quat_to_rpy


class FakeBridge:
    def __init__(self):
        self.cbs = {}

    def subscribe(self, topic, msg_type, qos=None, callback=None):
        self.cbs[topic] = callback


class FakeSink:
    def __init__(self):
        self.poses = []
        self.atts = []

    def ned_to_enu(self, i, x, y, z):
        return (y, 3.0 * i + x, -z)

    def note_pose(self, *a):
        self.poses.append(a)

    def note_attitude(self, *a):
        self.atts.append(a)


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_alignment_captured_once_then_applied():
    bridge, sink = FakeBridge(), FakeSink()
    sim_now = {"t": 100.0}
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: sim_now["t"])
    rec.start(position_type=Msg, attitude_type=Msg)
    cb = bridge.cbs["/px4_0/fmu/out/vehicle_local_position"]
    cb(Msg(timestamp=40_000_000, x=1.0, y=2.0, z=-5.0, heading=0.5, xy_valid=True))
    # offset = 100 - 40 = 60 (applied to every later sample);
    sim_now["t"] = 105.0
    cb(Msg(timestamp=45_000_000, x=3.0, y=4.0, z=-6.0, heading=0.6, xy_valid=True))
    assert sink.poses == [(100.0, 2.0, 1.0, 5.0, 0.5), (105.0, 4.0, 3.0, 6.0, 0.6)]


def test_no_recording_before_alignment_reference_available():
    bridge, sink = FakeSink.__mro__[1]() if False else FakeBridge(), FakeSink()
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: 0.0)
    rec.start(position_type=Msg, attitude_type=Msg)
    bridge.cbs["/px4_0/fmu/out/vehicle_local_position"](
        Msg(timestamp=40_000_000, x=1.0, y=2.0, z=-5.0, heading=0.0, xy_valid=True))
    assert sink.poses == []          # honest None: no ref, no record


def test_invalid_xy_skipped():
    bridge, sink = FakeBridge(), FakeSink()
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: 10.0)
    rec.start(position_type=Msg, attitude_type=Msg)
    bridge.cbs["/px4_0/fmu/out/vehicle_local_position"](
        Msg(timestamp=5_000_000, x=1.0, y=2.0, z=-5.0, heading=0.0, xy_valid=False))
    assert sink.poses == []


def test_attitude_recorded_as_rpy():
    bridge, sink = FakeBridge(), FakeSink()
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: 10.0)
    rec.start(position_type=Msg, attitude_type=Msg)
    bridge.cbs["/px4_0/fmu/out/vehicle_attitude"](
        Msg(timestamp=5_000_000, q=(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))))
    t, r, p, y = sink.atts[0]
    assert t == 10.0 and abs(y - math.pi / 2) < 1e-6 and abs(r) < 1e-9


def test_realign_recaptures_offset():
    bridge, sink = FakeBridge(), FakeSink()
    sim_now = {"t": 100.0}
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: sim_now["t"])
    rec.start(position_type=Msg, attitude_type=Msg)
    cb = bridge.cbs["/px4_0/fmu/out/vehicle_local_position"]
    cb(Msg(timestamp=40_000_000, x=0.0, y=0.0, z=0.0, heading=0.0, xy_valid=True))
    rec.realign()
    sim_now["t"] = 200.0
    cb(Msg(timestamp=50_000_000, x=0.0, y=0.0, z=0.0, heading=0.0, xy_valid=True))
    assert sink.poses[0][0] == 100.0 and sink.poses[1][0] == 200.0


def test_quat_to_rpy_identity_and_yaw90():
    assert _quat_to_rpy(1, 0, 0, 0) == (0.0, 0.0, 0.0)
    r, p, y = _quat_to_rpy(math.sqrt(0.5), 0, 0, math.sqrt(0.5))
    assert abs(y - math.pi / 2) < 1e-9


def test_offset_tracks_clock_skew():
    """A drifting sim clock must not strand the buffers: the EMA offset
    follows the skew, so converted stamps stay near ref (the live failure was
    pose_at(now) -> None after hours of ~0.1% drift)."""
    bridge = FakeBridge()
    sink = FakeSink()
    sim_now = {"t": 1000.0}
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: sim_now["t"])
    # first message anchors the offset
    rec._on_pose(Msg(timestamp=10_000_000, x=1.0, y=2.0, z=-3.0, heading=0.5))
    first_t = sink.poses[-1][0]
    assert first_t == 1000.0
    # both clocks tick ~1:1 (0.1 s/msg) with a 0.1% relative skew — the live
    # failure class; target rate is the SKEW rate, so the EMA lag stays ~ms
    for k in range(2000):
        sim_now["t"] += 0.1001
        rec._on_pose(Msg(timestamp=10_000_000 + (k + 1) * 100_000, x=1.0, y=2.0, z=-3.0, heading=0.5))
    err = abs(sink.poses[-1][0] - sim_now["t"])
    assert err < 0.2, f"offset did not track skew: {err:.2f}s off"


def test_boot_poison_stamp_dropped_and_offset_recaptured():
    """A wildly-wrong boot-transient timestamp must not poison the alignment:
    dropped, offset re-captured on the next sane message (observed live: one
    sample stamped ~17668080 s starved pose_at for a whole run)."""
    bridge, sink = FakeBridge(), FakeSink()
    sim_now = {"t": 60.0}
    rec = Px4StateRecorder(bridge, sink, i=0, sim_time_ref=lambda: sim_now["t"])
    rec.start(position_type=Msg, attitude_type=Msg)
    cb = bridge.cbs["/px4_0/fmu/out/vehicle_local_position"]
    # first sample is sane: offset = 60 - 40 = 20
    cb(Msg(timestamp=40_000_000, x=1.0, y=2.0, z=-5.0, heading=0.5, xy_valid=True))
    # boot poison: a stamp of ~17.7 MILLION seconds (t would land ~204 days out)
    cb(Msg(timestamp=17_668_080_910_000, x=1.0, y=2.0, z=-5.0, heading=0.5,
           xy_valid=True))
    assert len(sink.poses) == 1          # poison dropped, not recorded
    # next sane message re-captures cleanly (offset survives / re-derives)
    sim_now["t"] = 62.0
    cb(Msg(timestamp=42_000_000, x=3.0, y=4.0, z=-6.0, heading=0.6, xy_valid=True))
    assert sink.poses[-1] == (62.0, 4.0, 3.0, 6.0, 0.6)


def test_world_buffer_flushes_future_dated_head():
    """note_pose/note_attitude drop a future-dated poisoned head instead of
    letting it defeat _interp's coverage test forever."""
    from agents.world import World
    w = World()
    w.note_pose(17668080.91, 0.0, 0.0, 1.0, 0.0)      # the observed poison
    w.note_pose(60.10, 1.0, 2.0, 3.0, 0.5)            # first sane sample
    w.note_pose(60.20, 1.1, 2.1, 3.0, 0.5)
    assert w.pose_at(60.15) is not None               # interpolates again
    w.note_attitude(17668080.91, 0.0, 0.0, 0.0)
    w.note_attitude(60.10, 0.0, 0.1, 0.2)
    w.note_attitude(60.20, 0.0, 0.1, 0.2)
    assert w.attitude_at(60.15) is not None
