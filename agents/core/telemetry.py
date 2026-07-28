"""Px4StateRecorder: feeds a duck-typed sink (World) from PX4 telemetry (W1).

Subscribes vehicle_local_position + vehicle_attitude on the bridge and calls
sink.note_pose / sink.note_attitude with SIM-TIME-ALIGNED stamps. Lives in core
so `world` stays ROS-free; message objects arrive duck-typed.

Clock alignment (design R11): PX4 stamps are µs since PX4 boot; gz sim time is
seconds since world start. PX4 and gz run in lockstep, so a single constant
offset relates them. The offset is captured ONCE at the first message, using a
sim-time reference callable (e.g. cameras.stamp) read at the same instant:
    offset = sim_time_ref() - px4_t_seconds
Before the offset is captured (no ref yet), nothing is recorded — pose_at()
honestly returns None. The offset can be re-captured via realign().
"""
import math
import threading


def _quat_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """PX4 q (w,x,y,z) -> (roll, pitch, yaw), radians."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class Px4StateRecorder:
    """bridge -> sink (World) recorder with one-time clock alignment."""

    def __init__(self, bridge, sink, i: int = 0,
                 sim_time_ref=None) -> None:
        self._bridge = bridge
        self._sink = sink
        self._i = i
        self._ref = sim_time_ref          # callable -> float sim seconds, or None
        self._lock = threading.Lock()
        self._offset: float | None = None  # sim_seconds - px4_seconds

    def _sim_t(self, px4_stamp_us: float) -> float | None:
        with self._lock:
            if self._ref is None:
                return None
            ref = self._ref()
            if not ref:
                return None
            px4_t = px4_stamp_us * 1e-6
            if self._offset is None:
                self._offset = ref - px4_t
            else:
                # PX4's hrt and the gz clock are NOT rate-locked: the one-shot
                # offset goes stale at ~0.1% (seconds over a long session —
                # observed: pose_at/attitude_at returning None at "now",
                # starving VisionContacts of projections). Track the skew with
                # an EMA (τ ≈ 100 samples → ~50 ms ramp lag, jitter-smooth).
                self._offset += 0.01 * ((ref - px4_t) - self._offset)
            t = self._offset + px4_t
            # boot-poison guard (M3b forensics, 2026-07-21): a PX4 boot
            # transient can emit one wildly-wrong timestamp (observed: a
            # single sample stamped 17668080.91 s — ~204 days — sitting at
            # the World buffer HEAD forever; _interp never extrapolates, so
            # pose_at returned None for the WHOLE run and VisionContacts
            # never produced a measurement). Any stamp >30 s off the live
            # reference is garbage: drop it and re-capture the offset on the
            # next sane message.
            if abs(t - ref) > 30.0:
                self._offset = None
                return None
            return t

    def realign(self) -> None:
        """Drop the captured offset (e.g. after a sim anchor/reset); the next
        message re-captures it."""
        with self._lock:
            self._offset = None

    def start(self, position_type=None, attitude_type=None) -> None:
        """Subscribe the recorder. px4_msgs types are lazily imported at
        runtime; tests inject fakes."""
        if position_type is None or attitude_type is None:
            from px4_msgs.msg import (VehicleAttitude as _VA,
                                      VehicleLocalPosition as _VLP)
            position_type = _VLP if position_type is None else position_type
            attitude_type = _VA if attitude_type is None else attitude_type
        self._bridge.subscribe(
            f"/px4_{self._i}/fmu/out/vehicle_local_position",
            position_type, callback=self._on_pose)
        self._bridge.subscribe(
            f"/px4_{self._i}/fmu/out/vehicle_attitude",
            attitude_type, callback=self._on_att)

    def _on_pose(self, p) -> None:
        if not getattr(p, "xy_valid", True):
            return
        t = self._sim_t(getattr(p, "timestamp", 0))
        if t is None:
            return
        e, n, alt = self._sink.ned_to_enu(self._i, p.x, p.y, p.z)
        self._sink.note_pose(t, e, n, alt, float(getattr(p, "heading", 0.0)))

    def _on_att(self, m) -> None:
        t = self._sim_t(getattr(m, "timestamp", 0))
        if t is None:
            return
        q = getattr(m, "q", None)            # px4 array: no `or` — numpy truth
        if q is None:                         # is ambiguous on 4-element arrays
            q = (1.0, 0.0, 0.0, 0.0)
        roll, pitch, yaw = _quat_to_rpy(float(q[0]), float(q[1]),
                                        float(q[2]), float(q[3]))
        self._sink.note_attitude(t, roll, pitch, yaw)
