"""Gazebo Harmonic PythonSystemLoader module: drives one kinematic mover model
along its analytic trajectory, inside the physics loop.

Attached per-mover in the generated world SDF:
    <plugin filename="gz-sim-python-system-loader-system"
            name="gz::sim::systems::PythonSystemLoader">
      <module_name>mover_system</module_name>
    </plugin>
(the loader finds this file via GZ_SIM_SYSTEM_PLUGIN_PATH=<repo>/sim/plugins)

Each physics step the mover's LINK VELOCITY is set to the analytic
trajectory.vel_xy(spec, sim_t - t0); position drift against pos_xy is checked
each second and snapped only when it exceeds SNAP_M. Velocity-drive matters:
commanding world POSE every step destabilised PX4's EKF on the same machine
(persistent "vertical velocity unstable" / "Yaw estimate error" preflight
failures, arming denied — verified live: removing the movers restored
arming). Smooth velocities keep physics happy; the once-a-second drift gate
keeps motion an exact function of sim time for grading. The trajectory spec
comes from the world's movers sidecar (env MOVERS_JSON), keyed by model name.

Heading alignment (W1b): movers with "heading_align": true in the sidecar get
their YAW driven to the velocity direction by the same PX4-safe channel — a
proportional angular-VELOCITY command (yaw_drive, recomputed on the same
10-step pose read as the z-hold), never per-step pose commands. Without it
angular velocity is zeroed and the mover keeps its spawn yaw forever: cars
slid sideways through every leg not aligned with the t0 heading (W1a). The
rare drift-snap pose command also restores the trajectory heading then.

LINK-FRAME VELOCITY (W1b): gz's Link.SetLinearVelocity takes the velocity in
the LINK'S frame (gz/sim/Link.hh), not the world frame — so the world-frame
trajectory velocity must be rotated by -yaw (link_frame()). At yaw 0 the
rotation is identity, which is why the pre-alignment movers (frozen yaw 0)
tracked fine, and why heading_align (yaw follows velocity) breaks hard
without it — verified live 2026-08-01: an un-rotated car drove every
vertical leg BACKWARD at 4 m/s, position held only by the 1 Hz drift snap.

Phase anchor: a gz.msgs Empty on /movers/anchor re-zeroes t0 for every mover
at the next step — the eval runner publishes it during soft_reset so each
cell starts at trajectory phase 0. t0 auto-anchors on first step otherwise.

Host unit tests import only the pure helpers; gz bindings load lazily inside
the sim container.
"""
import json
import math
import os
import sys
from pathlib import Path

# self-locating: this file lives at <repo>/sim/plugins/, trajectory at
# <repo>/agents/world/ — the gz server process has no PYTHONPATH promise
_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from agents.world.trajectory import pos_xy, vel_xy  # noqa: E402

ANCHOR_TOPIC = "/movers/anchor"
SNAP_M = 1.0            # drift beyond this teleports the mover back on-spec
DRIFT_CHECK_STEPS = 250  # ~1 s at the 4 ms physics step
WZ_GAIN = 4.0           # heading_align: yaw-rate per rad of heading error
WZ_MAX = 2.0            # rad/s clamp — a 90 deg corner swings in well under 1 s


def yaw_drive(target: float, current: float) -> float:
    """Clamped proportional yaw-rate command toward the target heading (the
    error wraps across +/-pi). Pure helper: host tests need no gz bindings."""
    err = (target - current + math.pi) % (2.0 * math.pi) - math.pi
    return max(-WZ_MAX, min(WZ_MAX, WZ_GAIN * err))


def link_frame(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    """World-frame velocity rotated into the link frame (gz's
    Link.SetLinearVelocity is LINK-frame). Identity at yaw 0."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx + s * vy, -s * vx + c * vy)


def load_spec(sidecar_path: str, model_name: str) -> dict:
    """The mover's entry from the movers sidecar; raises with a legible message
    when the model has no entry (a generator/world mismatch, fail loud)."""
    with open(sidecar_path) as f:
        cfg = json.load(f)
    for m in cfg.get("movers", []):
        if m["name"] == model_name:
            return m
    raise KeyError(f"model {model_name!r} not in {sidecar_path} movers "
                   f"({[m['name'] for m in cfg.get('movers', [])]})")


def to_seconds(sim_time) -> float:
    """UpdateInfo.sim_time is a datetime.timedelta in the python bindings;
    accept plain numbers too so tests don't need gz."""
    return sim_time.total_seconds() if hasattr(sim_time, "total_seconds") \
        else float(sim_time)


class MoverSystem:
    def __init__(self):
        self._model = None
        self._link = None
        self._spec = None
        self._z = 0.0
        self._t0 = None
        self._anchor_req = False
        self._steps = 0
        self._vz = 0.0
        self._align = False      # sidecar "heading_align": yaw follows velocity
        self._wz = 0.0
        self._node = None   # kept alive: gz subscription dies with the Node

    def configure(self, entity, sdf, ecm, event_mgr):
        from gz.sim8 import Link, Model
        from gz.transport13 import Node
        from gz.msgs10.empty_pb2 import Empty

        self._model = Model(entity)
        self._link = Link(self._model.canonical_link(ecm))
        name = self._model.name(ecm)
        sidecar = os.environ.get("MOVERS_JSON", "")
        if not sidecar:
            raise RuntimeError("MOVERS_JSON env var not set for mover_system")
        self._spec = load_spec(sidecar, name)
        self._z = float(self._spec.get("z", 0.0))
        self._align = bool(self._spec.get("heading_align", False))
        self._node = Node()
        self._node.subscribe(Empty, ANCHOR_TOPIC, self._on_anchor)

    def _on_anchor(self, _msg):
        self._anchor_req = True

    def _heading(self, t: float) -> float:
        """Trajectory heading at t for aligned movers; 0.0 otherwise (the
        pre-W1b behaviour the dynamic/perceive worlds still rely on)."""
        if not self._align:
            return 0.0
        vx, vy = vel_xy(self._spec["traj"], t)
        return math.atan2(vy, vx)

    def _snap(self, ecm, x: float, y: float, yaw: float = 0.0):
        from gz.math7 import Pose3d
        self._model.set_world_pose_cmd(ecm, Pose3d(x, y, self._z, 0.0, 0.0, yaw))

    def pre_update(self, info, ecm):
        if info.paused:
            return
        from gz.math7 import Vector3d
        t = to_seconds(info.sim_time)
        trel = 0.0 if self._t0 is None else t - self._t0
        if self._t0 is None or self._anchor_req:
            self._t0 = t
            self._anchor_req = False
            trel = 0.0
            self._snap(ecm, *pos_xy(self._spec["traj"], 0.0), self._heading(0.0))
        vx, vy = vel_xy(self._spec["traj"], trel)
        # z-hold (fable-R2-1): vz was 0 and the kinematic mover's z DRIFTED up
        # to SNAP_M between 1 s drift checks — a ~1 m z error is a ~4–5 m
        # support-plane projection error at the gate's geometry (the "EKF lag"
        # chased for days). Proportional velocity-drive correction (PX4-safe,
        # same channel as vx/vy — no pose commands).
        self._steps += 1
        pose = self._link.world_pose(ecm)
        yaw = pose.rot().euler().z() if pose is not None else None
        if pose is not None and self._steps % 10 == 0:
            self._vz = max(-1.0, min(1.0, -1.0 * (pose.pos().z() - self._z)))
            if self._align:
                # heading_align (W1b): yaw chases atan2(vy,vx) through a
                # clamped proportional yaw-RATE (no pose commands; corners
                # read as a short pivot, not a sideways slide).
                self._wz = (yaw_drive(math.atan2(vy, vx), yaw)
                            if vx * vx + vy * vy > 1e-4 else 0.0)
        # SetLinearVelocity is LINK-frame (gz/sim/Link.hh) — rotate the
        # world-frame trajectory velocity by -yaw. Identity at yaw 0, which is
        # the only reason the pre-heading_align movers tracked without it.
        if yaw is not None:
            vx, vy = link_frame(vx, vy, yaw)
        self._link.set_linear_velocity(ecm, Vector3d(vx, vy, self._vz))
        self._link.set_angular_velocity(
            ecm, Vector3d(0.0, 0.0, self._wz if self._align else 0.0))
        if self._steps % DRIFT_CHECK_STEPS == 0 and pose is not None:
            ax, ay = pos_xy(self._spec["traj"], trel)
            dx, dy = pose.pos().x() - ax, pose.pos().y() - ay
            dz = pose.pos().z() - self._z
            if (dx * dx + dy * dy + dz * dz) ** 0.5 > SNAP_M:
                self._snap(ecm, ax, ay, self._heading(trel))


def get_system():
    return MoverSystem()
