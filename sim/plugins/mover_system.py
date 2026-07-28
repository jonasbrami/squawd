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

Phase anchor: a gz.msgs Empty on /movers/anchor re-zeroes t0 for every mover
at the next step — the eval runner publishes it during soft_reset so each
cell starts at trajectory phase 0. t0 auto-anchors on first step otherwise.

Host unit tests import only the pure helpers; gz bindings load lazily inside
the sim container.
"""
import json
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
        self._node = Node()
        self._node.subscribe(Empty, ANCHOR_TOPIC, self._on_anchor)

    def _on_anchor(self, _msg):
        self._anchor_req = True

    def _snap(self, ecm, x: float, y: float):
        from gz.math7 import Pose3d
        self._model.set_world_pose_cmd(ecm, Pose3d(x, y, self._z, 0.0, 0.0, 0.0))

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
            self._snap(ecm, *pos_xy(self._spec["traj"], 0.0))
        vx, vy = vel_xy(self._spec["traj"], trel)
        # z-hold (fable-R2-1): vz was 0 and the kinematic mover's z DRIFTED up
        # to SNAP_M between 1 s drift checks — a ~1 m z error is a ~4–5 m
        # support-plane projection error at the gate's geometry (the "EKF lag"
        # chased for days). Proportional velocity-drive correction (PX4-safe,
        # same channel as vx/vy — no pose commands).
        self._steps += 1
        if self._steps % 10 == 0:
            pose = self._link.world_pose(ecm)
            if pose is not None:
                self._vz = max(-1.0, min(1.0, -1.0 * (pose.pos().z() - self._z)))
        self._link.set_linear_velocity(ecm, Vector3d(vx, vy, self._vz))
        self._link.set_angular_velocity(ecm, Vector3d(0.0, 0.0, 0.0))
        if self._steps % DRIFT_CHECK_STEPS == 0:
            pose = self._link.world_pose(ecm)
            if pose is not None:
                ax, ay = pos_xy(self._spec["traj"], trel)
                dx, dy = pose.pos().x() - ax, pose.pos().y() - ay
                dz = pose.pos().z() - self._z
                if (dx * dx + dy * dy + dz * dz) ** 0.5 > SNAP_M:
                    self._snap(ecm, ax, ay)


def get_system():
    return MoverSystem()
