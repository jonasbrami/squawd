"""GzPoses: latest scripted-mover positions off gz-transport.

Subscribes /world/<w>/dynamic_pose/info (gz.msgs.Pose_V, ~49 Hz, sim-time
stamped) and keeps the newest pose per tracked model name. Consumers:
- the evals sampler snapshots mover positions in the SAME tick as drone
  poses, so oracle checks are pure within-snapshot geometry (no wall-clock
  vs sim-time drift);
- a drone agent's `scan` reports movers as live contacts.

Same ownership pattern as GzCameras (agents/core/camera.py): one gz Node,
callbacks kept alive, a lock around the latest-value dict. gz imports happen
in __init__ so pure consumers can be unit-tested with a fake.
"""
import threading


class GzPoses:
    """Holds the latest (x, y, z) per tracked model + last sim-time stamp."""

    def __init__(self, world: str, names: list[str]) -> None:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V

        self._names = set(names)
        self._lock = threading.Lock()
        self._poses: dict[str, tuple[float, float, float]] = {}
        self._sim_t: float = 0.0
        self._node = GzNode()
        self._cb = self._on_msg          # keep alive for gz
        self._node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self._cb)

    def _on_msg(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self._lock:
            self._sim_t = stamp
            for p in msg.pose:
                if p.name in self._names:
                    self._poses[p.name] = (p.position.x, p.position.y, p.position.z)

    def poses(self) -> dict[str, tuple[float, float, float]]:
        """Latest {name: (x, y, z)} for every tracked mover seen so far."""
        with self._lock:
            return dict(self._poses)

    def sim_time(self) -> float:
        """Sim-time stamp (seconds) of the newest pose message, 0.0 before any."""
        with self._lock:
            return self._sim_t
