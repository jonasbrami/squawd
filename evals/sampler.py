"""Sampler: poll the live World/RosBridge into a WorldTrack during a run.

snapshot_now is unit-testable with a fake World (no ROS). Sampler.run is the async
loop the runner starts before injecting the task and stops after; it tolerates the
brief windows where a drone has no valid fix (just omits it from that snapshot)."""
import asyncio
import time

from evals.worldstate import DronePose, Snapshot, WorldTrack


def snapshot_now(world, bridge, t: float, gzposes=None) -> Snapshot:
    st = world.drone_state(bridge, 0)
    poses = ({0: DronePose(e=st[0], n=st[1], alt=st[2], heading=st[3])}
             if st is not None else {})
    movers = gzposes.poses() if gzposes is not None else {}
    return Snapshot(t=t, poses=poses, movers=movers)


class Sampler:
    def __init__(self, world, bridge, objects, geofence_m, interval=0.5,
                 gzposes=None):
        self._world = world
        self._bridge = bridge
        self._objects = dict(objects)
        self._geofence_m = geofence_m
        self._interval = interval
        self._buildings = list(getattr(world, "buildings", []) or [])
        self._gzposes = gzposes
        self._snaps: list[Snapshot] = []
        self._running = False

    async def run(self) -> None:
        self._running = True
        t0 = time.monotonic()
        while self._running:
            self._snaps.append(snapshot_now(self._world, self._bridge,
                                            time.monotonic() - t0, self._gzposes))
            await asyncio.sleep(self._interval)

    def stop(self) -> None:
        self._running = False

    def track(self) -> WorldTrack:
        return WorldTrack(snapshots=list(self._snaps), objects=dict(self._objects),
                          geofence_m=self._geofence_m,
                          buildings=list(self._buildings))
