"""W3-run8 instrumented cornerwatch (in-container, R8-mandated harness
instrumentation): gz-truth corner crossings for car_1 PLUS the drone's
synchronized altitude pair — PX4 local alt (-lp.z, the /state alt) and the
gz-truth z of x500_depth_0 — at 10 Hz. The verdict script
(w3_session_verdict.py) classifies INVALID_ENV windows off this log:
|px4_alt - (gz_z + b)| > 1.5 m for >=0.5 s, with b the offset estimated at
the session's stable start. An INVALID_ENV window contributes NO pass/fail
samples.

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w3_run8/w3_cornerwatch.py [seconds]
"""
import os
import sys
import time

from agents.core.bus import RosBridge
from agents.core.gzposes import GzPoses

CORNERS = [(70.0, -30.0), (70.0, 30.0), (30.0, 30.0), (30.0, -30.0)]
CROSS_M = 3.0
ARM_M = 8.0


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 400.0
    bridge = RosBridge(node_name="w3_cornerwatch")
    from px4_msgs.msg import VehicleLocalPosition
    bridge.subscribe("/px4_0/fmu/out/vehicle_local_position",
                     VehicleLocalPosition)
    gz = GzPoses(os.environ.get("GZ_WORLD", "demo"),
                 ["car_1", "x500_depth_0"])
    bridge.start()
    print(f"BEGIN wall {time.time():.3f}", flush=True)
    t0 = time.monotonic()
    armed = {c: True for c in CORNERS}
    time.sleep(1.0)
    while time.monotonic() - t0 < seconds:
        t = time.monotonic() - t0
        lp = bridge.latest("/px4_0/fmu/out/vehicle_local_position")
        px4_alt = (-float(lp.z)) if lp is not None else float("nan")
        drone = gz.poses().get("x500_depth_0")
        gz_z = drone[2] if drone is not None and len(drone) > 2 else float("nan")
        print(f"ENV t={t:7.1f} px4_alt={px4_alt:6.2f} gz_z={gz_z:6.2f}",
              flush=True)
        p = gz.poses().get("car_1")
        if p is not None:
            for c in CORNERS:
                d = ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) ** 0.5
                if armed[c] and d <= CROSS_M:
                    print(f"t={t:7.1f} CORNER E{c[0]:.0f} N{c[1]:.0f} "
                          f"car_1 at E{p[0]:.1f} N{p[1]:.1f}", flush=True)
                    armed[c] = False
                elif not armed[c] and d >= ARM_M:
                    armed[c] = True
        time.sleep(0.1)


if __name__ == "__main__":
    main()
