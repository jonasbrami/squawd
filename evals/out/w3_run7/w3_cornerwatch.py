"""W3-run7 corner-truth logger (in-container): logs the gz-truth times
car_1 crosses each corner of its 40x60 m loop (within CROSS_M of the corner
point), at 10 Hz for N seconds — the truth column for the R7 corner
sub-gate's corner-by-corner table.

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w3_run7/w3_cornerwatch.py [seconds]
"""
import os
import sys
import time

from agents.core.gzposes import GzPoses

CORNERS = [(70.0, -30.0), (70.0, 30.0), (30.0, 30.0), (30.0, -30.0)]
CROSS_M = 3.0
ARM_M = 8.0          # re-arm distance: must leave this radius to re-trigger


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    gz = GzPoses(os.environ.get("GZ_WORLD", "demo"), ["car_1"])
    t0 = time.monotonic()
    armed = {c: True for c in CORNERS}
    time.sleep(1.0)
    while time.monotonic() - t0 < seconds:
        p = gz.poses().get("car_1")
        if p is not None:
            for c in CORNERS:
                d = ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) ** 0.5
                if armed[c] and d <= CROSS_M:
                    print(f"t={time.monotonic() - t0:7.1f} CORNER "
                          f"E{c[0]:.0f} N{c[1]:.0f} car_1 at "
                          f"E{p[0]:.1f} N{p[1]:.1f}", flush=True)
                    armed[c] = False
                elif not armed[c] and d >= ARM_M:
                    armed[c] = True
        time.sleep(0.1)


if __name__ == "__main__":
    main()
