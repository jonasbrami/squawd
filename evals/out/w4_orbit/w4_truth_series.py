"""W4 true-radius series (in-container): reconstructs car_1's gz-truth path
from cornerwatch corner crossings (leg-time interpolation — the measured
lap rate ~2.37 m/s, not the 4.0 spec), aligns the orbitwatch clock by
minimizing EKF-target-vs-truth distance, and writes true_radius.csv
(t, true horizontal radius from the drone's PX4 E/N).

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w4_orbit/w4_truth_series.py
"""
import json
import math
import re
import statistics

CORNERWATCH = "/tmp/cornerwatch_w4.log"
ORBITWATCH = "/tmp/orbitwatch_w4.log"
OUT = "/tmp/true_radius.csv"


def main() -> None:
    corners = []
    for line in open(CORNERWATCH):
        m = re.match(r"t=\s*([\d.]+) CORNER E(-?\d+) N(-?\d+)", line)
        if m:
            corners.append((float(m.group(1)), float(m.group(2)),
                            float(m.group(3))))
    corners.sort()

    def car_at(t):
        for i in range(len(corners) - 1):
            t0, e0, n0 = corners[i]
            t1, e1, n1 = corners[i + 1]
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return (e0 + (e1 - e0) * f, n0 + (n1 - n0) * f)
        return None

    rows = [json.loads(l) for l in open(ORBITWATCH)
            if l.strip().startswith("{")]
    seg = [r for r in rows if 30 <= r["t"] <= 70 and r.get("te") is not None]
    best = None
    for d10 in range(-60, 61):
        d = d10 / 10.0
        errs = [math.hypot(r["te"] - car_at(r["t"] + d)[0],
                           r["tn"] - car_at(r["t"] + d)[1])
                for r in seg if car_at(r["t"] + d)]
        if len(errs) < 50:
            continue
        m = statistics.mean(errs)
        if best is None or m < best[1]:
            best = (d, m)
    d, m = best
    print(f"clock offset ow=cw+({d:+.1f}s); EKF-vs-truth mean {m:.2f} m")
    n = 0
    with open(OUT, "w") as f:
        f.write("t,true_radius\n")
        for r in seg:
            c = car_at(r["t"] + d)
            if c:
                f.write(f"{r['t']},{math.hypot(r['de'] - c[0], r['dn'] - c[1]):.2f}\n")
                n += 1
    print(f"true series rows: {n} -> {OUT}")


if __name__ == "__main__":
    main()
