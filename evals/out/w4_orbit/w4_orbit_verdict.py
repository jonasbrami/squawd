"""W4 orbit verdict math (host side) over w4_orbitwatch's JSONL log.

Slices the orbit window (--t0/--t1 in orbitwatch seconds; defaults to all
rows with a radius), then reports: radius mean/p05/p95, speed mean/std,
yaw-rate mean/std (unwrapped heading diff), MEASURED health fraction, LOST
rows. Also re-scores INVALID_ENV off the instrumented cornerwatch log when
given (the R8 harness rule: those windows contribute no samples).

  python evals/out/w4_orbit/w4_orbit_verdict.py orbitwatch.log \
      [--t0 S] [--t1 S] [cornerwatch.log]
"""
import json
import math
import re
import statistics
import sys


def pct(vals, q):
    s = sorted(vals)
    if not s:
        return None
    k = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
    return s[k]


def main() -> None:
    path = sys.argv[1]
    t0 = float(sys.argv[sys.argv.index("--t0") + 1]) if "--t0" in sys.argv else None
    t1 = float(sys.argv[sys.argv.index("--t1") + 1]) if "--t1" in sys.argv else None
    cwpath = next((a for a in sys.argv[2:] if a.endswith("cornerwatch.log")
                   or "cornerwatch" in a), None)
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("BEGIN") or not line:
                continue
            r = json.loads(line)
            if (t0 is None or r["t"] >= t0) and (t1 is None or r["t"] <= t1):
                rows.append(r)
    # INVALID_ENV exclusion (R8 rule)
    invalid = []
    if cwpath:
        env, wall_cw = [], None
        with open(cwpath) as f:
            for line in f:
                if line.startswith("BEGIN wall"):
                    wall_cw = float(line.split()[2])
                m = re.match(r"ENV t=\s*([\d.]+) px4_alt=\s*([\d.-]+) "
                             r"gz_z=\s*([\d.-]+)", line)
                if m:
                    env.append((float(m.group(1)), float(m.group(2)),
                                float(m.group(3))))
        if env:
            base = statistics.mean(a - g for t, a, g in env[:30])
            bad = None
            for t, a, g in env:
                d = abs(a - (g + base))
                if d > 1.5 and bad is None:
                    bad = t
                elif d <= 1.5 and bad is not None:
                    if t - bad >= 0.5:
                        invalid.append((bad, t))
                    bad = None
            if bad is not None and env[-1][0] - bad >= 0.5:
                invalid.append((bad, env[-1][0]))
        print(f"INVALID_ENV windows (cornerwatch clock): "
              f"{[(round(a,1), round(b,1)) for a, b in invalid] or 'none'}")
    # orbitwatch clock == cornerwatch clock (same batch start ~1-2 s apart);
    # exclude rows inside invalid windows (approximate, +-2 s tolerance)
    def bad_at(t):
        return any(a - 2.0 <= t <= b + 2.0 for a, b in invalid)
    valid = [r for r in rows if not bad_at(r["t"])]
    if len(valid) != len(rows):
        print(f"excluded {len(rows) - len(valid)} rows in INVALID_ENV windows")
    rows = valid
    if not rows:
        print("no rows")
        return
    rad = [r["radius"] for r in rows if r.get("radius") is not None]
    spd = [r["spd"] for r in rows if r.get("spd") is not None]
    meas = [r for r in rows if r.get("health") == "MEASURED"]
    coast = [r for r in rows if r.get("health") == "COASTING"]
    lost = [r for r in rows if r.get("mode") == "LOST"]
    span = rows[-1]["t"] - rows[0]["t"]
    print(f"rows {len(rows)} span {span:.1f}s")
    if rad:
        print(f"radius: n={len(rad)} mean={statistics.mean(rad):.2f} "
              f"p05={pct(rad, 0.05):.2f} p50={pct(rad, 0.5):.2f} "
              f"p95={pct(rad, 0.95):.2f} min={min(rad):.2f} max={max(rad):.2f}")
        inb = [x for x in rad if 11 <= x <= 19]
        print(f"radius in 15±4: {len(inb)}/{len(rad)} "
              f"({100.0 * len(inb) / len(rad):.0f}%)")
    if spd:
        print(f"speed: mean={statistics.mean(spd):.2f} "
              f"std={statistics.pstdev(spd):.2f} "
              f"min={min(spd):.2f} max={max(spd):.2f}")
    # yaw-rate smoothness (successive unwrapped heading diffs)
    hdgs = [r["hdg"] for r in rows if r.get("hdg") is not None]
    ts = [r["t"] for r in rows if r.get("hdg") is not None]
    rates = []
    for (ta, ha), (tb, hb) in zip(zip(ts, hdgs), zip(ts[1:], hdgs[1:])):
        dt = tb - ta
        if dt <= 0:
            continue
        d = (hb - ha + 540.0) % 360.0 - 180.0
        rates.append(d / dt)
    if rates:
        print(f"yaw rate: mean={statistics.mean(rates):+.1f} dps "
              f"std={statistics.pstdev(rates):.1f} dps "
              f"p05={pct(rates, 0.05):+.1f} p95={pct(rates, 0.95):+.1f}")
    n = len(rows)
    print(f"health: MEASURED {len(meas)} ({100.0 * len(meas) / n:.0f}%) "
          f"COASTING {len(coast)} ({100.0 * len(coast) / n:.0f}%) "
          f"LOST rows {len(lost)}")
    # raw series for the plot (radius vs t)
    with open(path + ".radius_series.csv", "w") as f:
        f.write("t,radius\n")
        for r in rows:
            if r.get("radius") is not None:
                f.write(f"{r['t']},{r['radius']}\n")
    print(f"series -> {path}.radius_series.csv")


if __name__ == "__main__":
    main()
