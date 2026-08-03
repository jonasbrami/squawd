"""W3-run8 session verdict math (host side) — run-7 verdicts PLUS the
R8-mandated INVALID_ENV scoring.

Reads session.log (w3_session.py events + 1 Hz rows) and the instrumented
cornerwatch log (ENV rows: synchronized px4_alt / gz_z at 10 Hz). Estimates
the alt offset b from the session's stable start (first 30 ENV samples,
drone parked at staging), classifies INVALID_ENV windows (|px4_alt -
(gz_z + b)| > 1.5 m for >=0.5 s), and scores alt/gap bands EXCLUDING rows
inside INVALID_ENV windows (they contribute no pass/fail samples).

  python evals/out/w3_run8/w3_session_verdict.py session.log cornerwatch.log
"""
import json
import os
import re
import statistics
import sys

ACTIVE = ("ACQUIRING", "TRACKING", "MEASURED", "COASTING", "DESIGNATED")


def load_session(path):
    events, rows, wall = [], [], None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r.get("event") == "begin":
                wall = r["wall"]
            (events if "event" in r else rows).append(r)
    return events, rows, wall


def load_env(path):
    wall, env, corners = None, [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("BEGIN wall"):
                wall = float(line.split()[2])
            elif line.startswith("ENV t="):
                m = re.match(r"ENV t=\s*([\d.]+) px4_alt=\s*([\d.nan-]+) "
                             r"gz_z=\s*([\d.nan-]+)", line)
                if m:
                    env.append((float(m.group(1)), float(m.group(2)),
                                float(m.group(3))))
            elif "CORNER" in line:
                m = re.match(r"t=\s*([\d.]+) CORNER E(-?\d+) N(-?\d+)", line)
                if m:
                    corners.append((float(m.group(1)), int(m.group(2)),
                                    int(m.group(3))))
    return wall, env, corners


def main() -> None:
    spath = (sys.argv[1] if len(sys.argv) > 1 else
             os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "session.log"))
    cpath = (sys.argv[2] if len(sys.argv) > 2 else
             os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "cornerwatch.log"))
    events, rows, swall = load_session(spath)
    cwall, env, corners = load_env(cpath)

    # --- INVALID_ENV scoring (BEFORE alt/gap scoring) ---
    invalid = []
    offset = None
    if env:
        base = [(a - g) for t, a, g in env[:30]
                if a == a and g == g]          # NaN-safe
        if base:
            offset = statistics.mean(base)
        bad_start = None
        for t, a, g in env:
            bad = (offset is not None and a == a and g == g
                   and abs(a - (g + offset)) > 1.5)
            if bad and bad_start is None:
                bad_start = t
            elif not bad and bad_start is not None:
                if t - bad_start >= 0.5:
                    invalid.append((bad_start, t))
                bad_start = None
        if bad_start is not None and env[-1][0] - bad_start >= 0.5:
            invalid.append((bad_start, env[-1][0]))
    # env clock -> session clock
    dt = (swall - cwall) if (swall and cwall) else 0.0

    def env_bad_at(ts):
        """session-time ts inside an INVALID_ENV window?"""
        tc = ts + dt              # session t -> cornerwatch t (approx)
        return any(a - 0.5 <= tc <= b + 0.5 for a, b in invalid)

    print("== INVALID_ENV analysis ==")
    print(f"alt offset b={None if offset is None else round(offset, 2)} m "
          f"(first 30 ENV samples); env rows {len(env)}")
    if invalid:
        for a, b in invalid:
            lo = min(abs(a2 - (g + offset))
                     for t, a2, g in env if a <= t <= b and a2 == a2
                     and g == g) if offset is not None else None
            print(f"  INVALID_ENV cornerwatch t={a:.1f}..{b:.1f} "
                  f"(session ~{a - dt:.1f}..{b - dt:.1f}) |dev|>1.5 m "
                  f">=0.5 s — contributes NO samples")
    else:
        print("  no INVALID_ENV windows")

    print("\n== events ==")
    for e in events:
        if e.get("event") in ("lock", "lost"):
            print("  ", json.dumps(e))
    locks = [e for e in events if e["event"] == "lock" and e.get("ok")]
    losses = [e for e in events if e["event"] == "lost"]
    durs = [l["dur_s"] for l in losses]
    lat = [e["relock_latency_s"] for e in locks
           if e.get("relock_latency_s") is not None]
    print(f"\nengagements: {len(locks)}; durations {durs} "
          f"(>=20/55 s: {[(d >= 20, d >= 55) for d in durs]})")
    print(f"re-lock latencies: {lat} (<=8: {[l <= 8 for l in lat]})")
    act = [r for r in rows if r.get("track") in ACTIVE and r.get("target")]
    if rows:
        print(f"rows span {rows[-1]['t'] - rows[0]['t']:.0f}s; "
              f"active rows {len(act)} "
              f"(~{100.0 * len(act) / max(1, len(rows)):.0f}%)")
    valid = [r for r in rows if not env_bad_at(r.get("t", -1))]
    nbad = len(rows) - len(valid)
    if nbad:
        print(f"excluded {nbad} rows inside INVALID_ENV windows")
    alts = [r["alt"] for r in valid if r.get("alt") is not None]
    if alts:
        inb = [a for a in alts if 3.7 <= a <= 4.5]
        print(f"alt (valid): mean={statistics.mean(alts):.2f} "
              f"min={min(alts):.1f} max={max(alts):.1f} "
              f"in-3.7-4.5: {len(inb)}/{len(alts)}")
    gaps = [r["gap_m"] for r in valid if r.get("gap_m") is not None]
    if gaps:
        inb = [g for g in gaps if 12 <= g <= 20]
        sub = [round(g, 1) for g in gaps if g < 11]
        print(f"gap (valid): n={len(gaps)} mean={statistics.mean(gaps):.1f} "
              f"min={min(gaps):.1f} max={max(gaps):.1f} "
              f"in-12-20: {len(inb)}/{len(gaps)} "
              f"({100.0 * len(inb) / len(gaps):.0f}%) sub-11: {sub}")
    if corners:
        print("\ncorner crossings (cornerwatch t):")
        for t, e, n in corners:
            print(f"  t={t:7.1f} E{e} N{n}  (session ~{t - dt:.1f})")


if __name__ == "__main__":
    main()
