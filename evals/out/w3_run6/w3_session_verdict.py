"""W3-run5 session verdict math over session.log (host side).

Parses the w3_session.py event/status stream: engagement list (lock -> lost
durations), aggregate active seconds, re-lock latencies, alt band, gap band
(from the 1 Hz status rows' gap_m), min gap.

  python evals/out/w3_run5/w3_session_verdict.py [session.log]
"""
import json
import os
import statistics
import sys

PATH = (sys.argv[1] if len(sys.argv) > 1
        else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "session.log"))
ACTIVE = ("ACQUIRING", "TRACKING", "MEASURED", "COASTING", "DESIGNATED")


def main() -> None:
    events, rows = [], []
    with open(PATH) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            (events if "event" in r else rows).append(r)
    print("events:")
    for e in events:
        print("  ", json.dumps(e))
    locks = [e for e in events if e["event"] == "lock" and e.get("ok")]
    losses = [e for e in events if e["event"] == "lost"]
    print(f"\nengagements (locks ok): {len(locks)}")
    durs = [l["dur_s"] for l in losses]
    print(f"durations: {durs}  (>=20 s: {[d >= 20 for d in durs]})")
    lat = [e["relock_latency_s"] for e in locks
           if e.get("relock_latency_s") is not None]
    print(f"re-lock latencies: {lat}  (<=8 s: {[l <= 8 for l in lat]})")
    # aggregate active from the 1 Hz rows
    act = [r for r in rows if r.get("track") in ACTIVE and r.get("target")]
    if rows:
        span = rows[-1]["t"] - rows[0]["t"]
        print(f"rows span {span:.0f}s; active rows {len(act)} "
              f"(~{100.0 * len(act) / max(1, len(rows)):.0f}%)")
    alts = [r["alt"] for r in rows if r.get("alt") is not None]
    if alts:
        inb = [a for a in alts if 3.7 <= a <= 4.5]
        print(f"alt: mean={statistics.mean(alts):.2f} min={min(alts):.1f} "
              f"max={max(alts):.1f} in-3.7-4.5: {len(inb)}/{len(alts)}")
    gaps = [r["gap_m"] for r in rows if r.get("gap_m") is not None]
    if gaps:
        inb = [g for g in gaps if 12 <= g <= 20]
        print(f"gap (1 Hz rows): n={len(gaps)} mean={statistics.mean(gaps):.1f} "
              f"min={min(gaps):.1f} max={max(gaps):.1f} "
              f"in-12-20: {len(inb)}/{len(gaps)} "
              f"({100.0 * len(inb) / len(gaps):.0f}%) "
              f"sub-11: {[g for g in gaps if g < 11]}")


if __name__ == "__main__":
    main()
