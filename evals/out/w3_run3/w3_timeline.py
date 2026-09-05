"""W3-integration timeline sampler (host side).

Appends one JSON line per /state sample (every --period s for --seconds)
to evals/out/w3_integration/timeline.log, tagged with a phase --label, and
echoes a compact human line. The verdict math (LOCKED count, gap settling)
is done by w3_verdict.py over the same file.

  python evals/out/w3_integration/w3_timeline.py --label pursuit \
      --seconds 90 [--period 2]
"""
import argparse
import json
import os
import time
import urllib.request

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "timeline.log")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--url", default="http://localhost:8000")
    args = ap.parse_args()

    t0 = time.monotonic()
    with open(OUT, "a") as f:
        f.write(json.dumps({"phase": args.label, "event": "begin",
                            "wall": round(time.time(), 3)}) + "\n")
        f.flush()
        while time.monotonic() - t0 < args.seconds:
            t = round(time.monotonic() - t0, 1)
            try:
                with urllib.request.urlopen(f"{args.url}/state",
                                            timeout=5) as r:
                    s = json.loads(r.read())
            except Exception as e:
                print(f"[{args.label} {t:5.1f}s] /state error: {e}", flush=True)
                time.sleep(args.period)
                continue
            row = {"phase": args.label, "t": t,
                   "mode": s.get("mode"), "armed": s.get("armed"),
                   "alt": s.get("alt"), "east": s.get("east"),
                   "north": s.get("north"), "speed": s.get("speed"),
                   "track": s.get("track"), "beam": s.get("beam"),
                   "contacts": [c.get("name")
                                for c in s.get("contacts") or []],
                   "cam_stamp": s.get("cam_stamp"),
                   "sim_stamp": s.get("sim_stamp")}
            f.write(json.dumps(row) + "\n")
            f.flush()
            tr, bm = s.get("track") or {}, s.get("beam") or {}
            gap, rng = tr.get("gap_m"), bm.get("range_m")
            print(f"[{args.label} {t:5.1f}s] mode={s.get('mode')} "
                  f"alt={s.get('alt')} track={tr.get('state')} "
                  f"gap={None if gap is None else round(gap, 1)} "
                  f"beam={bm.get('status')} "
                  f"range={None if rng is None else round(rng, 1)} "
                  f"contacts={len(row['contacts'])}", flush=True)
            time.sleep(args.period)
        f.write(json.dumps({"phase": args.label, "event": "end",
                            "wall": round(time.time(), 3)}) + "\n")


if __name__ == "__main__":
    main()
