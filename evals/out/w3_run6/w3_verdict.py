"""W3-rerun verdict math over timeline.log (host side).

Per phase: track-state histogram, LOST excursions, distinct adopted target
ids, gap stats (overall + last-30 s mean), alt stats, beam statuses seen.

  python evals/out/w3_rerun/w3_verdict.py [phase ...]
"""
import json
import os
import statistics
import sys

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "timeline.log")


def main() -> None:
    wanted = set(sys.argv[1:])
    rows = []
    with open(OUT) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    phases = {}
    order = []
    for r in rows:
        ph = r.get("phase")
        if ph not in phases:
            phases[ph] = []
            order.append(ph)
        if not r.get("event"):
            phases[ph].append(r)
    for ph in order:
        if wanted and ph not in wanted:
            continue
        rs = phases[ph]
        if not rs:
            continue
        states, beams, targets, gaps, alts = {}, {}, [], [], []
        seq = []
        for r in rs:
            tr, bm = r.get("track") or {}, r.get("beam") or {}
            st = tr.get("state")
            states[st] = states.get(st, 0) + 1
            b = bm.get("status")
            beams[b] = beams.get(b, 0) + 1
            tgt = tr.get("target")
            if tgt and (not targets or targets[-1] != tgt):
                targets.append(tgt)
            seq.append((r.get("t"), st, tr.get("gap_m"), r.get("alt"),
                        r.get("speed")))
            if tr.get("gap_m") is not None:
                gaps.append(tr.get("gap_m"))
            if r.get("alt") is not None:
                alts.append(r.get("alt"))
        t_end = seq[-1][0] or 0
        last30 = [g for (t, st, g, a, s) in seq
                  if g is not None and t is not None and t >= t_end - 30.0]
        lost_t = [t for (t, st, g, a, s) in seq if st == "LOST"]
        print(f"== {ph}  n={len(rs)}  span 0-{t_end:.0f}s")
        print(f"   states: {states}")
        print(f"   beam:   {beams}")
        print(f"   targets adopted ({len(set(targets))}): {targets}")
        if gaps:
            print(f"   gap_m:  n={len(gaps)} mean={statistics.mean(gaps):.1f} "
                  f"min={min(gaps):.1f} max={max(gaps):.1f}")
        if last30:
            print(f"   gap last-30s mean={statistics.mean(last30):.1f} "
                  f"(n={len(last30)})")
        if alts:
            print(f"   alt:    mean={statistics.mean(alts):.2f} "
                  f"min={min(alts):.1f} max={max(alts):.1f}")
        print(f"   LOST samples: {len(lost_t)}"
              + (f" at t={lost_t[:8]}{'…' if len(lost_t) > 8 else ''}"
                 if lost_t else ""))


if __name__ == "__main__":
    main()
