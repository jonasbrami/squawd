"""W3-run5 gate-2 session driver (host side, LLM-free): the re-scoped
engagement-structure gate — 90 s session, click-engage whenever the track is
IDLE/LOST and a clickable car exists, re-lock <=8 s after any LOST.

Logs one JSON line per event (lock/lost/relock) plus per-second status rows
to stdout (tee to session.log). The verdict math (engagement list, active
aggregate, re-lock latencies) is done by w3_session_verdict.py.

  python evals/out/w3_run5/w3_session.py [--seconds 90] [--max-locks 6]
"""
import argparse
import json
import time
import urllib.request

FRAME_W, FRAME_H = 640, 360
ACTIVE = ("ACQUIRING", "TRACKING", "MEASURED", "COASTING", "DESIGNATED")


def get_json(url, data=None, timeout=5):
    req = urllib.request.Request(url)
    if data is not None:
        req = urllib.request.Request(
            url, data=json.dumps(data).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read())
        except Exception:
            return e.code, {"error": str(e)}
    except Exception as e:
        return None, {"error": str(e)}


def pick(contacts, cx=(160, 480), max_range=40.0, edge=1):
    """Clickable car: bbox fully in-frame, ranged, near-boresight."""
    cands = []
    for c in contacts or []:
        box = c.get("bbox_xyxy")
        if not box or c.get("cls") != "car":
            continue
        x1, y1, x2, y2 = box
        if x1 < edge or y1 < edge or x2 > FRAME_W - edge or \
                y2 > FRAME_H - edge:
            continue
        rng = c.get("range_m")
        if rng is None or rng > max_range:
            continue
        if not (cx[0] <= (x1 + x2) / 2.0 <= cx[1]):
            continue
        cands.append(((x2 - x1) * (y2 - y1), c))
    cands.sort(key=lambda t: -t[0])
    return cands[0][1] if cands else None


def why_not(contacts, cx=(160, 480), max_range=40.0, edge=1):
    """pick-failure forensics (run7 attempt-1: zero locks, gates silent):
    per car contact, the first gate it fails."""
    out = []
    for c in contacts or []:
        if c.get("cls") != "car":
            continue
        box = c.get("bbox_xyxy") or [0, 0, 0, 0]
        x1, y1, x2, y2 = box
        rng = c.get("range_m")
        if not c.get("bbox_xyxy"):
            r = "no-box"
        elif x1 < edge or y1 < edge or x2 > FRAME_W - edge or \
                y2 > FRAME_H - edge:
            r = "edge"
        elif rng is None:
            r = "range-none"
        elif rng > max_range:
            r = f"range>{max_range:g}"
        elif not (cx[0] <= (x1 + x2) / 2.0 <= cx[1]):
            r = "cx"
        else:
            r = "OK"
        out.append((c.get("name"), r,
                    None if rng is None else round(rng, 1),
                    [round(v) for v in box]))
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--max-locks", type=int, default=6)
    ap.add_argument("--align", action="store_true",
                    help="hold the session clock until any car contact is "
                         "visible (the 90 s gate measures the engagement "
                         "session, not the parking time between laps)")
    ap.add_argument("--url", default="http://localhost:8000")
    args = ap.parse_args()

    if args.align:
        print(json.dumps({"event": "align_wait"}), flush=True)
        while True:
            st, state = get_json(f"{args.url}/state")
            if st == 200 and any(c.get("cls") == "car"
                                 for c in state.get("contacts") or []):
                break
            time.sleep(0.5)
        print(json.dumps({"event": "align_found"}), flush=True)

    t0 = time.monotonic()
    print(json.dumps({"event": "begin", "wall": round(time.time(), 3)}),
          flush=True)
    locks = 0
    eng = None              # {"name", "start"}
    lost_t = None
    last_row = 0.0
    while time.monotonic() - t0 < args.seconds:
        t = time.monotonic() - t0
        st, state = get_json(f"{args.url}/state")
        if st != 200:
            time.sleep(0.5)
            continue
        tr = state.get("track") or {}
        tstate, tgt = tr.get("state"), tr.get("target")
        active = tstate in ACTIVE and tgt is not None
        if active and eng is None:
            eng = {"name": tgt, "start": t}   # op already engaged (first lock)
        if not active and eng is not None:
            print(json.dumps({"event": "lost", "t": round(t, 1),
                              "name": eng["name"],
                              "dur_s": round(t - eng["start"], 1)}), flush=True)
            eng, lost_t = None, t
        if not active and locks < args.max_locks:
            c = pick(state.get("contacts"))
            if c is None and t - last_row >= 4.9:
                reasons = why_not(state.get("contacts"))
                if reasons:
                    print(json.dumps({"t": round(t, 1),
                                      "pick_fail": reasons}), flush=True)
            if c is not None:
                x1, y1, x2, y2 = c["bbox_xyxy"]
                x, y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                st2, resp = get_json(f"{args.url}/api/lock", {"x": x, "y": y})
                ok = st2 == 200 and resp.get("ok")
                locks += ok and 1 or 0
                lat = (round(t - lost_t, 1)) if lost_t is not None else None
                print(json.dumps({"event": "lock", "t": round(t, 1),
                                  "name": c["name"], "status": st2, "ok": ok,
                                  "relock_latency_s": lat}), flush=True)
                if ok:
                    eng = {"name": resp.get("contact") or c["name"],
                           "start": t}
                    lost_t = None
                time.sleep(1.0)      # let the op dispatch before re-polling
        if t - last_row >= 1.0:
            last_row = t
            print(json.dumps({"t": round(t, 1), "track": tstate,
                              "target": tgt,
                              "gap_m": tr.get("gap_m"),
                              "alt": state.get("alt"),
                              "mode": state.get("mode")}), flush=True)
        time.sleep(0.5)
    if eng is not None:
        print(json.dumps({"event": "lost", "t": round(time.monotonic() - t0, 1),
                          "name": eng["name"],
                          "dur_s": round(time.monotonic() - t0 - eng["start"], 1),
                          "note": "session end"}), flush=True)


if __name__ == "__main__":
    main()
