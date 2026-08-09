"""Fine pursuit reproduction: wait for in-band vehicle, click-lock, then
sample /state at 5 Hz — centering error (heading vs absolute bearing),
bbox offset from frame center, elevation, gap, track state.
Writes repro_timeline.jsonl for analysis.
"""
import json
import time
import urllib.request

BASE = "http://localhost:8000"
OUT = "evals/out/show_2026-08-03/repro_timeline.jsonl"
WAIT_S, WATCH_S, MAX_RANGE = 300, 75, 32.0


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.load(r)


def post(path, payload):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def wrap180(a):
    return (a + 180) % 360 - 180


def row(t, phase, d, locked):
    tgt = next((c for c in d["contacts"] if c["name"] == locked), None) \
        if locked else None
    r = {"t": round(t, 2), "phase": phase, "mode": d["mode"],
         "alt": round(d["alt"], 1), "heading": round(d["heading"], 1),
         "track": d["track"]["state"], "target": d["track"].get("target"),
         "gap": d["track"].get("gap_m")}
    if tgt:
        bb = tgt.get("bbox_xyxy")
        r.update({
            "bearing": round(tgt.get("bearing_deg") or 0, 1),
            "elev": round(tgt.get("elevation_deg") or 0, 1),
            "cerr": round(wrap180(d["heading"] - (tgt.get("bearing_deg") or 0)), 1),
            "bb_cx_off": (round((bb[0] + bb[2]) / 2 - 320, 0) if bb else None),
            "bb_bot": (round(bb[3], 0) if bb else None),
            "health": tgt.get("health"), "age": tgt.get("age_s"),
            "range": round(tgt.get("range_m") or 0, 1)})
    return r


T0 = time.monotonic()
locked = None
with open(OUT, "a") as fh:
    while time.monotonic() - T0 < WAIT_S and locked is None:
        d = get("/state")
        fh.write(json.dumps(row(time.monotonic() - T0, "wait", d, None)) + "\n")
        for c in d["contacts"]:
            rng = c.get("range_m")
            if c["cls"] in ("car", "truck", "bus") and rng and rng <= MAX_RANGE \
                    and c.get("bbox_xyxy"):
                x1, y1, x2, y2 = c["bbox_xyxy"]
                st, resp = post("/api/lock",
                                {"x": (x1 + x2) / 2, "y": (y1 + y2) / 2})
                print("CLICK", c["name"], round(rng, 1), "->", st, resp, flush=True)
                if st == 200:
                    locked = resp.get("contact")
                break
        time.sleep(0.5)
    if locked:
        t1 = time.monotonic()
        while time.monotonic() - t1 < WATCH_S:
            d = get("/state")
            r = row(time.monotonic() - T0, "pursuit", d, locked)
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            print(r, flush=True)
            time.sleep(0.2)
    else:
        print("NO in-band vehicle", flush=True)
