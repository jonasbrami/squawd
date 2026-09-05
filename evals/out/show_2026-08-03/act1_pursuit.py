"""Show Act 1: wait for an in-band vehicle contact, click-lock it, watch.
Host-side: polls http://localhost:8000/state, POSTs /api/lock on bbox center,
then records a pursuit timeline to timeline_act1.jsonl.
"""
import json
import time
import urllib.request

BASE = "http://localhost:8000"
OUT = "evals/out/show_2026-08-03/timeline_act1.jsonl"
WAIT_S, WATCH_S, MAX_RANGE = 75, 45, 25.0


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


def log(fh, phase, d):
    row = {"t": round(time.monotonic() - T0, 1), "phase": phase,
           "alt": d["alt"], "mode": d["mode"],
           "track": d["track"], "beam": d["beam"]["status"],
           "contacts": [c["name"] for c in d["contacts"]],
           "annotations": [a["cls"] for a in d.get("annotations", [])]}
    fh.write(json.dumps(row) + "\n")
    fh.flush()
    print(row["t"], phase, row["mode"], row["track"]["state"],
          row["track"].get("target"), row["annotations"], flush=True)


T0 = time.monotonic()
locked = None
with open(OUT, "a") as fh:
    while time.monotonic() - T0 < WAIT_S and locked is None:
        d = get("/state")
        log(fh, "wait", d)
        for c in d["contacts"]:
            r = c.get("range_m")
            if c["cls"] in ("car", "truck", "bus") and r and r <= MAX_RANGE \
                    and c.get("bbox_xyxy"):
                x1, y1, x2, y2 = c["bbox_xyxy"]
                st, resp = post("/api/lock",
                                {"x": (x1 + x2) / 2, "y": (y1 + y2) / 2})
                print("CLICK", c["name"], round(r, 1), "->", st, resp,
                      flush=True)
                if st == 200:
                    locked = c["name"]
                break
        time.sleep(1.5)
    if locked:
        t1 = time.monotonic()
        while time.monotonic() - t1 < WATCH_S:
            d = get("/state")
            log(fh, "pursuit", d)
            time.sleep(2)
    else:
        print("NO in-band vehicle within", WAIT_S, "s", flush=True)
