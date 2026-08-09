#!/usr/bin/env python3
"""M4 demo-probe driver (HOST side): send one chat command to the pilot and
collect the campaign evidence set for `minutes`:
  - chat transcript (GET /chat?since=n deltas -> <label>_transcript.json)
  - /state timeline at 2 s cadence (pos/mode/track/contacts/annotations ->
    <label>_timeline.jsonl)
  - the tap proxy log is captured separately (it logs every sidecar call).

Usage: python3 evals/out/deep_m4/probe_drive.py <label> <minutes> <text...>
"""
import json
import sys
import time
import urllib.request

OUT = "evals/out/deep_m4"
URL = "http://localhost:8000"


def get(path, timeout=5):
    with urllib.request.urlopen(URL + path, timeout=timeout) as r:
        return json.load(r)


def post(path, body, timeout=5):
    req = urllib.request.Request(
        URL + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def main():
    label, minutes = sys.argv[1], float(sys.argv[2])
    text = " ".join(sys.argv[3:])
    off = get("/chat?since=0")["next"]
    print(post("/command", {"text": text}), flush=True)
    lines, timeline = [], []
    t0 = time.monotonic()
    last = 0.0
    while time.monotonic() - t0 < minutes * 60:
        t = time.monotonic() - t0
        try:
            d = get(f"/chat?since={off}")
            off = d["next"]
            for ln in d["lines"]:
                lines.append({"t": round(t, 1), "line": ln})
                print(f"[{t:7.1f}] {ln}", flush=True)
            if t - last >= 2.0:
                last = t
                st = get("/state")
                timeline.append({
                    "t": round(t, 1),
                    "e": st.get("east"), "n": st.get("north"),
                    "alt": st.get("alt"), "mode": st.get("mode"),
                    "armed": st.get("armed"), "yaw": st.get("yaw"),
                    "track": st.get("track"),
                    "contacts": [(c.get("name"), c.get("cls"),
                                  c.get("range_m"))
                                 for c in st.get("contacts") or []],
                    "annotations": [(a.get("cls"), a.get("conf"))
                                    for a in st.get("annotations") or []],
                })
        except Exception as e:
            print(f"[{t:7.1f}] poll error: {e}", flush=True)
        time.sleep(0.5)
    with open(f"{OUT}/{label}_transcript.json", "w") as f:
        json.dump(lines, f, indent=1)
    with open(f"{OUT}/{label}_timeline.jsonl", "w") as f:
        for row in timeline:
            f.write(json.dumps(row) + "\n")
    print(f"collected {len(lines)} chat lines, {len(timeline)} state rows",
          flush=True)


if __name__ == "__main__":
    main()
