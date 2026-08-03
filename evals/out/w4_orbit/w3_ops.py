"""W3-run3 op poster (host side, LLM-free): resolve the CURRENT track target
from /state and POST one /api/cmd op — the Back-Off (standoff 23), orbit,
stop, resume sequence steps, with the contact name fresh at post time (the
R1 readoption path can churn vis_* ids mid-pursuit).

  python evals/out/w3_run3/w3_ops.py standoff 23
  python evals/out/w3_run3/w3_ops.py orbit 20 8
  python evals/out/w3_run3/w3_ops.py stop
  python evals/out/w3_run3/w3_ops.py resume [name]

Prints the request/response as JSON on stdout (tee into click.log).
"""
import json
import sys
import urllib.request

FRAME_W, FRAME_H = 640, 360


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


def pick_name(state):
    """Current designated target if it is still a live contact; else the
    largest fully-in-frame ranged car contact."""
    contacts = state.get("contacts") or []
    names = [c.get("name") for c in contacts]
    tgt = (state.get("track") or {}).get("target")
    if tgt in names:
        return tgt
    cands = []
    for c in contacts:
        box = c.get("bbox_xyxy")
        if not box or c.get("cls") != "car":
            continue
        x1, y1, x2, y2 = box
        if x1 < 1 or y1 < 1 or x2 > FRAME_W - 1 or y2 > FRAME_H - 1:
            continue
        if c.get("range_m") is None:
            continue
        cands.append(((x2 - x1) * (y2 - y1), c["name"]))
    cands.sort(key=lambda t: -t[0])
    return cands[0][1] if cands else None


def main() -> None:
    op = sys.argv[1]
    url = "http://localhost:8000"
    st, state = get_json(f"{url}/state")
    if st != 200:
        raise SystemExit(f"/state -> {st}: {state}")
    if op == "stop":
        body = {"op": "stop"}
    else:
        name = sys.argv[2] if op == "resume" and len(sys.argv) > 2 \
            else pick_name(state)
        if not name:
            raise SystemExit(json.dumps(
                {"error": "no live contact to address",
                 "track": state.get("track")}))
        if op == "standoff":
            body = {"op": "standoff", "contact": name,
                    "range_m": float(sys.argv[2])}
        elif op == "orbit":
            body = {"op": "orbit", "contact": name,
                    "radius_m": float(sys.argv[2]),
                    "rate_dps": float(sys.argv[3])}
        elif op == "resume":
            body = {"op": "resume", "contact": name}
        else:
            raise SystemExit(f"unknown op {op!r}")
    st, resp = get_json(f"{url}/api/cmd", body)
    print(json.dumps({"post": body, "status": st, "resp": resp,
                      "track_before": state.get("track")}))


if __name__ == "__main__":
    main()
