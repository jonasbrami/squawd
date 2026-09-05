"""W3-integration headless click driver (host side, LLM-free).

The browser click, minus the browser: poll the cockpit /state until a
contact's bbox_xyxy sits fully inside the 640x360 frame, then POST
/api/lock with the box CENTER (exactly what the canvas click sends). On
409 (stale|ambiguous|miss) retry with the next sample, up to --tries over
--window seconds. Prints the winning request/response as JSON on stdout.

  python evals/out/w3_integration/w3_click.py [--cls car] [--tries 10] \
      [--window 60] [--url http://localhost:8000]
"""
import argparse
import json
import time
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


def pick(contacts, cls, min_h=0, max_range=None, edge=1, cx=None):
    """Best clickable contact: bbox fully inside the frame by `edge` px of
    margin (a box clipped by the frame edge flickers and the EKF drops it —
    the W3 fast-LOST), preferred class, largest area first (nearest == most
    stable box). min_h floors the box height (px) and max_range the
    contact's EKF range (slant m) — the proven engagement envelope is
    <=40 m slant (design §8)."""
    cands = []
    for c in contacts or []:
        box = c.get("bbox_xyxy")
        if not box or len(box) != 4:
            continue
        x1, y1, x2, y2 = box
        if x1 < edge or y1 < edge or x2 > FRAME_W - edge or \
                y2 > FRAME_H - edge:
            continue
        if y2 - y1 < min_h:
            continue
        rng = c.get("range_m")
        if max_range is not None and (rng is None or rng > max_range):
            continue            # None range == bearing-only (far/shallow): skip
        if cx is not None and not (cx[0] <= (x1 + x2) / 2.0 <= cx[1]):
            continue            # off-boresight: the pursuit's yaw slew loses it
        if cls and c.get("cls") != cls:
            continue
        cands.append(((x2 - x1) * (y2 - y1), c))
    cands.sort(key=lambda t: -t[0])
    return cands[0][1] if cands else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cls", default="car")
    ap.add_argument("--min-h", type=float, default=0)
    ap.add_argument("--max-range", type=float, default=None)
    ap.add_argument("--edge", type=float, default=1)
    ap.add_argument("--cx", default=None,
                    help="lo:hi gate on box-center x (boresight gate)")
    ap.add_argument("--then-orbit", action="store_true",
                    help="after a 200 lock, POST /api/cmd orbit r=15 w=15")
    ap.add_argument("--then-standoff", type=float, default=None,
                    metavar="RANGE_M",
                    help="after a 200 lock, POST /api/cmd standoff at this "
                         "range (the demo ops-bar Approach flow)")
    ap.add_argument("--tries", type=int, default=10)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--window", type=float, default=60.0)
    ap.add_argument("--url", default="http://localhost:8000")
    args = ap.parse_args()

    t0 = time.monotonic()
    attempt = 0
    while attempt < args.tries and time.monotonic() - t0 < args.window:
        attempt += 1
        st, state = get_json(f"{args.url}/state")
        if st != 200:
            print(f"[click {attempt}] /state -> {st}: {state}", flush=True)
            time.sleep(2.0)
            continue
        cx = None
        if args.cx:
            cx = tuple(float(v) for v in args.cx.split(":"))
        c = pick(state.get("contacts"), args.cls, args.min_h,
                 args.max_range, args.edge, cx) or \
            pick(state.get("contacts"), None, args.min_h, args.max_range,
                 args.edge, cx)
        if c is None:
            n = len(state.get("contacts") or [])
            print(f"[click {attempt}] no clickable contact "
                  f"(contacts={n}) — waiting", flush=True)
            time.sleep(args.period)    # the close transit lasts ~10-14 s/lap
            continue
        x1, y1, x2, y2 = c["bbox_xyxy"]
        x, y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        st, resp = get_json(f"{args.url}/api/lock", {"x": x, "y": y})
        print(f"[click {attempt}] {c['name']} box={[round(v,1) for v in c['bbox_xyxy']]} "
              f"center=({x:.1f},{y:.1f}) -> {st}: {resp}", flush=True)
        if st == 200 and resp.get("ok"):
            locked = resp.get("contact")
            print(json.dumps({"locked": locked, "x": x, "y": y,
                              "attempt": attempt,
                              "elapsed_s": round(time.monotonic() - t0, 1)}))
            if args.then_orbit:
                time.sleep(0.8)
                st2, state2 = get_json(f"{args.url}/state")
                names = [cc.get("name") for cc in
                         (state2.get("contacts") or [])] if st2 == 200 else []
                name = locked if locked in names else None
                if name is None:
                    c2 = pick(state2.get("contacts") if st2 == 200 else [],
                              args.cls) or pick(
                        state2.get("contacts") if st2 == 200 else [], None)
                    name = c2["name"] if c2 else None
                if name:
                    body = {"op": "orbit", "contact": name,
                            "radius_m": 15, "rate_dps": 15}
                    st3, resp3 = get_json(f"{args.url}/api/cmd", body)
                    print(json.dumps({"orbit": name,
                                      "status": st3, "resp": resp3}))
                else:
                    print(json.dumps({"orbit": None,
                                      "error": "no contact to orbit"}))
            if args.then_standoff:
                # the demo ops-bar flow: click (lock/shadow) then Approach —
                # the radial stand-off holds a ring the vfov floor can't
                # kill (the plain shadow closes to 10 m and the car drops
                # below the ±21° vfov at >=5 m alt — observed live).
                time.sleep(1.0)
                st2, state2 = get_json(f"{args.url}/state")
                names = [cc.get("name") for cc in
                         (state2.get("contacts") or [])] if st2 == 200 else []
                name = locked if locked in names else None
                if name is None:
                    c2 = pick(state2.get("contacts") if st2 == 200 else [],
                              args.cls) or pick(
                        state2.get("contacts") if st2 == 200 else [], None)
                    name = c2["name"] if c2 else None
                if name:
                    body = {"op": "standoff", "contact": name,
                            "range_m": args.then_standoff}
                    st3, resp3 = get_json(f"{args.url}/api/cmd", body)
                    print(json.dumps({"standoff": name,
                                      "status": st3, "resp": resp3}))
                else:
                    print(json.dumps({"standoff": None,
                                      "error": "no contact to hold"}))
            return
        time.sleep(4.0)               # 409 stale/ambiguous/miss: next sample
    raise SystemExit(f"no lock after {attempt} attempts / "
                     f"{time.monotonic() - t0:.0f}s")


if __name__ == "__main__":
    main()
