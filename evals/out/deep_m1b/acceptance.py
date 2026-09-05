"""M1b acceptance harness (HOST side, plain project venv).

Drives the live deep sidecar (172.17.0.1:8100, bearer from .deep_token) over
the three captured sim frames and records EVERYTHING to results.json for
docs/benchmarks/deep-perception-m1.md:

  a. vocabulary detect on each frame (building,house,tree,pole,car,truck,person)
  b. color-order proof: RGB (wire contract) vs channel-swapped copy
  c. SAM point-prompt (car centroid) + box-prompt (house), mask rle_decode
     cross-check (area_px == decoded True count, box-local dims)
  d. latency: cold (model load, extended-timeout client) vs warm p50/p95
  e. coexistence: cockpit /state detector latency + cam cadence while
     hammering the sidecar with 20 sequential detects
  f. live 429: two concurrent detects -> one BUSY

Frames are the UNTOUCHED gz RGB888 bytes (PNG is lossless), seq/sim_stamp
from the capture sidecars — the exact Frame wire contract.

Run:  uv run --no-sync python evals/out/deep_m1b/acceptance.py
"""
import base64
import json
import math
import os
import threading
import time
import urllib.request

import numpy as np
from PIL import Image

from agents.core.contact import Frame
from agents.perception.deep_client import DeepClient
from agents.vision.types import rle_decode

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
GW = "http://172.17.0.1:8100"
VOCAB = ["building", "house", "tree", "pole", "car", "truck", "person"]

FRAMES = ["frame_a_car_houses", "frame_b_houses_pole_car", "frame_c_gasstation"]
# prompt geometry picked by eye from the captures (640x360)
POINT_CAR = (326, 288)            # red hatchback centroid, frame A
BOX_HOUSE = [390, 216, 506, 302]  # gabled house, frame A
BOX_CANOPY = [179, 256, 512, 360]  # gas-station canopy, frame C


def load_frame(tag):
    meta = json.load(open(os.path.join(HERE, tag + ".json")))
    rgb = Image.open(os.path.join(HERE, tag + ".png")).convert("RGB").tobytes()
    return Frame(meta["seq"], meta["sim_stamp"], meta["w"], meta["h"], rgb)


def pct(xs, q):
    xs = sorted(xs)
    i = min(len(xs) - 1, max(0, int(math.ceil(q / 100 * len(xs))) - 1))
    return xs[i]


def state():
    with urllib.request.urlopen("http://127.0.0.1:8000/state",
                                timeout=2) as r:
        return json.loads(r.read())


def main():
    token = open(os.path.join(REPO, ".deep_token")).read().strip()
    client = DeepClient(GW, token=token)
    cold = DeepClient(GW, token=token,
                      read_timeout_detect=120, read_timeout_segment=120)
    out = {"frames": {}, "color_order": {}, "segment": {}, "latency": {},
           "coexistence": {}, "busy_429": {}}
    frames = {t: load_frame(t) for t in FRAMES}

    # -- cold loads (first calls pay torch/ultralytics import + weights) --
    t0 = time.monotonic()
    r = cold.detect(frames[FRAMES[0]], VOCAB, conf=0.25)
    out["latency"]["detect_cold_s"] = round(time.monotonic() - t0, 2)
    assert r.ok, r
    t0 = time.monotonic()
    r = cold.segment(frames[FRAMES[0]], points=[POINT_CAR])
    out["latency"]["segment_cold_s"] = round(time.monotonic() - t0, 2)
    assert r.ok, r
    out["health_loaded"] = client.health().data

    # -- a. vocabulary detect on each frame (conf 0.05; frame A also at the
    #       plan's 0.25 default to document the synthetic-frame threshold) --
    for tag, f in frames.items():
        r = client.detect(f, VOCAB, conf=0.05)
        assert r.ok, r
        out["frames"][tag] = {"conf": 0.05, "dets": r.data["dets"],
                              "latency_ms": r.data["latency_ms"]}
        print(f"[detect] {tag} @0.05: " + (", ".join(
            f"{d['cls']} {d['conf']:.2f} {[round(v) for v in d['xyxy']]}"
            for d in r.data["dets"]) or "(none)"), flush=True)
    r = client.detect(frames[FRAMES[0]], VOCAB, conf=0.25)
    out["frames"][FRAMES[0]]["dets_conf025"] = r.data["dets"]
    print(f"[detect] {FRAMES[0]} @0.25: {len(r.data['dets'])} dets", flush=True)

    # -- b. color-order proof on frame A: a color-discriminating vocabulary
    #       makes the channel order observable — the REAL red hatchback must
    #       answer 'red car' only when the wire order is honored --
    f = frames[FRAMES[0]]
    a = np.frombuffer(f.rgb, dtype=np.uint8).reshape(f.height, f.width, 3)
    swapped = Frame(f.seq, f.sim_stamp, f.width, f.height,
                    a[:, :, ::-1].copy().tobytes())      # R<->B swapped
    r_rgb = client.detect(f, ["red car", "blue car"], conf=0.01)
    r_bgr = client.detect(swapped, ["red car", "blue car"], conf=0.01)
    out["color_order"] = {
        "vocab": ["red car", "blue car"],
        "rgb_dets": [(d["cls"], d["conf"], d["xyxy"])
                     for d in r_rgb.data["dets"]],
        "swapped_dets": [(d["cls"], d["conf"], d["xyxy"])
                         for d in r_bgr.data["dets"]]}
    print("[color] correct-order:", out["color_order"]["rgb_dets"], flush=True)
    print("[color] swapped:", out["color_order"]["swapped_dets"], flush=True)

    # -- c. SAM point + box prompts --
    for name, kw, f in (("point_car", {"points": [POINT_CAR]},
                         frames[FRAMES[0]]),
                        ("box_house", {"box": BOX_HOUSE}, frames[FRAMES[0]]),
                        ("box_canopy", {"box": BOX_CANOPY},
                         frames[FRAMES[2]])):
        r = client.segment(f, **kw)
        assert r.ok, r
        d = r.data
        entry = {k: d[k] for k in ("xyxy", "centroid", "area_px", "score",
                                   "latency_ms")}
        if d["mask"] is not None:
            rows = rle_decode(base64.b64decode(d["mask"]["rle"]),
                              d["mask"]["w"], d["mask"]["h"])
            decoded_area = sum(v for row in rows for v in row)
            entry["mask_dims"] = (d["mask"]["w"], d["mask"]["h"])
            entry["decoded_area_matches"] = bool(decoded_area == d["area_px"])
        out["segment"][name] = entry
        print(f"[segment] {name}: {entry}", flush=True)

    # -- d. warm latency (one unmeasured warm-up first: the color-order runs
    #       leave a different vocabulary cached, and the first loop call would
    #       otherwise fold the set_classes switch into p95) --
    client.detect(frames[FRAMES[0]], VOCAB, conf=0.25)
    client.segment(frames[FRAMES[0]], points=[POINT_CAR])
    det_lat, seg_lat = [], []
    for _ in range(10):
        det_lat.append(client.detect(frames[FRAMES[0]], VOCAB,
                                     conf=0.25).data["latency_ms"])
        seg_lat.append(client.segment(frames[FRAMES[0]],
                                      points=[POINT_CAR]).data["latency_ms"])
    out["latency"]["detect_warm_ms"] = {
        "p50": round(pct(det_lat, 50), 1), "p95": round(pct(det_lat, 95), 1),
        "all": [round(x, 1) for x in det_lat]}
    out["latency"]["segment_warm_ms"] = {
        "p50": round(pct(seg_lat, 50), 1), "p95": round(pct(seg_lat, 95), 1),
        "all": [round(x, 1) for x in seg_lat]}
    print("[latency]", json.dumps(out["latency"], indent=1), flush=True)

    # -- e. coexistence: cockpit fast lane while hammering the sidecar --
    def sample(tag):
        s0 = state()
        time.sleep(2.0)
        s1 = state()
        out["coexistence"][tag] = {
            "detector_latency_ms": [s0["detector"]["latency_ms"],
                                    s1["detector"]["latency_ms"]],
            "cam_hz": round((s1["cam_seq"] - s0["cam_seq"]) / 2.0, 1)}
    sample("before")
    stop = threading.Event()

    def hammer():
        while not stop.is_set():
            client.detect(frames[FRAMES[0]], VOCAB, conf=0.25)
    t = threading.Thread(target=hammer)
    t.start()
    sample("during_20_detects")
    stop.set()
    t.join()
    print("[coexist]", json.dumps(out["coexistence"]), flush=True)

    # -- f. live 429: two concurrent detects --
    barrier = threading.Barrier(2)
    results = []

    def one():
        barrier.wait()
        results.append(client.detect(frames[FRAMES[1]], VOCAB, conf=0.25,
                                     ).status)
    ts = [threading.Thread(target=one) for _ in range(2)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    out["busy_429"] = {"statuses": sorted(results)}
    print("[429] two concurrent detects ->", sorted(results), flush=True)

    out["health_final"] = client.health().data
    with open(os.path.join(HERE, "results.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote results.json", flush=True)


if __name__ == "__main__":
    main()
