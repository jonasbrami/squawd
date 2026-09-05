#!/usr/bin/env python3
"""M4 item 3 — SAM mask IoU vs pseudo-GT over the recorded set.

For each iou target in labels.json (hand-picked by eye: frame tag, object,
prompt point + prompt box, gt kind):
  - SAM: POST /v1/segment to the sidecar (:8100 direct, bearer) with the
    POINT prompt and with the BOX prompt -> two masks (box-local RLE decoded
    into the full frame).
  - pseudo-GT: gt_kind "building" -> the repo's own projection of the
    demo_boxes.json footprint into the frame (agents/perception/projection
    intrinsics + the frame sidecar's recorded pose/attitude), convex hull of
    the 8 projected box corners; gt_kind "vehicle" -> the fast lane's OWN
    det mask: the repo OnnxBackend (coco-nano-seg-v2-640.onnx, the shipped
    VISION_MODEL) run offline on the exact recorded frame, det picked by
    cls + nearest to the prompt point.
  - IoU = |intersection| / |union| of full-frame binary masks.

CAVEAT (stated in the doc): pseudo-GT is NOT hand-labeled GT — building
footprints are world-file boxes (roof edges, no facade detail) and vehicle
masks are the fast lane's own output (its errors are shared by both sides).

Run on the HOST with the GPU venv (onnxruntime):
  .venv-train-gpu/bin/python evals/out/deep_m4/run_iou.py
"""
import base64
import json
import math
import os
import sys
import time

sys.path.insert(0, os.getcwd())
from agents.core.contact import Frame           # noqa: E402
from agents.perception.deep_client import DeepClient  # noqa: E402
from agents.perception.projection import HFOV_DEG, CAM_MOUNT_M  # noqa: E402
from agents.vision.types import rle_decode      # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames")
BOXES = "PX4-Autopilot/Tools/simulation/gz/worlds/demo_boxes.json"
MODEL = "models/coco-nano-seg-v2-640.onnx"
TOKEN = open(".deep_token").read().strip()
URL = "http://172.17.0.1:8100"

W, H = 640, 360
FX = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
FY = FX                                     # square pixels (vfov from aspect)


# ---------- building pseudo-GT: footprint projection ----------

def project_mask(meta, b):
    """Project one demo_boxes building (x,y,w,d,h, ENU) into the frame via the
    recorded pose/attitude -> (mask_rows, projected_corner_px). Level-camera
    pinhole + first-order roll/pitch correction (the ray_support_range
    convention inverted)."""
    p = meta["pose"]
    ce, cn = p["e"], p["n"]
    cz = p["alt"] + CAM_MOUNT_M
    hdg = math.radians(p["heading_deg"])
    roll = math.radians(meta["att_deg"]["roll"])
    pitch = math.radians(meta["att_deg"]["pitch"])
    pts = []
    for dx in (-b["w"] / 2, b["w"] / 2):
        for dy in (-b["d"] / 2, b["d"] / 2):
            for z in (0.0, b["h"]):
                de = b["x"] + dx - ce
                dn = b["y"] + dy - cn
                f = de * math.sin(hdg) + dn * math.cos(hdg)
                if f < 0.5:                 # behind the camera plane
                    continue
                r = de * math.cos(hdg) - dn * math.sin(hdg)
                u = z - cz
                ax0 = math.atan2(r, f)
                ay0 = math.atan2(-u, f)     # level-body depression (px angle)
                # image angles under hover roll/pitch (first order):
                ax = ax0
                ay = (ay0 + pitch - ax0 * math.sin(roll)) / max(0.98,
                                                                math.cos(roll))
                pts.append((W / 2 + FX * math.tan(ax),
                            H / 2 + FY * math.tan(ay)))
    if len(pts) < 3:
        return None, []
    hull = _convex_hull(pts)
    return _poly_mask(hull), hull


def _convex_hull(pts):
    """Monotonic chain; pts are (x, y) floats -> hull CCW."""
    pts = sorted(set((round(x, 3), round(y, 3)) for x, y in pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    up = []
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def _poly_mask(poly):
    from PIL import Image, ImageDraw
    im = Image.new("L", (W, H), 0)
    ImageDraw.Draw(im).polygon([(x, y) for x, y in poly], fill=1)
    return list(im.getdata())


# ---------- mask helpers ----------

def wire_mask(payload):
    """Sidecar segment payload -> full-frame flat mask list (box-local RLE at
    xyxy, m3_capture.py draw_mask semantics: tolerate dim mismatch)."""
    x1, y1, x2, y2 = [int(round(v)) for v in payload["xyxy"]]
    m = payload["mask"]
    rows = rle_decode(base64.b64decode(m["rle"]), m["w"], m["h"])
    out = [0] * (W * H)
    bw, bh = m["w"], m["h"]
    for yy in range(y2 - y1):
        sy = min(bh - 1, int(yy * bh / max(1, y2 - y1)))
        for xx in range(x2 - x1):
            sx = min(bw - 1, int(xx * bw / max(1, x2 - x1)))
            if rows[sy][sx] and 0 <= y1 + yy < H and 0 <= x1 + xx < W:
                out[(y1 + yy) * W + (x1 + xx)] = 1
    return out


def det_mask(det):
    """OnnxBackend Detection (box-local rle .mask + .xyxy) -> flat mask. Dims
    use the encoder's own box formula (pipeline.py:46-47)."""
    return wire_mask({"xyxy": det.xyxy,
                      "mask": {"rle": base64.b64encode(det.mask).decode(),
                               "w": max(1, int(det.xyxy[2]) - int(det.xyxy[0])),
                               "h": max(1, int(det.xyxy[3]) - int(det.xyxy[1]))}})


def iou(a, b):
    inter = sum(1 for x, y in zip(a, b) if x and y)
    union = sum(1 for x, y in zip(a, b) if x or y)
    return inter / union if union else 0.0, inter, union


def seg_with_retry(client, frame, *, points=None, box=None, tries=8):
    for _ in range(tries):
        r = client.segment(frame, points=points, box=box)
        if r.status == "OK":
            return r
        if r.status == "BUSY":
            time.sleep(0.4)
            continue
        raise SystemExit(f"segment failed: {r.status} {r.detail}")
    raise SystemExit("segment BUSY x8")


def main():
    from PIL import Image
    targets = json.load(open(os.path.join(FRAMES, "iou_targets.json")))
    buildings = {b["name"]: b for b in json.load(open(BOXES))["buildings"]}
    client = DeepClient(URL, TOKEN)

    backend = None
    rows = []
    for t in targets:
        tag = t["tag"]
        meta = json.load(open(os.path.join(FRAMES, f"{tag}.json")))
        im = Image.open(os.path.join(FRAMES, f"{tag}.png")).convert("RGB")
        frame = Frame(meta["seq"], meta["sim_stamp"], im.width, im.height,
                      im.tobytes())

        # pseudo-GT
        if t["gt_kind"] == "building":
            gt, hull = project_mask(meta, buildings[t["object"]])
            if gt is None:
                print(f"[{tag}/{t['object']}] projection failed"); continue
            gt_desc = f"proj({t['object']} footprint)"
        else:
            if backend is None:
                from agents.vision.backends import OnnxBackend
                backend = OnnxBackend(MODEL, MODEL.replace(".onnx", ".json"))
                backend.load()
            dets = backend.infer(frame, 0.25)
            px, py = t["point"]
            cands = [d for d in dets
                     if d.cls in ("car", "truck", "person")
                     and d.mask is not None
                     and d.xyxy[0] <= px <= d.xyxy[2]
                     and d.xyxy[1] <= py <= d.xyxy[3]]
            if not cands:
                print(f"[{tag}/{t['object']}] no fast det at point "
                      f"(dets={[(d.cls, round(d.conf,2)) for d in dets]})")
                continue
            det = max(cands, key=lambda d: d.conf)
            gt = det_mask(det)
            gt_desc = f"fastlane({det.cls} {det.conf:.2f})"

        res = {"tag": tag, "object": t["object"], "gt": gt_desc}
        for kind, kw in (("point", {"points": [t["point"]]}),
                         ("box", {"box": t["box"]})):
            r = seg_with_retry(client, frame, **kw)
            sm = wire_mask(r.data)
            v, inter, union = iou(sm, gt)
            res[kind] = {"iou": round(v, 3), "score": r.data.get("score"),
                         "area_px": r.data.get("area_px"),
                         "union": union}
            print(f"[{tag}/{t['object']} {kind:5s}] IoU={v:.3f} "
                  f"score={r.data.get('score')} gt={gt_desc}", flush=True)
        rows.append(res)
    with open(os.path.join(HERE, "iou_results.json"), "w") as f:
        json.dump(rows, f, indent=1)
    print("\n| target | gt | point IoU | box IoU |")
    print("|---|---|---|---|")
    for r in rows:
        print(f"| {r['tag']} / {r['object']} | {r['gt']} | "
              f"{r['point']['iou']} | {r['box']['iou']} |")


if __name__ == "__main__":
    main()
