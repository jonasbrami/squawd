#!/usr/bin/env python3
"""w0_assets_eval — W0.1 detector-on-rendered-assets gate (design 2026-07-28
§2 item 1): run the exported stock COCO yolo11n-seg ONNX DIRECTLY with
onnxruntime (production agents/vision code is NOT imported) over the frames
captured from the `assets` world, and report per-cast-member detection.

Detection eval (host or container):
  uv run --no-project --with onnxruntime --with numpy --with pillow \
      python scripts/w0_assets_eval.py

In-container fps probe (the gate's perf half):
  docker exec w0-assets bash -lc 'cd /workspace && uv run --no-project \
      --with onnxruntime --with numpy --with pillow \
      python scripts/w0_assets_eval.py --bench \
      --model models/coco-nano-seg-v1.onnx --out \
      evals/out/w0_detector_assets/bench416.json'

Preprocess/decode MIRRORS agents/vision/backends.py OnnxBackend (letterbox
114-pad, class-aware NMS iou=0.45, conf 0.25) so numbers are comparable to
the production <=25 ms precedent; mask assembly is reimplemented here for
the fps parity arm. Hit test per cast member: the world generator publishes
exact poses (sim/worlds/make_assets_world.py, imported), so each object's
expected screen box is projected from the camera replica geometry and a
detection counts when its class is in the object's expect list and the boxes
overlap (IoU>0.15 or either center inside the other box).
"""
import argparse
import glob
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sim.worlds.make_assets_world import (OBJECTS, CAMERAS, HOUSE, HFOV,  # noqa: E402
                                          CAM_W, CAM_H)

F_PX = (CAM_W / 2) / math.tan(HFOV / 2)          # 465.4 px/rad
VEHICLE_OR_PERSON = {"person", "bicycle", "car", "motorcycle", "bus", "truck"}
ANNOTATE = ["cam10_low_00.png", "cam25_low_00.png", "cam40_low_00.png",
            "cam10_high_00.png", "cam40_high_00.png", "cam_house_00.png"]


# ---- preprocess/decode (mirror of agents/vision/backends.py, no import) ----

def letterbox(img, size):
    import numpy as np
    w, h = img.size
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    a = np.asarray(img.resize((nw, nh))) if (nw, nh) != (w, h) else np.asarray(img)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = a
    x = canvas.transpose(2, 0, 1)[None].astype("float32") / 255.0
    return x, scale, (px, py)


def nms(boxes, scores, iou=0.45):
    import numpy as np
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_j = (boxes[order[1:], 2] - boxes[order[1:], 0]) * \
                 (boxes[order[1:], 3] - boxes[order[1:], 1])
        order = order[1:][inter / (area_i + area_j - inter + 1e-9) <= iou]
    return keep


def run_net(sess, img, size):
    x, scale, pad = letterbox(img, size)
    outs = sess.run(None, {sess.get_inputs()[0].name: x})
    return outs, scale, pad


def decode(outs, conf, scale, pad, fw, fh, size, with_masks=False):
    """-> [(cls_id, score, xyxy, mask_bool|None)]. Class-aware NMS (0.45)."""
    import numpy as np
    det, protos = outs[0][0], outs[1][0]           # (C, A), (32, mh, mw)
    nc = det.shape[0] - 4 - protos.shape[0]
    boxes_cxcywh = det[:4].T
    cls_scores = det[4:4 + nc].T
    coeffs = det[4 + nc:].T
    cls_id = cls_scores.argmax(1)
    scores = cls_scores.max(1)
    keep = scores >= conf
    if not keep.any():
        return []
    boxes_cxcywh, scores, cls_id, coeffs = (boxes_cxcywh[keep], scores[keep],
                                            cls_id[keep], coeffs[keep])
    x1 = (boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2 - pad[0]) / scale
    y1 = (boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2 - pad[1]) / scale
    x2 = (boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2 - pad[0]) / scale
    y2 = (boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2 - pad[1]) / scale
    boxes = np.stack([x1, y1, x2, y2], 1)
    out = []
    for c in np.unique(cls_id):
        idx = np.where(cls_id == c)[0]
        for i in nms(boxes[idx], scores[idx]):
            j = idx[i]
            mask = assemble_mask(coeffs[j], protos, boxes[j], scale, pad,
                                 size) if with_masks else None
            out.append((int(c), float(scores[j]),
                        tuple(float(v) for v in boxes[j]), mask))
    return out


def assemble_mask(coeff, protos, box, scale, pad, size):
    """Same algorithm as backends._assemble_mask (fps parity arm)."""
    import numpy as np
    mh, mw = protos.shape[1], protos.shape[2]
    kx, ky = mw / size, mh / size
    x1 = int(max(0, (box[0] * scale + pad[0]) * kx))
    y1 = int(max(0, (box[1] * scale + pad[1]) * ky))
    x2 = int(min(mw, (box[2] * scale + pad[0]) * kx) + 1)
    y2 = int(min(mh, (box[3] * scale + pad[1]) * ky) + 1)
    sub = protos[:, y1:y2, x1:x2]
    m = 1.0 / (1.0 + np.exp(-np.tensordot(coeff, sub, axes=(0, 0))))
    from PIL import Image as _I
    bw = max(1, int(box[2]) - int(box[0]))
    bh = max(1, int(box[3]) - int(box[1]))
    return np.asarray(_I.fromarray((m * 255).astype("uint8"))
                      .resize((bw, bh), _I.BILINEAR)) > 127


# ---- geometry: expected screen box per cast member -------------------------

def project(cam_pose, ox, oy, oyaw, length, width, height):
    """Expected screen box (xyxy px) + forward depth f (m). Full 3D:
    the high cameras are pitched down 0.55 rad, which COMPRESSES object
    bearings toward the image center — a pitch-blind horizontal projection
    misplaces expected boxes by up to ~60 px there (found 2026-08-01)."""
    cx, cy, cz, _, pitch, yaw = cam_pose
    dx, dy = ox - cx, oy - cy
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)        # azimuth frame
    lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
    dz = height / 2 - cz                                 # center rel z
    f = fwd * math.cos(pitch) - dz * math.sin(pitch)     # depth, cam frame
    h = fwd * math.sin(pitch) + dz * math.cos(pitch)     # up, cam frame
    # pinhole: pixel offset = F * tan(angle) = F * lat/f — NOT F * angle
    # (F*angle misplaces edge objects ~10% outward, found 2026-08-01)
    u = CAM_W / 2 - F_PX * (lat / f)
    v = CAM_H / 2 - F_PX * (h / f)
    # apparent horizontal extent: silhouette of an L x W footprint seen from
    # azimuth atan2(dy, dx) with the object's forward axis at world yaw oyaw
    theta = oyaw - math.atan2(dy, dx)
    extent = length * abs(math.sin(theta)) + width * abs(math.cos(theta))
    w = F_PX * extent / f
    hh = F_PX * height / f
    return (u - w / 2, v - hh / 2, u + w / 2, v + hh / 2), f


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    bb = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + bb - inter + 1e-9)


def center_in(pt, box):
    return box[0] <= pt[0] <= box[2] and box[1] <= pt[1] <= box[3]


def overlaps(det_box, exp_box):
    if box_iou(det_box, exp_box) > 0.15:
        return True
    ec = ((exp_box[0] + exp_box[2]) / 2, (exp_box[1] + exp_box[3]) / 2)
    dc = ((det_box[0] + det_box[2]) / 2, (det_box[1] + det_box[3]) / 2)
    return center_in(ec, det_box) or center_in(dc, exp_box)


def p50(vals):
    vals = sorted(vals)
    return vals[len(vals) // 2] if vals else None


def house_center():
    return (HOUSE["pose"][0] + HOUSE["center_offset"][0],
            HOUSE["pose"][1] + HOUSE["center_offset"][1])


# ---- detection eval ---------------------------------------------------------

def evaluate(args) -> int:
    import numpy as np
    import onnxruntime as ort
    from PIL import Image, ImageDraw

    manifest = json.load(open(args.model + ".json"))
    classes = manifest["classes"]
    sess = ort.InferenceSession(args.model + ".onnx",
                                providers=["CPUExecutionProvider"])
    size = sess.get_inputs()[0].shape[2]
    cams = {c["name"]: c for c in CAMERAS}
    by_row = {}
    for o in OBJECTS:
        by_row.setdefault(o["range"], []).append(o)

    frames = sorted(glob.glob(os.path.join(args.frames_dir, "*.png")))
    if not frames:
        print(f"no frames in {args.frames_dir}", file=sys.stderr)
        return 1

    # obj stats keyed (obj_name, height); house keyed separately
    stats, house = {}, {"frames": 0, "labels": {}, "bad": 0}
    brightness, frame_dets = {}, {}
    for fp in frames:
        fn = os.path.basename(fp)
        cam_name = fn.rsplit("_", 1)[0]
        cam = cams.get(cam_name)
        if cam is None:
            continue
        img = Image.open(fp).convert("RGB")
        arr = np.asarray(img)
        brightness.setdefault(cam_name, []).append(float(arr.mean()))
        outs, scale, pad = run_net(sess, img, size)
        dets = decode(outs, args.conf, scale, pad, img.width, img.height, size)
        frame_dets[fn] = [(classes[c], round(s, 3), [round(v, 1) for v in b])
                          for c, s, b, _ in dets]

        if cam_name == "cam_house":
            house["frames"] += 1
            hx, hy = house_center()
            exp, _ = project(cam["pose"], hx, hy, HOUSE["yaw"],
                             HOUSE["dims"][0], HOUSE["dims"][1],
                             HOUSE["dims"][2])
            for c, s, b, _ in dets:
                if overlaps(b, exp):
                    lbl = classes[c]
                    house["labels"][lbl] = house["labels"].get(lbl, 0) + 1
                    if lbl in VEHICLE_OR_PERSON:
                        house["bad"] += 1
            continue

        for o in by_row.get(cam["row"], []):
            height = cam_name.rsplit("_", 1)[1]    # low | high
            key = f"{o['name']}|{height}"
            st = stats.setdefault(key, {
                "key": o["key"], "range": o["range"], "height": height,
                "expect": o["expect"], "frames": 0, "hits": 0, "confs": [],
                "w_ratios": [], "labels": {}, "expected_w_px": None,
                "expected_box": None, "dist_m": None})
            st["frames"] += 1
            exp, dist = project(cam["pose"], o["pose"][0], o["pose"][1],
                                o["pose"][5], o["length"], o["width"],
                                o["height"])
            st["expected_box"] = [round(v, 1) for v in exp]
            st["expected_w_px"] = round(exp[2] - exp[0], 1)
            st["dist_m"] = round(dist, 1)
            best, best_iou = None, 0.0
            for c, s, b, _ in dets:
                if overlaps(b, exp):
                    iou = box_iou(b, exp)
                    if iou > best_iou or best is None:
                        best, best_iou = (c, s, b), max(iou, best_iou)
            if best is None:
                st["labels"]["<none>"] = st["labels"].get("<none>", 0) + 1
                continue
            c, s, b = best
            lbl = classes[c]
            st["labels"][lbl] = st["labels"].get(lbl, 0) + 1
            if lbl in o["expect"]:
                st["hits"] += 1
                st["confs"].append(s)
                st["w_ratios"].append((b[2] - b[0]) / (exp[2] - exp[0]))

    objects_out = {}
    for key, st in sorted(stats.items()):
        objects_out[key] = {
            "key": st["key"], "range": st["range"], "cam_height": st["height"],
            "expect": st["expect"], "frames": st["frames"], "hits": st["hits"],
            "det_rate": round(st["hits"] / st["frames"], 3),
            "conf_p50": (None if not st["confs"]
                         else round(float(np.median(st["confs"])), 3)),
            "box_w_ratio_p50": (None if not st["w_ratios"] else round(
                float(np.median(st["w_ratios"])), 3)),
            "labels": st["labels"], "expected_w_px": st["expected_w_px"],
            "expected_box_xyxy": st["expected_box"], "dist_m": st["dist_m"],
        }

    report = {
        "gate": "W0.1 detector-on-rendered-assets",
        "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "model": {"path": args.model + ".onnx", "sha256": manifest["sha256"],
                  "imgsz": size, "source": manifest["source"]},
        "conf": args.conf, "nms_iou": 0.45, "frames": len(frames),
        "frame_brightness_mean": {c: round(float(np.mean(v)), 1)
                                  for c, v in sorted(brightness.items())},
        "objects": objects_out,
        "house_negative": house,
        "detections_by_frame": frame_dets,
    }
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump(report, f, indent=1)
    print("report:", args.report)

    # ~6 annotated frames: expected box in white, detections labeled
    ann_dir = os.path.join(os.path.dirname(args.report), "annotated")
    os.makedirs(ann_dir, exist_ok=True)
    for fn in ANNOTATE:
        fp = os.path.join(args.frames_dir, fn)
        if not os.path.exists(fp):
            continue
        cam = cams[fn.rsplit("_", 1)[0]]
        img = Image.open(fp).convert("RGB")
        dr = ImageDraw.Draw(img)
        rows = ([HOUSE] if fn.startswith("cam_house")
                else by_row.get(cam["row"], []))
        for o in rows:
            if fn.startswith("cam_house"):
                hx, hy = house_center()
                exp, _ = project(cam["pose"], hx, hy, o["yaw"],
                                 o["dims"][0], o["dims"][1], o["dims"][2])
                tag = "EXP house"
            else:
                exp, _ = project(cam["pose"], o["pose"][0], o["pose"][1],
                                 o["pose"][5], o["length"], o["width"],
                                 o["height"])
                tag = f"EXP {o['key']}"
            dr.rectangle(exp, outline=(255, 255, 255), width=1)
            dr.text((exp[0] + 2, exp[1] + 1), tag, fill=(255, 255, 255))
        for lbl, s, b in frame_dets.get(fn, []):
            dr.rectangle(b, outline=(255, 220, 0), width=2)
            dr.text((b[0] + 2, max(0, b[1] - 9)), f"{lbl} {s:.2f}",
                    fill=(255, 220, 0))
        img.save(os.path.join(ann_dir, fn))
    print("annotated:", ann_dir)
    return 0


# ---- fps bench ---------------------------------------------------------------

def bench(args) -> int:
    import numpy as np
    import onnxruntime as ort
    from PIL import Image

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    size = sess.get_inputs()[0].shape[2]
    frames = sorted(glob.glob(os.path.join(args.frames_dir, "*.png")))
    imgs = [Image.open(fp).convert("RGB") for fp in frames[:12]]
    if not imgs:                                 # container bench before capture
        imgs = [Image.fromarray(
            (np.random.rand(CAM_H, CAM_W, 3) * 255).astype("uint8"))]

    def tick(with_masks):
        img = imgs[tick.i % len(imgs)]
        tick.i += 1
        t0 = time.perf_counter()
        outs, scale, pad = run_net(sess, img, size)
        t1 = time.perf_counter()
        decode(outs, args.conf, scale, pad, img.width, img.height, size,
               with_masks=with_masks)
        t2 = time.perf_counter()
        return (t1 - t0) * 1000, (t2 - t0) * 1000

    tick.i = 0
    for _ in range(8):                           # warmup (ORT arena, caches)
        tick(False)
    raw, pipe, masks = [], [], []
    for i in range(args.iters):
        r, p = tick(i % 3 == 0)                  # every 3rd: +mask assembly
        raw.append(r)
        pipe.append(p)
        if i % 3 == 0:
            masks.append(p)

    def summ(v):
        v = sorted(v)
        return {"mean": round(sum(v) / len(v), 2),
                "p50": round(v[len(v) // 2], 2),
                "p95": round(v[int(0.95 * (len(v) - 1))], 2),
                "max": round(v[-1], 2)}

    out = {"model": args.model, "imgsz": size, "iters": args.iters,
           "nproc": os.cpu_count(), "ort_version": ort.__version__,
           "raw_infer_ms": summ(raw),
           "pipeline_ms": summ(pipe),
           "pipeline_with_masks_ms": summ(masks),
           "note": ("raw=letterbox+sess.run; pipeline=raw+box decode+NMS; "
                    "with_masks adds per-det mask assembly (production "
                    "OnnxBackend.infer parity)")}
    print(json.dumps(out, indent=1))
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(out, f, indent=1)
        print("wrote", args.out)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--model", default="models/coco-nano-seg-v1",
                    help="bench: ONNX path; eval: path WITHOUT extension")
    ap.add_argument("--frames-dir", default="evals/out/w0_detector_assets/frames")
    ap.add_argument("--report", default="evals/out/w0_detector_assets/report.json")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iters", type=int, default=120)
    ap.add_argument("--out", default=None, help="bench: write json here")
    args = ap.parse_args()
    if args.bench:
        return bench(args)
    return evaluate(args)


if __name__ == "__main__":
    sys.exit(main())
