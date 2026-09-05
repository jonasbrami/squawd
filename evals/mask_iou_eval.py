#!/usr/bin/env python3
"""mask_iou_eval — M2.5 mask-quality + latency evaluation (host-side).

Runs the trained OnnxBackend over the val split and reports, per class:
  - mask IoU vs the geometric auto-labels (projection truth);
  - mask IoU vs ColorBlobBackend's independent color mask (target class —
    cross-modal sanity, geometry vs color);
  - box detection rate (IoU>0.5 vs label box);
  - inference latency at 640x360 (gate: <=25 ms/frame CPU).

  uv run --with onnxruntime --with numpy --with pillow \
      python evals/mask_iou_eval.py --data evals/out/dataset_v1 \
      --model models/mover-nano-seg-v1
"""
import argparse
import io
import json
import os
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, ".")
from agents.core.contact import Frame                     # noqa: E402
from agents.vision.backends import ColorBlobBackend, OnnxBackend  # noqa: E402
from agents.vision.types import rle_decode                # noqa: E402

NAMES = {0: "target", 1: "obstacle"}


def load_label(path, w, h):
    """YOLO-seg label file -> [(cls, mask bool[h,w], box xyxy px)]."""
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path):
        parts = line.split()
        if len(parts) < 7:
            continue
        cls = int(parts[0])
        xy = [float(v) for v in parts[1:]]
        pts = [(xy[i] * w, xy[i + 1] * h) for i in range(0, len(xy), 2)]
        img = Image.new("L", (w, h), 0)
        ImageDraw.Draw(img).polygon(pts, fill=1, outline=1)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        out.append((cls, np.asarray(img, dtype=bool),
                    (min(xs), min(ys), max(xs), max(ys))))
    return out


def det_mask(d, w, h):
    if d.mask is None:
        return None
    x1, y1, x2, y2 = (int(round(v)) for v in d.xyxy)
    rows = rle_decode(d.mask, x2 - x1, y2 - y1)
    # rows are uniform-width except possibly the LAST (ragged tail) — build
    # explicitly, np.asarray on ragged lists makes an object array
    sub = np.zeros((len(rows), x2 - x1), dtype=bool)
    for ri, row in enumerate(rows):
        sub[ri, : len(row)] = row
    full = np.zeros((h, w), dtype=bool)
    y1c, y2c = max(0, y1), min(h, y2)
    x1c, x2c = max(0, x1), min(w, x2)
    if y2c <= y1c or x2c <= x1c:
        return full
    sh = min(y2c - y1c, sub.shape[0])
    sw = min(x2c - x1c, sub.shape[1])
    full[y1c:y1c + sh, x1c:x1c + sw] = sub[:sh, :sw]
    return full


def iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return inter / union if union else 0.0


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    bb = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + bb - inter + 1e-9)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="evals/out/dataset_v1")
    ap.add_argument("--model", default="models/mover-nano-seg-v1")
    args = ap.parse_args()

    backend = OnnxBackend(args.model + ".onnx", args.model + ".json")
    blob = ColorBlobBackend()
    img_dir = os.path.join(args.data, "images", "val")
    lbl_dir = os.path.join(args.data, "labels", "val")
    files = sorted(f for f in os.listdir(img_dir) if f.endswith(".png"))
    if not files:
        print("no val images", file=sys.stderr)
        return 1

    stats = {c: {"geo": [], "blob": [], "hit": 0, "n": 0} for c in NAMES}
    lat = []
    for fn in files:
        img = Image.open(os.path.join(img_dir, fn)).convert("RGB")
        w, h = img.size
        frame = Frame(1, 0.0, w, h, img.tobytes())
        t0 = time.monotonic()
        dets = backend.infer(frame, 0.25)
        lat.append((time.monotonic() - t0) * 1000)
        labels = load_label(os.path.join(lbl_dir, fn.replace(".png", ".txt")),
                            w, h)
        blob_dets = blob.infer(frame, 0.25)
        for cls, lmask, lbox in labels:
            st = stats[cls]
            st["n"] += 1
            cand = [d for d in dets
                    if d.cls == NAMES[cls] and box_iou(d.xyxy, lbox) > 0.3]
            if not cand:
                continue
            best = max(cand, key=lambda d: box_iou(d.xyxy, lbox))
            if box_iou(best.xyxy, lbox) > 0.5:
                st["hit"] += 1
            dm = det_mask(best, w, h)
            if dm is not None:
                st["geo"].append(iou(dm, lmask))
            if cls == 0 and blob_dets:
                bm = det_mask(max(blob_dets, key=lambda d: d.conf), w, h)
                if bm is not None and dm is not None:
                    st["blob"].append(iou(dm, bm))

    print(f"val images: {len(files)}")
    for cls, name in NAMES.items():
        st = stats[cls]
        if not st["n"]:
            continue
        geo = st["geo"] or [0.0]
        bl = st["blob"]
        print(f"{name}: labels={st['n']} det_rate(IoU>0.5)="
              f"{st['hit'] / st['n']:.2f} maskIoU_vs_geo={np.mean(geo):.3f}"
              + (f" maskIoU_vs_blob={np.mean(bl):.3f}" if bl else ""))
    lat = sorted(lat)
    print(f"latency ms: p50={lat[len(lat) // 2]:.1f} "
          f"p95={lat[int(0.95 * (len(lat) - 1))]:.1f} max={lat[-1]:.1f} "
          f"(gate <=25)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
