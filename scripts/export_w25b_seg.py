#!/usr/bin/env python3
"""export_w25b_seg — W2.5b phase 2 artifact: fine-tuned 80-class
yolo11n-seg -> ONNX @640 + manifest (mirrors scripts/export_coco_seg.py).

  .venv-train-gpu/bin/python scripts/export_w25b_seg.py \
      --best runs/segment/w25b_s2/weights/best.pt

Artifacts (models/ weights are git-ignored, manifest committed per
models/README.md):
  models/coco-nano-seg-v2-640.onnx / .json

The manifest keeps the v1 keys (sha256/source/trained_at/output/classes)
and adds "finetuned_from": "yolo11n-seg.pt" (task requirement). Existing
mover/coco artifacts are not touched.
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import sys

IMGSZ = 640
OUT = "models/coco-nano-seg-v2-640"


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--best", default="runs/segment/w25b_s2/weights/best.pt")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    if not os.path.exists(args.best):
        print(f"missing weights: {args.best}", file=sys.stderr)
        return 1
    if os.path.exists(args.out + ".onnx"):
        print(f"refusing to overwrite {args.out}.onnx", file=sys.stderr)
        return 1

    from ultralytics import YOLO                    # .venv-train-gpu only

    model = YOLO(args.best)
    classes = [model.names[i] for i in sorted(model.names)]
    if len(classes) != 80:
        print(f"expected 80 COCO classes, got {len(classes)}", file=sys.stderr)
        return 1

    exported = model.export(format="onnx", imgsz=IMGSZ, simplify=True)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    shutil.move(exported, args.out + ".onnx")
    print("exported:", args.out + ".onnx", flush=True)

    manifest = {
        "sha256": sha256(args.out + ".onnx"),
        "source": ("yolo11n-seg W2.5b fine-tune on evals/out/w25b_dataset "
                   "(80-class COCO head kept; 3,487 gz + 5,000 COCO replay "
                   "train, gz-only val/test; codex R5 config: 8 ep freeze=10 "
                   "+ 22 ep freeze=5 lr0=2e-4, imgsz=640, batch=16, cos_lr, "
                   "patience=8, seed=7)"),
        "finetuned_from": "yolo11n-seg.pt",
        "trained_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "output": {"layout": "seg-v1", "nms": "external"},
        "classes": classes,
    }
    with open(args.out + ".json", "w") as f:
        json.dump(manifest, f, indent=2)
    print("manifest:", args.out + ".json", flush=True)
    print("sha256:", manifest["sha256"], flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
