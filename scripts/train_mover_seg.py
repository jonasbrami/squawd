#!/usr/bin/env python3
"""train_mover_seg — M2.5 training + export pipeline.

Fine-tunes yolo11n-seg on the auto-labeled mover dataset (scripts/
vision_dataset.py), exports ONNX + the SHA-256 manifest that OnnxBackend
verifies at load (ICD §6.2, layout "seg-v1", nms "external").

  .venv-train/bin/python scripts/train_mover_seg.py \
      --data evals/out/dataset_v1 --base ~/perception-lab/yolo11n-seg.pt \
      --out models/mover-nano-seg-v1

Artifacts (models/, weights git-ignored, manifest committed per models/README):
  <out>.onnx   the exported model
  <out>.json   {"sha256", "source", "trained_at", "output": {...}}
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import sys

# training config (the M2.5 "training config" deliverable — explicit, no magic)
TRAIN = dict(
    epochs=50,            # nano-seg on ~1.2k near-single-object frames
    patience=10,          # early stop on val
    imgsz=640,
    batch=16,
    device="cpu",
    workers=8,
    freeze=10,            # backbone frozen: 2-class gray-world boxes need the head
    seed=7,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="evals/out/dataset_v1")
    ap.add_argument("--base", default=os.path.expanduser(
        "~/perception-lab/yolo11n-seg.pt"))
    ap.add_argument("--out", default="models/mover-nano-seg-v1")
    ap.add_argument("--epochs", type=int, default=TRAIN["epochs"])
    args = ap.parse_args()

    yaml_path = os.path.join(args.data, "dataset.yaml")
    for p, what in ((yaml_path, "dataset.yaml"), (args.base, "base weights")):
        if not os.path.exists(p):
            print(f"missing {what}: {p}", file=sys.stderr)
            return 1

    from ultralytics import YOLO                    # .venv-train only

    cfg = dict(TRAIN, epochs=args.epochs)
    print("training config:", cfg, flush=True)
    model = YOLO(args.base)
    model.train(data=yaml_path, **cfg)

    best = os.path.join(model.trainer.save_dir, "weights", "best.pt")
    print("best weights:", best, flush=True)
    exported = YOLO(best).export(format="onnx", imgsz=cfg["imgsz"],
                                 simplify=True)
    print("exported:", exported, flush=True)

    out_onnx = args.out + ".onnx"
    os.makedirs(os.path.dirname(out_onnx) or ".", exist_ok=True)
    shutil.copyfile(exported, out_onnx)

    h = hashlib.sha256()
    with open(out_onnx, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    manifest = {
        "sha256": h.hexdigest(),
        "source": (f"yolo11n-seg fine-tune on {os.path.basename(args.data)} "
                   f"(geometric auto-labels, {cfg['epochs']} epochs max, "
                   f"freeze={cfg['freeze']}, seed={cfg['seed']})"),
        "trained_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "output": {"layout": "seg-v1", "nms": "external"},
    }
    with open(args.out + ".json", "w") as f:
        json.dump(manifest, f, indent=2)
    print("manifest:", args.out + ".json", flush=True)
    print("sha256:", manifest["sha256"], flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
