#!/usr/bin/env python3
"""export_coco_seg — W0.1 artifact: STOCK COCO yolo11n-seg -> ONNX @416.

Mirrors the export+manifest half of scripts/train_mover_seg.py (same
.venv-train interpreter, same YOLO(...).export(format="onnx", simplify=True)
call) but with NO fine-tune: this is the stock COCO model the demo
prototype's P2 general perception selects via VISION_MODEL=coco-nano-seg-v1
(design 2026-07-28 §2 item 1, §4). The manifest adds "classes" (the 80 COCO
names from model.names) — the mover manifest lacks it and
agents/vision/backends.py:_decode_seg falls back to ("target","obstacle").

  .venv-train/bin/python scripts/export_coco_seg.py

Artifacts (models/ weights are git-ignored, manifest committed per
models/README.md):
  models/coco-nano-seg-v1.onnx / .json                the shipped 416 artifact
  evals/out/w0_detector_assets/coco-nano-seg-640.onnx bench-only 640 export
                                                      (W0.1 fps comparison)
"""
import datetime
import hashlib
import json
import os
import shutil
import sys

IMGSZ = 416
BENCH_IMGSZ = 640
SCRATCH = "evals/out/w0_detector_assets/export"


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    base = os.path.expanduser("~/perception-lab/yolo11n-seg.pt")
    if not os.path.exists(base):
        print(f"missing base weights: {base}", file=sys.stderr)
        return 1
    os.makedirs(SCRATCH, exist_ok=True)
    os.makedirs("models", exist_ok=True)

    # Export from a COPY: ultralytics writes <name>.onnx next to the .pt, and
    # the lab dir already holds a 640 export used by perception-lab tooling.
    pt = os.path.join(SCRATCH, "yolo11n-seg.pt")
    if not os.path.exists(pt):
        shutil.copyfile(base, pt)

    from ultralytics import YOLO                    # .venv-train only

    model = YOLO(pt)
    classes = [model.names[i] for i in sorted(model.names)]
    if len(classes) != 80:
        print(f"expected 80 COCO classes, got {len(classes)}", file=sys.stderr)
        return 1

    out_onnx = "models/coco-nano-seg-v1.onnx"
    exported = model.export(format="onnx", imgsz=IMGSZ, simplify=True)
    shutil.move(exported, out_onnx)
    print("exported:", out_onnx, flush=True)

    exported640 = YOLO(pt).export(format="onnx", imgsz=BENCH_IMGSZ,
                                  simplify=True)
    shutil.move(exported640, "evals/out/w0_detector_assets/"
                             "coco-nano-seg-640.onnx")
    print("exported: evals/out/w0_detector_assets/coco-nano-seg-640.onnx "
          "(bench only)", flush=True)

    manifest = {
        "sha256": sha256(out_onnx),
        "source": (f"stock COCO yolo11n-seg (Ultralytics pretrained, AGPL-3.0; "
                   f"no fine-tune), exported imgsz={IMGSZ} simplify=True for "
                   f"the 10 Hz detector budget (W0.1)"),
        "trained_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "output": {"layout": "seg-v1", "nms": "external"},
        "classes": classes,
    }
    with open("models/coco-nano-seg-v1.json", "w") as f:
        json.dump(manifest, f, indent=2)
    print("manifest: models/coco-nano-seg-v1.json", flush=True)
    print("sha256:", manifest["sha256"], flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
