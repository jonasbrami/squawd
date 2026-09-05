#!/usr/bin/env python3
"""train_w25b_seg — W2.5b phase 2: COCO-preserving fine-tune of yolo11n-seg
on the W2.5b mixed dataset (codex R5 training config, spec
docs/benchmarks/w3-detector-codex-r5.md §3).

Two-stage because ultralytics `freeze` is static per train() call:
  stage 1:  8 epochs, freeze=10, default LR  (80-class head adapts to the
                                             demo domain, backbone frozen)
  stage 2: 22 epochs, freeze=5, lr0=2e-4     (from stage-1 best.pt)
common: imgsz=640, batch=16, cos_lr, patience=8, seed=7.

  nohup .venv-train-gpu/bin/python scripts/train_w25b_seg.py \
      > runs/segment/w25b_train.log 2>&1 &

Runs land in runs/segment/w25b_s1 and runs/segment/w25b_s2; the acceptance
model is runs/segment/w25b_s2/weights/best.pt. Export + manifest are a
separate step (scripts/export_w25b_seg.py) so training metrics can be
inspected before the artifact is produced.
"""
import argparse
import os
import sys

DATA = "evals/out/w25b_dataset/dataset-host.yaml"
BASE = os.path.expanduser("~/perception-lab/yolo11n-seg.pt")

COMMON = dict(
    imgsz=640,
    batch=16,
    cos_lr=True,
    patience=8,
    seed=7,
    workers=8,
    device=0,
    project="runs/segment",
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=DATA)
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--stage", choices=["1", "2", "all"], default="all")
    ap.add_argument("--s1-best", default=None,
                    help="stage-2 only: path to stage-1 best.pt")
    args = ap.parse_args()

    for p, what in ((args.data, "dataset yaml"), (args.base, "base weights")):
        if not os.path.exists(p):
            print(f"missing {what}: {p}", file=sys.stderr)
            return 1

    from ultralytics import YOLO                    # .venv-train-gpu only

    best1 = args.s1_best
    if args.stage in ("1", "all"):
        cfg1 = dict(COMMON, epochs=8, freeze=10, name="w25b_s1")
        print("stage 1 config:", cfg1, flush=True)
        m1 = YOLO(args.base)
        m1.train(data=args.data, **cfg1)
        best1 = os.path.join(m1.trainer.save_dir, "weights", "best.pt")
        print("stage 1 best:", best1, flush=True)

    if args.stage in ("2", "all"):
        if not best1 or not os.path.exists(best1):
            print(f"missing stage-1 best.pt: {best1}", file=sys.stderr)
            return 1
        # optimizer=auto IGNORES an explicit lr0 in ultralytics 8.4.103
        # ("ignoring 'lr0=0.0002' ... determining best automatically") —
        # pin AdamW so the codex R5 second-stage lr0=2e-4 is honored.
        cfg2 = dict(COMMON, epochs=22, freeze=5, optimizer="AdamW",
                    lr0=2e-4, name="w25b_s2")
        print("stage 2 config:", cfg2, flush=True)
        m2 = YOLO(best1)
        m2.train(data=args.data, **cfg2)
        best2 = os.path.join(m2.trainer.save_dir, "weights", "best.pt")
        print("stage 2 best:", best2, flush=True)
        print("ACCEPTANCE WEIGHTS:", best2, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
