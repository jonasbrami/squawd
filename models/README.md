# Model weights — provenance & licenses

This directory holds detector artifacts consumed by `agents/vision/backends.py`
(`VISION_WEIGHTS_DIR`). Weights are git-ignored; provenance is recorded here and
in each artifact's manifest (`<model>.json`: `{"sha256", "source", "trained_at",
"output": {...}}`), which `OnnxBackend` verifies at load.

## Licenses (design §6.1)

- **Ultralytics** (yolo26 / yolo11 families, code and pretrained weights):
  AGPL-3.0. This repo is fully open-source, which satisfies the AGPL obligations;
  fine-tuned/exported derivatives produced with Ultralytics remain AGPL-3.0.
  Enterprise license available if this project is ever closed/commercial.
- **Meta SAM 2 / 2.1** checkpoints: Apache-2.0.
- **VisDrone community fine-tunes** (e.g. mshamrai yolov8-visdrone): their own
  community terms — check the source before redistributing.
- **OBB-DOTA** pretrained models: Ultralytics AGPL + DOTA dataset terms.
- Detection *outputs* (boxes, masks) are not a covered work and are unencumbered.

## Provisioning

Copy or symlink weights here; do NOT commit them. The host lab
(`~/perception-lab`, symlinks into `~/scratch/yolo-webcam`) is the current
source of pre-trained weights. Custom mover models ship from the M2.5 training
pipeline as `mover-nano-seg-v<N>.onnx` + `mover-nano-seg-v<N>.json`.
