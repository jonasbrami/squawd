# Model weights — provenance & licenses

This directory holds detector artifacts consumed by `agents/vision/backends.py`
(`VISION_WEIGHTS_DIR`). Weights are git-ignored; provenance is recorded here and
in each artifact's manifest (`<model>.json`: `{"sha256", "source", "trained_at",
"output": {...}}`), which `OnnxBackend` verifies at load.

## Licenses (design §6.1)

- **Ultralytics** (YOLO families, code and pretrained weights): AGPL-3.0.
  Fine-tuned/exported derivatives produced with Ultralytics remain subject to
  the applicable Ultralytics terms. Review distribution obligations before
  publishing artifacts; an enterprise license is available for incompatible
  commercial use.
- **Meta SAM 2 / 2.1** checkpoints: Apache-2.0.
- **VisDrone community fine-tunes** (e.g. mshamrai yolov8-visdrone): their own
  community terms — check the source before redistributing.
- **OBB-DOTA** pretrained models: Ultralytics AGPL + DOTA dataset terms.
- Detection outputs, datasets, and exported artifacts can have different
  licensing obligations. Do not infer redistribution rights from this summary;
  retain source and license metadata in each manifest.

## Provisioning

Weights are local, git-ignored artifacts; manifests remain tracked. A clean
clone therefore needs explicit provisioning before camera inference can start.

### Fast ONNX lane

- `mover-nano-seg-v1.onnx` + manifest: two-class mover model produced by the
  M2.5 training/export pipeline.
- `coco-nano-seg-v1.onnx` and `coco-nano-seg-v1-640.onnx` + manifests: stock
  COCO yolo11n-seg exports with the 80 class names recorded in the manifest.
- `coco-nano-seg-v2-640.onnx` + manifest: demo-domain fine-tune accepted for the
  supported cockpit demo. It is selected explicitly with
  `VISION_MODEL=coco-nano-seg-v2-640.onnx`; `run_single_demo.sh demo` still
  defaults to v1 until the documented promotion gate is intentionally changed.

Relevant scripts are `train_mover_seg.py` (training and export),
`export_coco_seg.py`, `train_w25b_seg.py`, and `export_w25b_seg.py`. They require
their own training/source datasets; no script currently bootstraps all ONNX
artifacts from a clean clone. Verify every manifest SHA-256 at load.

### Deep host-GPU lane

Provision the two pinned upstream checkpoints with:

```bash
./scripts/provision_deep_models.sh
```

The script downloads to temporary paths, verifies hard-coded SHA-256 digests,
then writes the weight and manifest:

- `yolov8s-worldv2.pt` — YOLO-World (Ultralytics), AGPL-3.0.
- `sam2.1_t.pt` — Meta SAM 2.1, Apache-2.0.

Install the optional service environment and start it with:

```bash
uv venv .venv-train-gpu
uv pip install -p .venv-train-gpu -e '.[deep]'
./scripts/deep_perception.sh
```

See `docs/benchmarks/deep-perception-m1.md` through `-m4.md` for measured
operating points and limitations. Deep outputs are advisory; the fast COCO lane
remains contact authority.
