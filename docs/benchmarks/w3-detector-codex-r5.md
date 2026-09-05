1. **Primary: W2.5b, COCO-preserving demo-domain fine-tune.** Start from `yolo11n-seg.pt`, but keep an **80-class COCO head**. The current pipeline cannot do that unchanged: capture hard-codes two classes ([vision_dataset.py:174](/home/quenouille/drone/scripts/vision_dataset.py:174)), writes a two-class YAML ([vision_dataset.py:319](/home/quenouille/drone/scripts/vision_dataset.py:319)), and training consequently replaces the head ([train_mover_seg.py:46](/home/quenouille/drone/scripts/train_mover_seg.py:46)). Add a `demo-coco` profile mapping Hatchback/SUV→COCO `car`, TruckDelivery→`truck`, walkers→`person`, while retaining all 80 names. TinyRobot remains explicitly non-trackable; inventing a COCO label would corrupt semantics.

2. **Dataset: 5,160 Gazebo frames plus 5,000 COCO replay images.** Capture primarily in the real `demo` world—correct backgrounds, motion, clipping and render domain—not W0.1’s static grid.

   - Per vehicle mesh: 6 aspect bins (front, rear, left/right side, front/rear-quarter) × 3 slant bands (10–15, 15–22, 22–30 m) × 60 frames = **1,080**, or 3,240 total.
   - Persons: 6×3×40 = **720** across both walkers.
   - **1,200 negatives:** house/gas-station roofs, trees, lamps, road, partial/empty frames; deliberately include the “chair” roof aspects.
   - Balance altitudes 4/6/8/10/14 m within every cell; make 25% bottom-clipped and 15% horizontally edge-clipped.

   Extend the capture lattice at [vision_dataset.py:247](/home/quenouille/drone/scripts/vision_dataset.py:247). Rotate projected cuboid corners by truth quaternion—the current labels are axis-aligned ([vision_dataset.py:122](/home/quenouille/drone/scripts/vision_dataset.py:122)). Split **70/15/15 by complete capture run/trajectory seed**, not every tenth adjacent frame as currently done ([vision_dataset.py:226](/home/quenouille/drone/scripts/vision_dataset.py:226)). Keep W0.1 assets and new pursuit cameras entirely held out.

   Mix 5,000 stratified COCO-seg train images, including at least 500 each containing person/car/truck/bus and coverage of all 80 classes. Otherwise Gazebo images teach absent COCO classes to become background; pretrained initialization alone does not prevent forgetting.

3. **Training.** At [train_mover_seg.py:24](/home/quenouille/drone/scripts/train_mover_seg.py:24), add the mixed-data profile: 8 epochs `freeze=10`, then 22 epochs `freeze=5`, `imgsz=640`, batch 16, low second-stage LR (`lr0=2e-4`), cosine decay, patience 8, seed 7. Export an 80-name manifest like the stock exporter does ([export_coco_seg.py:57](/home/quenouille/drone/scripts/export_coco_seg.py:57)). Expected in-domain superclass recall is **85–95%**, versus run 5’s 6%; temporal-gap acceptance matters more than confidence uplift.

4. **W2.5b gate that unblocks W3 run 6.** Extend the truth-box evaluator’s existing overlap rule ([w0_assets_eval.py:248](/home/quenouille/drone/scripts/w0_assets_eval.py:248)) over held-out 10 Hz pursuit clips:

   - Vehicle-superclass recall ≥90% overall and ≥80% in every mesh×aspect×range cell.
   - Person recall ≥90% overall and ≥80% per aspect×range cell.
   - No miss streak >10 frames/1.0 second; miss-gap p95 ≤0.5 second.
   - Admitted-class false positives ≤0.5% of negative frames; zero house-roof vehicle/person contacts.
   - Preserve every previously passing W0.1 cell ([w0-detector-assets.md:49](/home/quenouille/drone/docs/benchmarks/w0-detector-assets.md:49)).
   - Against COCO val2017 or a fixed stratified subset: person/car/truck/bus AP50 drop ≤3 points and overall box/mask mAP drop ≤2 points.
   - Production ONNX p50 ≤50 ms, p95 ≤70 ms with Gazebo running.

5. **Committed fallback: identically fine-tuned `yolo11s-seg@640`.** Use it only if nano misses recall while the local ONNX benchmark remains p95 ≤90 ms. Official YOLO11 detection measurements put `s` around 1.6× `n` CPU latency, but segmentation must be benchmarked locally ([Ultralytics YOLO11 metrics](https://docs.ultralytics.com/models/yolo11#performance-metrics)).

   Resolution-only fallback is weak: scaling the measured 39.6 ms ([w0-detector-assets.md:75](/home/quenouille/drone/docs/benchmarks/w0-detector-assets.md:75)) by pixel area predicts **960≈89 ms** and **1280≈158 ms**, leaving no 10 Hz headroom or falling to ~6 Hz. Lowering confidence is not a fix: class scores are discarded before mask assembly ([backends.py:214](/home/quenouille/drone/agents/vision/backends.py:214)); run 5 lacks proposals. A stock-person plus custom-vehicle hybrid is third-line only: two nano passes nominally cost ~79 ms before contention and require result fusion.

6. **Rejected shortcuts:** COCO-friendly replacement cars falsify the demo claim; slowing pursuit does not repair rear-quarter domain recall; synthetic randomization may augment training but cannot replace exact Gazebo renders; lowering `conf<0.25` mainly admits false roofs.