# W0.1 — Detector-on-rendered-assets gate (2026-08-01)

Gate: prove (or disprove) that stock COCO `yolo11n-seg` ONNX detects the demo
world's Fuel object cast when rendered by Gazebo — the demo prototype's #1
risk (COCO-on-Fuel domain gap, design 2026-07-28 §2 item 1) — and record
in-container performance.

**Verdict: GATE PASSED with caveats.** Every COCO-mapped cast member is
detected at demo-relevant geometries (10 m and 25 m) at the canonical 3 m
hold; 40 m is resolution/aspect-limited; TinyRobot is invisible to COCO at
every range (needs its own answer). The 416 export meets the 10 Hz budget
with 5x headroom; 640 restores most 25–40 m detections at 2.2x cost, still
inside budget.

## Artifacts

| what | where |
|---|---|
| shipped model (416) | `models/coco-nano-seg-v1.onnx` — sha256 `360d06f3f123492d199b9f39faffa0287034f53c86b503ca9eb2055e81fbdc24` |
| manifest (+80 COCO `classes`) | `models/coco-nano-seg-v1.json` |
| bench-only 640 export | `evals/out/w0_detector_assets/coco-nano-seg-640.onnx` (+`.json`) |
| world generator | `sim/worlds/make_assets_world.py` (+ `swarm_sim.sh` `assets` branch) |
| capture / eval / bench scripts | `scripts/w0_assets_capture.py`, `scripts/w0_assets_eval.py`, `scripts/export_coco_seg.py` |
| machine report | `evals/out/w0_detector_assets/report.json` (416 + fps + 640 comparison), `report640.json` |
| frames / annotated | `evals/out/w0_detector_assets/frames/` (70), `annotated/` (6) |

## Method

- Export: stock COCO `yolo11n-seg.pt` (no fine-tune) → ONNX `imgsz=416`,
  `simplify=True`, via the existing `.venv-train` ultralytics path
  (mirror of `scripts/train_mover_seg.py`). Outputs verified: input
  `[1,3,416,416]`, det head `[1,116,3549]`, protos `[1,32,104,104]`.
- World: flat `default.sdf` + the cast (Hatchback, SUV, TruckDelivery,
  TinyRobot, Walking person, House 1) at 10 / 25 / 40 m rows; 7 STATIC
  cameras replicating the x500_depth IMX214 exactly (hfov 1.204 rad,
  640×360, far 100). `low` = 3 m pitch 0 (the exact production geometry);
  `high` = 12 m, pitched down 0.55 rad (necessary: with the natural
  untilted camera the 10/25 m rows are BELOW the frame from 12 m —
  `cam40_high` keeps pitch 0 as the faithful 12 m case).
- Frames: 10 per camera (70 total), `squawd:dev` container, llvmpipe.
- Eval: ONNX run DIRECTLY via onnxruntime (no `agents/` code), preprocessing
  and class-aware NMS (iou 0.45) mirrored from `OnnxBackend`, conf 0.25.
  Hit = expected class group (design §4 admission: vehicles = car/truck/bus)
  overlapping the projected expected box (exact poses known; projection is
  full-3D pinhole — pitch-aware, `F·tan` not `F·angle`, both bugs caught and
  fixed during harness validation). Object dims MEASURED from the Fuel
  meshes, not assumed.

## Detection @416 (10 frames per cell; conf p50)

| object | 10m low | 10m high | 25m low | 25m high | 40m low | 40m high |
|---|---|---|---|---|---|---|
| Hatchback | **10/10 car .83** | 0/10 (**chair** .44) | **10/10 car .74** | **10/10 car .76** | 0/10 | 0/10 |
| SUV | **10/10 truck .28** | 0/10 (**chair** .51) | 0/10 | **10/10 truck .27** | 0/10 | **10/10 car .29** |
| TruckDelivery | **10/10 car .34** | 0/10 | 0/10 | **10/10 truck .47** | 0/10 | 0/10 |
| TinyRobot | 0/10 | 0/10 | 0/10 | 0/10 | 0/10 | 0/10 |
| Walking person | **10/10 person .69** | **10/10 person .53** | **10/10 person .62** | **10/10 person .55** | 0/10 | 0/10 |
| House 1 (negative) | — | — | — | — | — | **0 false dets of ANY class in 10 frames — PASS** |

Label confusion is class-group only (SUV↔truck, TruckDelivery→car) — never
vehicle↔person. Box widths track the projected extent (w-ratio p50
0.74–1.27). Not shown: a BACKGROUND walker at 42 m in side view was detected
in `cam10_low` (person .53) — 40 m failure is aspect-dependent (front-view
person ≈ 7×21 px), not a hard range limit.

## 640 comparison (same frames, report640.json)

- Restores: SUV 25m-low (→10/10 truck .64), TruckDelivery 25m-low (→10/10
  truck .32), **walker 40m both heights (→10/10 person .49)**; lifts SUV
  10m-low conf .28→.46 (back above the §4 0.30 vehicle floor).
- Does NOT restore: Hatchback 40m, TruckDelivery 40m (contrast/size-limited,
  not resolution), steep 12m-top-down vehicles (SUV still "chair").
- TinyRobot: still invisible (one stray "car" label in 60 object-frames).

## In-container fps (squawd:dev, ORT 1.28 CPU, 20 cores i9-12900H, gz+PX4 running)

| model | raw infer p50/p95 | full pipeline p50/p95 (letterbox+decode+NMS+masks) |
|---|---|---|
| 416 | 16.8 / 18.9 ms | **18.3 / 19.7 ms** |
| 640 | 37.5 / 38.5 ms | **39.6 / 40.5 ms** |

## Per-class GO / NO-GO (demo cast)

- **Hatchback: GO** (10m/25m solid; 40m NO — plan engagements ≤30 m).
- **SUV: GO, marginal at 416** (detected 10–25m but conf .27–.29 sits at/below
  the §4 0.30 vehicle floor; 640 lifts it clear).
- **TruckDelivery: GO, marginal** (same conf story; 25m needs 640 or height).
- **Walking person: GO** (10/25m both heights, conf .5–.7; 40m needs 640).
- **TinyRobot: NO-GO as a COCO detection target** — no class, never detected,
  never misdetected either. Needs the custom-class route (design's
  "+ TinyRobot mapping") or accept it as oracle/blob-only.
- **House 1 negative check: PASS** (zero detections of any class on the house).

## imgsz verdict: 416 = GO for the 10 Hz budget

416 at p50 18.3 ms/frame (with masks) fits the ≤100 ms budget 5x over AND
beats the ≤25 ms precedent of the 2-class mover model. **But** the detection
data says 416 alone loses 25 m low-altitude vehicles and 40 m persons. For
the single-drone demo cockpit, 640 at ~40 ms also fits 10 Hz comfortably and
is the better operating point for click-to-lock at 25–40 m; for N>1
detectors sharing the box, 416 is the right default. Keep both exports.

## Surprises (all worked around; details in report.json notes)

1. **`<actor>` wedges headless-CPU gz outright** — 3× Walking person actors
   (even with `<library_animations>` stripped, 26 MB→4 MB) peg one core
   forever; `/clock` never publishes; includes-only/cameras-only worlds are
   healthy. The walker renders fine as a STATIC mesh visual (frozen stride
   pose) — which is also the demo's W1b plan. **W1b: actors are verified
   BROKEN headless; use static mesh visuals for persons.**
2. **House 1's Fuel include renders BLACK headless** (custom Ogre material
   script unsupported) — replaced with the same mesh + plain material at the
   Fuel scale (dae is in inches; ogre honors `<unit>`).
3. **Hatchback textures don't resolve from a fuel include** (relative +
   `model://` refs vs versioned cache) — fixed cache-side in the `assets`
   branch (texture copy into `meshes/`, version-less `model-links` symlinks
   on `GZ_SIM_RESOURCE_PATH`). Without it the car renders untextured gray,
   falsifying the very domain gap under test. **W1a's demo world will hit
   the same thing — reuse the branch's fix.**
4. **Car roofs from 12 m at 10 m read as "chair"** (conf .44–.51, 10/10) —
   steep look-down kills vehicle detection; at 12 m engage from ≥25 m slant.
   The §4 allowlist filters "chair" (no false contacts), but no trackable
   contact exists either.
5. TinyRobot references `libslotcar.so` (absent) — harmless for a static
   placement; matters if W1b wants it driven by that plugin.

## Follow-ups before W1b (full cast)

- Persons as static mesh visuals (frozen pose), NOT actors — update the
  W1b plan accordingly (mover_system can still drive pose; keep them out of
  skeletal animation).
- Reuse the texture/symlink fix in `make_demo_world.py`'s branch.
- Decide imgsz per deployment: 416 (multi-detector) vs 640 (single-drone
  demo, restores 25–40 m). Both exports exist; `VISION_MODEL` picks one.
- Revisit the 0.30 vehicle admission floor: at 416, SUV/TruckDelivery
  detections land at .27–.34 — a 0.25 floor + two-hit confirm admits them.
- TinyRobot: pick the custom-class/mapping route or exclude from the
  click-to-lock cast.
- Detection flicker at 40 m is confirmed real (design's assumption) —
  lock on first good detection, never demand streaks.
