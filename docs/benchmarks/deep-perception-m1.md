# Deep Perception M1 — sidecar service + real-weights acceptance (2026-08-03)

Milestone M1a/M1b of the deep-perception plan (`black-hawk-taskmaster-groot`,
codex r1 SHIP-WITH-CHANGES folded in): the host-GPU sidecar
(`agents/vision/deep/`, Starlette + uvicorn, bearer auth, one-lock 429) plus
the stdlib in-container client (`agents/perception/deep_client.py`), with real
pinned weights accepted on the live sim against real GPU inference.

**Verdict: SHIPPED.** Both models provisioned with hard-coded sha256 and
serving; warm detect p50 9.1 ms / segment p50 82 ms on the RTX 3070 Ti; the
10 Hz fast lane did not degrade under sidecar load (RENDER_BACKEND=intel);
container→host path proven with bearer (401 without). One real ultralytics
adapter bug found and fixed (CLIP device re-pin, below). Open-vocab quality
on the flat synthetic renders is WEAK at the plan's 0.25 default conf — see
the detect tables; M4 must set thresholds from the recorded-set numbers.

## Provisioning (M1b)

`scripts/provision_deep_models.sh` — official ultralytics-assets URLs only,
hard-coded sha256 verified before the manifest is written (codex CR2):

| model | size | sha256 | license |
|---|---|---|---|
| `models/yolov8s-worldv2.pt` | 25,923,032 B (24.7 MiB) | `9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792` | YOLO-World (Ultralytics) AGPL-3.0 |
| `models/sam2.1_t.pt` | 78,105,722 B (74.5 MiB) | `3c1e81ca9b037dd39d70a014ddb9a813d6c4c4e12555420db7eaff31689bd4e3` | Meta SAM 2.1 Apache-2.0 |

Manifests `models/yolov8s-worldv2.json` / `models/sam2.1_t.json` written in
the repo's `{sha256, source, downloaded_at}` shape; `models/README.md`
provenance lines appended by the script. Weights stay gitignored; re-run is
idempotent (re-download + re-verify, README grep-guarded). First-ever service
start additionally pulled ultralytics' CLIP dependency (AutoUpdate from
`git+https://github.com/ultralytics/CLIP.git`) and the ViT-B/32 weights
(~338 MB) on the first `set_classes` — see "Findings".

## Sidecar

`./scripts/deep_perception.sh` (host, `.venv-train-gpu`, torch 2.13.0+cu126,
ultralytics 8.4.103) — binds the discovered docker0 gateway `172.17.0.1:8100`
(never 0.0.0.0), bearer from gitignored `.deep_token`. Health with both
models loaded:

```json
{"ok": true, "device": "cuda",
 "models_loaded": ["sam2.1_t", "yolov8s-worldv2"], "vram_mb": 898}
```

`--selftest` (health-only in M1a) passes from the host via the same gateway
discovery. Stop the sidecar with `pkill -f agents.vision.deep.service` (it is
a plain foreground uvicorn under the launcher; M2 expects it running).

## Acceptance frames (live sim, world=demo)

The M0 purge deleted the W-series recordings, so three FRESH frames were
captured from the running pilot-sim container (drone airborne HOLD at
E48.9 N36.7 alt 16 m; yaw-in-place only, no translation — mavsdk
`goto_location` at the current position, `evals/out/deep_m1b/face.py`).
Raw gz RGB888 bytes saved losslessly (`capture_frames.py`, GzCameras — the
exact `Frame` wire contract), 640x360:

| frame | content (verified by eye) | sidecar |
|---|---|---|
| `evals/out/deep_m1b/frame_a_car_houses.png` | red hatchback (car_1) center ~30 m; house_1 + house_2 right; second vehicle far left | seq/stamp in `.json` |
| `evals/out/deep_m1b/frame_b_houses_pole_car.png` | house_1 left, house_2 center, lamp post, pine tree right, car_1 bottom edge | |
| `evals/out/deep_m1b/frame_c_gasstation.png` | gas-station canopy + totem, clean building shot | |

Harness: `evals/out/deep_m1b/acceptance.py` → `results.json` (all numbers
below are from it, against the live service through the real DeepClient).

## a. Vocabulary detect (YOLO-World s, boxes only — codex F7)

Prompts `building,house,tree,pole,car,truck,person`, conf 0.05 (see the
threshold note):

| frame | dets (cls conf xyxy) | judgment |
|---|---|---|
| A | `person 0.23 [311,278,340,303]`; `house 0.19 [572,235,640,280]`; `house 0.05 [572,235,640,313]` | boxes land on the real objects (car, far house) but the tiny red hatchback is MISLABELED `person`; near house_1 missed |
| B | `person 0.17 [395,346,427,360]`; `tree 0.06 [539,251,629,360]` | again the car reads as `person`; pine found; both houses missed |
| C | (none) | canopy not seen as `building` even at 0.05 |

At the plan's `look` default conf 0.25, frame A returns ZERO dets — the flat
gray untextured sim renders sit far from YOLO-World's photo domain, so
confidences compress to 0.05–0.25. M2's tool default and M4's per-concept
recall/FP numbers must be set from the recorded set, not assumed; candidate
sidecar operating point is conf ≈ 0.05–0.10 with the label-miss caveat above
(the fast COCO nano stays the mover authority regardless — deep outputs are
advisory).

## b. Color-order proof

Discriminator vocabulary `["red car", "blue car"]` on frame A through the
wire (client b64 → service → registry `frame_to_array` RGB→BGR):

- correct order: `red car 0.018 [310,278,341,304]` on the real red hatchback
  (the gray far vehicle reads `blue car 0.033` either way — channel-neutral)
- R/B-swapped copy: the SAME box flips to `blue car 0.011`

The wire carries RGB888 (the `Frame` contract, `agents/core/contact.py`);
`agents.vision.backends.frame_to_array` remains THE one RGB→BGR conversion
site and the registry's world adapter feeds ultralytics BGR. The label flip
on the same box proves the order end to end.

## c. SAM 2.1 segment (one-shot `predict`, codex F6)

| prompt | result | check |
|---|---|---|
| point `(326,288)` (car centroid, frame A) | tight xyxy `[311,279,340,304]`, centroid `(326.4,294.5)` 6.5 px from the point, area 380 px, score 0.904, 83 ms | box-local RLE 29x25 decodes to exactly area_px |
| box `[390,216,506,302]` (house_1, frame A) | tight xyxy `[396,235,498,296]` (inside the prompt), area 5414 px, score 0.955, 84 ms | RLE 102x61, decoded area matches |
| box `[179,256,512,360]` (canopy, frame C) | xyxy `[189,303,490,360]`, area 10393 px, score 0.617, 81 ms | RLE 301x57, decoded area matches |

`decoded_area_matches: true` on all three (rle_decode of the wire payload vs
`area_px`) — the box-local mask contract (codex F8) holds on the real path.

## d. Latency + VRAM

| call | cold (first in process) | warm p50 | warm p95 (n=10) |
|---|---|---|---|
| detect (7-prompt vocab) | 15.8 s (torch+ultralytics import, weights, CUDA init, CLIP embed) | 9.1 ms | 13.3 ms |
| segment (point) | 2.75 s (SAM weights + first inference) | 82.3 ms | 93.2 ms |

VRAM with both models loaded: 898 MiB torch-allocated (`/v1/health.vram_mb`),
2545 MiB total GPU-used per nvidia-smi (incl. desktop; ~2.5 GB of 8 GB — the
RTX 3070 Ti has ample headroom). Cold-start cost is process-lifetime once;
the launcher keeps the service up, so M2 tools see warm-path latency.

## e. Coexistence (preliminary for the M3 gate)

Sim RUNNING, RENDER_BACKEND=intel (gz renders on the iGPU — it does NOT share
the RTX). Cockpit `/state` sampled before vs while hammering the sidecar
with sequential detects:

| | detector.latency_ms | cam cadence |
|---|---|---|
| idle sidecar | 54.4 / 41.6 | 9.5 Hz |
| under sidecar hammer | 41.8 / 42.8 | 9.0 Hz |

No fast-lane degradation (latency unchanged within its normal jitter; the cam
delta is sampling noise over a 2 s window). The nvidia-render case
(RENDER_BACKEND=nvidia) still needs the M3 A/B — gz would share this GPU.

## f. Live 429

Two concurrent detects from barrier-synced threads: `['BUSY', 'OK']` — the
second caller is rejected immediately (no queue), matching the unit tests.

## g. Container → host path (the M2 seam)

From inside pilot-sim via the docker0 gateway:

- `GET http://172.17.0.1:8100/v1/health` with the bearer from
  `/workspace/.deep_token` → 200 with the health JSON above (raw urllib AND
  the repo `DeepClient`; token never printed)
- same request without a token → 401

## Adapter fix vs the M1a code (real-model mismatch, per the milestone rules)

1. **CLIP device re-pin (registry `_WorldModel.set_classes`)** — ultralytics
   8.4.103: a CUDA `predict()` moves the cached CLIP text encoder's weights
   but not its `self.device` attribute (`nn/text_model.py` tokenizes onto
   `self.device`). A vocabulary CHANGE after the first GPU predict therefore
   crashed with `cpu tokens vs cuda:0 weights` in `F.embedding`. The adapter
   now re-pins `clip_model.to(dev)` + `clip_model.device = dev` before every
   `set_classes`. Verified live: consecutive distinct vocabularies through
   the service succeed. (M1a's fake-model tests could not see this — no CUDA
   in tests by design.)
2. **First-run CLIP provisioning** — the first-ever `set_classes` triggers
   ultralytics' AutoUpdate of `clip` from git plus a ~338 MB ViT-B/32
   download, and ultralytics prints "restart runtime for updates to take
   effect"; the detect served by that first process returned empty. After a
   restart (this milestone's running service) everything is warm and correct.
   Ops note: on a fresh host, start the sidecar once, let the first call
   finish, then restart it (or pre-warm in provisioning) — recorded here so
   M2 doesn't trip over it.

## Gates

- `pytest tests/test_deep_registry.py tests/test_deep_service.py
  tests/test_deep_client.py tests/test_import_rules.py` — 44 passed (after
  the adapter fix; no other product-code changes besides the sha256
  constants).
- Provision script re-run: idempotent (manifests rewritten identical, README
  not duplicated).
- Suite baseline 680 (M1a) unchanged — no fast-lane or shared-code edits.

## Left for M2+

`run_single_demo.sh` `--add-host host.docker.internal:host-gateway` +
`DEEP_PERCEPTION_URL`/`DEEP_TOKEN` passthrough; `look`/`pinpoint` tools with
`asyncio.to_thread`; sidecar conf operating point from the M4 recorded set.
The sidecar is LEFT RUNNING (pid via `pgrep -f agents.vision.deep.service`).
