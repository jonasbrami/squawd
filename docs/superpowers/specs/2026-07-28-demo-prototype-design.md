# Demo Cockpit Prototype — design (v0.4, W3 saga + W2.5b)

> **Purpose.** Turn the eval rig into a product-feeling demo: a richer world
> with many followable objects, a general COCO detector, click-to-lock in the
> cockpit, and locked-object operations (orbit, stand-off, stop/resume) —
> riding the existing flight/perception seams, NOT the LLM, for interactions
> that must be deterministic.
>
> v0.2: codex gpt-5.6-sol-high review
> (`docs/superpowers/specs/reviews/2026-07-28-demo-prototype-codex-review.md`)
> folded in (W0, arbitration, fusion gating, milestone reorder).
> v0.3: W0 COMPLETE — results below; /api/lock is server-side hit-test with
> `{x,y}` frame pixels (supersedes v0.2's client-side design); conf floors
> and imgsz adjusted on gate evidence; persons are static mesh visuals
> (actors wedge headless gz); TinyRobot excluded from click-to-lock.
>
> Relation to the paused M0→M6 goal: PARALLEL demo track. Same seams, no gate
> interference. The M6 rungs are parked in docs/PROJECT-STATE.md.

## 1. The golden-path demo

1. `./scripts/run_single_demo.sh demo` → cockpit at :8000 shows a
   neighborhood: houses, gas station, trees, lamp posts; cars looping the
   streets, pedestrians on loops.
2. The overlay paints COCO boxes+masks (car, person, truck…), each with a
   stable `vis_{cls}_{k}` name and confidence.
3. The user CLICKS a car → corner-bracket reticle latches, the drone pursues
   to a ~10 m stand-off, ToF beam fuses → LOCKED pill with live range.
4. Contextual ops bar: **Orbit** (circles the mover, camera locked),
   **Approach/Back off** (stand-off radius), **Stop**, **Resume**.
5. Estop always wins instantly; operator UI commands preempt the LLM pilot
   via the serialized arbiter (estop > operator lease > LLM).

## 2. W0 — foundation — COMPLETE 2026-07-28 (suite 509→530 green)

1. **Detector-on-rendered-assets: PASS with caveats**
   (`docs/benchmarks/w0-detector-assets.md`, `evals/out/w0_detector_assets/`):
   - Artifact: `models/coco-nano-seg-v1.onnx` + `.json` manifest (mover
     schema + 80 COCO `classes`; sha256 360d06f3…dc24). Export path:
     `scripts/export_coco_seg.py` (existing ultralytics pattern).
   - @416: Hatchback car .74–.83 GO; person .53–.69 GO; SUV truck .27–.29
     MARGINAL; TruckDelivery .34–.47 MARGINAL; 40 m mostly dead at 416,
     RESTORED at 640. House-1 negative: 0 detections — PASS. TinyRobot:
     NO-GO (no COCO class; never misdetected either) → excluded from
     click-to-lock (custom-class route is a later decision).
   - fps in-container (ORT CPU, gz+PX4 running): **416 p50 18.3 / p95 19.7
     ms; 640 p50 39.6 / p95 40.5 ms** — both inside the 100 ms 10 Hz budget;
     **use 640 for the single-drone demo** (restores 25 m-low + 40 m cells).
   - Conf floors (evidence-adjusted): 0.25 all classes + two-hit confirm for
     tracking admission (SUV/TruckDelivery sit at .27–.34, below v0.2's 0.30
     vehicle floor).
   - Operational facts: NO `<actor>` headless (persons = static mesh
     visuals); steep look-down from 12 m at ~10 m classifies car roofs as
     "chair" → engage ≥25 m slant; Fuel include textures need the cache-side
     fix (in the swarm_sim.sh `assets` branch — REUSE in the demo branch);
     House-1 Fuel include renders black headless → mesh + plain material.
2. **Tracker-`none` fix: DONE** — `agents/vision/trackers/none.py`
   (`NoOpTracker`, first-class `"none"` registry entry); `Detector(...,
   tracker=)` kwarg actually wires the configured tracker (run.py passes
   cfg.tracker). The request_lock path can no longer raise.
3. **Crowd-safe identity: DONE** — server-side `hit_test(snap, x, y,
   frame_stamp)` in `agents/observatory/overlay.py` (unique-box hit →
   contact; ambiguous/miss/stale → 409 reason); `POST /api/lock {x,y}` in
   `server.py` publishes `{"op":"lock","contact":name}` on `/pilot/cmd`.
   Browser hit-testing is dropped (single testable source of truth).
4. **Command arbiter: DONE** — `agents/pilot/arbiter.py` `CommandArbiter`
   (estop latched > operator lease, 90 s default, > LLM; `guard_llm` raises
   `OperatorActiveError` while leased; lease ends on completion/stop/timeout;
   estop semantics unchanged, test_estop.py untouched). 9 arbiter tests.

## 3. P1 — Rich world (`demo`), staged

- **W1a**: 2 movers — Hatchback + SUV, mesh visuals on the proven
  velocity-drive mover pattern (`sim/plugins/mover_system.py`), 3–5 m/s
  waypoint_loop/circle; 3 landmarks (House 1 mesh+plain material, Gas
  Station, Pine Tree). Proves generator, fuel caching, resource-path +
  texture fix, render cost.
- **W1b**: full cast — + TruckDelivery, 2 Walking persons (static mesh
  visuals, slow loops), remaining landmarks (Lamp Post, Oak tree). NO actors.
- **Files**: NEW `sim/worlds/make_demo_world.py` (clone of
  `make_perceive_world.py`/`make_assets_world.py` + W0 texture fix, writes
  `demo_boxes.json`); `sim/launch/swarm_sim.sh` `demo` branch;
  `scripts/run_single_demo.sh demo` unchanged.
- **Base**: PX4 `default.sdf` (flat). Baylands rejected. `city` re-validate
  later.

## 4. P2 — General perception (COCO)

- **Detector**: `models/coco-nano-seg-v1.onnx` @ **640** for the demo;
  `VISION_BACKEND=onnx VISION_MODEL=coco-nano-seg-v1.onnx`. Code change:
  `_decode_seg` hardcodes `("target","obstacle")`
  (`agents/vision/backends.py:221`) → read names from the manifest
  `"classes"` table.
- **Tracker**: KEEP CV-EKF `VisionContacts` (`vis_{cls}_{k}` IDs). No
  ByteTrack.
- **Admission**: dynamic-class allowlist (car, truck, bus, person, bicycle,
  motorcycle); 0.25 + two-hit confirm for trackable contacts (per W0.1).
- **Click perception path**: `VisionContacts.designate()` →
  `Detector.request_lock` (fixed in W0.2). Lock on the FIRST good detection
  (flicker at range is normal; never demand streaks).
- **Wire schema**: extend `PerceptionSnapshot.det_json`
  (`agents/vision/pipeline.py:35`) to carry masks (RLE) for UI drawing.

## 5. P3 — Click-to-lock + locked operations

- **Hit-test**: server-side (W0.3): `POST /api/lock {x,y}` → unique-box
  contact or 409 stale/ambiguous/miss. W3 wires the browser click → endpoint
  (CSS px → frame px via the native 640×360 canvas).
- **Command path**: `/api/lock` → `/pilot/cmd` → **CommandArbiter** (W0.4)
  → `ops.track(name, mode=...)` registered in ActiveToolRegistry. W3 wiring:
  cmd subscriber + arbiter.submit_operator in `agents/pilot/run.py`; LLM
  tool calls go through `arbiter.guard_llm`; registry gen-aware clear.
  Bearing-only contacts acquire for free (`ops.py:505-529`).
- **Orbit-while-locked** (`track mode="orbit"`, `radius`, `rate_dps`) in
  `trk.control_ref` (`track.py:81-103`): `θ += ω/CTRL_HZ`,
  `ref = tgt + R(cosθ, sinθ)`, `ff = tangential + (est.ve, est.vn)`.
  **Init θ from current relative position** (no phase jump). Explicit work:
  - **ToF fusion is shadow-gated** (`beam.py:59 in_fusion_envelope`:
    mode=="shadow" + own-speed ≤3 m/s) → extend envelope to `orbit`
    deliberately; check the speed clause vs tangential speed.
  - Servo FF streaming: assert via fixture (pattern: tests/test_track_ops.py)
    that orbit refs actually stream feedforward.
  - 7 m keep-out bubble (`ops.py:737`) bounds min radius.
  - Stand-off = `range_m` arg with EXPLICIT radial init
    (`ref = tgt + R·(me−tgt)/|me−tgt|`).
  - **Stop** = `/pilot/cmd {op:"stop"}` → arbiter release + hold;
    **Resume** re-locks (which op calls `arbiter.release()` — W3 UX detail).
- **UI beauty**: crosshair cursor + hover-highlight + click ripple;
  corner-bracket reticle + pulsing ring on the designated contact
  (`drawDet` :357); LOCKED pill (`.trackbanner`); contextual ops bar
  (Orbit / Approach / Back off / Stop / Resume, `.btn`); clickable contact
  list; video-first layout; PPI orbit-radius ring (`:540-541`).

## 6. Milestones

| # | Milestone | Status / validates |
|---|---|---|
| **W0** | Foundation spikes | **DONE 2026-07-28** (§2: detection report + fps; 21 new tests, suite 530) |
| W1a | `demo` world: 2 movers + landmarks | **DONE** (boot + doctor + evidence frames; suite 539) |
| W1b | Full cast + heading-align + link-frame velocity fix | **DONE** (5 movers nose-first, textured landmarks; suite 548) |
| W2 | COCO detector live in-container | **DONE** (manifest classes, admission floors, masks on wire; suite 559; `/state` COCO contacts live) |
| W3a/W3b | Click-to-lock + ops bar + cmd arbiter + orbit controller | **DONE** (suite 582; /api/lock + /api/cmd live) |
| **W2.5b** | Demo-domain fine-tune (codex R5) | **DONE** — v2 accepted R6 (`coco-nano-seg-v2-640.onnx`, sha f7007721…): pursuit-vehicle recall **94.3 %** at 10–22 m qualified slant, **0/1100 negatives**, live miss streak ≤2.0 s; person display/best-effort only |
| W3 gate | Click→pursuit (RE-SCOPED, codex R4) | **DONE — PASS-with-caveats** (run 8, `w3-run8.md`): **64.8 s endurance engagement through all 4 corners** of a full car_1 lap, same id, MEASURED ≤2 s per corner; ops + estop mid-track proven (`tool cancelled: True`). Caveats: gap band 72 % (<80 %), LOST→re-lock lap-window-limited (35.1 s), pursuit op 60 s cap |
| W4 | Orbit/stand-off live tuning | **DONE — PASS-with-caveats** (`w4-orbit.md`): 44 s live orbit + stop/resume; lane held its EKF reference at p50 14.7 m. Caveat: radius accuracy limited by EKF-vs-truth ghost offset ~7 m (follow-up) |
| W5 | Golden-path polish | **DONE** (`w5-golden-path.md` + `evals/out/w5_golden/`): recorded end-to-end run (beats + 11:46 POV mp4 + timeline/ops logs + beat frames), RUN-DEMO runbook, suite 638+ green |

## 7. Open decisions — RESOLVED

1. Person movers: **mover_system static mesh visuals** (actors wedge
   headless gz — W0.1 evidence).
2. Stand-off: **`range_m` arg** on track, explicit radial init.
3. Confidence: **0.25 + two-hit confirm** (W0.1 evidence: SUV/Truck .27-.34).
4. `/pilot/cmd`: **always preempt via operator lease**; estop latched above
   both; lease ends on completion/stop/timeout.
5. imgsz: **640** for the single-drone demo (p50 39.6 ms, 2.5× budget).
6. TinyRobot: **excluded from click-to-lock** (no COCO class; custom-class
   route is a post-prototype decision).

## 8. UNVERIFIED / risks (post-W0)

- COCO detection beyond 40 m at 640 (restore verified at 25 m-low + 40 m-high
  only) — demo engagements should stay ≤40 m slant.
- Person textures under headless EGL (baylands trees render black already).
- Glass-to-glass video latency (frame_seq/staleness handled server-side).
- Orbit gain smoothness (KP=0.7, accel cap vs circling reference) — W4.
- Registry-tracker dormant modules (`trackers/dnn.py|template.py|sam.py`
  absent) + `VISION_TRACKER=auto` has no resolver (maps to `none`) —
  separate cleanup, not on the prototype path.

---

## 9. The W3 integration saga (2026-07-28 → 08-02) and W2.5b

Five live validation runs, each failing one layer deeper, each layer fixed
by a codex-reviewed change (reviews in `docs/benchmarks/w3-*-codex-r*.md`,
evidence in `docs/benchmarks/w3-{integration,rerun,run3,run4,run5}.md`):

| run | died at | mechanism | fix (verified live in the NEXT run) |
|---|---|---|---|
| 1 | 2–20 s | contact ID churn (class flaps, 2 s grace, readoption gate) + M3b alt sag to 2.3 m | **R1**: vehicle superclass assoc, lost_s/rebind 5 s, readoption relaxation, hold_altitude opt-out, HUD honesty (suite 595) |
| 2 | 8–9 s | vfov blind cone: 7 m keep-out at 6 m alt ⇒ target leaves frame | **R2**: `R_min(H)` radial floor in FlightOps (suite 601) |
| 3 | 27.4 s | corner transient: 1 m/s² shaper cuts inside the ring | **R3**: demo shadow → direct lane + measured-bearing yaw + radial escape barrier + ring R_min+2 (suite 604) |
| 4 | ~11 s | stale coast yaw (frozen on last measurement) + edge-clip rejection | **R4**: predicted-bearing coast yaw (2 s horizon) + gate re-scope to 3 engagements/90 s at 4 m (suite 607) |
| 5 | ~12 s | **detector wall**: stock COCO nano@640 ≈6% recall at pursuit aspects (rear-quarter, bottom-clipped) | **W2.5b**: demo-domain fine-tune (below) |

What's proven live through the saga: click chain (9×200 + correct 409s),
cmd→arbiter→ops.track dispatch, LLM-free golden path (zero backend
requests), alt/gap bands held, zero ID churn, grace recoveries ≤3.7 s,
geometry law held (no sub-floor samples, no corner cuts), estop mid-track.

**W2.5b (codex R5)**: fine-tune yolo11n-seg FROM COCO weights (80-class
head intact; Hatchback/SUV→car, TruckDelivery→truck, walkers→person) on
5,160 gz pursuit-aspect frames (6 aspects × 3 slant bands per vehicle,
persons, 1,200 hard negatives, 25% bottom-clipped) + 5,000 stratified COCO
replay images (anti-forgetting). Training: 8 ep freeze=10 → 22 ep freeze=5,
lr0=2e-4, imgsz=640, seed 7. Acceptance bar (unblocks W3 run 6): vehicle
recall ≥90% overall / ≥80% per cell, person ≥90%/≥80%, no miss streak
>1.0 s, COCO val AP50 drop ≤3 pts, ONNX p50 ≤50 ms. Expected lift:
6% → 85–95%. Fallbacks in order: yolo11s-seg (only if p95 ≤90 ms);
imgsz 960/1280 REJECTED (~89/~158 ms, no 10 Hz headroom); conf lowering
useless (no proposals below threshold).

---

## 10. Follow-ups (post-W5, 2026-08-02)

1. **Radius/ghost accuracy** (W4): EKF-vs-truth target offset ~7 m on
   orbit (ghost dives to ~0 m at death); CV range bias ~10 % on the v2
   footpoint + CV filter lag — next codex round's tracker item.
2. **Person pursuit excluded**: persons are display/best-effort only (the
   86.3 %-recall trade-off accepted at R6); engage CARS for any gate.
3. **v2 default promotion gated** on a quiet-host interleaved latency
   re-bench (v1↔v2): p50 ≤50 ms / p95 ≤70 ms / ≤10 % regression vs v1 —
   until then `VISION_MODEL=coco-nano-seg-v2-640.onnx` is an explicit
   override, NOT the demo default.
4. **Qualified slant band 10–22 m** for v2 pursuit geometry; engage
   inside it.
5. **TinyRobot custom-class route** (no COCO class) — post-prototype
   decision, untouched here.
6. **Sim-host environmental notes (run-8 attempt 1 + W5 boots 1–3)**:
   detector-empty camera stall window; a ~28° view twitch at lock with
   MPC_TILTMAX_AIR=12° confirmed set (cause unresolved); PX4 EKF yaw
   non-convergence under host load ≥~30 (time-sync storms → "blind land"
   failsafes) — boot on a quiet host and check `Ready for takeoff!`
   before driving.
