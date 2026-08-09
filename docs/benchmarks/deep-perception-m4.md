# Deep Perception M4 — metrics acceptance: recorded-set recall/FP, SAM IoU, latency, pursuit regression, demo probes (2026-08-03)

Final milestone of the deep-perception plan (`black-hawk-taskmaster-groot`):
the acceptance is NUMBERS, not anecdote — a 29-frame hand-labeled recorded
set from the live sim; per-concept presence-level recall/FP for YOLO-World
at the two operating points (conf 0.05, 0.10); SAM 2.1 mask IoU vs the
repo's own pseudo-GT; look()/pinpoint() end-to-end latency; the fast-lane
regression + load check during a live pursuit with the slowlane at its
shipped default; and the two demo probes (the wave-1 S2 phantom-truck
regression, and a building-aware op). M1–M3 product code FROZEN throughout
(no product-code changes in M4; this doc + eval artifacts only).

**Verdict: SHIPPED-WITH-QUANTIFIED-LIMITS.** The deep lane is a solid
ADVISORY layer: movers (car ≤40 m, truck ≤30 m, person ≤83 m, trees) are
detected reliably at BOTH operating points (recall 67–100 %), false
positives on empty scenes are ~zero, SAM masks are geometry-grade on
houses and vehicles (IoU 0.55–0.84), end-to-end tool latency is
LLM-negligible (production arm: look p50 29 ms, pinpoint p50 120 ms),
and the pursuit window shows no fast-lane regression vs the M3
baseline. The quantified weaknesses: **buildings/houses are marginal at
conf 0.10 (recall 20–40 % — the flat-render confidence compression,
M1b's finding, now measured)**, **pole is effectively blind (0/8 at
both confs)**, the gas-station canopy is not seen as a building even
at 0.05, close-up (≤30 m) dark roofs are missed, and trucks ≥66 m read
as "car". Recommended operating point: **conf 0.05 with the advisory
semantics exactly as shipped** (0.10 loses the building class entirely
for no meaningful FP gain — FPs are already rare at 0.05).

Ops incident before the numbers (affects RUN-DEMO, see §Ops): the box
entered a state where PX4 SITL arming was refused for ~90 min (EKF
yaw/height/GPS-drift preflight flapping under host load + memory
pressure); the recovery recipe is **`arm()` first, then `takeoff()`** —
plain arm catches a clean EKF window, takeoff-after-arm does not re-run
the failing check the way the takeoff command does. No product-code
change made; the stack was restarted twice via `run_single_demo.sh`
(exact M3 env: kimi/k3-256k, intel render, VISION_MODEL v2, tap :8101).

## 1. Recorded set (29 frames)

`evals/out/deep_m4/frames/*.png` — raw lossless gz RGB888 (the exact
`Frame` wire contract, `capture_frames.py` pattern) + JSON sidecars
`{seq, sim_stamp, pose(e,n,alt,heading), att(roll,pitch,yaw), movers
(live GzPoses truth), shot spec, mover-window result}`. Captured from
parked hovers (FlightOps.goto transits, yaw-in-place framing; mover
shots gated on live-truth range/bearing windows, ranges in the table
are TRUE measured values). `capture_set.py` (29-shot matrix), labels
hand-assigned by eye per frame (`labels.json`) with the convention:
house ⇒ building also present; gas_station ⇒ building only; car =
car_1/car_2, truck = car_3; pole = lamp posts only (the gas totem is
not a pole); frame-edge slivers excluded from recall denominators.

| zone | shots | content |
|---|---|---|
| A (49,-18,9) | a1–a3 | house_1 30 m close; house_2 54 m + oak 70 m + lamp_post 27 m; empty SE |
| B (50,-18,4.5) | b1–b4,b2b | car_1 13.6/20.4/24.0/37.2 m rear-quarter + 23.6 m front-quarter (truth-gated windows) |
| C (64,26,8) | c1–c4 | gas station 35 m; oak 46 m; car_1 28.2 m near-front; houses 76–100 m far |
| D (102,-14,4.5) | d1–d4,d2b | truck 16.7/18/22/29 m side+rear-quarter; house_1 63 m + lamp_post 76 m |
| E (50,-24,3.5) | e1–e5 | walker_1 13/17/22 m side; pine 43 m; lamp_post_1 25 m |
| F (0,0,25) | f1–f7 | gas station 100 m; house_1 69 m; pine_2 86 m + lamp_post_2 61 m + car_2 71 m + walker_2 83 m; two pure-empty FP-bait frames |

Every frame was viewed and labeled; several frames carry bonus content
(e.g. b1 also has walker_1; b3 the gas station at 85 m; f6 a car at
68 m). Two pure sky/ground empties (f3, f5) + near-empties for the FP
denominators.

## 2. Per-concept recall / FP (YOLO-World, presence-level)

Runner: `run_recall.py` — the repo DeepClient direct to the sidecar
:8100 (not the tap), vocab = slowlane vocab + movers
(`building,house,tree,pole,tower,car,truck,person`), both operating
points, raw dets in `dets.json`, overlays in `overlays/` (every FP and
mislabel below was verified by eye on the overlay).

### conf 0.05 (the shipped operating point)

| concept | recall (present) | FP (absent) | n present | n absent |
|---|---|---|---|---|
| building | 10/15 = 67 % | 0/12 = 0 % | 15 | 12 |
| house | 7/10 = 70 % | 2/18 = 11 % | 10 | 18 |
| tree | 12/12 = 100 % | 1/17 = 6 % | 12 | 17 |
| pole | 0/8 = 0 % | 0/21 = 0 % | 8 | 21 |
| car | 6/8 = 75 % | 2/21 = 10 % | 8 | 21 |
| truck | 4/6 = 67 % | 0/23 = 0 % | 6 | 23 |
| person | 9/9 = 100 % | 1/20 = 5 % | 9 | 20 |

### conf 0.10

| concept | recall (present) | FP (absent) | n present | n absent |
|---|---|---|---|---|
| building | 3/15 = 20 % | 0/12 = 0 % | 15 | 12 |
| house | 4/10 = 40 % | 0/18 = 0 % | 10 | 18 |
| tree | 12/12 = 100 % | 1/17 = 6 % | 12 | 17 |
| pole | 0/8 = 0 % | 0/21 = 0 % | 8 | 21 |
| car | 6/8 = 75 % | 1/21 = 5 % | 8 | 21 |
| truck | 4/6 = 67 % | 0/23 = 0 % | 6 | 23 |
| person | 8/9 = 89 % | 0/20 = 0 % | 9 | 20 |

### What the numbers say (all overlay-verified)

- **Movers are the strength**: car 6/6 ≤40 m (misses only at 68–71 m),
  truck 4/4 ≤29 m (misses at 66 m and 80 m — read as `car 0.41/0.14`
  instead, i.e. vehicle-vs-vehicle confusion, not loss), person 9/9
  (13–83 m, conf 0.74–0.87 up close), tree 12/12 (27–109 m).
- **Buildings/houses are the weak class, and 0.10 kills them**: house
  70 % → 40 %, building 67 % → 20 %. Confidences on the flat gray
  renders sit at 0.05–0.21 (the M1b compression, now quantified over
  15 present frames). Misses at 0.05: the gas-station canopy (35 m,
  c1 — reproduces M1b's frame-C finding), close-up dark roofs (b1's
  house_1 dead-center huge, e1/e2 — too-close flat gables don't match
  "house"), gas at 66–75 m (d1, b2b).
- **pole is blind**: 0/8 at both confs (lamp posts at 24–94 m, incl. a
  prominent 25 m fill-frame post in e5). Thin 0.6 m untextured poles
  never fire; they read as `tree` (d4, 0.13) or `tower` (d4, 0.07)
  when they read as anything.
- **FPs are rare and benign**: empty frames (a3/f3/f5/f6/f7) produced
  ZERO dets at both confs. All FPs are label-confusion on real objects,
  never phantoms on empty ground: truck→`car` ×2 (c2 0.14, d2 0.06 —
  the only car FPs), gas-station→`house` ×2 (b2b/b3 0.09 — the only
  house FPs), lamp-post→`tree` ×1 (d4 0.13), gas-front→`person` ×1
  (f1 0.06, gone at 0.10). **The M1b red-car→"person" mislabel did NOT
  recur** — b1's `person 0.80` is the real walker (tight box, overlay
  verified). The unscored slowlane-vocab `tower` fires on buildings
  (a1 0.11, d1 0.20) and once on a lamp post — an annotation-only
  annoyance worth knowing when reading cockpit annotations.
- **Operating-point verdict**: 0.10 buys almost nothing in FP
  suppression (FPs are already ~0–11 % at 0.05) and destroys
  building/house recall. **Stay at 0.05** (as shipped); 0.10 is only
  defensible for mover-only queries.

## 3. SAM mask IoU vs pseudo-GT (12 targets)

Runner: `run_iou.py` — SAM 2.1 one-shot `segment` (point and box
prompts, sidecar :8100 direct) vs pseudo-GT: buildings = the repo's own
projection of the `demo_boxes.json` footprint into the frame
(`agents/perception/projection.py` intrinsics + the sidecar's recorded
pose/attitude; convex hull of the 8 projected box corners), vehicles =
**the fast lane's own det masks** (the repo `OnnxBackend`
coco-nano-seg-v2-640 run offline on the exact recorded frame).
**Pseudo-GT caveat**: not hand-labeled GT — footprints are world-file
boxes (no facade detail, roof overhang error ~1 m) and vehicle masks
share the fast lane's own biases; IoU here measures geometric
usefulness, not absolute accuracy.

| target (range) | pseudo-GT | point IoU | box IoU |
|---|---|---|---|
| a1 house_1 (30 m) | footprint proj | 0.735 | 0.744 |
| d4 house_1 (63 m) | footprint proj | 0.734 | 0.729 |
| f2 house_1 (70 m) | footprint proj | 0.765 | 0.761 |
| e5 house_2 (25 m) | footprint proj | 0.663 | 0.548 |
| c1 gas_station (35 m) | footprint proj | 0.058 | 0.340 |
| f1 gas_station (100 m) | footprint proj | 0.218 | 0.229 |
| b1 car_1 (13.6 m) | fastlane car 0.82 | 0.707 | 0.710 |
| b2 car_1 (20 m) | fastlane car 0.90 | 0.611 | 0.601 |
| c3 car_1 (28 m) | fastlane car 0.91 | 0.628 | 0.615 |
| d2 car_3 truck (18 m) | fastlane truck 0.94 | 0.818 | 0.837 |
| d3 car_3 truck (29 m) | fastlane truck 0.94 | 0.785 | 0.791 |
| e2 walker_1 (17 m) | fastlane person 0.80 | 0.500 | 0.502 |

- **Houses: 0.66–0.77** (n=4) — the masks are geometry-grade: tight on
  the gables/roof, usable for facade centroids and cockpit silhouettes.
- **Vehicles: 0.60–0.84** (n=5; truck best 0.79–0.84) — masks sit on
  the vehicle body; box prompts ≈ point prompts (±0.02, no prompt-type
  advantage at these scales).
- **Gas station: 0.06–0.34** — SAM masks the canopy slab, not the
  20.6×30 m footprint-with-void the pseudo-GT projects; low IoU is
  substantially a pseudo-GT mismatch for open structures, but the
  honest takeaway is that pinpoint masks on the gas station outline
  the canopy, not the "building".
- Walker 0.50 — a 19 px-wide figure; mask covers torso+legs
  partially. Fine for pointing, not mensuration.

## 4. look()/pinpoint() end-to-end latency (tool entry → text result)

Harness: `run_latency.py` in-container — the real M2 tools
(`make_deep_tools`) over the injected `GzCameras` frame source and the
pilot's own env client (DEEP_PERCEPTION_URL = the :8101 tap, i.e. the
exact production path incl. its ~1–2 ms proxy hop). Warm, n=20 each,
BUSY texts excluded and counted (0 occurred in either arm):

| call | production arm (pilot + slowlane live) | uncontended arm (pilot paused) |
|---|---|---|
| look (cached vocab "truck") | p50 29.1 ms / p95 40.3 ms | p50 18.4 ms / p95 37.3 ms |
| pinpoint (point prompt) | p50 120.3 ms / p95 134.0 ms | p50 97.8 ms / p95 107.8 ms |
| look (FRESH vocab each call) | p50 35.4 ms (27.8–63.5, n=6) | p50 21.6 ms (19.8–30.2, n=6) |

Two honest findings: (1) the set_classes re-embed for a fresh
single-word vocabulary costs only ~+6 ms median over cached
(29.1 → 35.4) — M2's 30–48 ms tap numbers were multi-word fresh
vocabs, same path; (2) the pilot's pre-existing 13.5-core CPU burn
(M3 deviation 2) inflates the container-side path (frame snapshot +
b64 JSON + tap hop) by ~+11/22 ms p50 look/pinpoint — the sidecar's
GPU inference itself is unchanged (tap wire p50 23 ms during the same
window). Even at the production-arm p95, both tools are LLM-negligible.
(`latency_results.json`; the uncontended arm's first run had a 1-frame
NOT_READY warmup artifact, excluded.)

## 5. Fast-lane regression + load during a live pursuit (slowlane default-ON)

Window: 2.5 min, drone armed at 5 m by the car_1 loop, slowlane at the
shipped default (ON, 0.3 Hz), TWO click-lock pursuit engagements
(headless `w3_click.py` lock + `/api/cmd standoff` 18 m, then
`w3_session.py` re-locks: vis_car_22 15.6 s, vis_car_23 16.6 s active
pursuit). Collector: `pursuit_collect.py` (M3's ab_collect + host
load). vs the M3 A/B OFF arm:

| metric | M3 OFF arm (5.5 min) | M4 pursuit window (2.5 min) | verdict |
|---|---|---|---|
| gz RTF mean (min) | 0.921 (0.262) | 1.030 (0.674) | no degradation |
| PX4 time jumps/min | 3.26 | 4.0 | same pre-existing family |
| detector.latency_ms mean / p95 | 40.5 / 44.3 | 43.6 / 44.4 | within M3-ON jitter (42.6/44.7) |
| cam cadence | 9.76 Hz | 9.45 Hz | ~unchanged (pursuit EKF load) |
| sidecar detect p50 / p95 (tap wire) | — (0 calls) | 23 / 39 ms (n=45) | matches M3 ON arm (22/31) |
| VRAM (whole GPU) | 2636→2631 MiB | 2547→2556 MiB | flat |
| host load1 (mean) | — | 23.5 | the 13.5-core pilot burn dominates (pre-existing, M3 deviation 2) |
| slowlane counters | 263 ticks / 0 calls | +42 ticks / +42 calls / 42 ok | every tick served |

**No fast-lane regression during live pursuit with the slowlane at its
shipped default.** (The window's two RTF outliers 0.674/1.739 bracket
the mean — gz stats-reader artifacts, recorded honestly in
`pursuit_m4.jsonl`.)

## 6. Demo probe (a) — "orbit the truck at 20 meters" (S2 regression)

Fresh pilot session (post-restart, no prior chat). Evidence:
`probe_a_transcript.json`, `probe_a_timeline.jsonl` (238 rows, 2 s
cadence), `probe_a_orbit.png` (cockpit-render overlay), tap.log.

What happened (t = seconds after the command): the k3 climbed from
5 m to 16 m on its own (out of the close-range blind cone), **called
`look("truck")` at t≈41 (tap: `prompts=['truck'] conf=0.05 -> 200,
27 ms`) BEFORE committing** — the self-verification the plan was
written for — acquired the truck through the fast lane
(`vis_truck_0 conf 0.89`), transited east to the loop, and from t≈100
flew a continuous **20.0 m mean-radius circle (min 19.9, max 27.0 on
one outlier sample; full 360° coverage) around E119 N−35 for the
entire remaining ~6 minutes**, reporting at t=81:

> Located truck (vis_truck_0, conf 0.89) at E119 N-35 and completed a
> clockwise orbit at 20m radius, 16m altitude, camera on the truck
> throughout.

**Was the center real? YES — oracle-verified.** E119 N−35 lies ON
car_3's real loop (E85–120/N−40..0); the analytic trajectory
(`agents/world/trajectory.py`, sim-anchored) puts car_3 within 2–6 m
of the center during the detection window, and the first FRESH contact
ranges match the oracle position (t=54: contact 54.7 m vs
oracle-implied 56.4 m; later `vis_truck_*` rows include frozen
coasting-EKF ranges — noted, not measurement). Contrast wave-1
S2 (`demo-scenarios-2026-08-02.md`): the k3 orbited E91 N67 — a
truck-class FP contact in the gas-station area, outside car_3's route,
"orbited the phantom perfectly and never engaged the real truck".
**M4's orbit centered the REAL truck's measured position: the S2
phantom regression is fixed at the evidence level.**

Caveats, recorded honestly: (1) the orbit anchors the DETECTION point
— car_3 keeps circulating on its 50 s loop, so the orbit becomes a
static-center circle around where the truck WAS (the M2 probe (b)
aftermath pattern); the report's "camera on the truck throughout" is
an overclaim for the later arcs. (2) The cockpit `mode` still reads
raw `#21` (OFFBOARD) — the same cosmetic pre-existing enum gap S2
noted. (3) The deep lane's role was verification; the orbit itself
was flown by the fast-lane/world path, as designed.

**Verdict: PASS** — look()-before-commit demonstrated; the center was
the real car_3 (not a phantom); radius error ≤0.5 % of the 20 m
request.

## 7. Demo probe (b) — "face house_1, then pinpoint on the house facade"

Same pilot session, immediately after probe (a). Chat: *"face house_1,
then use pinpoint on the house facade and tell me what you see"*.
Evidence: `probe_b_transcript.json`, `probe_b_timeline.jsonl` (208
rows), `probe_b_mask_payload.json` (the full /pilot/deep payload),
`probe_b_mask_render.png` (mask re-drawn on the held viewpoint),
`probe_b_scene.png`, tap.log.

What happened: the k3 repositioned to (108.5,−17.8, 16 m), turned to
yaw 242.5° — the timeline's facing error vs house_1's true bearing is
**±0.1° and held for 6+ minutes** — then called
`look("house,building,window,roof")` (tap 14:40:40, conf 0.05), then
`pinpoint(322,261)` (tap 14:40:44, seq 16540, 101 ms), and reported:

> Faced house_1 (heading -117°). Deep look detected the house (conf
> 0.25) ~63m ahead at E53 N-46, matching house_1's known position
> (E50 N-48). Pinpoint segmented the facade: mask centered ahead in
> frame at pixel (322,261), tight box [258,220,406,309], area ~8.8k
> px, mask score 0.95 — a clean, well-defined house front centered in
> view. No people or vehicles visible around it.

Verification: the look's ground_intersection put the house at
E53 N−46 vs true E49.6 N−48.4 (3.6 m error over the true 66.4 m
range — the visible-bottom ray projection working as designed), and
the k3 explicitly cross-checked it against the known map instead of
trusting the deep output (the advisory discipline). The mask payload:
`{cls: "house"` — **label pairing via the look() seed worked** —
`area 8843 px, score 0.955, centroid (326.5,269.4), 5.5 px from the
prompt point, box-local RLE 262 B}`, and the render shows the
silhouette sitting tightly on the house's roof/facade
(`probe_b_mask_render.png`). The building-aware op is real: face →
look (cross-checked advisory) → pinpoint (label-paired mask) →
accurate plain-language description.

Caveat: the report says "facade"; from this angle the mask covers the
near roof slope + gable as one silhouette (a sim box-house has no
facade texture for SAM to separate) — semantically close enough at
advisory level, noted for completeness.

**Verdict: PASS.**

## 8. Ops notes (the arming incident)

Between ~16:00 and ~17:45 the box refused SITL arming: preflight
WARN spam (`Yaw estimate error`, `GPS H/V Pos Drift too high`,
`height estimate not stable`, `vertical velocity unstable`) at 2–3/min,
recurring across TWO fresh container instances. Diagnosis evidence:
`estimator_status_flags` showed `cs_yaw_align/cs_gps` solidly TRUE for
hundreds of consecutive seconds while arming was still refused;
mavsdk `health_all_ok` flipped TRUE in quiet windows; the failures
correlated with the `time jump detected` timesync churn, which
correlated with host load ~19–22 + ~5 GB swap pressure. Arming
succeeded instantly when the load collapsed — and the durable recipe
turned out to be: **`action.arm()` FIRST (catches a clean EKF window),
then `action.takeoff()`** — 9/9 blind `takeoff()` attempts failed while
arm-then-takeoff worked on the first calm-window try. M3 hit the same
family as a 15-min post-boot stall; this episode lasted ~90 min under
heavier memory pressure. Not a product-code bug — nothing in the repo
changed between the working M3 state and the failing state — but
RUN-DEMO's "host load" warning deserves this sharper note (see
§RUN-DEMO touch-up below).

## Deep perception acceptance summary (M1→M4)

**What the system can now do (measured):**
- Name movers from chat/tools reliably: car ≤40 m (6/6), truck ≤30 m
  (4/4), person ≤83 m (9/9), tree ≤109 m (12/12) at conf 0.05;
  self-verification with look() before committing to an op demonstrated
  live twice (M2 + M4 probe a).
- Annotate statics at 0.3 Hz into the cockpit with ≤0.5 s frame-age
  expiry (M3) — building/house annotations are advisory-grade (67–70 %
  recall, ~0 % phantom FP on empty scenes).
- Mask buildings and vehicles on demand (pinpoint): house IoU
  0.66–0.77 vs footprint projection, vehicle IoU 0.60–0.84 vs the fast
  lane's own masks; masks publish to the cockpit (box-local RLE,
  frame_seq-keyed).
- Answer in LLM-negligible time: look p50 29 ms / pinpoint p50 120 ms
  end-to-end through the production path (18/98 ms uncontended).
- Coexist with the 10 Hz fast lane: no detector latency/cadence/RTF/
  PX4 degradation during a live pursuit with the slowlane ON (matches
  the M3 A/B GREEN decision).

**What it still can't do (quantified):**
- Buildings at conf ≥0.10: recall collapses to 20–40 % (flat-render
  confidence compression) — the sidecar must stay at 0.05, and deep
  building outputs remain ADVISORY.
- Poles: 0/8 — no lamp-post detection at any tested conf; the slowlane
  vocab's `tower` partially backstops this (fires on some posts and on
  buildings, annotation-only).
- The gas-station canopy: never a `building` even at 0.05; pinpoint
  masks outline the canopy slab, not the station (IoU ≤0.34 vs
  footprint pseudo-GT).
- Close-up (≤30 m) dark roofs: missed as house/building (b1/e1/e2) —
  the detector wants whole-structure context.
- Trucks ≥66 m and cars ≥68 m: out of reach (trucks demote to `car`).
- FP-suspect advisory: still never triggered in a natural run
  (M3 honest-zero stands; the path is unit-pinned only).

**Recommended operating points:** conf **0.05** for all deep detect
paths (as shipped: look default, slowlane DEEP_SLOWLANE_CONF); 0.10
only for mover-only queries where its marginally cleaner output helps;
SAM prompts: box ≈ point (±0.02 IoU) — use whatever's handy; treat
pinpoint masks on open structures (canopies) as "the slab", not the
footprint; **pinpoint points: aim at the object, low-center beats
upper-frame** — M2's heuristic confirmed: an upper-frame point on an
empty scene grabbed a degenerate 100k-px sky mask (latency harness),
while probe (b)'s object-centered point scored 0.955; deep outputs
stay advisory, the fast COCO lane stays the mover authority (unchanged
contract).

## RUN-DEMO touch-up

`docs/RUN-DEMO.md` prereqs: the EKF/load note was stale in both
directions — today's refusal episode happened at load ~20 + swap
pressure (the note said ≥ ~30) and had a working recipe the note
lacked. Updated in place: the sharper trigger condition and the
**`arm()` → `takeoff()`** recovery sequence.
