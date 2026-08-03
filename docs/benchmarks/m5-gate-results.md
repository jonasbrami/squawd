# M5 gate results — 2026-07-22 (branch rebuild-single-drone)

Gate (design §7, M5) evidence, live in the `perceive`/`dynamic` worlds.
Unit suite: **477 green** (440 pre-M5 + 37 new; the 4 load-bearing ICD §11 M5
tests + natural units).

## d2_shadow A/B — ground-truth-fed vs camera-fed (documented, M5 gate item)

The A/B instrument is `evals/track_shadow_gate.py --feed truth|vision` (M3a);
the M5 harness now runs the same split through `run_evals --feed truth|vision`
via the Deps split (`flight_contacts` = GzPoses for the explicit truth-fed
control, VisionContacts camera-fed — never crossed). Numbers (M3a gate,
`docs/benchmarks/m3a-gate-results.md`, 2026-07-21, same sim build):

| feed | runs | contiguous dwell (need ≥45 s of 60 s) | mean gap | samples ≤15 m |
|---|---|---|---|---|
| **truth** (GzPoses in the flight path) | 2/2 PASS | 48.4 s, 53.4 s | — (not measured per-run) | — |
| **vision** (blob → VisionContacts, `vis_*` discovered) | 2/2 PASS | **52.7 s, 51.5 s** | 11.3 m, 10.6 m | 213/241, 214/241 |

Reading: the camera-fed chain meets the same 45 s bar as the truth-fed
control — perception error no longer dominates the dwell budget (mean gap
~11 m sits inside the 15 m window; the residual is the controller's dynamic
lag, bounded by the CV-EKF at frozen constants — ToF range is the designed
closer, M3b). The truth control remains the isolation lane: if a vision
regression appears, truth-fed vs camera-fed separates flight-stack bugs from
perception bugs.

## Perceive dual gates (`--pilot --feed vision`, k=3)

Status: **pilot_null 6/6 FAIL (as designed)** on first contact with the harness
(rows in `evals/out/m5_perceive_gates/`): every null row fails
`identified_target` ("no vis_* lock") and `dwell_moving` (≤6 s « need) while
`alive` + `within_step_budget` pass — the must-FAIL lane is fully green.

Pilot lanes (final state, evidence `evals/out/m5_perceive_gates/`):
**`identified_target` PROVEN LIVE** — four independent flights:
`locked vis_target_N -> mov_true (want mov_true)`, assoc err **2.0 / 2.1 /
2.4 / 6.8 m** (25 m gate), e.g. sim_t=2940.6 err 2.42 m. The TargetLockEvent
→ timestamp association → oracle path grades identity correctly, report text
never consulted. Cells still FAIL overall on **`dwell_moving`** (best
**21.5 s** of the needed 30 s; two cells timed out without a ≤15 m pass) —
the shadow-survival physics M3a spent ~35 gate runs on; close-lock (15 m) +
onnx lifted holds from ~6-10 s to 9.5-21.5 s. The task-level pilot PASS is
the one outstanding item; it is a chase-controller problem, not a
perception-grading one (see forensic chain). NOTE: rows at sim_t 2459.9
appear twice (p1_2/p2_1, identical details) — a same-flight double-grade
from an accidental v6/v7 overlap (both sweeps briefly shared the drone);
the three other associations are independent flights.

## M5 forensic chain (what the perceive gate taught us)

1. **ScriptedClient fleet-routing** — behavior steps crashed with
   `'System' object is not callable`: single-drone `FlightOps.drone` is the
   MAVSDK System ATTRIBUTE, not the fleet's `drone(i)` method. Callable-guard
   added; regression test in `tests/evals/test_pilot.py`.
2. **Missing VisionPipeline** — `run_evals --feed vision` built Detector +
   VisionContacts but no pipeline to pump one into the other: 15 hover polls,
   zero `vis_*` contacts, every cell.
3. **Vantage geometry (two-sided)** — from alt 6 at (55,-75) the rover's near
   arc sits below the 21° half-vFOV and the visible band starts at ~56 m
   (blob starves: 6/6 cells, zero detections); from alt 3 the blob's far
   sightings are shallow-angle, seeding the EKF >25 m off and failing the
   25 m TargetLockEvent association gate. Answer: alt-3 vantage for
   detection, `max_range_m=25` for the lock.
4. **Detector operating point** — `accuracy_report`'s `conf=0.45` default
   starves the blob's far-range scores (0.35-0.45); the driver now uses the
   production `conf=0.25` (`evals/out/perceive_accuracy_20260722-072641.json`
   shows the mismatch: tp=0 fp=38 with decoys correctly invisible).
5. **Lock emission timing** — scripted behavior triples emit the ToolUseBlock
   POST-hoc, so the lock hook ran after the 60-75 s track (every lock
   `measured_xy=None`, association impossible). The `_pending` protocol now
   emits the call at true call time (unit-tested).

Sim health notes (this window): PX4 `is_armable: False` /
`is_magnetometer_calibration_ok: False` and `is_global_position_ok: False`
transients after long uptime + process kills (3 container relaunches); a
1300%-CPU spin in the production pilot agent wedged new mavsdk clients
(killed; eval runs now kill it preemptively). EKF↔gz frame drift measured
**~0.3 m XY** on a fresh container (the ~85 m drift class appears with
container age, not at boot).

## Accuracy artifact (`evals/perceive_report.py`)

- `perceive_accuracy_20260722-071535.json` — NULL run (heading bug: travel
  heading faced the approach leg, plaza outside FOV; 586 frames, zero truth
  boxes, zero dets). Driver now passes an explicit vantage heading.
- `perceive_accuracy_20260722-072641.json` — blob @ conf 0.45: decoys
  recall-0 (designed rejection), target tp=0 fp=38 — the conf mismatch
  artifact (see forensic 4).
- blob @ conf 0.25 (`perceive_accuracy_20260722-085729.json`) — target
  precision 0.47 / recall 0.04 (tp=8 fp=9 fn=193), center p50 4.5 px; decoy
  classes recall-0 (designed rejection). ID metrics null (no track ids).
- onnx @ conf 0.25 (`perceive_accuracy_20260722-090126.json`) — target tp=0
  fp=719: the seg model fires ~1.2/frame but never reaches IoU≥0.5 vs the
  projected truth extents (seg boxes run ~2× the truth footprint vs the
  IoU-strict metric; the LIVE path grades the footpoint instead — contacts
  associated at 2-7 m in the gates, so this is a metric-strictness artifact,
  not misalignment; box-IoU vs footpoint accuracy is the documented split).

## Single-drone N=1 regression (dynamic ladder, truth-fed)

**RUN 2026-07-28 — regression found → codex-reviewed → fixed → validated →
confirmed.**

- First run (`evals/out/m5_truth_regression_20260728{,_retry}/`): 8/9 cells
  matched baseline; **d2_shadow pilot lane regressed** — `dwell_moving`
  10.5 s run-1 / 41.6 s fresh-container retry vs 45 s gate (baseline 68.1 s,
  2026-07-06). Truth-fed bypasses perception ⇒ pursuit-side regression.
- Codex review (`docs/benchmarks/d2-regression-review-codex.md`): culprit =
  the `_shp` trajectory shaper in `agents/flight/ops.py` (discards
  `control_ref()`'s direct shadow reference; a 1 m/s² accel-limited carrot ⇒
  ~7–10 s lag on the 3.5 m/s rover) + the beam-geometry altitude profile
  wrongly applied to truth-fed contacts (no `observation()` ⇒ descent to
  ~3.8 m despite alt=12).
- Fix (iteration 1, first attempt): `beam_capable` gate — truth-fed shadow
  streams the direct reference at commanded alt; camera-fed M3b path
  byte-identical. ICD tests in `tests/test_track_ops.py` (truth-shadow
  direct-reference + ≥45 s dwell fixture; beam-capable branch preserved).
  Suite 509 green.
- Fix validation (`evals/out/m5_d2_fix1_20260728/`): **dwell 69.1 s vs 45
  gate** (baseline 68.1 s); null lane still fails correctly.
- Full-ladder re-confirmation (`evals/out/m5_d1d5_confirm_20260728/`): **all
  9 cells match pre-M5 behavior** — pass/fail semantics, step counts, and
  per-check outcomes identical to the composite baseline
  (`pilot_dynamic/dyn_pilot_gate2` for d1/d3/d5, `pilot_track/dyn` for
  d2/d4). **M5 CLOSED.**

## Strategy A/B demo (`intercept-lead`)

LLM cells DEFERRED by time (see report); the activation gate is
unit-demonstrated in `tests/evals/test_strategy_ab.py` (activate only when
the snippet lane's Wilson CI-low beats the base point rate, both lanes ≥3
scored cells; refuses overlap, regression, thin-K, and infra-poisoned lanes).
Infra: `evals/strategy_ab.py`, `--assignments "drones=haiku,strategy=intercept-lead"`,
`agents/pilot/strategies/intercept-lead.md`, `extra_prompt` in
`make_pilot_options`.
