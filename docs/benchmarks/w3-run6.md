# W3-run6 — validation verdict (demo world, v2 detector, codex R4 gate, 2026-08-02)

**Scope.** FIRST live validation with the fine-tuned detector
(`models/coco-nano-seg-v2-640.onnx`, sha256 f7007721da1d…, pursuit-vehicle
recall 94.3 % qualified at 10–22 m slant) via EXPLICIT
`VISION_MODEL=coco-nano-seg-v2-640.onnx` env override (NOT promoted in
`run_single_demo.sh` — the script's `${VISION_MODEL:-…}` already lets the
env win; verified reached: container env + pilot process environ both v2).
ONE run + ONE fresh-container diagnostic repeat, then stop. Gate: codex R4
numbers (4 m staging, session structure, orbit 15/8, stop/resume, estop).
**No production code changed** — no integration bug surfaced; the residual
failure is in the tracker's corner model (§3). Suite untouched (623+ as
handed off).

**Verdict in one line:** **W3 FAIL on the session-structure counts — but
the v2 detector works** (live vehicle-det presence **67 %/40 %/95 %** vs
run 5's ~6 %), and it UNMASKED the true residual: the CV-EKF's
constant-velocity ghost through the mover's 90° corner — the association
gate rejects the turned measurements while the detector delivers a car det
at conf 0.72–0.93 in **every** cycle. All three engagements died 2–5 s after
the SE corner (70,−30), never from detector drought.

## 1. Per-gate verdicts

| Gate | Attempt 1 (v2, fresh container) | Diagnostic repeat (v2, fresh container) |
|---|---|---|
| Setup (4 m, 14–18 m) | (47.0,−13.2) facing S, 16.8 m to leg | (49.9,−14.0) facing S, 16 m |
| 1. Click → lock | **PASS** — 200 `vis_car_3` | **PASS** — 200 `vis_car_3` |
| 2. Session structure | **FAIL** | **FAIL** |
| — engagements ≥3 | 2 ✗ | 1 ✗ |
| — each ≥20 s | [14.6, 13.1] ✗ | [15.1] ✗ |
| — aggregate ≥60/90 s | ~28 % ✗ | ~16 % ✗ |
| — re-lock ≤8 s | 43.2 s ✗ (lap-dictated) | none (no window) ✗ |
| — alt 3.7–4.5 | engagement rows in-band ✓ (parking tail drifted to 4.6) | engagement rows 3.8–3.9 ✓ (HOLD tail drifted to 3.5) |
| — gap 12–20 ≥80 % | **100 %** (25/25, mean 14.8) ✓ | strict 5/14; **13/14 within 6 mm** (frozen coast reads 11.994) — reported honestly |
| — no measured sample <11 m | **PASS** (min 12.8) | **PASS** (min 12.0) |
| 3. Orbit 15/8 | not run (blocked) | not run (blocked) |
| 4. Stop → resume | not run (blocked) | not run (blocked) |
| 5. Estop | not run (blocked) | not run (blocked) |

Re-lock mechanics: every post-LOST click that had a window returned 200 and
re-engaged — the structural path works; the 50 s lap caps window
availability (43.2 s / none).

## 2. Recall: run 5's 6 % → v2 live

- Attempt 1 engagement windows: vehicle-det present **67 %** (76/113) and
  **40 %** (42/104) of 10 Hz cycles.
- Retry engagement window: **95 %** (122/128) — the qualified 94.3 %
  showing up live; death window (last 6 s) **53/53 cycles** with a
  `truck` 0.72–0.93 det. Longest drought in the retry engagement: **0.0 s**
  (offline miss-streak ≤2.0 s confirmed live).
- Flicker/readoption: id churn 0 across all engagements; ToF zero LOCKED
  (OUT-OF-ENVELOPE/NO-RETURN/SEARCHING — expected at 4 m).

## 3. Mechanism — the corner ghost (newly isolated, association layer)

All three engagements died at the same locus: the car rounds the SE corner
(70,−30), eastbound → northbound, 2–5 s before the LOST:

1. The CV-EKF predicts constant-velocity through the turn; the real
   measurements swing away and the NN/NIS association gate starts rejecting
   them — while the detector keeps delivering (52 % / 95 % presence in the
   death windows). Track → COASTING at the ghost point.
2. The drone coast-holds; the car recedes along the new leg at 4 m/s. Within
   seconds the re-capture geometry is gone: boxes shrink/recede toward the
   horizon (`[388,235,434,251]` → `[334,151,374,166]`), the ground-plane
   projection degrades, measurements turn bearing-only (attempt 1: range
   `None` at coast onset; double-birth `vis_car_4` orphaned bearing-only —
   readoption requires a POSITIONED candidate, so it cannot fire).
3. 5 s grace expires with zero association → LOST at 13.1–15.1 s.

This is NOT the run-5 recall problem (solved by v2), NOT the R2/R3 geometry
(ring held 12–17.7 m, zero sub-11), NOT the R4 yaw (prediction-steered).
It is the tracker's corner model: the R4 note's own caveat — "a
constant-velocity ghost is fiction through the mover's 90° corners" — now
measured live. Round-7 candidates: turn-rate-aware process-noise inflation
on the designated track, a corner re-capture path that widens the
association gate for the designated target on fresh v2 dets (the superclass
map already covers the observed car↔truck flap), or readoption extended to
bearing-only candidates with a bearing-fan search.

## 4. Evidence

`evals/out/w3_run6/`: attempt 1 — `session.log`, `timeline.log`,
`session_timeline_echo.log`, `autopsy_attempt1.log` (the double-birth
window), `detwatch_attempt1.log`, `pilot_attempt1.log`. Retry —
`retry/session.log`, `retry/session_timeline_echo.log`,
`retry/autopsy_retry.log`, `retry/detwatch_retry.log` (the 53/53 death
window), `retry/pilot_retry.log`. LLM-free CONFIRMED both (pilot.logs
boot-lines only). Tooling: `w3_session.py` (+`--align`),
`w3_session_verdict.py`, `w3_click.py`, `w3_position.py`, `w3_reposition.py`,
`w3_ops.py`, `w3_timeline.py`, `w3_capture.py`, `w3_autopsy.py`,
`w3_detwatch.py`, `w3_verdict.py`.
