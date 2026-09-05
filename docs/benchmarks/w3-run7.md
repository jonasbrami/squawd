# W3-run7 — validation verdict (demo world, v2 detector + R7 maneuver mode, 2026-08-02)

**Scope.** ONE run + ONE fresh-container diagnostic repeat of the W3 golden
path with the R7 designated-track corner-maneuver mode live (suite 635 green;
COCO-vehicle-only, arms on sign-consistent lateral innovations, Q×25,
widened re-capture gate) and the v2 detector via explicit
`VISION_MODEL=coco-nano-seg-v2-640.onnx` (override verified in container +
pilot environ; not promoted). Gate: R4 session gate + R7 corner sub-gate
(one contiguous engagement survives a full ~50 s lap, ≥4 corners, same id,
MEASURED ≤2 s after each). **No production code changed** — the failures
below are one confirmed R7-success + one environmental glitch + one
visibility loss, no integration bug. Suite untouched.

**Verdict in one line:** **W3 FAIL on the structure counts and the corner
sub-gate — but with the first real corner survival on record.** The R7
maneuver mode worked where it was allowed to: engagement 1 rode the SE
corner with a 0.5–1.0 s flicker and re-MEASURED at ~2.0 s (run 6 died
there every time). The lap attempt then died at the NE corner to an
environmental PX4 z-estimate excursion (drone dipped to ~0.4 m at speed),
and engagement 2 died at the SW corner with the car simply leaving the
frame (dets=0 for 5+ s). Attempt 1 produced zero locks to an environmental
CV-range inflation of that boot (image content ~8° high; 36.8 m reported
for a 16 m car — did not reproduce in the retry boot, sanity-checked).

## 1. Per-gate verdicts (retry = the substantive attempt)

| Gate | Attempt 1 | Diagnostic repeat |
|---|---|---|
| Setup (4 m, 14–18 m) | (49.9,−13.9) facing S, 16 m | (50.0,−14.0) facing S, 16 m; pre-session sanity PASS (range 13.8–15.5 m, box rows ~305) |
| 1. Click → lock | **FAIL** — zero clickable contacts all session (CV ranges inflated ~2.3×; box bottoms ~70 px high vs geometry) | **PASS** — 200 `vis_car_4`; **re-lock 200 `vis_car_5` at 6.5 s ≤8 ✓** |
| 2. Session structure | FAIL (0 engagements) | **FAIL** — 2 engagements (≥3 req) |
| — each ≥20 s | — | **[28.1, 20.6] ✓ both** |
| — aggregate ≥60/90 s | 0 | 48.7 s (~52 %) ✗ |
| — re-lock ≤8 s | — | **6.5 s ✓** |
| — alt 3.7–4.5 | — | 83/88 rows ✗ — the z-excursion (−0.3) |
| — gap 12–20 ≥80 % | — | 61 % (28/46; mean 14.2) ✗ |
| — no measured <11 m | — | **VIOLATED** — 12 samples 8.1–10.7 m (SE-corner undershoot + sag window) |
| 3. Corner sub-gate (full lap, ≥4 corners, 1 id) | FAIL | **FAIL** — best: 2 corners in one id |
| 4. Orbit 15/8 | not run | not run (blocked) |
| 5. Stop → resume | not run | not run (blocked) |
| 6. Estop | not run | not run (blocked) |

## 2. Corner-by-corner table (retry; cornerwatch gz-truth × 10 Hz autopsy health)

| Corner | Truth crossing | Engagement | Outcome |
|---|---|---|---|
| C1 SE (70,−30) | sess ~8.7 s | eng1 `vis_car_4` | **SURVIVED** — MEASURED→COASTING 0.5 s→MEASURED at ~2.0 s post-corner (marginal ≤2 s); the maneuver recovery run 6 never had. Gap undershot to 8.1 m during the chase (R_guard=13 — the R3 barrier did not hold the corner either; covered by v2 recall + maneuver) |
| C2 NE (70,30) | sess ~25.0 s | eng1 `vis_car_4` | **FAILED** — coincident PX4 z-estimate excursion: /state alt 2.6→**0.4/−0.3**→3.4 at 2–6 m/s near (84,26); CV projection rejected (gap froze 8.55), coast, LOST at grace (engagement total 28.1 s). Same environmental family as the 2026-08-01 z-drift incident; PX4 log shows no EKF event at default verbosity |
| C3 SW (30,−30) | sess ~51.3 s | eng2 `vis_car_5` | **FAILED** — car left the frame outright: healthy 0.82–0.9 car dets → `dets=[]` for 5+ s (no conf flicker, no FPs); LOST at grace (20.6 s). Visibility/yaw layer, not association |

## 3. Flicker/readoption/maneuver, recall, ToF

- Id churn: 0 across both retry engagements (ids held through LOST).
- Maneuver arms: not exposed in the snapshot; inferred = **1 corner
  recovery observed (C1)** — the first in six runs.
- Recall (v2): vehicle-det presence 67–95 % of cycles in engagement windows;
  C3's zero-det window was geometric (out of frame), not recall.
- ToF: zero LOCKED — OUT-OF-ENVELOPE/NO-RETURN/SEARCHING (expected at 4 m).
- LLM-free: CONFIRMED both attempts (pilot.logs boot-lines only, 7 lines).

## 4. Mechanism read (new vs recurring)

C1 proves the R7 layer works when the environment lets it. The two
remaining death modes are NOT the corner ghost: (a) the sim's PX4
z-estimate stability (environmental — struck mid-corner at C2; a
validation-harness hardening question, e.g. gz truth watchdog in the
session driver), and (b) pursuit-yaw visibility at corners (C3 — the nose
didn't keep the turning car in frame; whether that's the predicted-bearing
horizon cap or ring geometry at the SW turn needs the next design round).
The gap-band violations (8.1–10.7 m) trace to the C1 corner undershoot of
the R3 barrier + the sag window — reported, not excused.

## 5. Evidence

`evals/out/w3_run7/`: attempt 1 — `session.log` (zero locks, all-IDLE),
`timeline.log`, `autopsy_attempt1.log` (36.8 m inflated range),
`detwatch_attempt1.log`, `cornerwatch_attempt1.log`, `pilot_attempt1.log`.
Retry — `retry/session.log`, `retry/session_timeline_echo.log` (timeline
clock +40 s vs session, noted), `retry/autopsy_retry.log`,
`retry/detwatch_retry.log`, `retry/cornerwatch_retry.log`,
`retry/pilot_retry.log`, `retry/shot_final.png`. Tooling: `w3_session.py`
(+pick-failure forensics), `w3_session_verdict.py`, `w3_cornerwatch.py`,
`w3_click.py`, `w3_position.py`, `w3_reposition.py`, `w3_ops.py`,
`w3_timeline.py`, `w3_capture.py`, `w3_autopsy.py`, `w3_detwatch.py`,
`w3_verdict.py`.
