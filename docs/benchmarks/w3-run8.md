# W3-run8 — validation verdict (demo world, v2 + R8 edge barrier + instrumented harness, 2026-08-02)

**Scope.** ONE run + ONE fresh-container diagnostic repeat at the codex R8
acceptance shape (R8 image-edge barrier live, suite 638 green; v2 via
explicit `VISION_MODEL` override, env-verified both containers), with the
R8-mandated harness instrumentation built FIRST: `w3_cornerwatch.py` now
logs synchronized PX4 alt + gz-truth z of `x500_depth_0` at 10 Hz, and
`w3_session_verdict.py` scores **INVALID_ENV** (`|px4_alt − (gz_z + b)| >
1.5 m for ≥0.5 s`, b = stable-start offset) BEFORE alt/gap scoring —
INVALID_ENV rows contribute no pass/fail samples. **No production code
changed** (no integration bug surfaced). Suite untouched.

**Verdict in one line:** **PASS-with-caveats.** The R8 centerpiece —
ENDURANCE — passed on the retry: **one contiguous engagement of 64.8 s
covering all 4 corners of a full car_1 lap with the same contact id and
MEASURED within 2 s of every corner** (run 7's best was 2 corners, run 6's
zero). Estop mid-track confirmed (`tool cancelled: True`). The named
caveats: gap band 72 % (<80 %), Back-off 18 unmeasured (track died before
posting), LOST→re-lock 35.1 s (lap-window-limited), the pursuit op's 60 s
cap means the last ~4.8 s of the 64.8 s engagement was designation
persistence, and attempt 1 was environmental (camera-stall + a ~28° view
twitch the 12° tilt cap was already set for — cause unresolved).

## 1. Per-step verdicts (retry = substantive attempt)

| Step | Attempt 1 | Diagnostic repeat |
|---|---|---|
| Click → lock (≤2 s dispatch) | 200 ×1 (session) | **PASS** — 7× 200; dispatch ~1.5 s (lock t=2.0→ACQUIRING 3.5; 55.7→57.2) |
| ENDURANCE ≥55 s, ≥4 corners, 1 id | FAIL (10.6 s) | **PASS — 64.8 s, 4 corners, `vis_car_16`, zero churn** |
| 3 clicks ≤2 s | ✗ | **PASS** (≥5 clicks at ~1.5–2 s) |
| release/re-click ≤8 s | — | LOST→re-lock **35.1 s ✗** (lap-dictated; window-gated); stop→resume re-engage **✓ ~5 s** |
| Approach 14 | — | POST 200 ✓ (gap 15.8; track died ~12 s in — convergence unmeasured) |
| Back-off 18 | — | **✗ not posted** (track dead at post time) |
| Orbit 15/8, 30 s | — | ~34 s OFFBOARD; radius mean **16.7 m**, 29/34 samples in 15±4 — window flagged **INVALID_ENV** (below) |
| Stop → hold | — | **PASS** (HOLD, spd 0.1) |
| Resume → re-engage | — | **PASS** (OFFBOARD ≤5 s, same contact) |
| Estop mid-track | — | **PASS** — `estop: drone_0 HOLDING (estop) (tool cancelled: True)`; PRE OFFBOARD/ACQUIRING → POST HOLD spd 0.3 |
| alt 3.7–4.5 (valid rows) | — | **98/98 ✓** (mean 3.94) |
| gap 12–20 ≥80 % | — | **72 % (43/60) ✗** — corner-recovery swings to 27.6 m |
| no measured <11 m | — | **PASS** (min 11.7) |
| zero id churn | — | **PASS** within engagements |
| ToF | — | OUT-OF-ENVELOPE/NO-RETURN/SEARCHING; zero LOCKED (expected at 4 m — VISION LOCK gate) |

## 2. Corner table (retry endurance engagement, cornerwatch gz-truth × autopsy health)

| Corner | Crossing (sess t) | Outcome |
|---|---|---|
| B SE (70,−30) | ~59.3 | MEASURED through crossing (first coast +2.6 s), recovered MEASURED +4.0 — **pass, marginal** |
| C NE (70,30) | ~74.9 | MEASURED through; 0.3 s flicker; MEASURED +2.9 — **pass** |
| D NW (30,30) | ~85.5 | MEASURED through; 0.5 s flicker; MEASURED +1.7 — **pass, clean** |
| E SW (30,−30) | ~101.4 | MEASURED through, no coast in window — **pass** |

Engagement span: autopsy t=97.4 (ACQUIRING) → 162.2 (LOST) = **64.8 s**.
Detector presence in the window: 74 % of 10 Hz cycles (v2; run 5 was 6 %).

## 3. INVALID_ENV analysis (R8-mandated)

b = +2.03 m (stable-start offset, first 30 ENV rows). **One INVALID_ENV
window: cornerwatch t=405.8–499.9 (|dev| > 1.5 m sustained)** — the
ops/orbit era, where /state alt read 4.6–5.0 while gz-truth z disagreed.
Those rows contribute no pass/fail samples: the orbit's alt-band breach is
environmental, and its radius numbers (CV-range, same z input) are reported
raw-with-caveat. Excluding the window, alt scored 98/98 in-band. Attempt
1's gap-collapse window was checked the same way — **no** INVALID_ENV
(deviation ≤0.4 m throughout; the 17.9→8.0 m EKF collapse there was a view
twitch, not an altitude-estimate excursion).

## 4. Attempt-1 post-mortem (environmental)

61 s of the 100 s session had zero car dets (detector-empty through a full
south-leg transit — camera/renderer stall signature; the camera read
healthy at 9.7 Hz pre/post). The one engagement died at 10.6 s: a ~28°
view swing at lock+1–2 s (det boxes swept rows 292→−4→356 coherently)
ghosted the designated EKF to 8.0 m while the true car stayed ~17–20 m
(double-birth `vis_car_5` at truth; readoption radius 8 m < 10 m → LOST).
MPC_TILTMAX_AIR read 12.0 post-hoc (the tune had landed), so the swing was
either inside the pre-tune window or a controller transient — cause
unresolved, reported honestly.

## 5. Notes, fixes, LLM-free

- **Fixes: NONE** (no production diff; evidence tooling only:
  instrumented cornerwatch, INVALID_ENV verdict scoring, roll/pitch in
  timeline rows, `w3_preflight.py`, `w3_reposition.py`).
- LLM-free: CONFIRMED — all pilot.logs boot-lines only (7 lines); every
  action via FlightOps scripts + `/api/lock` + `/pilot/cmd` + `/pilot/estop`.
- Estop-path caution for future runs: `ros2 topic echo --once /pilot/chat`
  replays STALE latched history — liveness must be checked with a live
  echo window, or the estop looks dead when it is not (as it briefly did
  here).

## 6. Evidence

`evals/out/w3_run8/`: attempt 1 — `session.log`, `timeline.log`,
`session_timeline_echo.log`, `autopsy_attempt1.log`, `detwatch_attempt1.log`,
`cornerwatch_attempt1.log` (ENV rows), `pilot_attempt1.log`. Retry —
`retry/session.log`, `retry/session_timeline_echo.log`,
`retry/ops_timeline_echo.log`, `retry/ops_click.log` (7 locks, all ops
POSTs), `retry/autopsy_retry.log`, `retry/detwatch_retry.log`,
`retry/cornerwatch_retry.log` (instrumented ENV log), `retry/pilot_retry.log`,
`retry/shot_final.png`. Tooling: `w3_session.py` (+begin-wall, pick-failure
forensics), `w3_session_verdict.py` (INVALID_ENV), `w3_cornerwatch.py`
(instrumented), `w3_preflight.py`, `w3_click.py`, `w3_position.py`,
`w3_reposition.py`, `w3_ops.py`, `w3_timeline.py`, `w3_capture.py`,
`w3_autopsy.py`, `w3_detwatch.py`, `w3_verdict.py`.
