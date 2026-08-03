# W3-run4 — validation verdict (demo world, codex R3 gate, 2026-08-02)

**Scope.** ONE validation run + ONE fresh-container diagnostic repeat of the
W3 golden path against the R3 corner fix (camera-fed `hold_altitude` shadow on
the direct lane + measured-bearing yaw + radial escape barrier at
R_guard=R_min+1=19 m + lock ring R_min+2=20 m; suite 604 green at handoff).
Codex R3 gate at 6 m: click → 90 s no-LOST (alt 5.5–6.7, gap 18–26 ≥80/90 s,
no measured gap <17 m) → orbit 20/8 → stop/resume → estop. Protocol: stop
after the repeat regardless. **No production code changed** — no integration
bug surfaced; the residual failure is a design/perception issue (§3).
Evidence tooling only: `w3_detwatch.py` added (raw det trace).

**Verdict in one line:** **W3 FAIL on gate 2 duration — but the R1–R3
geometry circle is CLOSED.** Both engagements died at ~11 s with **geometry
fully legal** (zero sub-17 m samples, gap means 24.1/24.8 m, no corner cut —
the barrier never needed to fire). The residual killer is NEW and in the
perception/yaw layer: detector confidence dips at pursuit onset + the
coast-phase yaw freezing on the stale measured bearing, letting the car walk
to the frame edge where its clipped boxes can't re-associate inside the 5 s
grace (§3, det-watch evidence).

## 1. Per-gate verdicts

| Gate | Attempt 1 (fresh container) | Diagnostic repeat (fresh container) |
|---|---|---|
| Setup (6 m, 25–30 m) | (46.9,−4.7) facing S, 25 m to south leg | (50.0,−5.0) facing S, 25.3 m to south leg |
| 1. Click → lock | **PASS** — 200 `vis_car_3` (attempt 13, boresight gate) | **PASS** — 200 `vis_car_3` (attempt 12) |
| 2. Pursuit 90 s | **FAIL** — LOST at **11.6 s** | **FAIL** — LOST at **10.9 s** |
| — no LOST for 90 s | ✗ | ✗ |
| — alt 5.5–6.7 | 6.2–6.5 while alive ✓ | 6.0–6.4 while alive ✓ |
| — gap 18–26 ≥80/90 s | 24/43 MEASURED samples in band (mean **24.1**, 20.8–26.5) ✗ | 26/37 (mean **24.8**, 22.9–27.9) ✗ |
| — no measured gap <17 m | **PASS — zero** (min 20.8) | **PASS — zero** (min 22.9) |
| 3. Orbit 20/8 | not run (blocked) | not run (blocked) |
| 4. Stop → resume | not run (blocked) | not run (blocked) |
| 5. Estop | not run (blocked) | not run (blocked) |

Honest engagement stats (10 Hz autopsies, both attempts): **id churn 0**
(`vis_car_3` kept its id throughout — superclass association solid);
**flickers 3 / recoveries 2** in each (grace bridged 1–3.7 s dropouts);
ToF **zero LOCKED** — OUT-OF-ENVELOPE / NO-RETURN / SEARCHING only
(expected at 6 m over flat roofs; HUD honesty path).

## 2. What R3 fixed (geometry: confirmed closed)

- Ring held: gaps stayed 20.8–27.9 m through both engagements; the 20 m
  lock ring + direct lane tracked the moving car without the R3-run's
  14.2–15.1 m corner cuts. The escape barrier never fired (never needed).
- Alt law held: 6.0–6.5 m, no sag.
- The (70,30) watch corner was never reached either time (deaths at ~11 s,
  before the car's first corner transit completed the leg) — corner behavior
  UNTESTED this run.

## 3. The residual killer (det-watch evidence, retry)

Death chain (retry, t=lock≈26.7 on the det-watch clock):

1. **Detector conf dip at pursuit onset** (t=27.2–28.1, ~1 s): the car det
  (0.35–0.87 pre-lock, strong) vanishes for ~1 s exactly as the drone spools
  up — while `person` dets persist (not global blur). Track → COASTING.
2. **Coast-yaw freeze:** the R3 yaw preference uses `tr.bearing_deg` — the
  LAST MEASURED bearing (`contacts.py:932`), frozen during a coast. The car
  (4 m/s eastbound, ~25 m) crosses ~8°/s and walks left while the nose holds
  the stale bearing.
3. **Edge-clip starvation:** car dets return only clipped at the left frame
  edge (`[14,287,106,328]`, `[0,248,114,308]`, conf 0.25–0.78, intermittent)
  — the exact edge-flicker condition the click gates avoid at lock time —
  and fail re-association. 5 s grace expires → LOST (t≈37.8; 10.9 s
  engagement). Attempt 1 shows the same signature (3rd flicker never
  re-associates with dets=1–3 present).

**Same circle or new?** NEW dominant mechanism. R1 (blind-cone close-in),
R3-run (ring corner-cut) are both closed by the R2/R3 laws; what remains is
perception+yaw: (a) yaw should steer on the PREDICTED bearing during a coast
(EKF-projected, steering-only — never for association), (b) re-association
tolerance for edge-clipped boxes on the designated target, and/or (c) a
detector-scope decision (COCO-nano@640 recall at 24–28 m on the Fuel
hatchback is marginal; conf dips of 1–3 s are routine). This decides codex
round 4 (yaw/coast semantics — small, testable) vs re-scoping the gate's
90 s continuous-contact requirement to this detector's reality.

## 4. Execution notes

- Instrumentation: autopsy + det-watch pre-launched before the click both
  attempts (full 10 Hz engagement records); the 2 s host sampler started
  ~10–20 s post-lock both times (tool latency) — it corroborates HOLD/LOST
  tails; the autopsy is the engagement record.
- Screenshots: `shot_lock.png` (att-1) and `retry/shot_lock.png` both caught
  the immediate post-LOST banner (ops died inside the capture latency); no
  mid-corner frame exists (car never reached (70,30) while tracked).
- **LLM-free: CONFIRMED** — both pilot.logs boot-lines only (7 lines); all
  motion via FlightOps scripts + `/api/lock` + `/pilot/cmd`.
- Suite untouched (604 green as handed off; zero production diffs this run).

## 5. Evidence

`evals/out/w3_run4/`: attempt 1 — `click.log`, `timeline.log`,
`pursuit_echo.log`, `autopsy_vis_car_3.log` (1764 rows), `pilot_attempt1.log`,
`shot_lock.png`. Retry — `retry/click.log`, `retry/timeline.log`,
`retry/pursuit_echo.log`, `retry/autopsy_vis_car_3.log` (2767 rows),
`retry/detwatch.log` (2770 rows, raw det trace incl. the edge-clip window),
`retry/pilot_retry.log`, `retry/shot_lock.png`. Tooling: `w3_click.py`
(no-fallback), `w3_position.py`, `w3_reposition.py`, `w3_ops.py`,
`w3_timeline.py`, `w3_capture.py`, `w3_autopsy.py`, `w3_detwatch.py`,
`w3_verdict.py`.
