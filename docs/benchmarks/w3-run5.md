# W3-run5 — validation verdict (demo world, codex R4 re-scoped gate, 2026-08-02)

**Scope.** ONE validation run + ONE fresh-container diagnostic repeat of the
W3 golden path at the RE-SCOPED R4 gate (4 m staging, engagement-structure
90 s session replacing the continuous-pursuit gate) with the R4 fixes live
(coast-phase yaw on the EKF-predicted bearing, 2 s horizon cap,
`ops.py:793-817`; ops-bar 4 m defaults; suite 607 green at handoff). Staging
per R4: takeoff 4 m, ~16–20 m from the mover. Protocol: stop after the
repeat regardless. **No production code changed** — no integration bug
surfaced; the residual failure is detector recall (§3). Evidence tooling
only: `w3_session.py` (re-lock watchdog/session driver),
`w3_session_verdict.py`, `--align` session-clock option.

**Verdict in one line:** **W3 FAIL — every structural count of the
re-scoped gate missed in both attempts; the mechanism is RECURRING
(run-4's perception starvation), now harsher at 4 m/14 m: the COCO-nano@640
detector's recall on the Fuel hatchback under pursuit geometry (~6 % of
cycles in the death window) starves every engagement in 7–12 s, and the
car's 50 s lap makes ≤8 s re-locks physically impossible when a window is
missed.** Geometry, association, grace and the R4 yaw law all behave (gap
min 13.0 ≥11, 91–100 % of samples in the 12–20 band, alt in-band, zero id
churn, 200 re-locks when windows existed).

## 1. Per-gate verdicts

| Gate | Attempt 1 (fresh container) | Diagnostic repeat (fresh container) |
|---|---|---|
| Setup (4 m, 16–20 m) | (46.6,−11.2) facing S, 18.8 m to south leg | (49.9,−12.0) facing S, 18 m |
| 1. Click → lock | **PASS** — 200 `vis_car_4` (ring 14 m) | **PASS** — 200 `vis_car_3` at session t=0.5 (aligned clock) |
| 2. Session structure | **FAIL** | **FAIL** |
| — engagements ≥3 | 1 ✗ | 2 ✗ |
| — each ≥20 s contiguous | [12.0] ✗ | [12.1, 7.0] ✗ |
| — aggregate ≥60/90 s | ~12 % ✗ | ~20 % ✗ |
| — re-lock ≤8 s | none (no window) ✗ | 43.2 s ✗ (next lap's window) |
| — alt 3.7–4.5 | **PASS** 89/89 rows (mean 4.06) | **PASS** 87/87 (mean 4.26) |
| — gap 12–20 ≥80 % | **PASS** 91 % (10/11; mean 15.8) | **PASS** 100 % (17/17; mean 14.1) |
| — no measured sample <11 m | **PASS** (min 13.8) | **PASS** (min 13.0) |
| 3. Orbit 15/8 | not run (blocked) | not run (blocked) |
| 4. Stop → resume | not run (blocked) | not run (blocked) |
| 5. Estop | not run (blocked) | not run (blocked) |

Attempt-1 note: the session clock started at staging (pre-`--align`), so
60 of its 90 s were parking between laps — the retry's aligned clock is the
fair reading of the gate; both are reported.

## 2. What works (honest credit)

- The 14 m ring at 4 m: gap converged 26.0→13.8 (att-1) and settled
  13.0–15.3 m (retry) — the R2/R3/R4 geometry stack holds the re-scoped
  envelope; zero sub-11 m samples, zero corner excursions.
- Association/grace: zero id churn across all engagements; recoveries when
  dets existed.
- Re-lock mechanics: both post-LOST clicks returned 200 and re-engaged
  (OFFBOARD, ACQUIRING) — the structural gate's re-lock path works when a
  clickable window exists.
- LLM-free: CONFIRMED both attempts (pilot.logs boot-lines only, 7 lines).

## 3. Mechanism (det-watch evidence) — RECURRING perception starvation

Death window, engagement 1 (retry): the car sat in-frame at ~14–16 m
(lower-third boxes, e.g. `[222,318,346,360]` — bottom-clipped at the 21.1°
floor on the 14 m ring) yet the detector produced a car det in only **7 of
~114 cycles (6 %)** at conf 0.26–0.63; 5 s grace expired → LOST. Engagement
2 died identically in 7.0 s. Run-4's conf-dip/edge-walk at 24–28 m is the
same phenomenon at 14–16 m: the nano@640 model does not hold the hatchback
mesh under the pursuit's rear-quarter, low-depression aspects. The R4
coast-yaw prediction cannot bridge a zero-measurement window (nothing to
associate), and the 50 s mover lap caps re-lock availability (43.2 s
observed) — both are environmental constraints of this world/model pairing,
not wiring.

**Same circle or new?** Recurring — the perception layer that became
dominant in run 4. The decision this feeds: a detector-scope change
(fine-tune the nano on demo-world aspects / raise imgsz / accept
mask-weaker boxes at birth) or a further gate re-scope (engagement floor
below detector-independent 20 s is impossible while recall is ~6 % under
pursuit aspects).

## 4. Evidence

`evals/out/w3_run5/`: attempt 1 — `session.log` (driver events + 1 Hz rows),
`timeline.log`, `session_timeline_echo.log`, `autopsy_attempt1.log`,
`detwatch_attempt1.log`, `pilot_attempt1.log`. Retry — `retry/session.log`,
`retry/session_timeline_echo.log`, `retry/autopsy_retry.log`,
`retry/detwatch_retry.log` (the 6 %-recall window), `retry/pilot_retry.log`,
`retry/shot_final.png`. Tooling: `w3_session.py`, `w3_session_verdict.py`,
`w3_click.py`, `w3_position.py`, `w3_reposition.py`, `w3_ops.py`,
`w3_timeline.py`, `w3_capture.py`, `w3_autopsy.py`, `w3_detwatch.py`,
`w3_verdict.py`.
