# W3-run3 — validation verdict (demo world, codex R2 gate, 2026-08-02)

**Scope.** ONE validation run + ONE fresh-container diagnostic repeat of the
W3 golden path against the R2 geometry law (`min_pursuit_range_m`,
`agents/perception/projection.py:22-40`; shadow default `range_m=R_min(6)=18`,
standoff/orbit clamped, orbit gets `hold_altitude=True`; suite 601 green at
handoff). Codex R2 gate at 6 m hold: click → pursuit 90 s no-LOST (alt
5.5–6.7, gap 20–25 m ≥80/90 s after ONE Back-Off to 23 m) → orbit 20 m/8 dps
30 s → stop/resume → estop. Protocol: stop after the repeat regardless. **No
production code changed** — no integration bug surfaced; the residual failure
is a dynamics/design issue (below). Evidence tooling only:
`evals/out/w3_run3/w3_click.py` lost its any-class fallback (see notes).

**Verdict in one line:** **W3 FAIL on gate 2 (pursuit duration) — the SAME
blind-cone circle, one layer deeper.** The R1+R2 fix stack demonstrably
works — the retry engagement survived **27.4 s** (R1: <10 s), with **zero
id churn** (R1: `vis_car_19→20→…`), **3 grace recoveries** through detector
flicker, and a **22.3 m mean gap** on the 18 m ring — until the mover's 90°
corner cut the ring inside the frame floor again and the contact died in the
cone (final coast at **15.1 m**, R_min=18). This is not a new mechanism and
not perception association: it is **ring tracking through the mover's
corners** — the next codex round's problem (see §3).

## 1. Per-gate verdicts

| Gate | Attempt 1 (fresh container) | Diagnostic repeat (fresh container) |
|---|---|---|
| Setup (6 m, 25–30 m) | (52.6,0) facing W, 26 m to west leg | (50.0,−4.7) facing S, 25.3 m to south leg |
| 1. Click → lock | **PASS** — 200 `vis_car_7` (boresight, ranged, attempt 28) | **PASS** — 200 `vis_car_9` (attempt 7, mid-leg eastbound); an earlier any-class fallback locked `vis_person_2` (execution note §4) |
| 2. Pursuit 90 s | **FAIL** — LOST at ~6–7 s (COASTING gap **14.2** m at first sample — already inside the 18 m ring) | **FAIL** — LOST at **27.4 s** |
| — alt band 5.5–6.7 | 6.2–6.4 while alive ✓ | 5.6–6.4 (timeline), 5.9–6.4 in window ✓ |
| — gap 20–25 ≥80/90 s | no band data (died at spool-up) | **44 %** of MEASURED samples in band (mean 22.3, min 14.9, max 29.7) ✗ |
| 3. Orbit 20/8, 30 s | not run (blocked) | not run (blocked) |
| 4. Stop → resume | not run (blocked) | stop→200 HOLD executed once (walker cleanup); resume not reached |
| 5. Estop | not run (blocked) | not run (blocked) |

Back-Off (standoff 23 m): in both attempts the POST landed **after** the op
had already LOST-broken (`track=LOST`, no live contact) — the band
requirement is reported from the lock's own 18 m ring data.

## 2. The retry engagement (10 Hz autopsy, the honest numbers)

Lock t=8.4 (autopsy clock) → LOST t=35.8 = **27.4 s**
(R1 best: <10 s; pre-fix: 2–20 s):

- **Readoptions/id churn: 0.** `vis_car_9` kept its id for the entire
  engagement — the superclass association keys killed the R1 churn.
- **Flickers: 4 (MEASURED→COASTING); recoveries: 3 (COASTING→MEASURED)** at
  t=14.7/18.5/22.7 — the 5 s grace + association bridged detector gaps up to
  3.7 s, exactly the R1 design intent. The 4th flicker (in the cone) killed it.
- **Gap (EKF range while MEASURED, n=147): mean 22.3 m** — 28.0 at lock →
  29.7 → 17.8/18.3 (on the 18 m ring) → 24.1/22.8 (holding) → **15.1 coast →
  LOST**. In-band (20–25): 44 %.
- **ToF: zero LOCKED** — OUT-OF-ENVELOPE ×101, NO-RETURN ×114, SEARCHING ×54
  (expected at 6 m over flat roofs; HUD would read VISION LOCK + raw status).
- Alt held 5.6–6.4 m; the R2 `hold_altitude` + ring kept the geometry viable
  for 3× the R1 lifetime on straight legs.

## 3. Mechanism for codex round 3 — the corner, not the cone radius

Both attempts died the same way: the pursuit's accel-limited shaped servo
(1 m/s², `agents/flight/ops.py:817-831`) cannot hold the 18 m ring through
car_1's 90° corners at 4 m/s (lateral need ≈v²/r≈0.9 m/s² *while* closing —
the servo cuts inside). Attempt 1: cut on the initial spool-up chase (gap
14.2 at the first COASTING). Retry: clean ring hold on two straight legs,
then the NE corner (car westbound onto the north leg at (70,30)) swung the
ring across the drone — final rest (64.6,25.2) vs the corner — coast at
15.1 m, car under the ±21.1° frame floor, dets stop, 5 s grace expires with
no re-detection (the cone is a *geometric* blind spot — association/grace
cannot bridge it), LOST. **R_min(H) is a steady-state law; what fails is the
transient.** Candidate directions: bearing-rate-aware ring expansion
(widen the radius when |dθ/dt| is high, i.e., corners), a corner-cut
interlock (cap closing speed while the target's turn rate is high), or
re-planning the ring on the outside of the turn. Also worth reviewing: the
CV ground-plane range at ≥26 m slant is ~3 m/px sensitive — birth ranges
this shallow are noisy (the 27.3→17.8 jump was a re-measure, not motion).

## 4. Execution notes (honesty)

- **Walker mis-click (retry, operator tooling):** the reused `w3_click.py`
  any-class fallback locked `vis_person_2` (walker_1) before the car entered
  frame; a `stop` op was issued (200, HOLD) and the fallback was removed from
  the *evidence copy* (`evals/out/w3_run3/w3_click.py`); the excursion
  spoiled the staging, so the drone was repositioned (goto-only
  `w3_reposition.py`) before the car lock. The person lock itself behaved
  correctly (hit-test + dispatch + structured LOST when it left view).
- **Instrumentation latency:** the 2 s host sampler started after each op's
  early life both times (tool-launch latency); the fine-grained evidence is
  the in-container 10 Hz autopsy. The retry autopsy was launched pre-lock
  and captured the full engagement.
- Screenshots: the ops died before/during capture windows; the honest
  photographic evidence is `retry/shot_final.png` (post-LOST banner). No
  staged "engaged" shots exist — the timeline + autopsy logs are the record.
- **LLM-free: CONFIRMED** — both pilot.logs boot-lines only (7 lines); all
  motion via `FlightOps` scripts + `/api/lock` + `/pilot/cmd`.
- Suite untouched (601 green as handed off; zero production diffs this run).

## 5. Evidence

`evals/out/w3_run3/`: attempt 1 — `click.log`, `timeline.log`,
`pursuit_echo.log`, `autopsy_vis_car_7.log` (1263 rows), `pilot_attempt1.log`.
Retry — `retry/click.log` (walker stop + car lock), `retry/timeline.log`,
`retry/pursuit_echo.log`, `retry/autopsy_vis_car_9.log` (2815 rows, full
27.4 s engagement), `retry/autopsy_walker_episode.log`,
`retry/pilot_retry.log`, `retry/shot_final.png`. Tooling: `w3_click.py`
(no-fallback), `w3_position.py`, `w3_reposition.py`, `w3_ops.py` (fresh-target
op poster), `w3_timeline.py`, `w3_capture.py`, `w3_autopsy.py`,
`w3_verdict.py`.
