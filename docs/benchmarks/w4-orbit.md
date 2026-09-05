# W4-orbit — live orbit validation verdict (demo world, v2 detector, 2026-08-02)

**Scope.** ONE dedicated orbit run at the W4 spec (design §6): click →
orbit 15 m @ 8 dps for ≥45 s with radius/gap statistics + stop/resume, and
up to two tuning iterations if radius accuracy is off (circle rule). Fresh
container, v2 via explicit override (env-verified), cockpit in-container,
LLM-free (pilot.log boot-lines only, 7 lines). New instruments (evidence
tooling only): `w4_orbitwatch.py` (10 Hz synchronized drone PX4 E/N/alt/
speed/heading + designated-target EKF e/n/health + derived radius),
`w4_orbit_verdict.py`, `w4_truth_series.py` (gz-truth path reconstruction
from corner crossings), plus the R8 instrumented cornerwatch for
INVALID_ENV scoring. **No production code changed; no tuning iteration
spent** (justification in §3).

**Verdict in one line:** **FAIL on the gate numbers (contiguity 44.0 s vs
≥45 s; 42 % of samples in the 15±4 band; true radius ~20 m) — but NOT
through the orbit gains.** The orbit lane tracks its EKF reference at
p50 14.7 m (mean deviation −1.1 m, inside the ±2 tuning trigger); the
misses come from the EKF target estimate itself (~7 m ghost offset from
gz-truth with a terminal collapse to 0 m) and an edge walk-out death at
44 s. A gain knob would have been cosmetic — the circle rule's reason to
stop, not to iterate.

## 1. Gate items

| Item | Result |
|---|---|
| Setup (4 m, 14–18 m, v2 verified) | **PASS** — (50.0,−14.0) facing S, 16 m to south leg; lock 200 `vis_car_3` (attempt 11) |
| Orbit contiguous ≥45 s | **FAIL** — orbit phase ~39–44 s (LOST at orbitwatch t=72.5; engagement span 44.0 s from lock dispatch) |
| radius mean/p05/p95 (15±4) | EKF gap: **mean 13.86, p05 2.53, p50 14.68, p95 20.69**; in-band 110/264 (**42 %**). TRUE radius (gz path, clock-aligned −2.9 s): **mean 19.7, p50 18.9, p95 25.7, min 13.1, max 26.6** |
| tangential smoothness | speed mean 3.38, std 1.55, max 5.80 m/s; yaw rate mean −3.3 dps, std 11.5, p05 −20.0 / p95 +21.6 dps — broadly physical for an 8 dps orbit around a 2.4 m/s moving center (measured lap rate from corner truth, not the 4.0 spec), but no clean rate hold |
| LOST events | 1 — at 44.0 s (see §2); no in-session re-lock attempted |
| MEASURED fraction | **62 %** (COASTING 38 %) |
| Stop → hold | **PASS** (HOLD, spd 0.1, op canceled) |
| Resume → re-engage | **PASS** — OFFBOARD ≤5 s, same contact (`vis_car_13`, gap 20.6 m at confirm) |
| INVALID_ENV | **none** in the orbit window (dev −0.69…+0.08 m; a sub-threshold ~0.6 m z bias mid-orbit contributes ~0.9 m of the EKF-vs-truth shortfall) |
| LLM-free | **CONFIRMED** (pilot.log boot lines only) |

## 2. The death (det-watch evidence)

Orbitwatch t≈68–70.2: the car's box (conf 0.81–0.9, healthy) slid steadily
LEFT across the frame (x 385→0) and out the left edge; `dets=[]` for 2.3 s;
grace expired → LOST at 72.5. During the COASTING the R4 predicted-bearing
yaw (2 s horizon cap) could not keep up with the frame-edge transit — the
same visibility-loss family as W3's C3, here at 25–45 m receding range
(box rows 185–227, near the horizon — the R8 floor-edge barrier correctly
never fired).

## 3. Why no tuning iteration

The rule triggers on radius mean dev >±2 m or visible thrash, with knobs
"tangential FF scale, KP on the radial error, accel cap". The evidence
assigns the misses elsewhere:

- The orbit lane (direct lane, `control_ref` orbit branch, `OrbitPhase`
  advanced 8 dps/tick, measured-bearing yaw) held the EKF-reference at
  **p50 14.68 m** — the control is not the error term.
- The error term is the reference itself: EKF-vs-truth mean distance
  **6.94 m** (best clock alignment), with a terminal ghost collapse
  (EKF radius → 0.05 m at t≈66 while the true car was ~17–26 m away —
  see `radius_series.png`, the blue dive). Gains cannot fix a circling
  center that is 7 m off and occasionally teleports.
- The contiguity miss is a perception/coast-horizon edge walk-out (§2),
  not a tracking instability.

Spending a knob here would tune the number, not the failure — reported
instead. The codex-facing candidates are the same family as W3 run 6/7's:
EKF range bias on the v2 footpoint (+~10 % z/projection bias), ghost-jump
admission control on a designated orbit target, and coast-phase yaw
horizon at frame-edge geometry.

## 4. Evidence

`evals/out/w4_orbit/`: `click.log` (2 locks + orbit/stop/resume POSTs),
`orbitwatch.log` (1214 rows, 10 Hz), `orbitwatch.log.radius_series.csv`,
`true_radius.csv` (239 rows, aligned), **`radius_series.png`** (EKF gap vs
true radius), `detwatch.log`, `cornerwatch.log` (ENV instrument),
`pilot.log` (LLM-free), `shot_orbit_1/2/3.png` (post-LOST frames — the
orbit died before the scheduled captures; honestly noted, no staged live
shots exist). Tooling: `w4_orbitwatch.py`, `w4_orbit_verdict.py`,
`w4_truth_series.py`, `w3_preflight.py`, plus the run-8 script set.
