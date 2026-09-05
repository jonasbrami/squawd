# M3a gate results — 2026-07-21 (branch rebuild-single-drone): **PASS**

Gate (design §7, M3a) results, all live in the dynamic world:

| Item | Result |
|---|---|
| camera-fed `track` shadows mov_1, discover `vis_*` id from detect output | **PASS 2/2** — contiguous dwell **52.7 s** and **51.5 s** of the needed 45 s (60 s tracks, mean gap 11.3/10.6 m, 213/214 of 241 samples inside 15 m), vision-only rendezvous (no truth in navigation; truth only for scoring) |
| ground-truth control `d2_shadow` | **PASS 2/2** — dwell 48.4 s and 53.4 s |
| dropout >2 s ⇒ structured LOST, no flyaway | **PASS** — unit-tested (O2 suite) + observed live ~30×: offboard stops, hold, legible LOST text |
| velocity-direct ≥ EMA on intercept convergence | **PASS** — feed_direct (provider velocity) converges in **10.5 s** vs EMA 26.7 s (2.5×), `evals/intercept_ab.py` |
| pytest | **386 green** |

Instruments: `evals/track_shadow_gate.py` (vision/truth feeds, dwell scoring),
`evals/intercept_ab.py` (O3 A/B).

## The journey (~35 gate runs, two reviewer rounds with fable+codex)

The camera-fed shadow resisted until the full stack below landed. Reviewers
(fable round 1–3, codex round 1–3) found the three biggest items:

1. **Pitch sign bug** (`dep = ay − pitch`, was `ay + pitch`) — every geom
   projection during a maneuver was corrupted (fable R1 headline).
2. **MAVSDK ff is NOT a constraint** → trajectory shaper (codex P1) and
   MPC_TILTMAX_AIR=12° / MPC_XY_VEL_MAX=6 (fable Q4; offboard bypasses PX4
   jerk limits, issue #18033).
3. **mover z drift** (sim mover commanded vz=0; ±1 m z = ±5 m projection
   error) → proportional vz hold in `sim/plugins/mover_system.py`
   (fable R2 headline).

Plus: recorder clock-skew EMA; blob thresholds on the real rendered orange;
frame-cadence aim + bearing association; 6° depression envelope; support_z
0.6; edge-clip guard; honest attitude fallback; bearing fallback for NIS
rejects; rebind predicts to match time (+ bearing-rebind + unique-best);
Detector 10 Hz conf 0.2; EKF warm-up hover; aim-lead + measured-bearing yaw;
feedforward velocity clamp; name-churn adoption (unique, predicted gate);
COASTING-aware pursuit; lost wall-clock backstop; visibility guard (7 m ref
floor); gap-dependent altitude; bbox angular-height range channel (σ≈0.8 m);
offboard priming retries; soft-start blending; Land-mode arming lock fix;
factory-state wipe at launch; clean handoff (positioned + closing/in-frame);
led vision-only rendezvous; truth control speed cap 6 m/s and −4 standoff.

Residual known limitation: the start-geometry variance is reduced, not
eliminated — unfavorable orbit phases still cost the first seconds (the
handoff waits them out). The estimate's orbit-phase oscillation (±4–7 m) is
bounded by the CV-EKF at frozen constants; ToF range (M3b) is the designed
closer.
