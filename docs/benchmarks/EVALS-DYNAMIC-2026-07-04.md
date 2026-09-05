# Dynamic scenarios (moving targets & obstacles) — first tier sweep, 2026-07-04

The new `dynamic` world (5 scripted movers, analytic trajectories, phase-anchored
per cell) and the d1–d5 ladder, graded by within-snapshot drone↔mover geometry.
Design: `docs/superpowers/specs/2026-07-03-dynamic-scenarios-design.md`.
Raw: `evals/out/dyn_{opus,sonnet,haiku}_v2/` (+ `dyn_v2_merged/`), calibration
round `dyn_{opus,sonnet,haiku}/`, pilot gates `evals/out/pilot_dynamic/`.

## The dual-baseline gate (what it caught before any LLM flew)

Every rung ships a must-PASS reference pilot AND a must-FAIL naive baseline
(`null_pilot`). Five gate iterations caught, in order:
1. **`set_speed` was a no-op across leg boundaries** — PX4's DO_CHANGE_SPEED dies
   at the next `goto_location` (a "12 m/s" dash measured ~5 m/s). Longstanding
   flight-tool bug, invisible until a timing task; now sets `MPC_XY_CRUISE`
   (reset restores it).
2. The reference intercept missed by 14.6 m — accel lag vs the clean solve; fixed
   by aiming 2.5 s past the meeting point (arrive early, let it come to you).
3. **The goto ceiling, measured**: a blocking goto costs ~4 s fixed overhead, so
   NO discrete-hop strategy holds a tight dwell on a ≥4 m/s loop. d2 was
   recalibrated into the trajectory-authoring rung: reference = fit the circle,
   fly a matched-speed `run_mission` lap (96.6 s dwell); strongest goto strategy
   = 2.0 s dwell. A 48× cliff between strategy classes.
4. The naive d3 dash originally slipped the patrol window — root cause was (1).

Final gate state: all 5 rungs pilot-PASS / null-FAIL (K=2 on the contested ones).

## v1 sweep = calibration round (opus 2/10, sonnet 0/10, haiku 1/10)

All-tier collapse → suspect the harness first (our own rule). Transcripts showed
budgets taxed the LLMs for costs the scripted pilots never pay: 1–2 SDK
ToolSearch steps + one `scan` step per mover observation + ~5 s deliberation per
step. Opus MADE the d1 intercept and failed on 9 steps > 8. Budgets +4 steps /
+30–60 s (v2); prompts, mover deadlines, path budgets, geometry unchanged.

## v2 results (K=2, seed 11, budgets recalibrated)

| rung | capability probed | opus | sonnet | haiku |
|---|---|---|---|---|
| d1 rendezvous (route given) | space-time arithmetic + patience | **2/2** | 0/2* | 1/2 |
| d2 shadow the loop | recognize hops can't hold → author trajectory | 0/2 | 0/2 | 0/2 |
| d3 timing gate | read patrol phase, wait, dash | **1/2**† | 0/2 | 0/2 |
| d4 estimate-and-intercept | 2 scans → velocity → lead point | 0/2 | 0/2 | 0/2 |
| d5 perimeter defense | intercept under deadline | 0/2‡ | 0/2 | 0/2 |
| **total** | | **3/10** | 0/10 | 1/10 |

\* sonnet made BOTH d1 intercepts and lost both cells on step budget (13 > 12) —
the o2_slalom signature again, now at K=2.
† the d3 miss failed ONLY the step budget (gcs 92%).
‡ **opus caught the intruder both times — at 5.6 m and 0.1 m separation — but
AFTER it crossed the 50 m perimeter.** It wins the chase and loses the race;
the deadline knob measures deliberation cost directly.

**The suite is the hardest axis yet and orders the tiers with headroom above
opus**: flat 24/24 → obstacles 6/8 → dynamic 3/10.

## Tier signatures under dynamics (from transcripts)

- **opus — the observer**: on d2 it scanned the rover 9–11× and hovered 6–8×
  over ~4 minutes, built an accurate picture, and never converted it into a
  plan (1–2 gotos, no `run_mission`). On d4 rep0 it never moved at all beyond
  takeoff (8 scans, 3 hovers). Perfect information, no commitment.
- **sonnet — safe, verbose, no lead**: respects every constraint it can see
  (fence strips clean 4/4 cells, fuel budgets clean), meets static-ish goals
  (d1 intercepts), but never once computed a lead — d4/d5 chased last-seen
  positions — and exceeds every step budget doing it.
- **haiku — right instinct, wrong execution**: the ONLY tier to reach for
  `run_mission` on d2 (3 attempts, v1) — the correct strategy class — but the
  missions didn't match the loop, and elsewhere it thrashes (13–25 steps).

## What discriminates (for the next tooling/prompting loop)

1. Observation → commitment: every tier can measure a mover (scan differencing
   works); none reliably converts velocity into a lead point. This is the PLAN
   nudge's dynamic sibling: "after two scans of a mover, compute its course and
   speed, and aim where it WILL be" is the next prompt experiment.
2. d2's cliff (hops can't hold → author a trajectory) is discoverable — haiku
   found the strategy class unprompted — so a `run_mission`-mentioning nudge
   probably converts d2 for opus/sonnet. Whether that's "tooling" or "coaching"
   is a design choice for the ladder's identity.
3. Step budgets ARE a real measurement (deliberation efficiency under time
   pressure), but they're now the dominant failure mode for sonnet everywhere;
   consider reporting a shadow "unbounded-steps" pass rate alongside, so
   correctness and efficiency read separately.

## Caveats

K=2 (Wilson bounds wide; patterns across rungs are the signal); single seed
(phase-anchored movers make repeats deterministic-ish, so K mostly measures
model variance); trajectories still not persisted (transcripts only).
