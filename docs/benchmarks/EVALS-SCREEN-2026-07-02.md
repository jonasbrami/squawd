# Tier screening after the tooling fix — 2026-07-02

12 tasks × {opus, sonnet, haiku} × K=2, flat world, blocking tools, isolated
parallel sims. Merged reports: `evals/out/screen_v3_merged/` (RESULTS/LADDERS/TOOLS).
Baseline (fire-and-forget tooling, corrected oracle): `evals/out/rerun_ordering/`.

## Headline

| tier | pass | vs baseline | cost (24 cells) | out-tok p50/cell | api s/step p50 |
|------|------|-------------|-----------------|------------------|----------------|
| opus | **24/24** | p1/p2 were 0/3, c1 1/3 | $3.90 | 1061 | 4.9 |
| sonnet | **24/24** | p1/p2/p3/c1 were 0/3 | $5.49 | 1212 | 3.5 |
| haiku | **21/24** | (was best under old tooling) | $0.88 | 2606 | 4.5 |

- The old plan-depth wall (~0% every tier) and the c1 inversion (haiku 100%,
  opus 0%) were **tooling artifacts**, gone entirely under blocking `goto`.
  K=2 caveat: 2/2 has a Wilson 95% lower bound of only 34% — "24/24 across 12
  diverse tasks" is the meaningful signal, not any single cell.
- **haiku is the only tier with a visible knee**, and it is exactly where the
  ladder design predicted: `s6_bearing` 1/2 (compass-bearing computation — the
  failed rep flew ~91 m off, the anchor-confusion failure mode), `am5_noflyzone`
  1/2 (constraint-respecting detour, burned the step budget doglegging),
  `p1_route2` 1/2 (flaky; deadline).
- haiku is ~4.4× cheaper than opus and ~6× cheaper than sonnet per cell, at
  87.5% vs 100%. Model latency per decision is comparable across tiers
  (3.5–4.9 s/step) — flight time dominates cell wall-clock, so **correctness,
  not latency, should drive drone-role assignment**.
- Notable behaviors: sonnet flew a clean 5-line boustrophedon survey via
  `run_mission` and landed on task completion (which exposed the reset ferry
  bug); haiku emits ~2.4× more output tokens than opus for the same tasks and
  occasionally attempts non-existent tools (`Bash`×7).

## What it took to get clean numbers (harness findings)

1. **Blocking tools** (`goto`/`fly` return on arrival, `hover(seconds=N)`) —
   removed the fire-and-forget override trap that zeroed plan_depth for every
   tier. Validated by a no-LLM pilot gate: 7/7 tasks, 100% of oracle checks.
2. **Parallel sims MUST set unique `ROS_DOMAIN_ID`** — DDS multicast crosses
   the docker bridge; with the default domain all sims' `/px4_0/*` topics merge
   and every runner reads a blend of three drones (proven by topic echo).
   Two full screening rounds were discarded to this.
3. **Post-turn halt** — a deadline-cancelled blocking goto leaves its PX4
   setpoint flying; without `hold()` drones ended cells up to 770 m out.
4. **Reset ferry** — agents legitimately land away from home; RTL on a
   disarmed vehicle is a mode-change no-op (observed live). Ferry = arm +
   takeoff (retried through PX4's transient DENIED) keyed on the DISARMED
   state (parked EKF altitude drifts ~2 m, so altitude thresholds lie).
5. **Fresh-sim escalation** (`evals/scripts/drive_sweep.sh`) — some PX4 states
   stay unrecoverable in place; the driver restarts the container and resumes
   (sonnet needed 2 restarts; results unaffected since resume skips scored
   cells and infra rows re-run).

## Haiku knee at K=8 (escalation, same day)

| cell | pass | Wilson 95% |
|------|------|------------|
| s6_bearing | 7/8 | [53%–98%] |
| am5_noflyzone | 5/8 | [31%–86%] |
| p1_route2 | 6/8 | [41%–93%] |

Zero infra failures across the 18 escalation cells (halt + ferry + isolation
hold under sustained load). Against the 0.8 deployment threshold (Wilson LOWER
bound ≥ 80%), none of the three certify: haiku's knee sits at
**negative-constraint routing** (am5, ~5/8 — fails by burning its step budget
doglegging) with bearing computation and even 2-leg routes not yet certifiable.
Read: haiku is fine for single-goal flights and cheap fleet work; give
constraint-bounded or geometry-computing taskings to sonnet/opus until prompts
or tools close the gap.

## Round 2 (same day): ceiling rungs + the PLAN nudge

**Ceiling rungs** (p6 tight-budget route, p7 battery-knapsack, c3 five-constraint
survey; pilot gate 3/3 before any LLM cells; K=2):

| task | opus | sonnet | haiku |
|------|------|--------|-------|
| p6_tight_route | 2/2 | 2/2 | 2/2 (3-4 steps — used run_mission!) |
| p7_knapsack | 2/2 | 2/2 (~100s thinking) | 2/2 |
| c3_constrained_survey | 1/2 | 0/2 | 1/2 |

- **c3 is the first above-opus rung**: everyone mostly fails it, on the step
  budget (17 vs 16) and once the ceiling (16.4m). The five-way constraint
  conjunction binds; steps budget may deserve +1 if we want it AT opus level
  rather than above.
- **p7's knapsack fell to every tier** — the machine-verified greedy trap
  (greedy=3 < optimum=4) did not fool any current Claude; small route
  optimization is now table stakes. Prediction falsified, which is fine: that
  is what probes are for.
- haiku solved p6 in 3-4 steps by authoring the whole patrol as ONE
  run_mission — tool-choice diversity the transcripts now make visible.

**PLAN prompt nudge** (system prompt gains: write the waypoint plan first and
check every leg against every constraint): haiku's knee cells re-run at K=8:

| cell | pre-nudge | post-nudge |
|------|-----------|------------|
| am5_noflyzone | 5/8 | **8/8** [68–100%] |
| s6_bearing | 7/8 | **8/8** [68–100%] |
| p1_route2 | 6/8 | 7/8 [53–98%] |

One prompt paragraph recovered haiku's constraint-routing gap (18/24 → 23/24
on the knee cells). This is the "improve tooling/prompting to raise capacity"
loop working end-to-end: find knee → localize with K=8 → prompt fix → verify.

**New harness lessons** (all committed): PX4 re-sets HOME AT ARMING, so a
reset ferry must fly to world home itself — RTL after re-arming returns to the
re-arm spot (observed 99.9m-from-home loops); stale copied credentials surface
as `<synthetic>` 401 cells — now flagged infra by `client_failed`, never scored.

## Next
- Add harder rungs above the current ceiling (opus/sonnet saturated at 24/24):
  tighter step budgets, multi-constraint capstones (c2 variants), and the
  planned paraphrase splits (prompt-wording effect is still unmeasured).
- Obstacle ladder stays blocked on a usable flat-with-buildings world.
