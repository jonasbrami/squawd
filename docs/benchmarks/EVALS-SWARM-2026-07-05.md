# Swarm coordination evals (C0 operator) — E1 screening, 2026-07-05

First-ever LLM coordination numbers on this stack: ONE model flying BOTH
drones of an N=2 fleet through the w1–w5 ladder (design:
`docs/superpowers/specs/2026-07-04-swarm-coordination-evals-design.md`).
Raw: `evals/out/swarm_{opus,sonnet,haiku}/` (+ `swarm_e1_merged/`), pilot
gates `evals/out/pilot_swarm{,_dyn}/`. K=2, seed 11.

## Results

| rung | axis | opus | sonnet | haiku |
|---|---|---|---|---|
| w1 split-reach | fleet split under time pressure | **2/2** | 1/2 | 0/2 |
| w2 allocation | assign 4 targets, fleet fuel budget | **2/2** | **2/2** | **2/2** |
| w3 crossing | deconflicted swap (8 m, 3D) | **2/2** | 0/2 | 0/2 |
| w4 double-intercept (dynamic) | synchronized tags, 25 s window | 0/2* | 0/2 | 0/2 |
| w5 sync-mark | simultaneity, distinct drones | **2/2** | 1/2 | **2/2** |
| **total** | | **8/10** | 4/10 | 4/10 |

\* opus made both intercepts inside the window once (v1, 19 steps) and missed
on the re-run — w4 is knife-edge FOR opus and out of reach below it.

**Headlines.**
- **Allocation is table stakes (6/6 all tiers)** — like the knapsack before
  it, rational target assignment doesn't discriminate current Claudes.
- **Deconfliction discriminates hard**: opus altitude-layered the crossing
  20/40 m (better than the reference's 22/10) and passed with ~20 m
  separation; sonnet stages correctly but runs out of time/steps
  mid-swap; haiku thrashes. w3 is the coordination analogue of the obstacle
  suite.
- **Synchronized dynamics (w4) is the new above-all-tiers rung**: it needs
  the ambush insight (park on the mover's path — the reference pilot itself
  required exactly that fix at the gate). One marginal opus solve in 4
  attempts; steps saturate at any budget because the intercepts don't happen
  — goto-hop chases equilibrate ~20 m out (the measured 4 s-overhead ceiling
  from the dynamic suite, now at fleet scale).
- **haiku's split personality**: perfect on simultaneity (w5 2/2 — timing
  coordination!) and allocation, zero on split-reach and crossing — it fails
  on execution pace (deadlines/steps), not coordination logic.
- **`goto_all` was adopted by every tier unprompted** (opus 22 calls,
  sonnet 51, haiku 23; zero `run_mission`) — the concurrency primitive's
  ergonomics landed.

## What E1's first pass caught (the gate philosophy, again)

Opus's initial w3 0/2 was TWO harness artifacts stacked on a correct plan:
1. **`goto_all` rejected the drone names the harness itself teaches**
   (`"d0"` namespaces, `"drone_1"` scan contacts) — opus's layered plan
   bounced and it fell back to per-drone gotos. Fixed: ID coercion.
2. **Pad climb-through**: sequential takeoffs cross each other's altitude
   3 m apart over the pads; at LLM deliberation pace this lands after any
   fixed grace window. Fixed positionally (`exempt_near_spawn_m`): the
   terminal area is never graded, a mid-field violation always is — the
   null still fails by geometry, not timing luck.

Also found live: **the reset ferry was being undone by the RTL wave**
(ferried drones re-arm away from home, so PX4 home = the stranding point;
RTL caught them mid-descent over world home and flew them straight back
out — ferry reported success, fuse tripped). Fixed: ferried drones are
excluded from RTL and the ferry waits for touchdown. Plus the recurring
stale-OAuth 401s (pilot gates never authenticate, so cred rot is invisible
until the first LLM cell).

w4's budget got ONE principled bump (18→22 = reference + the fleet paying
the observation tax twice) and was then frozen when steps saturated at the
new ceiling too — step exhaustion there is the symptom of missed
intercepts, not the cause (v1/v2 rows archived as `results_w4v{1,2}.jsonl`
... v1 in-place, current = v2).

## Next (per the spec's build order)

E2 fleet scaling (w7 at N=2/4/8), C1 commander layer on the SAME tasks
(delegation tax), E4 role-mix matrix (commander=opus;drones=haiku — the
cost/quality frontier). Candidate tooling experiment first: a fleet PLAN
nudge ("assign, deconflict, and schedule before moving") — w3/w4 look as
prompt-sensitive as haiku's am5 knee was.

## Caveats

K=2 (patterns across rungs are the signal); one seed; N=2 only; w4's
reference is timing-calibrated to this host (gate concern, logged); the
operator layer shares one context across the fleet — C1 will test whether
delegation helps or hurts.
