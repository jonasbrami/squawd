# Track Primitive: LLM Plans, Classical Controller Executes

**Date:** 2026-07-07 · **Branch:** feat/dynamic-scenarios · **Tiers:** opus =
claude-opus-4-8, sonnet = claude-sonnet-5, haiku = claude-haiku-4-5

## The question

The dynamic-scenario sweeps showed LLM pilots plan well but lose at real-time
pursuit: a blocking `goto` carries ~4 s of fixed overhead per leg, so
discrete-hop chasing can't hold a 3–6 m/s mover that turns. The hypothesis
(the user's): keep the LLM as the *planner* — pick the target, mode, altitude,
standoff, speed cap — and hand the 10 Hz pursuit loop to a **classical
controller** exposed as one tool. Does that recover the tasks that real-time
execution was blocking?

## What was built

`track(target, mode, alt, duration_s, within_m, speed, standoff)` and the
fleet-concurrent `track_all(tracks=[…])` (design + plan:
`docs/superpowers/specs|plans/2026-07-06-track-primitive*.md`). The controller
(`agents/flight/track.py`) estimates target velocity (finite-diff + EMA) and
streams PX4 offboard position-carrot + velocity-feedforward setpoints at
10 Hz — PX4's own cascade is the PD law. **shadow** holds station on a moving
target (PD-on-moving-reference); **intercept** flies a closed-form
lead-collision course recomputed every tick. The LLM never enters the 10 Hz
loop; it calls one tool and reads back a gap/dwell/velocity summary it must
verify against the task.

Live-smoke (evals-dyn): intercepted a 6 m/s transit in 13 s with an exact
velocity estimate; shadowed the plaza rover at **2.2 m mean gap / 56 s dwell**
in a 60 s call — the d2 requirement (45 s within 15 m) that no goto strategy
ever met.

## Headline result — the A/B on identical tasks

Tasks, oracles, and budgets are **frozen**; the only manipulation is the
toolset, so the pre-track sweeps are a clean historical control arm.

| task (bottleneck) | pre-track (every tier) | with track |
|---|---|---|
| **d2_shadow** (sustained tracking) | opus 0/2 · sonnet 0/2 · haiku 0/2 | **opus 2/2 · sonnet 2/2** · haiku 0/2 |
| **w4_double_intercept** (coordinated execution) | opus 0/2 · sonnet 0/2 · haiku 0/2 | **opus 1/2 · haiku 1/2** · sonnet 0/2 |
| **d4_estimate_intercept** (hard deadline) | 0/2 all tiers | 0/2 all tiers |

Both d2 and w4 were **categorically impossible** — 0/2 for every LLM tier —
before the primitive. Track unlocks them. d4 does not move, and *why* it
doesn't is the most interesting part.

## Reading the three tasks

**d2_shadow — unlocked (execution-bound).** Pre-track this was the
"trajectory-authoring rung": the only way to hold a 45 s dwell on the looping
rover was to fit its track from observations and fly a synchronized
`run_mission` lap — which no tier managed. With `track`, opus and sonnet pass
2/2 in **7–12 steps** (one scan, one track call). The classical controller
turned an author-a-trajectory task into a one-call task for capable models.
Haiku still fails 0/2 — but on the **observation tax**, not tracking: it burned
`ToolSearch×48` flailing to discover its own tools instead of flying (one repeat
never left the pad). The tracking was never the obstacle for haiku; orchestrating
a single clean call was.

**w4_double_intercept — unlocked (coordination-bound).** Two contacts ~230 m
apart must be tagged within 25 s of each other; one drone can't do both.
Pre-track: 0/2 all tiers, and E3 showed the commander layer couldn't rescue it
either — the bottleneck was drone-level pursuit. With `track_all` (two
concurrent onboard intercepts), opus and haiku reach 1/2, and **every remaining
failure is a coordination choice, not a tracking failure**: opus's other repeat
missed by a hair (intercepts 25.0 s apart, window 25 s); sonnet serialized its
two tracks (71.6 s apart) instead of issuing one concurrent `track_all`, or flew
one drone 799 m chasing wrong; haiku's other repeat froze (never flew). The
controller removed the real-time execution barrier and **exposed the
coordination signal the task was built to measure** — exactly the C0-operator
skill (allocate, deconflict, fire simultaneously) rather than an impossible
pursuit wall.

**d4_estimate_intercept — not unlocked (latency-bound), and this sharpens the
thesis.** A 6 m/s aircraft must be caught within 10 m *before* it reaches a
deadline zone, on a ≤300 m fuel budget. Still 0/2 — but the controller **nails
the geometry**: opus, sonnet, and haiku all close to **0.1–1.4 m**. They fail
because they're **dispatched too late**. The prompt says "work out its course
from successive scans," and opus obeys — `scan×11`, then `track×1` — by which
time the fast mover has entered the deadline zone. The scripted pilot passes d4
2/2 precisely because it tracks *immediately* (take_off → track, 2 steps); the
LLM's deliberation eats the margin. Track removes the **execution** bottleneck;
it cannot remove the **decision-latency** bottleneck a hard deadline punishes.

## The through-line

> A classical executor unlocks the tasks whose bottleneck was **execution**
> (holding a moving target, firing two intercepts at once) and leaves the tasks
> whose bottleneck is **decision latency** (committing before a deadline) exactly
> where they were.

This is a precise confirmation of the user's framing — "Claude is good for
planning, not real-time tracking; rely on old-school control for the real-time
part." The split works. And it locates the *next* frontier: for deadline tasks,
the win is not a better controller but a faster commit — the LLM must learn that
`track` estimates the course itself, so the optimal play is to dispatch it
immediately rather than scan first.

## Cost / effort notes

- d2 opus: ~$0.64, 7–11 steps, gap_p50 11 s (patient, deliberate). sonnet
  ~$1.39 (kept some gotos alongside track). haiku ~$0 but 48 ToolSearch calls.
- w4: opus passing repeat 19 steps; haiku passing repeat 13 steps at a fraction
  of opus cost — the E4 "intelligence in the cockpit" economics recur, now with
  a classical cockpit: a cheap model that dispatches `track_all` cleanly beats an
  expensive one that mismanages it.

## Method / validity

- Gates first (dual-baseline): d2/d4/w4 track-pilots PASS 2/2, the goto-chaser
  nulls still FAIL 2/2 — the primitive is the ideal toolpath, the tasks still
  discriminate. `evals/out/pilot_track/`.
- Guidance core has 33 unit tests (intercept quadratic, estimator, dwell log);
  the tool wiring and ENU→NED transform were reviewed (opus) and live-smoked.
- Two harness frictions surfaced and are recorded for next time: (1) eval LLM
  cells share the account's OAuth token with the live session — concurrent
  refresh rotates it and 401-thrashes; run one gentle pass with a fresh token,
  never a rapid restart loop. (2) A flailed cell can leave a drone landed-far-out
  and disarmed that the reset ferry can't re-arm; recovery is one clean container
  restart, not a loop.

## Data

`evals/out/track_dyn/` (d2, d4 × 3 tiers × k2), `evals/out/track_w4/` (w4 × 3
tiers × k2), `evals/out/pilot_track/` (gates). Pre-track control arm:
`evals/out/dyn_v2_merged/`, `evals/out/swarm_e1_merged/`.

## d4 follow-up — the latency floor is real (tested)

The hypothesis: tell the model, in the `track` tool description, that the
controller measures the target's velocity itself, so it should dispatch
immediately instead of scanning first. Tested (`evals/out/track_d4_immediate/`,
frozen oracle). The hint **worked directionally** — dispatch got leaner (opus
8–9 steps vs 10–14; sonnet 5–7) and every intercept still closes to 0.1–2.4 m —
but d4 stays **0/2**. The residual killer is the model's *thinking time between
tool calls* (gap_p50 ~8 s), which the zero-latency scripted pilot never pays.
The deadline is calibrated tighter than a single LLM reasoning turn, so **no
controller can unlock d4 with an LLM in the loop** — d4 is a pure
decision-latency probe. That is the sharpest possible confirmation of the
through-line: the classical executor removes execution cost to zero, and what's
left standing is exactly the latency the LLM cannot shed.

## Open follow-ups
- Reactive obstacle avoidance as a velocity-space repulsion term in the same
  10 Hz loop (the design doc's noted extension) — the user's "obstacle
  avoidance" ask, deferred from this pass.
- A sonnet-vs-opus coordination study on w4 now that execution is free: the
  failures are purely `track_all`-vs-serialized decisions.
