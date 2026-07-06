# Track Primitive — LLM Plans, Classical Controller Executes

**Problem.** The dynamic-scenario sweeps showed LLM pilots are good at mission
planning but lose on real-time pursuit: a blocking `goto` costs ~4 s of fixed
overhead per leg, so discrete-hop chasing caps out far below a 3–6 m/s mover
(first dynamic sweep: opus 3/10; d2 shadow requires trajectory authoring; w4
double-intercept 0/2 at every tier, and E3 showed the commander layer can't
rescue it because the bottleneck is drone-level pursuit skill).

**Hypothesis.** Splitting the stack — the LLM sets *precise tracking
instructions* (which contact, shadow vs intercept, altitude, standoff,
duration, speed cap) and a classical onboard controller closes the loop at
10 Hz — recovers the dynamic tasks. This mirrors the 2024–26 UAV-LLM
literature: direct per-step LLM control fails on anything dynamic; the LLM
belongs at the semantic layer.

## Architecture

New module `agents/flight/track.py` + one FlightOps method + two MCP tools.

**Controller (per research brief, 2026-07-06):**
- Target source: `GzPoses` (~49 Hz ground truth — the same source `scan`
  already exposes to the LLM, so the tool adds *control rate*, not
  *information*).
- Velocity estimator: finite difference between control ticks + EMA on the
  velocity output (alpha ≈ 0.35 on the new sample — equivalent to a
  Benedict–Bordner alpha-beta given clean input).
- SHADOW law (stay within R of a moving target): PD-on-moving-reference with
  velocity feedforward, delegated to PX4's own cascade by streaming
  `offboard.set_position_velocity_ned(p_ref, v̂_t)` at 10 Hz, where
  `p_ref = p_target + standoff` at commanded altitude. PX4 computes
  `v_des = v_ff + MPC_XY_P·(p_sp − p̂)` — exactly the recommended law, no
  reimplementation.
- INTERCEPT law: closed-form lead intercept recomputed every tick — solve
  `(v̂·v̂ − s²)t² + 2(r·v̂)t + r·r = 0` (r = target − drone, s = speed cap)
  for the smallest positive t_go; stream `p_i = p_target + v̂·t_go` as the
  position setpoint with velocity feedforward `s·unit(p_i − p_drone)`. With
  s = 12 > mover 2–8 m/s a positive root always exists; degenerate cases
  (slow/no velocity estimate yet) fall back to aiming at the target.
- Frames: gz world ENU → PX4 local NED via a per-call offset computed from a
  simultaneous read of `world_xy()` (world ENU) and
  `/fmu/out/vehicle_local_position` (local NED); the frames share
  north/east alignment so the transform is a constant offset + axis swap.
- Offboard lifecycle: stream a few setpoints, `offboard.start()`, run the
  loop, `offboard.stop()` (returns to Hold). If offboard drops (PX4
  failsafe), the tool returns an error string; the LLM re-plans.
- Hard caps: duration ≤ 120 s per call (same discipline as `hover`), speed
  clamped to 12 m/s, building-footprint refusal on the commanded reference
  (reuse `goto`'s check each tick; if the carrot enters a footprint below
  roof+3 m, hold altitude of roof+3 m instead — tracking must not wedge the
  drone against a wall).

**Tool surface:**
- Per-drone `track(target, mode="shadow"|"intercept", alt, duration_s,
  within_m, speed, standoff_east, standoff_north)` — blocking, one step buys
  up to 120 s of closed-loop pursuit. Returns a summary the LLM can verify
  against the task: tracked time, min/mean gap, *best contiguous dwell within
  within_m*, target velocity estimate, intercept time/gap (intercept mode
  returns early on success).
- Fleet `track_all(tracks=[{drone, ...same args}])` — concurrent per-drone
  tracks via `asyncio.gather` (same rationale as `goto_all`: sequential
  blocking calls serialize the fleet; w4 needs two simultaneous intercepts).
- System prompts gain one TRACK line describing when to use it (moving
  mov_* contacts) — planning guidance stays with the LLM: it must still find
  the contact, choose mode/params, and verify the summary.

## Evaluation design

Tasks, oracles, budgets stay FROZEN — the manipulation is the toolset, and
the pre-tool sweeps are the control arm (historical A/B on identical specs).

1. **Gates first** (dual-baseline, unchanged nulls): rewrite d2/d4/w4 `pilot:`
   scripts to use track/track_all (1–2 steps); `null_pilot:` (goto-chasers)
   unchanged and must still FAIL. A passing track-pilot simultaneously
   validates the controller through the real oracle path.
2. **Tier sweep with track available**: d2 + d4 × {opus, sonnet, haiku} × k=2
   (evals-dyn, N=1); w4 × {opus, sonnet, haiku} × k=2 (evals-fleetdyn, N=2).
   Headline questions: does the primitive (a) unlock w4 (0/2 → ?), (b) shrink
   the tier gap on dynamic tasks (does haiku+track beat bare opus — the E4
   "intelligence in the cockpit" lesson, but with the cockpit replaced by a
   controller)?
3. If time permits: d1/d3/d5 re-run with track available (regression + lift).

**Out of scope tonight:** reactive obstacle avoidance beyond the existing
building-footprint clamp (noted as follow-up: a velocity-space repulsion term
in the same 10 Hz loop is the natural extension); tracking without ground
truth (vision-based estimation).
