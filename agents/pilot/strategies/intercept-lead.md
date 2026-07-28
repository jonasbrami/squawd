# Strategy: intercept-lead (CANDIDATE — not active)

Status: candidate for the d4/intercept family. Activates only if an M5 A/B
(`drones=<tier>` vs `drones=<tier>,strategy=intercept-lead`) shows measured
lift per `evals/strategy_ab.lift_decision`.

When a task is to STOP or INTERCEPT a moving contact (not to shadow it):

- Call `track(target, mode='intercept', within_m=...)` IMMEDIATELY after
  `take_off` — the onboard controller measures the target's velocity and
  computes the lead itself. Do not spend calls observing first.
- If the target is fleeing or the deadline is tight, raise cruise speed with
  `set_speed(speed=...)` BEFORE the track call; intercept returns early on
  contact, so speed converts directly into deadline margin.
- If the call returns LOST, re-acquire with `scan` and re-issue `track(...)`
  once; only fall back to `goto(...)` waypoints when two intercepts fail.
