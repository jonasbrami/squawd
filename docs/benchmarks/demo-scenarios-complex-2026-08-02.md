# Demo scenarios — COMPLEX LONG-DURATION ENGAGEMENTS (wave 2, 2026-08-02)

**Stack.** Fresh container per wave-1 recovery + one mid-wave recycle (see
C5): world `demo`, detector `coco-nano-seg-v2-640.onnx` (explicit override,
env-verified), pilot LLM Kimi K3-256k (`SQUAWD_MODEL=k3-256k`, env-verified;
`CLAUDE_CODE_EFFORT_LEVEL` was again NOT visible in the pilot process env —
the launch script does not forward it, recorded honestly). No product code
changed; bugs recorded, not fixed. Car staging: 6 m (50,48)/S (wave-1's
proof that 10 m is out-of-band by geometry). Evidence root:
`evals/out/demo_scenarios/c<N>_<slug>/`. Metrics per the wave spec
(time-in-track %, engagement = one contiguous in-track run, intervention =
one operator/LLM re-lock, corner survival = ≥8 s past a ~90° turn).

**Headline.** The long window changes the picture in one important way:
engagements STILL die in 8–36 s on most windows, but when a window catches,
the pursuit now survives corners repeatedly (one engagement: 53.6 s through
three 90° corners). Fragility doesn't compound into instant death every
time — it compounds into *window starvation*: most of a 3-minute mission is
spent LOST and waiting for the next lap, and one in ~6 windows delivers the
lap-long engagement. The chat path (K3) reports all of this honestly,
including its own failures.

## C1 — "convoy escort" (click, 3-minute shadow window) — **FAIL on time-in-track / PASS on corner survival**

- Engagements (length distribution): **[36.4 s, 53.6 s]** on 2 locks
  (1 initial + 1 re-lock, latency 54.9 s). Interventions: 2.
- **time-in-track = 48.3 %** of the 185 s session window (**58.8 %**
  measured from the first lock) — both under the 60 % bar; the LOST gaps
  (33.5 s align-wait + 54.9 s lap-paced re-lock) sink the metric.
- **Corner survival: PASSED strongly.** Engagement 1 continued **26.9 s**
  past the NW corner (then died AT the next corner); engagement 2
  continued **46 s past corner 1 and 17.4 s past corner 2**, dying at the
  third (corners from the gz-truth cornerwatch, `c1_convoy_escort/`).
- Beam: NO-RETURN/OUT-OF-ENVELOPE/SEARCHING, never LOCKED (expected).
- Evidence: `c1_convoy_escort/` (session.log, timeline.log, shot_c1.png).

## C2 — "op chain" (one lock: shadow ≥30 → orbit ≥30 → standoff 18 ≥30 → shadow) — **FAIL at leg 3 (2/4 transitions)**

- Leg 1 shadow: **~30 s ✓** (vis_car_35, gap 23–35). Leg 2 orbit: POST
  200 on the retained id, **ran ~22 s measurably** (gap settling toward
  the ring) — short of 30. Leg 3 standoff: **not posted — the target died
  mid-orbit-leg** (~55 s into the chain). Leg 4 not reached.
- First attempt's leg-1 shadow had already broken at ~11 s; the recorded
  chain is the second lock (cx-gate relaxation documented in chain.log).
- Target identity retained across both executed legs (no churn while
  alive). The weakest link is the engagement layer, not the dispatch:
  every posted transition dispatched and ran.
- Evidence: `c2_op_chain/` (chain.log, timeline.log ×2).

## C3 — "moving ring" (standoff 18 through ≥2 corners) — **FAIL**

- Two windows: ring converged to the R2 clamp both times (gap →**21.2**,
  →**18.4** vs the 18 m setpoint — within ±6 while alive ✓), then died at
  **~20 s and ~10 s**, **before the first corner** in both. The ±6-of-
  setpoint ≥50 % criterion is moot; the corner behavior question
  (lag/cut/blow-out) is unanswered because the ring never reaches a
  corner — which is itself the answer.
- Evidence: `c3_moving_ring/` (ring.log, timeline.log).

## C4 — "re-task" (car → truck → walker re-designation mid-flow) — **PASS (mechanics)**

- Both re-designations **accepted and engaged within ~5 s each**: truck
  (`vis_truck_12`, OFFBOARD gap 31.0) and walker (`vis_person_23`,
  OFFBOARD gap 18.2). The arbiter's preempt-and-redispatch path is clean
  (no wedge, no error; each new lock dispatched immediately).
- Caveats, honestly: the "truck" was a gas-station-area truck-class
  contact at ~60–64 m (the real car_3 is never visible from car_1
  vantages — same FP family as wave-1's S2 phantom); both engagements
  then died in ~13 s (baseline, not a re-designation fault); the
  preemption was observed on already-dying ops rather than a
  mid-stride live one (seconds' difference, same code path).
- Visibility geometry had to be engineered for the walker leg
  (8 m alt puts a 19 m walker below the frame floor; 6 m works).
- Evidence: `c4_retask/` (retask.log, timeline.log).

## C5 — "mover orbit endurance" (orbit r15 w8, 2-minute window) — **FAIL, with an environmental catastrophe**

- First window: lock at 54.9 m (out of band — tooling gate tightened
  after), orbit died at ~10 s. Second window: lock at 23.1 m, orbit
  converged 29.4→18.2→17.3 m and **kept streaming ~90 s** — but at
  ~19:49 the **PX4 EKF exploded**: /state went to alt **−11,417,821 m**,
  position ±6–7 million m, speed 4.9×10⁹ m/s, and PX4 logged
  **"Preflight Fail: Imbalanced propeller detected"** (garbage sensor
  input under the time-sync storm). gz truth says the drone was
  physically fine at 6.6 m throughout.
- **Safety note**: three estop-hold attempts all failed with
  `StatusCode.UNAVAILABLE` (the DDS control link was as storm-degraded
  as the estimate) — the op kept streaming setpoints against a nonsense
  state until the container was recycled. An honest zombie, not a
  survival: the gap froze at 17.3 m from the explosion onward.
- Verdict: FAIL on genuine ≥90 s survival; the 2nd orbit's pre-explosion
  behavior (converged to ~17–18 m and held) is the qualified good news.
- Evidence: `c5_mover_orbit/` (orbit.log, timeline.log ×2,
  px4_explosion.log).

## C6 — "long mission brief" (chat K3) — **FAIL on gate numbers / honest-reporting credit**

Mission: *take off to 8 m, shadow the red car 2 min within 40 m, orbit
once, hover and report.*

- Takeoff ✓ (8.7 m, chat→takeoff ~30 s, chat→engage ~40 s). Shadow
  began ✓ on vis_car_1, closing 43.4→**8.8 m** (within 40 ✓), then LOST
  at **+25.9 s of the 120 s phase** — time-in-track for the shadow phase
  **~22 %** (bar 50 %).
- K3's own LOST recovery: ONE move — **climbed to 16 m** and (by its
  account) rotated through four headings and re-scanned; **no re-lock**.
  The climb was geometrically counterproductive: at 16 m alt the nearby
  car (<30 m horizontal) sits *below the camera's frame floor*, so its
  "no car contacts anywhere in the area" was the blind cone's doing —
  car_1 kept driving its loop directly under it.
- Orbit phase: honestly skipped ("Orbit could not be performed without
  a target"). Hover + report ✓.
- **Narrative fact-check (full text in `c6_long_brief/final_report.txt`):**
  "Took off to 8m" ✓ (8.7); "min gap 8.4m" ≈ ✓ (measured 8.8, 0.4 m
  optimistic); "LOST at t+26s of the 120s shadow" ✓ **exact** (25.9 s);
  "climbed to 16m" ✓ (16.1–16.3); "no car contacts anywhere… only a
  truck and pedestrians" — geometrically false but perception-true
  (blind cone, above); "hovering at (E49, N37), alt 16m" ✓ exact;
  surroundings list ✓ matches the world cast. **Zero fabrication; one
  geometry-misread it couldn't know about.**
- Evidence: `c6_long_brief/` (chat.log, final_report.txt, timeline.log,
  cornerwatch.log, shot_c6.png).

---

## WHAT THE LONG WINDOW REVEALS

**Does engagement fragility compound or recover over minutes?** Both, in
layers. Within a window it behaves exactly like the short runs — most
engagements die in 8–36 s at corners or visibility edges. Across a
3-minute window the fix stack now *recovers meaningfully when a window
catches*: C1's second engagement lived 53.6 s through three corners, and
re-locks always succeed when a lap window exists (27–110 s, lap-dictated).
The compound cost isn't worse deaths — it's the LOST-and-wait fraction:
**48–59 % time-in-track** over 3 minutes, and the rest is window
starvation.

**Which op transition is the weakest link?** None of them — the dispatch
layer (lock/orbit/standoff/resume/re-designate) executed every posted
transition in C2 and C4 without a wedge. The weak link is strictly the
*contact's life*: chains break when the EKF id dies, mid-leg, ~10–55 s
in, whichever op is running. (In C5 the weak link was lower still: the
PX4 estimate itself, which exploded mid-orbit under the time-sync storm —
estop included, RPC UNAVAILABLE.)

**Can the stack do a useful 3-minute mission today?** A guarded yes —
with an operator (or an LLM) driving re-locks and the mission spec
priced in laps, not seconds: you get ~50–60 % tracked time, several
corner survivals, a converging ring, and honest reporting. A hands-off
3-minute continuous tail: no — not until engagement life median is in
minutes, not tens of seconds.

**The single fix that would buy the most endurance:** bridge the
re-detection → re-engagement loop. Today, a LOST op exits and waits for
a human/LLM to re-click; the contact is often still in-frame (C1's
re-locks landed within seconds of a window opening). An automatic
re-acquire that re-designates the freshest same-superclass contact
within the rebind window would convert ~50 s lap-paced gaps into ~5–10 s
flickers and roughly double time-in-track without touching the tracker.

**Updated top-3 weaknesses** (changed from wave 1 — wave 1's #3, the
altitude/band geometry gap, is now understood and engineered around at
6 m; wave 1's #1 stands but with a hopeful twist):

1. **Engagement life (median ~10–30 s, corners as the usual locus)** —
   unchanged in rank, but wave 2 adds the bright data point that *good*
   windows now survive corners repeatedly (53.6 s/3 corners); the gap is
   consistency, not existence.
2. **Target/reference truthfulness** — EKF ghost offset (~7 m orbit
   radius), plus building-class truck FPs that bait both the EKF (C4's
   64 m "truck") and the LLM (wave-1 S2's phantom orbit; C6's search
   geometry misread). The demo can shadow the wrong object convincingly.
3. **Sim-stack robustness under load** (NEW) — the time-sync storm can
   not only starve the EKF (wave-1's blind-land boots) but *explode* it
   mid-flight (C5: alt −11.4 M m, garbage speeds, "imbalanced
   propeller"), and take the estop's RPC down with it. A validation
   harness needs a gz-truth watchdog + estop retry loop; the demo needs
   a quiet host.

*Config footnote: `CLAUDE_CODE_EFFORT_LEVEL` absent from the pilot env
both boots (launch script doesn't forward it); v2 override not default;
K3 produced no usage/ttfa telemetry — all latencies are wall-clock.*
