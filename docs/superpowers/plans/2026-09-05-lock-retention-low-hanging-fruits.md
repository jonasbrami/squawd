# Lock-retention low-hanging fruits

**Date:** 2026-09-05
**Status:** proposed, not implemented
**Scope:** body-fixed-camera target tracking in the single-drone simulator

## Goal

Increase the time that a designated moving target remains visible and safely
tracked without putting the LLM in the real-time loop or weakening identity,
safety, cancellation, or estop checks.

The starting evidence is the bounded 2026-08-09 campaign. Default shadow held
an active lock for 8.5--10.6 seconds; the best existing mode, 20 m standoff,
held it for 28.4 seconds but still failed before a complete lap. Image-only
association and a startup command ramp did not solve target frame exit. See
[`docs/benchmarks/lock-camera-motion-experiments-2026-08-09.md`](../../benchmarks/lock-camera-motion-experiments-2026-08-09.md).

## Constraints

- The LLM selects only the target and high-level behavior. A deterministic
  controller owns camera visibility and flight commands.
- Production contacts come from `VisionContacts`; Gazebo mover truth is only
  for explicit baselines and grading.
- Every movement continues through the shared `FlightOps` owner, active-tool
  cancellation, `Envelope`, PX4 limits, and the independent estop supervisor.
- A stale or ambiguous contact cannot authorize translation. The safe terminal
  behavior is `HOLD`.
- Do not broaden association gates merely to improve duration; nearby
  distractors must be present in the acceptance test.
- Use recorded/replay inputs before spending fresh simulator or LLM budget.

## Ranked changes

| Priority | Change | Estimated effort | Expected value |
|---|---|---:|---:|
| 1 | Make 20 m standoff the conservative demo default | Very low | Immediate improvement from the best measured existing mode |
| 2 | Add bbox-edge visibility states with hysteresis | Low--medium | Prevent frame exit instead of repairing identity afterward |
| 3 | Apply tracking-specific speed, acceleration, yaw-rate, and tilt limits | Low--medium | Reduce camera displacement during pursuit |
| 4 | A/B downward camera cant and vertical field of view in simulation | Low | Quantify cheap visual margin before proposing a gimbal |
| 5 | Add bounded stationary-yaw reacquisition | Medium | Recover safely after a genuine loss |
| 6 | Add the SQLite contact-ledger MVP | Medium | Give programmatic recovery and the agent durable history |
| 7 | Start a replay corpus for controller regression | Medium | Make later controller work cheaper and reproducible |

## Phase 1: conservative operating point

Change only the supported demo/operator default to a 20 m standoff and document
the chosen speed and yaw-rate caps. Keep the lower-level API explicit so evals
can request other modes and distances.

Before changing a default, repeat the existing 20 m configuration in a fresh
container with the same staging and detector. Accept it as the conservative
default only if it materially exceeds the repeated shadow baseline without
more identity switches, envelope violations, or unsafe terminal states.

This is an operational improvement, not a claim of robust lock retention.

## Phase 2: visibility guard

Add a small deterministic state machine driven by fresh designated-contact
image geometry:

```text
CLEAR     bbox inside inner region       pursue normally
WARNING   bbox enters outer margin       reduce translation/acceleration
CRITICAL  bbox approaches frame edge     stop translation and yaw toward target
STALE     no fresh compatible detection  HOLD and enter bounded coast policy
LOST      coast deadline expires         HOLD; require explicit reacquisition
```

Use normalized bbox coordinates, separate horizontal and vertical margins,
hysteresis, and minimum dwell times to prevent state chatter. The state machine
must distinguish a genuinely fresh measurement from a projected or coasting
contact. It may reduce or stop commands; it must never extend freshness or
declare identity.

The first version should be deliberately simple. Attitude-based image-motion
prediction belongs in a later controller only if bbox margins alone show
measurable improvement.

## Phase 3: tracking-specific motion limits

Give pursuit a conservative motion profile rather than relying only on global
PX4 maxima:

- lower horizontal acceleration and jerk;
- lower maximum speed while the target is in `WARNING`;
- bounded yaw rate and smooth heading changes;
- an effective tilt target in the 5--7 degree range for the first evaluation;
- a short, stationary acquisition dwell before translation.

The exact values are experiment parameters, not permanent constants. Log the
requested setpoints, observed roll/pitch, bbox position, visibility state, and
contact lifecycle so improvement can be attributed to the guard rather than to
an accidentally easier trajectory.

## Phase 4: cheap camera-geometry A/B

Without changing the physical-control architecture, evaluate:

- camera pitch: 0, 5, 10, and 15 degrees downward;
- wider vertical field of view at unchanged output resolution;
- the best camera variant with and without the visibility guard;
- optionally, a modest altitude change only after checking detector scale and
  standoff semantics together.

Measure angular resolution and detector recall as well as lock duration. A
wider or downward-canted camera trades forward coverage or target pixels for
margin and must not be described as equivalent to a stabilized gimbal.

## Phase 5: bounded reacquisition

After `LOST`, a deterministic routine may:

1. command `HOLD` through `FlightOps`;
2. grow the last track covariance with age;
3. search a bounded yaw sector while holding position and altitude;
4. rank only fresh `VisionContacts` candidates by class, motion, position,
   appearance when available, and ambiguity;
5. re-designate exactly one validated candidate or remain in `HOLD` on timeout.

Translation from stale coordinates is out of scope. Cancellation and estop
must preempt the routine. This work should share its safety contract with the
separate contact-memory proposal, but it does not require SQL for its first
short-horizon version.

## Contact memory follows retention

The durable ledger remains useful, but it solves correlation and historical
querying after observation loss, not camera stabilization. Implement it after
the visibility work unless evaluation shows that short-term identity churn,
rather than frame exit, is the dominant remaining failure.

Detectors, `VisionContacts`, ToF fusion, and deterministic association write
the ledger directly. The LLM receives bounded, read-only SQL access and cannot
insert observations, merge identities, relabel objects, or promote stale rows
to live flight authority. The complete design is in
[`docs/superpowers/specs/2026-08-09-agent-queryable-contact-memory.md`](../specs/2026-08-09-agent-queryable-contact-memory.md).

SQLite in WAL mode is the intended single-drone MVP; PostgreSQL/PostGIS is a
future scaling choice, not a prerequisite.

## Evaluation ladder

Run each implementation change through the same ladder:

1. Unit tests for state transitions, hysteresis, stale measurements,
   cancellation, estop, and envelope integration.
2. Deterministic replay with at least four target corners and one nearby
   same-class distractor.
3. Fresh-container scripted/no-LLM baseline using production camera contacts.
4. One bounded live comparison against default shadow and 20 m standoff.
5. Only after the scripted gate passes, one bounded model-driven demonstration;
   the model must use the same high-level tool and controller.

Record:

- measured, coasting, and lost duration;
- complete-lap success and number of frame exits;
- minimum bbox margin and fraction of time in each visibility state;
- observed roll/pitch, speed, and acceleration;
- identity switches and false reacquisitions;
- envelope, cancellation, estop, and final vehicle state;
- detector configuration and frame/inference latency.

## Acceptance target

The first meaningful target is one complete target lap with one identity, no
frame exit, no unsafe movement, and a final safe state in at least three fresh
scripted runs. A same-class distractor must be included in at least one run.

If three genuine interventions fail without measurable convergence, stop the
live loop, retain the evidence, and seek independent review. The next hardware
level to simulate is a stabilized two-axis gimbal with explicit rate and
saturation limits, not progressively wider association gates.

## Small project-wide follow-ups

These are useful but should remain separate changes from the lock controller:

- start the cockpit from the supported demo launcher or make the two-process
  behavior explicit in its command output;
- bind the unauthenticated cockpit to `127.0.0.1` by default;
- disable the arbitrary-code `run_mission` escape hatch by default;
- complete `Envelope` integration for every movement primitive;
- show `MEASURED`, `COASTING`, and `LOST` prominently in the cockpit, with a
  bounded reacquisition action rather than an implicit resume;
- extend readiness diagnostics for camera, detector, model credentials, deep
  sidecar, PX4 health, and the contact-memory store when present.

These items require their own review and verification. In particular, changing
network binding, tool availability, or safety-envelope behavior is a supported
interface change rather than documentation-only cleanup.
