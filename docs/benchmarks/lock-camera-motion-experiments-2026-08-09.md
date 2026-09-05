# Lock retention under body-fixed camera motion — bounded experiments

**Date:** 2026-08-09
**Scope:** single-drone demo world, COCO v2 detector, no LLM requests
**Status:** three interventions tested without convergence; stop and redesign

## Question

Does the lock fail primarily because the vehicle body moves the fixed forward
camera, and can an existing pursuit mode, image-space association, or a bounded
startup command ramp preserve the lock?

The aircraft was staged at approximately 4 m altitude near the `car_1` loop.
Each engagement used the normal cockpit click path and production
`VisionContacts`; Gazebo mover truth was used only for staging and evaluation
instrumentation. Raw detections, fused contact state, vehicle position,
attitude, and final state were sampled without invoking a model.

## Results

| Variant | Active lock | Attitude / observation | Verdict |
|---|---:|---|---|
| Default shadow, repeated baseline | 8.5 s and 10.6 s | Target repeatedly reached the frame floor | Failed |
| Existing standoff, 20 m | 28.4 s | Fresh contact for 46.4% of window; roll/pitch about ±12° | Best tested, but failed |
| Existing orbit, 20 m at 4°/s | 17.8 s | Fresh contact for 35.2% of window; roll/pitch about ±12° | Failed |
| Short-horizon image-bbox association + shadow | 18.8 s | Initial roll/pitch reached roughly 26°; target hit row 359/360 | Failed |
| Image association + 2 s settle / 3 s command release | 18.5 s | First 2 s stayed level; later max roll 12.3°, pitch 11.8°; target still hit row 360 | Failed |

The last experiment proved the causal mechanism more narrowly: holding
horizontal position suppressed the initial attitude spike, but releasing the
direct pursuit command still swept the target down through the frame. The
contact then remained `COASTING` for roughly seven seconds before `LOST`.
During parts of that coast, the detector again emitted car boxes, but they were
not safely associated with the designated identity.

PX4 reported `MPC_TILTMAX_AIR=12`, yet the unguarded engagement briefly
exceeded 20° pitch/roll. The guarded run stayed close to the configured limit,
so parameter application is not the complete explanation. Even a legitimate
12° body attitude consumes most of this level camera's approximately ±21°
vertical field of view when the vehicle is already low in the image.

## Conclusions

1. The owner's diagnosis is correct: pursuit motion and a body-fixed camera
   are tightly coupled. A position controller can satisfy its flight objective
   while destroying its own observation geometry.
2. More standoff helps, but it did not make the lock robust through a lap.
3. Image-space association alone cannot recover an object that leaves the
   image, and broadening its gate further would increase false identity merges.
4. A startup ramp treats only the first command discontinuity. It does not
   solve later corners or sustained attitude demand.
5. The contact ledger proposed separately would improve historical correlation
   and deliberate reacquisition after loss. It would **not** preserve a live
   visual lock or authorize pursuit from stale coordinates.

The experimental tracker and command-ramp code were discarded after this gate;
production defaults are unchanged.

## Recommended next bounded design

The strongest setup change is a stabilized two-axis gimbal (or an independently
stabilized tracking camera) whose fast controller keeps the designated target
in frame while `FlightOps` owns translation. Gimbal limits, rates, and
saturation must feed a classical visibility constraint; the LLM should only
choose a high-level target and behavior.

If hardware/model changes are out of scope, test a software-only lane before
changing production pursuit:

1. Build a deterministic replay from recorded frames, vehicle attitude, and
   detections.
2. Predict image motion from roll, pitch, yaw, and target range; evaluate a
   visibility-aware controller that constrains the predicted bbox to a safe
   image region while retaining the keep-out and flight envelope.
3. Compare against default shadow and 20 m standoff over at least four corners,
   including a nearby distractor vehicle to measure false reassociation.
4. Require one identity, no frame exits, no unsafe movement, and a full-lap
   measured/coasting target before returning to a fresh-container flight gate.

A wider or downward-canted camera can add margin and is worth simulating, but
it trades angular resolution or forward coverage and does not decouple sensing
from vehicle attitude. Treat it as a measured camera-design A/B, not a complete
tracking solution.
