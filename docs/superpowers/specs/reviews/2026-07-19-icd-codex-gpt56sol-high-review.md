Verdict: **NO-GO for M1.** The ICD captures the intended final architecture—13 tools, no `look`, local CV-EKF perception, range-provider abstraction—but it is not yet an executable contract. Five blockers require interface changes, three of them directly affecting M1.

## Findings

### Blockers

1. **M1 cannot satisfy both the ICD and the v4.2 milestone plan.**

Evidence:

- The ICD’s `PilotAgent` requires `Detector` and `VisionContacts`, and `make_pilot_options` requires both ([ICD §5.5](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:466), [§7.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:582)).
- Those objects do not exist until M2 and M3a. The design says M1 retains `GzPoses` as the FlightOps contact source ([design M1](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1317)).
- M1 says “13 tools … minus `detect`; NO `look`.” The current 13 include `look` ([tools.py](/home/quenouille/drone/agents/flight/tools.py:170)); removing it before adding `detect` leaves **12**, not 13.
- The final ICD inventory itself is correct: 13 tools, replacing `look` with `detect`.

Concrete fix:

- Add a milestone compatibility table:

  - M1: `ContactProvider=GzPoses`, `detector=None`, 12 tools.
  - M2: detector present, `detect` added, 13 tools; flight still uses `GzPoses`.
  - M3a: `VisionContacts` replaces `GzPoses`.
  - M3b: designation/acquisition/ToF fusion enabled.

- Make the final constructors capability-aware, e.g. `perception: PerceptionService | None`, rather than requiring future concrete classes.
- Specify how M1 constructs provisional `GzPoses`; the current construction order omits it.
- Correct the M1 gate to 12 tools, or explicitly move an honest `detect → NOT_READY` stub into M1 and retain 13. The former matches the written milestone intent better.

2. **`ContactView` creates the exact forbidden `vision → flight` dependency.**

Evidence:

- `ContactView` is defined in `agents/flight/contacts.py` ([ICD §1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:129)).
- `VisionContacts.view()` and `all_views()` return and therefore must construct it ([ICD §6.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:533)).
- The dependency law explicitly forbids `agents.vision` importing `agents.flight` ([ICD §0.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:31)).

String annotations do not solve construction of the value.

Concrete fix:

- Move neutral DTOs and enums—`ContactView`, `ContactHealth`, `RangeSource`, wire views—to `agents/core/contact.py`.
- Keep only the consumer-side `ContactProvider` Protocol in `flight`.
- Have both `vision` and `flight` import the DTO from `core`.

3. **M1 has no coherent owner for FlightOps, the safety envelope, or estop cancellation.**

Evidence:

- The current binding pattern constructs `FlightOps` as a local inside `make_drone_options` ([tools.py](/home/quenouille/drone/agents/flight/tools.py:181)).
- The ICD preserves that factory shape, but it does not pass `Envelope` to it or return the resulting `FlightOps` ([ICD §5.5](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:466)).
- `PilotAgent` supposedly invokes the same FlightOps directly during estop, but its signature contains neither a drone/System nor a FlightOps ([ICD §7.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:582)).
- The only halt operation today is private `_halt()` ([ops.py](/home/quenouille/drone/agents/flight/ops.py:361)).
- Cancellation and a concurrent direct `hold()`/`land()` can interleave with an active offboard or mission `finally` block. A single asyncio loop does not serialize separate tasks.

Concrete fix:

- Construct exactly one `System`, `Envelope`, and `FlightOps` in a runtime assembler and inject that same `FlightOps` into both PilotAgent and the tool registry.
- Replace the factory with something like `make_pilot_options(*, ops, detector, report, active_tools, ...)`.
- Add public `FlightOps.emergency_hold()` and `emergency_land()`.
- Define an `ActiveToolRegistry`/operation arbiter: register the current tool task, cancel it, await its cleanup, then execute the emergency action under `asyncio.shield`.
- Add an operation generation/lease so canceled controllers cannot resume sending stale setpoints.
- State whether wait-false actions may overlap subsequent tools; currently this remains unarbitrated.

4. **The flight↔vision Protocol cannot implement bearing-only acquisition or designation.**

Evidence:

- `ContactProvider` exposes only world poses, sim time, and velocity ([ICD §5.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:349)).
- A newly born bearing-only contact has no metric `e,n,z`; nevertheless O4/O6 require `face`, `goto`, and `track` to use its bearing/elevation ([ICD §5.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:443)).
- Neither `ContactView` nor the Protocol contains `bearing_rel`, `elevation_rel`, covariance, or validity flags.
- There is no `designate()` call, even though v4.2 permits one designated target ([design §3.10](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:830)).
- `support_z` is accepted by FlightOps but has no path into VisionContacts.
- Raw `RangeProvider` is injected into both `FlightOps` and `VisionContacts`, creating two plausible ToF owners.

Concrete fix:

- Define a neutral `ContactObservation` with optional `e/n/z`, bearing and elevation, measurement stamp, covariance/quality, source and health.
- Split the seam into `ContactReader` and `TargetDesignator`, with explicit operations such as `designate(name, support_z=None)`, `clear_designation()`, and `observation(name)`.
- Define `TrackingContext(mode, commanded_speed, own_alt, task_ceiling)` passed from FlightOps to the fusion owner.
- Make **VisionContacts the sole owner of range sampling, beam association and EKF fusion**. Remove `rangefinder` from FlightOps.
- Make a FlightOps `TrackSession` the sole owner of acquisition motion/retries. Keep `ContactHealth` separate from `TrackSessionState`.

5. **The range provider cannot perform v4.2’s required frame↔sample timestamp join.**

Evidence:

- `robust(window_s)` has no requested timestamp ([ICD §2.5](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:248)).
- v4.2 requires joining range to a specific frame within 20–50 ms and preserving the selected sample timestamp ([design §3.10](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:904)).
- Calling `latest()` or `robust()` during `VisionContacts.update(frame, dets)` may return a sample newer or older than that frame.

Concrete fix:

- Replace or supplement it with:

  `robust_at(sim_stamp, *, window_s=.12, sync_tolerance_s=.05) -> RangeSample | None`.

- Specify whether the fit window ends at `sim_stamp`, how stale/future samples are rejected, and which clock drives `STALE`.
- State that `true_range_m` in the impairment API means raw ideal sensor range, never `GzPoses` oracle truth.
- Move impairment/reduction work out of the Gazebo callback or normatively require it to be bounded and side-effect-free; the current callback rule says callbacks “never call out,” while `GzRangeProvider` must call the injected impairment model.

### Majors

6. **There is no atomic authoritative perception snapshot.**

Evidence:

- Pilot separately reads detector output, updates contacts, then reconstructs `DetectionsMsg` ([ICD §7.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:594)).
- The detector thread can finish another inference between those reads.
- `DetectionView` is referenced but never defined ([ICD §5.5](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:482)).
- The wire contact contains `cls/conf/xyxy`, but `ContactView` does not.
- The v4.2 output requires bearing and mask reference; the ICD wire schema omits both ([design §3.10](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:917)).
- The design’s `RANGE_LOCKED/WORLD_TRACKED` states are absent from the ICD.

Concrete fix:

- Add a versioned immutable `PerceptionSnapshot` containing one exact `Frame` identity, raw detections, fused contacts, detector health, beam state and track-session state.
- Make `VisionPipeline.update()` return/publish that snapshot atomically.
- Define mask encoding, optional fields, enum values, schema version and serialization in one shared module.
- Pilot should relay/serialize the snapshot, not rejoin independently read state.

7. **CV-EKF and acquisition are requirements, not implementable contracts.**

Missing normative details include:

- `TrackerConfig` and EKF state/covariance schema.
- Measurement models for geom, ToF and bearing-only observations.
- Source covariance values or configuration source.
- Process noise/acceleration limits and dt bounds.
- Normalized-innovation threshold and source-change confirmation rules.
- Track-birth, class gating, tie-break and ID-slot reuse rules.
- Whether `age_s`, coast and loss use sim time or monotonic time.
- Exact transition guards and timeouts for `DETECTED → DESIGNATED → ACQUIRING → RANGE_LOCKED → WORLD_TRACKED`.
- Retry/backoff schedule and lock confirmation count.
- Beam-slip and reacquisition behavior.
- Atomic reset and snapshot behavior.

Evidence: [ICD §6.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:533), [design acquisition SM](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:830).

Concrete fix:

- Add explicit `TrackerConfig`, `TrackState`, `Measurement`, `AssociationResult`, `ContactHealth`, and `TrackSessionState` schemas plus a transition table. Defaults must be test vectors, not prose such as “2σ” or “bounded retries.”

8. **`BeamAssociator.associate()` lacks the inputs needed to perform its contract.**

Evidence:

- It returns “Detection index/id,” but detections have no IDs.
- It promises a range-consistency gate without receiving the predicted range/covariance.
- It receives neither the source `Frame`, frame dimensions, designation, camera intrinsics/extrinsics nor decoded mask geometry.
- `cam_to_beam_offset_m` has no coordinate convention.
- RLE mask encoding is undefined.

See [ICD §6.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:564).

Concrete fix:

- Use a signature such as `associate(frame, detections, sample, pose, attitude, designated_index, predicted_range, predicted_sigma) -> BeamAssociation`.
- Define `BeamAssociation(status, detection_index, residual_m, footprint_px, reason)`.
- Specify camera and beam frames, axis directions, mask codec and exact erosion/containment rules.

9. **The error taxonomy is internally inconsistent and has uncovered paths.**

Evidence:

- `ValueError` means both `INVALID_PARAM` and prefix-sensitive `NOT_READY` ([ICD §5.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:443), [§9](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:655)).
- Arrival timeout is assigned `BLOCKED`, but FlightOps has no typed blocked path.
- `EnvelopeViolation` subclasses `ValueError` yet carries an arbitrary code.
- Wrappers catch `Exception`, but `asyncio.CancelledError` is not caught by that handler on supported Python versions.
- If the whole SDK turn is canceled, there may be no consumer to receive an `ESTOPPED` tool result.
- Backend, MAVSDK transport, serialization and unexpected-programming failures have no code.
- `run_mission` still returns `(bool, str)` rather than the same error representation used elsewhere.

Concrete fix:

- Introduce `ToolFailure(code: ToolCode, text: str, is_error: bool=True)` and typed subclasses or one structured result.
- Add explicit mapping order, including cancellation.
- Add `INTERNAL` or `FAILED` for unexpected failures, logged with traceback server-side.
- Decide whether estop is a tool result or an out-of-band action event; record it consistently in chat/evals.
- Specify the exact SDK result dictionary for success, failure and degraded `LOST`.

10. **World telemetry ownership and clock conversion are absent.**

Evidence:

- The topic table says telemetry feeds World buffers, but no component subscribes and calls `note_pose`/`note_attitude`.
- The ICD does not specify PX4 timestamp fields, offset capture, quaternion conversion, heading/yaw wrap interpolation or reset behavior.
- Linear interpolation across `+π/-π` is incorrect.
- The current `World` has no buffering or subscription code ([model.py](/home/quenouille/drone/agents/world/model.py:29)).

Concrete fix:

- Add a core-side structural adapter such as `Px4StateRecorder(bridge, sink, i=0)` where `World` is the duck-typed sink.
- Specify use of `timestamp_sample`, clock-offset initialization/reset, shortest-angle interpolation, buffer locking and behavior before alignment is established.

11. **The “core imports lazily and is sim-free” claim is false for the ported baseline.**

Evidence:

- ICD §2 says Gazebo imports happen lazily and everything is safe without ROS/Gazebo ([ICD §2](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:173)).
- `core.camera` imports Gazebo at module scope ([camera.py](/home/quenouille/drone/agents/core/camera.py:17)).
- `core.bus` imports ROS at module scope ([bus.py](/home/quenouille/drone/agents/core/bus.py:9)).
- M1 says core is ported unchanged.

Concrete fix:

- Either refactor camera imports into construction and split pure QoS declarations from ROS objects, or narrow the ICD claim to specific pure modules. Update the test seam accordingly.

12. **Detector/backend boundaries and freshness are underspecified.**

Evidence:

- `backends.py` needs `Detection` from `detector.py`, while `detector.py` names `DetectorBackend`, inviting a module import cycle.
- `healthy()` only considers three inference exceptions; a dead/stale camera can leave old output appearing healthy.
- `stop()` does not promise join/termination.
- `face()` settling does not guarantee that `detect()` returns an inference captured after settling.

Concrete fix:

- Move `Detection` and backend Protocols to `vision/types.py`.
- Add `InferenceResult(frame, detections, completed_monotonic)` and `wait_next(after_seq, timeout)`.
- Define startup, stale-camera, backend-failed and stopped states.
- Require `stop()` to join with a deadline.
- After `face`, `detect` must wait for a processed frame newer than the pre-face inference; 5° alone does not guarantee a fresh on-target image.

13. **The safety envelope is not fully enforceable from its contract.**

Missing semantics:

- Geofence center/home coordinate.
- Relationship between `max_alt_m` and `geofence_alt_m`.
- How task ceilings enter ordinary tools.
- Orbit perimeter validation, not just center validation.
- Relative `fly` endpoint validation.
- Enforcement for `run_mission`, which can bypass every FlightOps primitive.
- Clamp reporting schema.
- Whether PX4 parameter-setting failure is fatal or degraded.

Evidence: [ICD §5.2](/home/quenouille/drone/docs/superpowers/specs/2026-07-19-interface-specification.md:362), [design §13 item 3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1725).

Concrete fix:

- Make the code envelope authoritative, centered on launch/home, and derive PX4 parameters from it.
- Add per-primitive validation methods and a checked `run_mission` boundary—or explicitly admit that arbitrary mission code cannot satisfy the central-envelope claim.

14. **Topic and video contracts do not support the promised authoritative overlay.**

Evidence:

- The ICD omits `vehicle_status`, although v4.2 and the current observatory need it for flight mode ([design topic inventory](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:335), [server.py](/home/quenouille/drone/agents/observatory/server.py:37)).
- `/pilot/detections` uses `CHAT_QOS` depth 100, causing late joiners to replay roughly 20 seconds of stale 5 Hz state.
- The ICD calls the current VideoHub pump “unchanged,” but it currently reads `seq()` and `raw()` independently ([video.py](/home/quenouille/drone/agents/observatory/video.py:150)); v4.2 requires `snapshot()` exclusively.
- `frame_stamp(i)` records only “last encoded,” not the stamp of each queued/broadcast H.264 access unit.
- The `/ws_cam` binary framing schema is absent.

Concrete fix:

- Add `vehicle_status`.
- Add `STATE_QOS = RELIABLE/TRANSIENT_LOCAL/KEEP_LAST(1)` for detections.
- Encode from one `Frame` snapshot.
- Carry `seq` and `sim_stamp` with every encoded frame/access unit and define the WebSocket framing exactly.

### Minors

15. **The ported API inventory omits `jpeg_b64`.**

The current class implements it ([camera.py](/home/quenouille/drone/agents/core/camera.py:91)), and v4.2 explicitly says it remains for legacy callers, while ICD §2.3 omits it.

Fix: include it as deprecated/legacy, or revise the design and migrate its tests/callers explicitly.

16. **The layer diagram is more permissive than the matrix.**

The arrow chain suggests ordinary imports from each preceding layer, while the matrix forbids all `agents.*` imports from `world` and `perception`.

Fix: make the matrix normative and express allowed package edges as an adjacency list used directly by the AST test.

### Nit

17. **The design’s target flow still says “14 tools.”**

[Design §2.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:198) says 14; the normative inventory later and the ICD correctly say 13. Fix the stale diagram label.

## File:line audit

The principal baseline provenance is otherwise accurate:

- `store.py:14–52`, `bus.py:34–84`, `camera.py:41–93`, `gzposes.py:17–60`.
- `world/model.py:29–87`, `perception.py:17–135`, `flight/track.py:1–135`, `flight/ops.py:62–422`.
- `swarm/drone.py:22–74`, including connect/geofence at 45–60.
- `swarm/run.py:47–67` construction and line 73 singleton lock.
- `observatory/video.py:113–176`.
- `evals/runner.py:213–233` client wiring and 239–259 `_drive`.

The inaccurate semantic claims are the lazy-import statement, “unchanged” VideoHub pump, omission of `jpeg_b64`, and the claim that current signatures can directly support M1’s new Pilot construction.

## Missing contracts by milestone

M1 still needs exact contracts for:

- Pilot runtime construction, MAVSDK endpoint and provisional `GzPoses` creation.
- Shared FlightOps ownership and shutdown lifecycle.
- Estop task registration/cancel/await/emergency-action sequencing.
- All tool JSON schemas: required fields, optional fields, `additionalProperties`, and exact result dictionaries.
- Error mapper and typed operation failures.
- Envelope center/path/per-primitive enforcement.
- Fail-closed connection/readiness schema and doctor script exit/output contract.
- Agent inbox polling, stop behavior and command concurrency.

M2 still needs:

- PX4 telemetry recorder and clock-alignment contract.
- Detector lifecycle, stale-output semantics and post-face freshness wait.
- Backend manifest schema and ONNX output tensor contract.
- Atomic `InferenceResult`/`PerceptionSnapshot`.
- Timestamp-addressable range-provider API.
- Impairment configuration and delayed-sample behavior.
- Camera/beam extrinsic and projection sign conventions.
- Typed offline accuracy inputs. `accuracy_report(frames, truths, detector)` is incoherent because a live threaded Detector does not infer over the supplied `frames`.

M3 still needs:

- Optional-coordinate bearing observation schema.
- Designation and control-context Protocol operations.
- EKF configuration and measurement interfaces.
- Association/birth/rebind/ID rules.
- Beam association inputs and result schema.
- Acquisition state enum and transition table.
- Support-plane propagation from `track` into fusion.
- Loss/coast clock semantics.
- Atomic fusion snapshot and exact `detect`/`scan` text format.

## Answers to ICD §12

1. **Keep raw detections and fused contacts in one topic**, because their atomic frame relationship is valuable. Make it one versioned `PerceptionSnapshot` over `STATE_QOS` depth 1. Split only optional high-volume debug data later.

2. **Do not represent an uninitialized bearing-only track with a held numeric z.** Make `e/n/z` optional. A track that previously had metric lock may publish a predicted z, explicitly marked `position_src="predicted"` with age/covariance. A newly born bearing-only track should expose bearing/elevation and `e=n=z=None`.

3. **Keep geofence values in `Envelope`.** It is the application-level source of truth. Derive PX4 geofence parameters from it at connect time as defense in depth. Specify launch/home as the center and eliminate or define the `max_alt_m`/`geofence_alt_m` duplication.

4. **Detector should not own fusion.** Put detector→contacts ticking and atomic snapshot creation in a `VisionPipeline` class inside `vision`; Pilot starts/stops it and relays snapshots. This keeps Detector reusable without putting perception logic in the Pilot assembler.

5. **Keep 5°/5 s only as a provisional camera-facing threshold.** Require the error to remain inside tolerance for multiple telemetry samples and wait for a subsequent inference. ToF acquisition needs a separate much tighter beam-lock criterion derived from beam FOV and footprint containment; 5° is far too loose for the rangefinder.

## M1 decision

**NO-GO.**

Minimum changes before M1 starts:

1. Resolve M1’s 12-vs-13 tool count and phase the constructors.
2. Define one shared FlightOps owner plus the estop/operation arbiter.
3. Wire the same Envelope into tools and emergency handling.
4. Publish complete tool schemas and typed error mapping.
5. Specify provisional `GzPoses` construction and fail-closed startup.
6. Correct or implement the claimed core lazy-import behavior.

Once those are incorporated, M1 can proceed independently of the later vision/acquisition blockers. No repository files were modified.