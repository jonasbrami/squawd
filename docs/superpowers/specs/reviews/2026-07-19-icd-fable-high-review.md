# ICD Review — 2026-07-19-interface-specification.md (v1, pre-review)

Reviewer basis: ICD + design spec v4.2 read in full; every `file:line` claim checked against `feat/dynamic-scenarios` @ `7622618`.

**Verification statement first:** the ICD's repo citations are accurate. I confirmed `store.py:14-52`, `bus.py:34-84`, `camera.py:41-93`, `gzposes.py:17-60`, `model.py:29-87`, `perception.py:17-135`, `track.py:1-135` (all four constants and every listed symbol exist), `ops.py:62-422` (including `:245-246` no-mover-feed raise, `:291-295` silent continue, `:305` yaw-on-contact, `:361-370` `_halt`, `:387-422` `run_mission`), `tools.py` (13 current tools incl. `look`; prompt at `:192-242`; fleet `:246-319`), `drone.py:22-74`/`:45-60`, `run.py:30-44`/`:47-67`/`:73`, `video.py:113-176`, `runner.py` (`TIERS:22-28`, `Deps:154-160`, `client_for:213-233` with GzPoses injection at `:223-226`, `_drive:239-259`, `:363-366`, `:399-403`), `evals/pilot.py:289-300`, and that every §3.9 test file exists while `agents/vision/` and `agents/pilot/` do not (correctly marked NEW). The 13-tool surface, no-`look`, CV-EKF, rangefinder provider, acquisition machine, error-code set, and topic inventory all match design v4.2. The problems below are in the seams the ICD leaves open, not in what it cites.

---

## Findings

### BLOCKER-1 — `make_pilot_options` wiring makes the estop supervisor (and envelope/rangefinder injection) unimplementable as signed

**Evidence:** ICD §5.5 signature takes `(i, drone, world, bridge, n, cameras, detector, contacts, report, env, model, cli_path)` — no `envelope`, no `rangefinder`, yet §5.4's `FlightOps.__init__` requires both. Following the existing pattern the ICD inherits (`agents/flight/tools.py:181-190` constructs `FlightOps` *inside* the options builder and buries it in tool closures), the pilot process ends up with **no handle on the FlightOps instance** — but §7.1's estop supervisor must call "FlightOps halt/hold or land directly," and evals' `Deps` split needs the same instance identity. An implementer will either reach into the MCP server's closures or construct a *second* FlightOps for the estop path — the exact ambiguous-ownership tangle this document exists to prevent.
**Fix:** invert the construction: `PilotAgent` (or `pilot/run.py`) builds `ops = FlightOps(drone, world, bridge, 0, 1, contacts, envelope, rangefinder)` and `make_pilot_options(ops, cameras, detector, report, ...)` takes it. One owner, estop and tools share the instance, envelope/rangefinder params stop being missing. Update §7.2's construction order accordingly.

### BLOCKER-2 — the fusion tick has no task ownership; the naive port starves it and every vision-fed `track` dies

**Evidence:** §7.1: "Owns the fusion tick: on each new `detector.detections()` → `contacts.update()`". §6.3: "called by the pilot's asyncio loop after each new inference." Neither says it is an **independent asyncio task**. The template the pilot is rebuilt from (`agents/swarm/drone.py:62-74`) awaits the entire agent turn inside its poll loop — port that shape and during a 120 s `track()` tool call no `contacts.update()` ever runs: `poses()`/`sim_time()` freeze, O2's absence timer fires, every track returns LOST (or coasts on stale state). The estop supervisor gets an explicit "independent asyncio task" sentence; the fusion tick — which is load-bearing for the M3a gate — does not.
**Fix:** add to §7.1: the fusion tick is a dedicated asyncio task started in `run()`, polling `detector.detections()` at ≥ detector hz, deduped by `Frame.seq`, never blocked by agent turns or tool calls. Add the starved-tick case to §11's pilot test row (fake SDK client holding a turn open while the tick keeps publishing).

### MAJOR-1 — the error taxonomy (§9) has no production mechanism for `BLOCKED` and contradicts §7.1 on `ESTOPPED`

**Evidence:** (a) `BLOCKED` is listed for "arrival timeout (STILL ENROUTE), offboard refused" — but today STILL ENROUTE is a *success-string suffix* (`ops.py:109`, `_await_arrival` "NEVER raises"), and offboard-refused raises `ValueError` (`ops.py:282`), which §5.4's own rule maps to `INVALID_PARAM`. No exception type, sentinel, or return convention carries BLOCKED out of ops; a wrapper would have to sniff result strings. (b) The `NOT_READY` path is a "ValueError with `NOT_READY:` prefix" — stringly-typed error discrimination inside one exception class, next to a properly-typed `EnvelopeViolation`. (c) `ESTOPPED`: §9 says any in-flight tool returns it, but §7.1 says the estop "cancels the in-flight agent turn + active tool task" — a cancelled turn consumes no tool result, so no LLM can ever see `ESTOPPED`. The code is unreachable as specified.
**Fix:** define typed exceptions alongside `EnvelopeViolation` (`NotReadyError`, `BlockedError`) and delete the prefix hack; make `_await_arrival`'s timeout raise `BlockedError` (or document BLOCKED as an `is_error=True` mapped *return*, like LOST is a mapped return). For estop, pick one: cancel only the tool task and let the wrapper catch `CancelledError` → shielded halt → return `ESTOPPED: ...` into the still-live turn; or keep turn-cancellation and respecify ESTOPPED as context injected into the *next* prompt ("your last action was estopped by the operator"). §9's table must match whichever.

### MAJOR-2 — the acquisition state machine has two owners and its states can't reach the cockpit

**Evidence:** Design §3.10 defines DESIGNATED → ACQUIRING → RANGE_LOCKED → WORLD_TRACKED → COASTING → LOST, and design §3.7's track banner renders exactly those. The ICD splits the state across modules without an assignment: §5.4 O6 has `ops.track` "starting in ACQUIRING" with beam-lock retries; §6.3 has `contacts.health()` emitting ACQUIRING. Two writers of one conceptual state is the definition of ambiguous ownership. Meanwhile `DetectionsMsg` (§1) carries only per-contact `health` (MEASURED|COASTING|ACQUIRING|LOST) and `beam.status` — **RANGE_LOCKED and WORLD_TRACKED appear in no wire schema**, so M4's specified HUD banner is unbuildable from the specified topic. Nobody is named as producer of the `beam` block either.
**Fix:** assign ownership in §5.4/§6.3: `ops.track` owns the mission-level SM (designation, acquisition, retries); `VisionContacts` owns only per-track measurement health; the pilot composes `DetectionsMsg`. Add a `"track": {"state": str, "target": str|null, "gap_m": float|null}` block to `DetectionsMsg` (or an explicit statement that the banner derives as `health+beam` with the mapping written out), and name the `beam` block's producer (contacts, via BeamAssociator state).

### MAJOR-3 — W1's feed (bridge → World buffers) has no owner: clock alignment and NED→ENU conversion are unassigned

**Evidence:** §3 gives `note_pose`/`note_attitude`/`pose_at`/`attitude_at`; §8.1 says "uXRCE → World buffer". But §7.2's construction order never registers the `bridge.subscribe(..., callback=...)` that calls `note_pose`, and three nontrivial transformations live in that unowned callback: PX4 µs-since-boot → sim-time offset capture (design R11: "captured once at subscribe, documented"), NED→ENU with spawn offsets (currently only inside `World.drone_state`, `model.py:57-64` — the callback would duplicate it), and quaternion → roll/pitch/yaw for `vehicle_attitude`. `world` may import stdlib only (§0.1), so the feeder can't live there with ROS types; an implementer must invent a home and will plausibly violate the matrix or fork the frame math.
**Fix:** add a signature with an owner — e.g. `World.attach(bridge, i)` taking a duck-typed bridge (world stays ROS-free; msg objects arrive as duck-typed callbacks, same as `drone_state` today), or a small `core` feeder. Specify the offset-capture rule and that the ENU conversion is shared with `drone_state`, not duplicated.

### MAJOR-4 — the rangefinder never appears in the assembly, and the M2 composite model breaks the ICD's own `CAM_TOPIC`

**Evidence:** §7.2's construction order (`bridge → world → cameras → detector → contacts → envelope → agent`) omits `GzRangeProvider` and `ImpairmentModel` entirely, though `VisionContacts` and `FlightOps` both take a `rangefinder`. There is no topic-template constant for the 3×3 lidar bundle (`GzRangeProvider(topic, ...)` — caller invents it), and design M2 renames the sim model to `x500_depth_range` — which changes the **camera** topic path too, while ICD §2.3 hard-codes `CAM_TOPIC = "/world/{world}/model/x500_depth_{i}/..."` (matching today's `camera.py:24-25`).
**Fix:** add `RANGE_TOPIC` to §2.5 as a documented template; parameterize the model name in both `CAM_TOPIC` and `RANGE_TOPIC` (env or constant, one place); insert `rangefinder(+impair)` into §7.2's order between cameras and contacts.

### MAJOR-5 — `detect`'s output format — the most LLM-visible contract in the system — references an undefined type

**Evidence:** §5.5: "`detect` … formats `DetectionView` lines". `DetectionView` is defined nowhere in the ICD (§1 has `Detection`, `ContactView`, `DetectionsMsg` — no `DetectionView`). The prompt (design §4.1/§4.2) promises "contact id, class, confidence, relative bearing, estimated distance and world position" but the exact line grammar, ordering, staleness annotation, and the empty-result text are unspecified. Also unspecified: `ColorBlobBackend`'s `cls` label, which determines the actual contact IDs (`vis_{cls}_{k}`) every eval gate and transcript will contain.
**Fix:** define the `detect` result grammar verbatim in §5.5 (one example line + the empty and degraded cases), and pin the blob backend's class label (e.g. `"target"` → `vis_target_0`, matching design §2.5's example).

### MAJOR-6 — the flight↔vision seam is only half-built: `Detector` crosses into flight with no Protocol

**Evidence:** §5.1 builds `ContactProvider` precisely so flight never imports vision — then §5.5 hands a `vision.Detector` instance straight into `flight/tools.py` (`detector: "Detector"`), whose `detect` wrapper calls `detections()`, `healthy()` (and per §1, `latency_ms()` for the DetectionsMsg — though that's pilot-side). The string annotation dodges the import-rules AST test but not the coupling; the first implementer running mypy will add `from agents.vision.detector import Detector` under `TYPE_CHECKING` at best, at module scope at worst.
**Fix:** either define a `DetectorLike` Protocol in flight next to `ContactProvider` (methods: `detections()`, `healthy()`), or don't pass the detector into flight at all — pass a `detect_text: Callable[[str | None], str]` closure composed in the pilot layer (which may import everything). The second is cleaner and shrinks the 12-parameter `make_pilot_options`.

### MAJOR-7 — `BeamAssociator.associate` cannot be implemented from its signature

**Evidence:** §6.4: `associate(dets, sample, attitude) -> str | None`. Projecting the beam footprint disc into the frame requires image dimensions and intrinsics (hfov) — neither is passed nor in the constructor. The comment says it returns "the contact candidate's Detection index/id", but `Detection` (§1) has no id — detections are pre-association — so `str` is the wrong type for an index.
**Fix:** `associate(frame: Frame, dets, sample, attitude) -> int | None` (index into `dets`), hfov/dims from `frame` + a constructor intrinsics param.

### MAJOR-8 — the observatory import allowlist fails against the code the ICD says to keep

**Evidence:** §0.1 allows observatory `agents.core.*, stdlib, av, websockets`. The actual server the "REWRITE-lite" starts from is Starlette + uvicorn (`server.py:16-19`) and necessarily imports `std_msgs`/`px4_msgs` message types to subscribe (`server.py:22-23`) — all four absent from the MAY column. As written, `tests/test_import_rules.py` either fails the ported server on day one or forces an unplanned raw-`websockets` rewrite, contradicting REWRITE-lite.
**Fix:** scope the matrix to what it can actually enforce (`agents.*`, ROS, gz — the spaghetti-relevant edges) and let third-party deps be governed by requirements pinning; or enumerate `starlette, uvicorn, std_msgs, px4_msgs` for observatory. Note ROS *msg-type* imports are also needed by the pilot and evals rows.

### Minor findings

1. **§0.2 cross-process impossibility:** "read-only callers: observatory relay" of `VisionContacts`' locked getters — the observatory is a separate process (§0.2's own table) and reads only `/pilot/detections` (§0.1 decoupling #2). Should say "pilot's detections-publisher task, eval sampler".
2. **`scan` z=None contract mismatch:** §4.1 renders `z=None` as "alt unk", but `ContactProvider.poses()` z is always a float (held estimate, §1 `ContactView.z`). Nothing says how `ops.scan` learns a contact is bearing-only. Have `scan` consume `all_views()` and pass `z=None` when `range_src == "bearing"` (see open point 2).
3. **`lost_s` has two owners:** `VisionContacts(lost_s=2.0)` (§6.3) and O2's independent "absence exceeds `lost_s=2.0`" timer in ops (§5.4). Configure them differently and behavior forks. Make ops trust `health()` when present; the absence timer is only the fallback for providers without `health` — say so.
4. **`jpeg_b64` discrepancy:** design §3.1 says "`seq/has/raw/jpeg/jpeg_b64` kept"; ICD §2.3 silently drops `jpeg_b64` (`camera.py:91-93`). Post-v4.2 dropping it is right (its consumer was `look`) — but state the deletion and the `test_video*` migration, or the AST-of-record disagrees with the design.
5. **`vehicle_status` missing from §8.1:** design §3.1 has it ("`vehicle_status` (+ battery)"); the HUD flight strip (design §3.7) shows flight mode. The ICD topic table only has battery — `/state` can't serve "flight mode" as §8.2 promises.
6. **`/pilot/detections` on CHAT_QOS** (TRANSIENT_LOCAL, KEEP_LAST 100): every late joiner replays up to 100 stale frames. The 0.5 s staleness guard absorbs it, but a latched depth-1 profile is the honest contract ("late joiners get the latest", design §3.6 — says *latest*, not *last hundred*).
7. **Cross-reference typos:** §0.4 "13 total (§5.2)" → §5.5; §2.4 `velocities()` tagged "(O1)" — it's the O3 half (design §3.4 cites §3.5 O3); §2.4 "satisfies ContactProvider except `ranges()`/`health()`" — those aren't Protocol members, so GzPoses satisfies it fully (the exception applies to the *extended read model*).
8. **Design-spec staleness the ICD correctly overrides** (flag for the design's next rev): §2.3 mermaid still says "14 tools"; §3.5 "the 12 swarm-era tools" (today's server has 13 incl. `look`; 13 − look + detect = 13).
9. **`Envelope.check_xy` center undefined:** radius 300 m around what — world origin, spawn, or home captured at `connect()`? Also state that `connect()` derives PX4 `GF_*` from the `Envelope` instance (today `drone.py:55-57` hardcodes 300/80) so the two layers cannot diverge.
10. **`note_target_lock` timing:** `truth: GzPoses` holds only the latest pose; "association at that sim_stamp" is honest only if invoked synchronously at `Trace.observe` time. Say so (≤ one mover-tick error), or take the sampler's WorldTrack instead.
11. **§8.3 "unchanged pump" vs C1:** design §3.1 mandates VideoHub use `snapshot()` exclusively; the current pump duck-types `seq/raw` (`video.py:122-123`). `frame_stamp(i)` forces the switch — "unchanged" is wrong by one method; specify the new duck-type (`snapshot(i) -> Frame | None`).
12. **MAVSDK System creation is unowned:** `PilotAgent.__init__` (§7.1) has no `drone`/System param and §7.2 never creates one (`drone.py:31`: `System(port=50051+i)`). Presumably PilotAgent constructs it internally — one sentence, plus the port.
13. **`ImpairmentModel.apply` semantics:** what is `true_range_m` when all bundle hits are None; who stamps `STALE` (read-time age check in `latest()`/`robust()`?) — pin both.

### Nits

- `RosBridge` node-name default changes ("swarm_bridge" → "pilot_bridge") — fine, but it's an unmarked diff in a "[PORT + CMD_QOS]" section.
- `Detector.detections()` consumers dedupe by `Frame.seq` — stated for the Detector's *input*, not for the pilot's consumption of its output; one sentence.
- `hover`'s 120 s cap (`ops.py:210`) isn't in §5.4's behavioral contracts though M1's estop gate leans on `hover(seconds=60)`.
- `DetectionsMsg` rounds all floats to 2 decimals — fine at 10 Hz, but say explicitly that `sim_stamp` at 2 decimals is still unique per frame (100 ms > 10 ms granularity).

---

## Answers to §12's open points

1. **Split `DetectionsMsg`?** Keep it single. The overlay must render dets and fused contacts *from the same inference* atomically; two topics force the UI to re-join by seq across QoS boundaries — a race the single message makes impossible by construction. The payload is a few KB at ≤5 Hz. If bandwidth ever matters, gate the raw `dets` array behind a debug flag rather than splitting. (Do adopt the depth-1 latched QoS from minor 6, and add the missing track-state block from MAJOR-2.)
2. **`ContactView.z` for bearing-only?** Keep the held float in `poses()`/`ContactView.z` — control consumers (`track`, `goto`, `clamp_ref_alt`) need *a* number to fly at, and `range_src="bearing"` + `range_conf` already carry the provenance. Making z `None` would break the 3-tuple `poses()` contract GzPoses shares. But resolve the display half explicitly: `scan`/`detect` render "alt unk" by consulting `range_src` (via `all_views()`), not by z becoming None (minor 2). Keep, with that one sentence added.
3. **Is `Envelope` the right home for `geofence_*`?** Yes — both layers, one source. The dataclass is the fast, legible, testable pre-check at the tool boundary (LLM gets `INVALID_PARAM` text instead of a silent PX4 refusal); PX4 geofence stays as defense-in-depth exactly as design §13 item 3 prescribes. The missing piece is coupling them: `connect()` must set `GF_MAX_HOR_DIST/GF_MAX_VER_DIST` *from* the `Envelope` instance so the constants can't fork (minor 9).
4. **Fusion tick: Detector or pilot?** Pilot — the current choice is right (Detector stays a pure inference thread, reusable by `perceive_eval` fixtures without dragging fusion state along; evals call `contacts.update()` directly with scripted detections). But the choice only works if the tick is an *explicit independent asyncio task* — as written it's the top concurrency hazard in the document (BLOCKER-2). Keep pilot ownership; add the task sentence and the starvation test.
5. **`face` at 5°/5 s?** Keep 5° for v1. At 69° hfov / 640 px, 5° ≈ 46 px off-boresight — comfortably inside the ±35° detect FOV, which is all `face` guarantees ("a post-face `detect` gets an on-target frame"). Do **not** tighten it for beam-lock reasons: the 0.3–0.5° beam is steered by `track`'s own 10 Hz loop (O6), never by `face`. Two improvements worth one line each: return the residual error in the timeout case (so the LLM can decide to retry), and re-measure the 5 s timeout on the M1 bench under wind/gust params before M3b freezes acquisition timing on top of it.

---

## Go / No-Go for M1

**GO, conditioned on two pre-implementation edits** — both inside M1's own scope:

1. **Fix BLOCKER-1** (invert `FlightOps` construction so the pilot owns the instance and the estop supervisor can reach it; add envelope to the wiring). Without this, M1's estop gate ("cancels a `hover(seconds=60)` and holds within ~2 s") cannot be built against the signed interface.
2. **Resolve MAJOR-1's M1 half** (typed `NotReadyError`/`BlockedError` instead of the ValueError-prefix hack, and pick the ESTOPPED delivery mechanism) — the M1 gate asserts "stable tool-result codes across all tools"; two of six codes currently have no producible path.

Everything else (BLOCKER-2, MAJORS 2–7) lands in M2/M3 scope and must be fixed in the ICD before those milestones start, but does not block M1. MAJOR-8 should be fixed whenever `test_import_rules.py` is written — which is M1 — but the fix is a one-line matrix edit, not a design change. The document is unusually well-grounded (every repo citation verified true, and the layering + Protocol seam genuinely do preclude import cycles); its gaps are concentrated where new state meets old loops — assembly order, task ownership, and error transport — which is exactly where the next spaghetti would grow.
