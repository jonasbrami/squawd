# Principal-engineer review

Verdict: **NO-GO as written.** The high-level split—LLM plans, local perception measures, classical control executes—is sound, and the stack can support it. The proposed contracts are not yet safe or internally consistent enough to implement M2–M5, however. In particular, timestamping, projection, loss-of-contact behavior, contact identity, and eval attribution need redesign before coding proceeds.

Review context: the current checkout is `feat/dynamic-scenarios` at `7622618`, not `main`; the reviewed spec is untracked. I could not run pytest because the review sandbox has no writable temporary directory, so test-gate claims remain unverified.

## Blockers

### B1 — Contact loss stops the PX4 offboard stream

Evidence: when a contact is absent, `track()` only sleeps and sends no position/velocity setpoint ([ops.py:291](/home/quenouille/drone/agents/flight/ops.py:291)). PX4 offboard requires a continuing setpoint stream. Cleanup happens only when the entire call exits ([ops.py:309](/home/quenouille/drone/agents/flight/ops.py:309)). Yet the spec says `track()` remains unchanged and that a >1 s dropout reports degradation without flyaway ([M3 gate](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1017)).

Concrete fix: make contact state explicit: `MEASURED`, `COASTING`, `LOST`, with age. Continue streaming the last safe/predicted setpoint while coasting. On `LOST`, immediately stop offboard, command `hold()`, and return a structured degraded result. Test PX4’s actual offboard-loss action and set the corresponding failsafe parameters deliberately.

### B2 — Frame, detection, and ownship pose cannot be synchronized

Evidence: camera consumers call `seq()`, `raw()`, and proposed `stamp()` independently ([camera.py:64](/home/quenouille/drone/agents/core/camera.py:64), [camera.py:73](/home/quenouille/drone/agents/core/camera.py:73)); a camera callback can replace the frame between those calls. `World.drone_state()` holds only the latest pose, with no pose history or timestamp alignment ([model.py:57](/home/quenouille/drone/agents/world/model.py:57)). `GzPoses` likewise keeps only the newest truth sample ([gzposes.py:37](/home/quenouille/drone/agents/core/gzposes.py:37)). Therefore neither “same-frame look annotation” nor “same-tick accuracy” is guaranteed.

Concrete fix: introduce one immutable atomic object:

```python
Frame(seq, sim_stamp, width, height, rgb)
```

returned by `cameras.snapshot(i)`. Detection must retain that exact frame identity. Maintain timestamped ownship pose/attitude and ground-truth ring buffers, interpolate them to the frame’s sim timestamp, and join eval samples by timestamp rather than polling “in the same tick.”

### B3 — The box-to-world geometry is invalid for this target set

Evidence:

- The vertical projection uses a linear pixel-to-angle approximation, while horizontal bearing correctly uses a pinhole `atan` model ([projection specification](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:377)).
- V1 ignores vehicle pitch and roll ([spec §6.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:926)); those errors are largest during a fast chase, not only during hover.
- The dynamic world has airborne movers at z=8–12 m ([make_dynamic_world.py:34](/home/quenouille/drone/sim/worlds/make_dynamic_world.py:34)), while `box_ground_range()` assumes the contact footpoint lies on z=0 and `VisionContacts` emits z=0.
- The controller is supposed to fly at 12 m in `d2_shadow` ([d2_shadow.yaml:33](/home/quenouille/drone/evals/tasks/dynamic/d2_shadow.yaml:33)); several other targets are almost level with the camera, where ground-plane range is unobservable or explosive.

Concrete fix: use full camera intrinsics and extrinsics plus timestamp-aligned vehicle attitude. Prefer aligned depth from the existing `gz_x500_depth` model for range. If depth is unavailable, intersect a 3-D ray with a declared target support plane and limit v1 to ground movers. Airborne movers must be bearing-only until another ranging method exists.

### B4 — The command channel can replay old flight commands and cannot interrupt a flight action

Evidence: `/pilot/user_input` is specified with transient-local chat QoS ([spec topic inventory](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:307)). A restarted pilot can receive retained commands and starts its cursor at zero, like the current loop ([drone.py:62](/home/quenouille/drone/agents/swarm/drone.py:62)). The loop drains commands serially; a 60–120 s `track()` prevents processing “stop,” “hold,” or “land.”

Concrete fix: use `RELIABLE/VOLATILE` command QoS, command IDs, acknowledgements, and persisted deduplication. Put flight actions under a cancellable supervisor. Emergency hold/land must bypass the LLM queue and cancel the active tool safely.

### B5 — `identified_target` cannot be graded by the proposed eval contract

Evidence: the spec alternately says the check examines `track`/`report` names ([spec §3.8](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:644)) and that the oracle never grades report text ([spec §4.3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:781)). Today `grade()` receives only `WorldTrack`, oracle specs, and a run-meta dict containing steps/crash ([runner.py:399](/home/quenouille/drone/evals/runner.py:399)); tool trace and contact-to-truth association are absent. Visual names such as `vis_person_0` do not encode a Gazebo mover identity.

Concrete fix: record a structured `TargetLockEvent(contact_id, frame_stamp, selected_by_tool)` in the trace. At evaluation time only, associate that contact’s measurement to ground truth at the same timestamp and pass the resulting truth ID into `run_meta`. Never infer success from free-form report text.

## Major findings

### Mj1 — Current baseline architecture is misstated

The interactive swarm is not currently ground-truth-fed: `DroneAgent` does not construct or pass `GzPoses` ([drone.py:25](/home/quenouille/drone/agents/swarm/drone.py:25), [drone.py:36](/home/quenouille/drone/agents/swarm/drone.py:36)), and `agents/swarm/run.py` never creates one ([run.py:47](/home/quenouille/drone/agents/swarm/run.py:47)). Ground truth is injected by the eval harness ([runner.py:222](/home/quenouille/drone/evals/runner.py:222)). Thus interactive `track()` currently raises “no mover feed”; only eval/operator paths are ground-truth-fed.

Fix: correct §1/§2 to distinguish interactive and eval wiring. Use that distinction as the M1 baseline.

### Mj2 — The detector is duplicated across processes, despite being described as one perception source

The spec intentionally creates one detector in the pilot and another in the observatory ([spec §3.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:461)). They will have different sequence counters, timing, contact IDs, and possibly results; CPU work is duplicated. The UI overlay will not necessarily represent what actually fed flight control.

Fix: run camera acquisition and perception in one process and publish timestamped detections/contact state to pilot and observatory, or make the pilot process authoritative and expose its results over ROS/IPC.

### Mj3 — Contact naming and the M3 gate are inconsistent

`VisionContacts` specifies `vis_{cls}_{k}`, where `k` is an association index. The M3 gate hardcodes `vis_mov_1` ([spec M3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1022)). A first observed mover would normally be index 0; vision cannot know it corresponds to Gazebo’s `mov_1`.

Fix: have `detect()` return opaque stable IDs, and pass the selected ID directly to `track()`. Gates must discover the ID from detections, not assume a truth-derived name.

### Mj4 — The decoy plan is under-specified and incompatible with the color fallback

All target movers share the same orange material ([make_dynamic_world.py:55](/home/quenouille/drone/sim/worlds/make_dynamic_world.py:55)). A color-blob backend cannot distinguish a true target from orange decoys, and greedy nearest-neighbor association will ID-switch when same-class targets cross.

Fix: define the target-vs-decoy visual evidence explicitly: distinct class, marker, shape, or temporal behavior. Add ID-switch rate and track-fragmentation gates. Use motion plus box overlap and appearance/class gating, not ground-point nearest neighbor alone.

### Mj5 — MCP schemas make every declared property required

The SDK converts the shorthand dictionary schema into JSON Schema with every property listed in `required`. Existing transcripts already show `goto` rejecting omitted `target`. The proposed optional `detect.classes` will therefore be required too. Current tests only assert registration, not schemas ([test_drone_tools.py:4](/home/quenouille/drone/tests/test_drone_tools.py:4)).

Fix: use full JSON Schema objects with explicit `required` arrays and `additionalProperties: false`; add schema-level invocation tests for omitted optional fields.

### Mj6 — The prompt describes sensing transitions that the tools do not support

`face()` and `orbit()` resolve only drones/buildings via `World.resolve_xy()` ([model.py:71](/home/quenouille/drone/agents/world/model.py:71), [ops.py:332](/home/quenouille/drone/agents/flight/ops.py:332)). They cannot target a `vis_*` contact. `face()` also returns immediately after issuing the yaw command, without waiting for heading convergence or a new post-turn frame. Thus prompt advice to “face it, then detect” is race-prone.

Fix: allow flight target resolution through the contact provider, add `face(bearing_deg=...)`, wait for heading tolerance, and require a camera frame newer than completion of the turn before `detect()` returns.

### Mj7 — The prompting/eval budget contract is false

The spec says eval prompts include budget lines. In reality `run_cell()` passes `spec.prompt` unchanged ([runner.py:360](/home/quenouille/drone/evals/runner.py:360)); budgets are enforced externally by `_drive()` ([runner.py:239](/home/quenouille/drone/evals/runner.py:239)). The model is not told its wall-clock or tool-call limit unless the YAML prose happens to mention them.

Fix: generate one canonical task-injection envelope containing wall-clock, remaining step budget, safety constraints, and report contract. Test the exact rendered prompt.

### Mj8 — `look` dedupe will rarely save tokens

At 10 Hz, an LLM’s second `look` call seconds later almost always sees a new sequence, so “same seq” dedupe does not address repeated semantically identical views. Also, independent `jpeg_b64()` and detector reads cannot guarantee the image and annotation share a frame.

Fix: operate on the atomic frame object, and implement a minimum age/scene-change policy or an explicit `force=true` parameter. Report that `detect` saves image tokens; it is not “free,” since its textual tool result still consumes context tokens.

### Mj9 — Kimi is technically feasible, but the env recipe is not the official one

Official Kimi documentation confirms the subscription Anthropic base URL, `kimi-for-coding`, image/tool capabilities, and 300–1,200 requests per five hours. It instructs Claude Code to use `ANTHROPIC_API_KEY`, includes the Fable tier variable, and requires onboarding/third-party-mode setup—not `ANTHROPIC_AUTH_TOKEN` as the spec proposes. See [Kimi’s Claude Code configuration](https://www.kimi.com/code/docs/en/third-party-tools/other-coding-agents) and [Kimi Code overview](https://www.kimi.com/code/docs/en/).

The docs also position the subscription service for coding agents and the open platform for product integrations, so non-coding drone-control use should be confirmed against current terms.

Fix: move a minimal Kimi SDK/MCP/image spike before M1. Use the official environment recipe, initialize isolated config/onboarding, verify actual network destination through CLI debug logs rather than assuming `/status` works through SDK `query()`, and confirm allowed usage.

### Mj10 — Kimi cost capture is not guaranteed

The spec says `ResultMessage.total_cost_usd` works regardless of backend. The code only stores whatever the CLI returns ([runner.py:99](/home/quenouille/drone/evals/runner.py:99)); a subscription endpoint may return null, zero, or a Claude-price estimate unrelated to Kimi quota.

Fix: treat Kimi monetary cost as unavailable unless calibrated. Capture request count, token usage, latency, quota errors, and membership tier separately.

### Mj11 — The eval port needs separate truth and flight dependencies

Current `Deps.gzposes` is used both by the sampler and by flight tools ([runner.py:155](/home/quenouille/drone/evals/runner.py:155), [runner.py:223](/home/quenouille/drone/evals/runner.py:223)). The proposed design needs truth for grading while feeding vision to flight. Scripted pilots construct `FlightOps` directly with `gzposes` ([pilot.py:289](/home/quenouille/drone/evals/pilot.py:289)) and cannot exercise `detect()`.

Fix: split `oracle_truth`, `flight_contacts`, and `detector` dependencies. Add a scripted perception client that invokes the same detect/lock/track path as the LLM. Add `VisionContacts.reset()` for every eval cell so filters and IDs cannot leak across anchored resets.

### Mj12 — M1 deletion cannot leave pytest green as scoped

M1 deletes `commander.py`, `fleet.py`, and swarm eval layers, but existing tests directly import them, including [test_track_tool.py:8](/home/quenouille/drone/tests/test_track_tool.py:8), `test_fleet_ops.py`, `test_operator_tools.py`, `test_commander.py`, and several eval tests.

Fix: either shelve deprecated modules for one release, or enumerate every production/test/import deletion and migration in M1. Do not delete shared fleet code until the single-drone replacement and eval port are green.

### Mj13 — “NumPy-free ONNX Runtime” is not realistic

The current project has no NumPy dependency ([pyproject.toml:6](/home/quenouille/drone/pyproject.toml:6)), but Python ONNX Runtime normally consumes NumPy tensors and depends on NumPy. The plan also omits precise YOLO11 output decoding, embedded-vs-external NMS, artifact checksum, and dependency/container changes from M2.

Fix: accept NumPy as a runtime dependency or implement a tested alternative binding. Pin `onnxruntime`, document the exported model’s exact input/output contract and NMS mode, and ship a versioned checksum/provenance manifest.

### Mj14 — Arbitrary model-controlled code and built-in CLI tools remain exposed

`run_mission()` executes model-authored Python with imports allowed in the pilot process ([ops.py:387](/home/quenouille/drone/agents/flight/ops.py:387), [ops.py:404](/home/quenouille/drone/agents/flight/ops.py:404)). `ClaudeAgentOptions` does not set `tools=[]`; current transcripts demonstrate built-in `Bash` remains available despite the stated 14-tool surface.

Fix: remove `run_mission` from the initial rebuild, or run it in an isolated, credential-free subprocess with a narrow RPC API. Explicitly disable built-in CLI tools and test the complete exposed tool list.

## Minor findings

- **Historical evidence is not reproducible.** The spec says 461 `look` calls ([spec §1.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:27)); parsing all current `evals/out/**/transcripts.jsonl` yielded 89. Fix: cite the exact artifact/query or soften the claim.
- **The 26–35% CPU claim is misapplied.** Those numbers are for six drones at 720p ([RESULTS.md:52](/home/quenouille/drone/docs/benchmarks/RESULTS.md:52)), not N=1. Fix: benchmark the intended single-drone detector workload.
- **Topic inventory is incomplete.** The existing observatory also needs vehicle status and battery topics ([server.py:37](/home/quenouille/drone/agents/observatory/server.py:37)); correct projection needs attitude and timestamped pose. Fix: list all runtime and IPC topics.
- **“Ground truth only for grading” is overstated.** `scan()` and obstacle avoidance continue to use exact sidecar building geometry ([perception.py:74](/home/quenouille/drone/agents/perception/perception.py:74), [track.py:99](/home/quenouille/drone/agents/flight/track.py:99)). Fix: call it a known static map, distinguish it from live object truth, and state that assumption.
- **Detector startup failure is too broad.** A missing model currently means `start()` raises and can prevent all piloting. Fix: degrade sensing while retaining safe manual flight, with a prominent health state.
- **Source-version citations are not reproducible.** The lock pins SDK 0.2.107, the host has 0.2.87, and the spec cites internal `types.py:91`. Fix: cite locked version plus symbol/commit, not mutable package line numbers.

## Citation/interface audit summary

The cited repository spans for `LatestStore`/`TopicLog`, `RosBridge`/QoS, `GzCameras`, `GzPoses`, `World`, perception trig, track math, `DroneAgent`, assembler order, `VideoHub`, eval registry, and scripted pilot all exist and substantially match their listed signatures.

The material exceptions are:

- interactive track is not currently wired to `GzPoses`;
- camera ownership is per process, not system-wide;
- “12 async FlightOps primitives” is inaccurate: there are 10 public async operations plus synchronous `scan`;
- missing contacts are not safely tolerated;
- optional MCP fields are required by the shorthand schemas;
- eval prompts do not contain budget lines;
- `identified_target` has no data path into the oracle;
- SDK environment merging/model forwarding are verified, but exact internal line citations are version-dependent.

## Milestone verdicts

| Milestone | Verdict | Required before go |
|---|---|---|
| M1 | **NO-GO** | Fix command QoS/replay, emergency cancellation, exposed-tool policy, schema optionality, test/deletion scope, and correct the baseline description. |
| M2 | **NO-GO** | Define atomic frames, synchronized pose/attitude, calibrated camera/depth geometry, pinned model artifact/dependencies, and deterministic gates. |
| M3 | **NO-GO** | Implement continuous offboard behavior through coasting/loss, structured contact health, stable opaque IDs, contact-aware facing, and reset semantics. |
| M4 | **NO-GO pending M2** | Decide one authoritative perception process and stream its exact frame/detection IDs to the UI. |
| M5 | **NO-GO** | Split truth vs flight dependencies; design structured selection attribution, timestamp joins, detector-aware scripted pilots, and filter reset per cell. |
| M6 | **NO-GO in current order** | Run the Kimi MCP/image/tool-call spike before M1, use official configuration, confirm subscription usage terms, and define non-monetary quota metrics. |

The architecture is recoverable, but M2/M3 need a richer perception/control contract than the proposed `GzPoses` duck type. That two-method abstraction discards exactly the information—measurement age, uncertainty, velocity, health, frame identity, and provenance—that safe vision-fed control and credible evals require.