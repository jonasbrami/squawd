## Bottom line

Borrow AerialClaw’s operational patterns, not its subsystems. I would import no AerialClaw module wholesale. Its architecture is broader, more mutable, and substantially less oracle-grounded than the rebuild design.

Against our [design principles](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:72), [authoritative perception architecture](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:470), and [M0–M6 plan](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1234):

- ADOPT one policy: fail closed on backend/source identity.
- ADAPT nine ideas into smaller, oracle-gated forms.
- REJECT five subsystems outright for M0–M6.

This review is against AerialClaw commit `e01adaa73c38fb10ec4bc5e8c0f71915cc7490a8`.

## Candidate decisions

1. Soft-skill Markdown + dynamic generation — ADAPT  
   Cost: M. Slots: M1 infrastructure, M5 activation; dynamic drafts post-M6.

   Actual: `SoftSkillManager` scans Markdown and extracts a `## 概述` summary, with direct create/update/delete operations ([soft_skill_manager.py](/tmp/aerialclaw-src/skills/soft_skill_manager.py:22)). The planner can make two LLM calls: draft from the catalog, then refine after loading full documents ([planner_agent.py](/tmp/aerialclaw-src/brain/planner_agent.py:249)). But the active agent loop only retrieves full docs for three hard-coded keyword families ([agent_loop.py](/tmp/aerialclaw-src/brain/agent_loop.py:372)), and “executing” a soft skill merely returns the first 800 characters of its document as a successful result ([exector.py](/tmp/aerialclaw-src/runtime/exector.py:107)).

   Dynamic generation is weaker still: the LLM output is name-sanitized but not schema/tool-validated ([dynamic_skill_gen.py](/tmp/aerialclaw-src/skills/dynamic_skill_gen.py:126)); the integrated auto-generation path passes `skill_chain` records to code expecting `skill_trace`, so it currently detects no patterns ([agent_loop.py](/tmp/aerialclaw-src/brain/agent_loop.py:824), [dynamic_skill_gen.py](/tmp/aerialclaw-src/skills/dynamic_skill_gen.py:48)). Existing documents have already drifted into AirSim coordinates, cloud `perceive`, and nonexistent modules such as `base_adapter.py` and `adapter_factory.py`.

   Value/fit: concise, reviewable strategy documents and catalog→detail disclosure fit LLM orchestration well. Runtime self-writing, self-activation, and self-deletion conflict with KISS and oracle gating.

   Adaptation: create versioned, human-reviewed `agents/pilot/strategies/*.md`, generated from or validated against the real tool registry. A strategy becomes active only after passing named M5 oracle tasks. Permit LLM-generated drafts only offline; never auto-enable or auto-delete them.

   Verdict: ADAPT — keep progressive disclosure, remove autonomous strategy mutation.

2. Four-layer memory, reflection engine, vector store — REJECT  
   Cost: L. Earliest plausible slot: after M5/M6.

   Actual: “four layers” means a 20-item deque plus three independent vector stores—episodic, skill, and world ([memory_manager.py](/tmp/aerialclaw-src/memory/memory_manager.py:29), [memory_manager.py](/tmp/aerialclaw-src/memory/memory_manager.py:64)). Each query takes up to `top_k` from every store and injects the merged results into planning without a global threshold ([memory_manager.py](/tmp/aerialclaw-src/memory/memory_manager.py:84)). The preferred backend loads `all-MiniLM-L6-v2`; Chroma is created with an ephemeral client, and cosine similarity imports NumPy outside `vision/` ([vector_store.py](/tmp/aerialclaw-src/memory/vector_store.py:40), [vector_store.py](/tmp/aerialclaw-src/memory/vector_store.py:133), [vector_store.py](/tmp/aerialclaw-src/memory/vector_store.py:151)).

   Reflection asks an LLM for lessons, skill ratings, recommended parameters, and world facts, then writes those claims into Markdown and vector memory automatically ([reflection_engine.py](/tmp/aerialclaw-src/memory/reflection_engine.py:286)). Mission success is generally “all tool calls returned success,” not geometric success ([agent_runtime.py](/tmp/aerialclaw-src/runtime/agent_runtime.py:145)). There is even a data mismatch where the detailed episode summary passed by reflection is ignored by `store_episode()`.

   Value/fit: past failure retrieval might eventually improve orchestration, but this implementation creates self-confirming, ungraded beliefs, adds large dependencies, and violates NumPy confinement. OURS already has complete tool traces ([runner.py](/home/quenouille/drone/evals/runner.py:58)) and deterministic grading ([oracle.py](/home/quenouille/drone/evals/oracle.py:1)).

   Verdict: REJECT — if later evidence justifies memory, start with a single append-only oracle-labelled JSONL/SQLite episode ledger and exact metadata filters, not embeddings or LLM reflection.

3. Approval gate — REJECT  
   Cost: S code, large workflow cost. No M0–M6 slot.

   Actual: there is no runtime approval gate. AerialClaw contains an unused `ApprovalRequiredError` declaration ([errors.py](/tmp/aerialclaw-src/core/errors.py:49)) and approval lists in YAML ([safety_config.yaml](/tmp/aerialclaw-src/config/safety_config.yaml:44)); execution dispatch does not consult them.

   Value/fit: confirmation can matter on real hardware, but confirming every takeoff, move, land, or RTL would hang autonomous evals and duplicates the operator’s initial authorization. It is weaker than an enforced envelope plus LLM-bypass estop.

   Verdict: REJECT — no implementation exists to adopt, and per-command approval conflicts with autonomous M1/M5 runs. Revisit only for a real-hardware operating mode.

4. Command filter / parameter-range validation — ADAPT  
   Cost: M. Slots: M1, M3, gates in M5.

   Actual: there is no central filter or typed parameter schema. Skill schemas are descriptive strings ([base_skill.py](/tmp/aerialclaw-src/skills/base_skill.py:40)). `FlyTo` forces speed to at least 15 m/s ([motor_skills.py](/tmp/aerialclaw-src/skills/motor_skills.py:205)), while the MAVSDK adapter silently clamps it to 0.5–10 m/s ([mavsdk_adapter.py](/tmp/aerialclaw-src/adapters/mavsdk_adapter.py:389)). That contradicts both the prompt and the nominal safety configuration.

   Value/fit: very high. Classical boundary validation is exactly where untrusted LLM parameters should be constrained.

   Adaptation: one NumPy-free `SafetyEnvelope`/validator used by `flight/tools.py`, `flight/ops.py`, and `flight/track.py`. Reject missing, non-finite, malformed, or out-of-range user commands; only clamp documented controller-generated references such as `clamp_ref_alt`. Return requested/effective values and stable refusal codes. Add oracle cases proving invalid commands cause no motion and altitude/clearance ceilings hold.

   Verdict: ADAPT — implement the idea centrally; do not copy AerialClaw’s inconsistent local clamps.

5. Mock adapter — ADAPT narrowly  
   Cost: S. Slots: M1–M3 unit tests only.

   Actual: its adapter teleports state, sleeps at most 0.5 seconds, computes travel distance from the origin rather than the current position, and applies body velocities directly to north/east without yaw rotation ([mock_adapter.py](/tmp/aerialclaw-src/adapters/mock_adapter.py:14), [mock_adapter.py](/tmp/aerialclaw-src/adapters/mock_adapter.py:86)). It cannot model dynamics, failure, offboard state, or sim time.

   Value/fit: useful for deterministic unit tests, dangerous as a flight or eval backend.

   Adaptation: retain only narrow fakes already anticipated by the spec—`FakeContacts`, `FakeRangeProvider`, detector fixtures, and small MAVSDK action stubs. Never make a generic mock adapter selectable by the pilot or geometric eval runner.

   Verdict: ADAPT — protocol-level test doubles, never simulated mission success.

6. Manual/AI switching + WASD — ADAPT  
   Cost: M. Slots: M1 authority core, M4 UI, M5 gate.

   Actual: AI→manual sets a stop event but does not wait for the current blocking action to relinquish control ([server.py](/tmp/aerialclaw-src/server.py:1451)). The WASD UI correctly sends zero velocity on key release/unmount ([CockpitView.jsx](/tmp/aerialclaw-src/ui/src/components/CockpitView.jsx:89)), but the server’s velocity handler does not check manual mode, execution ownership, authentication, or parameter bounds ([server.py](/tmp/aerialclaw-src/server.py:2526)). Thus manual and AI commands can race.

   Value/fit: manual recovery and inspection nudges are valuable, but only under the “one control authority” principle.

   Adaptation: introduce an atomic `AUTONOMY | MANUAL | ESTOP` arbiter. AI→manual must cancel the pilot turn, stop offboard/hold, then grant a short-lived deadman lease. Every velocity packet must carry that lease and pass the central envelope; release, disconnect, or timeout causes hold. Gate in SITL: never simultaneous authority, takeover bounded in time, release causes bounded drift.

   Verdict: ADAPT — take the deadman UX, rebuild the authority semantics.

7. Five-camera setup — REJECT  
   Cost: L. Would expand M2, M2.5, M3, M4, and M5.

   Actual: despite a “4 cameras, 15fps” comment, the SDF defines front, rear, left, right, and down cameras, all 640×480 at 5 Hz ([model.sdf](/tmp/aerialclaw-src/sim/models/x500_lidar_2d_cam/model.sdf:22), [model.sdf](/tmp/aerialclaw-src/sim/models/x500_lidar_2d_cam/model.sdf:183)). The sensor bridge subscribes to all five and imports NumPy outside `vision/` ([gz_sensor_bridge.py](/tmp/aerialclaw-src/sim/gz_sensor_bridge.py:33)). A separate `GzCamera` implementation duplicates image ownership and subscriptions ([gz_camera.py](/tmp/aerialclaw-src/perception/gz_camera.py:28)).

   Value/fit: better passive coverage, but at five render streams, additional inference/association load, extra extrinsics, and loss of one authoritative pipeline. OURS intentionally uses vehicle facing/orbit and permits only the ToF model addition in M2.

   Verdict: REJECT — revisit a single downward camera only if an oracle coverage ablation proves the front camera cannot meet a named task.

8. 2D 360° LiDAR — REJECT  
   Cost: L. Would disrupt M2/M3/M5.

   Actual: the repository does not contain the LiDAR SDF; it includes external `model://lidar_2d_v2` ([model.sdf](/tmp/aerialclaw-src/sim/models/x500_lidar_2d_cam/model.sdf:9)). Runtime use is a 3 m directional precheck over four sectors that fails open on exceptions ([motor_skills.py](/tmp/aerialclaw-src/skills/motor_skills.py:589)). Its alleged camera/LiDAR “fusion” simply pairs detected object `i` with LiDAR obstacle `i` ([perception_skills.py](/tmp/aerialclaw-src/skills/perception_skills.py:196)); the detector feeding it is currently a hard-coded person/vehicle result ([perception_skills.py](/tmp/aerialclaw-src/skills/perception_skills.py:94)).

   Value/fit: horizontal proximity sensing can be useful, but a body-mounted 2D scan on a pitching multicopter is not general 3D clearance. It duplicates the known-map refusal path and conflicts with the specified narrow ToF role ([rangefinder design](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:767)).

   Verdict: REJECT — require an obstacle oracle failure first, then design a local occupancy solution if needed.

9. Health-gated startup — ADAPT  
   Cost: M. Slots: M1 foundation, M2 sensing, M4 observatory.

   Actual: `doctor_gazebo.sh` is a useful read-only checklist with actionable failures ([doctor_gazebo.sh](/tmp/aerialclaw-src/scripts/doctor_gazebo.sh:1)). Quickstart hard-gates the PX4 MAVSDK connection and verifies the selected adapter is actually `px4` ([sim_quickstart.sh](/tmp/aerialclaw-src/scripts/sim_quickstart.sh:149)). But `/api/status` reports only initialization/mode/execution state ([server.py](/tmp/aerialclaw-src/server.py:867)), while sensor failure merely warns and startup continues ([sim_quickstart.sh](/tmp/aerialclaw-src/scripts/sim_quickstart.sh:498)).

   Value/fit: strong fit if health is capability-specific rather than an all-or-nothing boot gate; the spec explicitly allows degraded sensing boot.

   Adaptation: add a structured readiness snapshot covering backend identity, MAVSDK connection, telemetry/pose freshness, camera frame age, detector health/inference age, contact health, and range status. Gate each tool on only what it needs; estop/hold/land remain available. Add injected-staleness tests where motion is refused, followed by a normal oracle task after recovery.

   Verdict: ADAPT — copy the diagnostic posture, replace HTTP reachability with freshness-based readiness.

10. Universal REST + WebSocket device protocol — REJECT  
    Cost: L. Post-M6 at earliest.

    Actual: it is a server-side schema, not an SDK—the documentation explicitly says Python, Arduino, and ROS2 production clients do not exist ([DEVICE_PROTOCOL.md](/tmp/aerialclaw-src/docs/DEVICE_PROTOCOL.md:564)). Registration and tokens are in-memory. The action endpoint does not authenticate the caller ([server.py](/tmp/aerialclaw-src/server.py:1087)); after WebSocket authentication, heartbeat/state/sensor/result handlers do not verify that the sender’s socket owns the claimed device ID ([server.py](/tmp/aerialclaw-src/server.py:1621). There is no implemented heartbeat expiry/safety monitor despite the documentation’s claim.

    Value/fit: none for a single fixed PX4/Gazebo/MAVSDK system. It creates a second control plane, larger attack surface, and false hardware generality.

    Verdict: REJECT — use narrow internal protocols such as `RangeProvider`; do not build a universal device bus.

11. `SOUL.md` / `BODY.md` identity documents — ADAPT  
    Cost: S. Slot: M1, refreshed by M2 health/tool additions.

    Actual: `SOUL.md` is mostly personality and first-person framing ([SOUL.md](/tmp/aerialclaw-src/robot_profile/SOUL.md:1)). `BODY.md` is generated from adapters, sensors, and skill registry, but mixes discovered data with hard-coded speed, altitude, localization, and visual-correction claims ([body_generator.py](/tmp/aerialclaw-src/robot_profile/body_generator.py:22)). The committed file is visibly stale: PX4 adapter, disconnected state, AirSim coordinates, and no sensors ([BODY.md](/tmp/aerialclaw-src/robot_profile/BODY.md:1)).

    Value/fit: separating stable role rules from current capability truth is useful. Mutable identity files and personality prose add prompt drift.

    Adaptation: keep the spec’s concise operating principles as static prompt text; generate a read-only capability card at prompt construction from the exact tool registry, backend tier, safety envelope, and current health. Never persist it as runtime-authored Markdown.

    Verdict: ADAPT — generated capability truth, not identity theater.

12. Skill evolution and statistics — ADAPT statistics; reject evolution  
    Cost: S. Slot: M5.

    Actual: `SkillMemory` counts tool-return success and average duration in process memory ([skill_memory.py](/tmp/aerialclaw-src/memory/skill_memory.py:28)). `SkillEvolution` stores LLM-generated `good/acceptable/poor` ratings and recommended-parameter drift in JSON ([skill_evolution.py](/tmp/aerialclaw-src/memory/skill_evolution.py:34)). These labels are not tied to geometric completion.

    Value/fit: regression statistics are valuable; LLM-driven parameter mutation is not.

    Adaptation: extend OURS’ existing `Trace` and `evals/report.py` with primitive latency/error-code counts and scenario-level oracle pass rates, grouped by model, detector artifact, and difficulty. Statistics remain observational and never rewrite prompts, strategies, or controller parameters.

    Verdict: ADAPT — oracle-derived telemetry only.

13. `safety_config` envelope — ADAPT  
    Cost: M, shared with candidate 4. Slots: M1/M3/M5.

    Actual: the YAML claims its limits are hard-coded and unmodifiable, but no runtime Python loads it. It declares 10 m/s maximum speed ([safety_config.yaml](/tmp/aerialclaw-src/config/safety_config.yaml:55)), while motor skills demand at least 15 m/s. Battery, heartbeat, geofence, approval profiles, and blacklist settings are similarly unenforced.

    Value/fit: a single envelope source of truth is essential; dead YAML is actively misleading.

    Adaptation: use one immutable Python configuration intersected with task-specific altitude/area constraints. Enforce it at tool admission and controller reference generation, with PX4 geofence/failsafes as defense in depth. Do not copy approval profiles, shell blacklists, or “permissive” modes.

    Verdict: ADAPT — central executable invariant, not declarative theater.

## Additional useful findings

14. Explicit no-silent-mock/source policy — ADOPT  
    Cost: S. Slots: M1/M2.

    AerialClaw’s adapter manager now refuses silent mock fallback unless explicitly enabled ([adapter_manager.py](/tmp/aerialclaw-src/adapters/adapter_manager.py:100)). That rule directly supports “truth is for grading” and fixed-stack operation.

    Apply it more strictly: production pilot startup must report `px4 + connected`; detector/range/contact outputs always carry source, timestamp, and health; fabricated detections are impossible outside isolated unit tests.

    Verdict: ADOPT — cheap protection against meaningless green runs.

15. Structured result/error taxonomy — ADAPT  
    Cost: M. Slots: M1, extended M3/M5.

    AerialClaw defines exceptions with fix hints, but most safety subclasses are unused ([errors.py](/tmp/aerialclaw-src/core/errors.py:10)). The useful idea is stable machine-readable outcomes.

    Add small result codes such as `NOT_READY`, `INVALID_PARAM`, `BLOCKED`, `LOST`, `TIMEOUT`, and `ESTOPPED` alongside human-readable measurements. This improves LLM replanning, observatory diagnosis, and M5 aggregation without an exception hierarchy.

    Verdict: ADAPT — stable outcomes, not the unused class tree.

## Explicit non-adoptions

The rejection of “LLM every step” needs one qualification.

AerialClaw’s loop invokes the LLM once per completed high-level skill, then dispatches a blocking action ([agent_loop.py](/tmp/aerialclaw-src/brain/agent_loop.py:417)). That is not a 10 Hz control loop, and event-driven replanning after `ARRIVED`, `BLOCKED`, or `LOST` is compatible with our principle. Our SDK tool loop already provides that behavior.

What should be rejected is AerialClaw’s bespoke loop:

- The LLM can declare `done` without a geometric check.
- Repeated behavior triggers another LLM-generated tactic.
- “Safe return” first asks the LLM to plan the return ([agent_loop.py](/tmp/aerialclaw-src/brain/agent_loop.py:635)).
- Runtime reflection then learns from that self-declared outcome.

OURS should retain high-level, event-driven LLM orchestration, while 10 Hz pursuit, estop, hold, RTL fallback, range association, and safety enforcement remain classical.

The cloud-VLM rejection is fully confirmed for the perception/control path. AerialClaw base64-encodes frames to an OpenAI-compatible endpoint ([vlm_analyzer.py](/tmp/aerialclaw-src/perception/vlm_analyzer.py:62)); passive perception asks it to estimate obstacle distances and writes those estimates into the WorldModel ([passive_perception.py](/tmp/aerialclaw-src/perception/passive_perception.py:31), [passive_perception.py](/tmp/aerialclaw-src/perception/passive_perception.py:117)). That is uncalibrated, network-dependent geometry.

The one acceptable exception is already in our spec: Claude-tier `look` for optional open-ended scene semantics, explicitly non-authoritative. It must never supply range, track state, clearance, or controller input. Kimi receives no images, ever ([spec T4](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:652)).

Also do not take:

- Generic multi-simulator adapter/factory architecture.
- Runtime HTTP, file-read, or file-write cognitive skills.
- Mutable `WORLD_MAP.md` scene truth.
- Duplicate camera/perception owners.
- Mock detections labelled successful.
- LLM-written controller parameters or safety advice.

## Ranked integration order by value/cost

1. ADOPT fail-closed backend and source provenance — S, M1/M2.
2. ADAPT central safety envelope + parameter validator — M, M1; extend in M3/M5.
3. ADAPT capability-specific health/readiness and doctor checks — M, M1/M2; expose in M4.
4. ADAPT stable result/error codes — M, M1/M3/M5.
5. ADAPT generated capability card — S, M1/M2.
6. ADAPT oracle-derived primitive statistics — S, M5.
7. ADAPT narrow protocol fakes — S, naturally inside M1–M3 tests.
8. ADAPT curated Markdown strategies — M; loader can land in M1, but activation should wait for M5 oracle evidence.
9. ADAPT manual takeover with deadman lease — M, after core M4; gate in M5.
10. ADAPT offline strategy-draft generation only — M, post-M6 and only if repeated oracle traces justify it.

The principal milestone conflict is M1 scope: items 1–5 could overload baseline parity. Items 1–3 are safety-critical enough to include, while the capability card and full error taxonomy can be thin initially. Strategy activation, statistics, manual piloting, memory, and all self-evolution must not precede M5’s oracle port. Multi-camera and 2D LiDAR directly conflict with M2’s explicitly single permitted sensor addition and should not enter the plan.