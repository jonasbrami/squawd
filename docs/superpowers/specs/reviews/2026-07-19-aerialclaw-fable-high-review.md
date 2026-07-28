# AerialClaw → Single-Drone Rebuild: Borrow Review

**Basis:** our spec `docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md` (v4 FINAL) + `agents/`, `evals/` on `feat/dynamic-scenarios`; their code read directly at `/tmp/aerialclaw-src` (all file:line refs are theirs unless prefixed "ours").

**TLDR verdict:** AerialClaw is a two-tier codebase — the ops tooling (doctor/preflight scripts, mock-adapter CI policy) is genuinely well-engineered, while nearly everything on its marketing surface (four-layer memory, approval gate, safety envelope, skill evolution, device protocol) is either dead code, dashboard-ware, or documentation fiction that no Python ever loads. We should **ADOPT one pattern** (health-gated startup), **ADAPT three small ideas** (mock ops for CI + anti-silent-mock policy, a consolidated enforced envelope, eval-gated strategy snippets), and **REJECT the other nine candidates**. Their live agent loop and perception stack are direct empirical confirmation of our principles 1 and 2 — their own guardrail code documents the failure modes.

---

## Candidate-by-candidate

### 1. Soft-skill Markdown strategy docs + dynamic skill generation — ADAPT (static half) / REJECT (dynamic half)

**(a) What it actually is.** Soft skills are Markdown files in `skills/soft_docs/` (no frontmatter, loose `##` sections: overview / when-to-use / numbered flow / cautions). `SoftSkillManager` (`soft_skill_manager.py:25-108`) scans the dir and extracts only the `## 概述` overview; the one-line summaries go into every planner tick, and the **full** doc is injected only for three hardcoded skill names selected by substring keyword match on the goal (`agent_loop.py:383-390`). "Executing" a soft skill is a no-op that returns the doc text (truncated to 800 chars) back into the loop for the LLM to follow (`exector.py:107-134`). Content is real pilot-style heuristics — lawnmower pattern with 15–20 m spacing, "approach in 20 m increments, observe between each", z-sign conventions. Dynamic generation (`dynamic_skill_gen.py:28-208`): at task end, a Counter mines repeated skill-tuples (≥2 occurrences) from `data/skill_chains.json`, one LLM call writes a new `.md` into `soft_docs/` — **no Python is generated or exec'd anywhere** (grep-verified); the "sandbox" in their safety config guards code that doesn't exist. Skill retirement exists but defaults to `dry_run=True` with no live caller.

**(b) Genuine value.** The kernel is real: task-family strategy text measurably shapes LLM planner behavior, and their docs encode the same kind of knowledge our PLAN prompt section does (spec §4.1). The dynamic half has negative value: LLM-authored mutable prompt state, unvalidated (no check that referenced skills even exist), with a broken pruning path.

**(c) Fit.** Static strategy text is orchestration-layer prose — no principle conflict. Runtime self-modification of the prompt directly breaks principle 4: an eval result is meaningless if the prompt mutated mid-sweep, and it breaks our per-cell reset discipline (ours: `VisionContacts.reset()`, anchored repeats).

**(d) Cost.** S. Slots into **M5**: optional per-task-family strategy paragraph (task YAML field or prompt section), A/B'd on the eval ladder — kept only if it moves pass rates.

**(e) Verdict: ADAPT** — hand-written, version-controlled, eval-gated strategy snippets; **REJECT** dynamic generation (unmeasurable self-modification; even their retirement path is dead).

### 2. Four-layer memory + reflection engine + vector store — REJECT

**(a) What it actually is.** Three disconnected memory stacks. The advertised "four layers" (`memory_manager.py:74-80`) are a 20-item RAM deque plus three `VectorStore` collections — and `MemoryManager` is instantiated in exactly one place, lazily, to serve **read-only dashboard stats endpoints** (`server.py:2697-2760`); its write path requires a `memory_manager` argument no live caller ever passes (`reflection_engine.py:446-471`, `agent_loop.py:712`). `consolidate()` is a logged no-op with a TODO (`memory_manager.py:229-244`). The live loop actually uses flat markdown files (`MEMORY.md` appended via a hand-rolled section splicer) plus one `experiences` vector store — local MiniLM with TF-IDF fallback, JSON file rewritten in full on every add, brute-force cosine that truncates vectors to `min_len` to paper over incomparable TF-IDF dimensions (`vector_store.py:246-256`). The reflection engine is one real LLM call per task (`reflection_engine.py:394-397`) whose richest sinks (vector memory, skill stats, `SKILLS.md` — a file that doesn't exist in the repo, so that sink silently no-ops at `:213-214`) are all disconnected on the live path: it degrades to "LLM appends markdown bullets." No cloud dependency anywhere — that part is clean.

**(b) Value.** Minimal for us. Our pilot flies short tasked missions; persistent cross-task memory is actively harmful to eval reproducibility (cross-cell contamination). Post-run "lessons" for us live in `docs/benchmarks/` written from oracle-graded data, which is strictly better than LLM self-reflection.

**(c) Fit.** Conflicts with principle 4 (ungraded state influencing behavior across cells) and KISS (their own system demonstrates the accretion failure: three subsystems, the most sophisticated one dead).

**(e) Verdict: REJECT** — it's mostly dead in the donor codebase; the live remnant is markdown-append, which we don't need.

### 3. Approval gate — REJECT

**(a) What it actually is.** **It does not exist.** No confirm-before-execute anywhere: `execute_skill`/`ai_task` run skills with zero confirmation; the documented `APPROVAL_REQUIRED (202)` / `SAFETY_VIOLATION (403)` codes (`docs/DEVICE_PROTOCOL.md:489-491`) are never emitted; the `approval_levels` YAML is never loaded (grep across all `.py`: zero hits). The nearest artifact is the `ask_user` skill (`cognitive_skills.py:448-503`) — **AI-initiated**, blocking on a class-level global `threading.Event` (concurrent asks collide), and on a 60 s timeout the AI **proceeds autonomously** ("操作员未回答，自行判断").

**(b/c) Value/fit.** Nothing to borrow — the feature is fiction, and the ask-then-proceed-on-timeout pattern is an anti-pattern for a gate. Our estop supervisor (spec §3.6: independent task, LLM-bypass, cancels a 120 s tool mid-flight) is strictly stronger and already designed with an M1 gate.

**(e) Verdict: REJECT** — nothing exists to adopt; our estop covers the real operator-override need.

### 4. Command filter (param-range validation) — ADAPT (concept only)

**(a) What it actually is.** No config-driven filter exists. The only enforced limits are ~4 hardcoded per-adapter clamps: `_MIN_ALT=2.0, _MAX_ALT=200.0` (`mavsdk_adapter.py:27`, `px4_adapter.py:15`), speed `min(max(s,0.5),10.0)`, silent clamp + `logger.warning` — never reject, never ask. These **contradict** the YAML (120 m vs 200 m) and each other: `motor_skills.py:207` *raises* the speed floor to 15 m/s, which the adapter then caps at 10. `velocity_control` (`server.py:2526`) validates nothing at all. Preconditions are parsed from strings but almost every skill declares `preconditions=[]`, so the check is vacuous.

**(b) Value.** The concept — one declarative envelope checked at the command boundary — is sound, and their implementation is the perfect negative example: config/code drift, silent clamping, per-layer disagreement. We already have the pieces (geofence at connect, `clamp_ref_alt`, `MAX_SPEED_MPS`, task ceilings) scattered across FlightOps/track.

**(c) Fit.** Pure classical-execution-layer; supports principle 1.

**(d) Cost.** S. **M1** (tools rewrite is happening anyway) or M3a: one envelope dataclass in `flight/ops.py`, checked pre-dispatch, with **explicit legible rejection/clamp text in the tool result** (tool results must be verifiable — principle 4), covered by unit tests. Constants live in code next to enforcement — no YAML.

**(e) Verdict: ADAPT** — consolidate our existing clamps into one tested envelope with explicit reporting; import their lesson (enforced-or-delete), not their code (there is none).

### 5. Mock adapter — ADAPT

**(a) What it actually is.** `mock_adapter.py` is mostly instant teleport (`takeoff` sets position and sleeps 0.1 s; `fly_to_ned` teleports and sleeps ≤0.5 s); the one honest bit is a single-Euler-step `set_velocity_body` (`:100-124`) added so the cockpit path "isn't a fake mock-control success." Its real value is the surrounding policy: `smoke_mock.sh` runs repo-check → compileall → pytest → UI build in CI against the mock, and `adapter_manager.py:104-115` **refuses to silently fall back to mock** unless `AERIALCLAW_ALLOW_MOCK_FALLBACK=1`; `sim_quickstart.sh:179-202` polls adapter status and hard-fails unless `adapter=="px4" and connected`.

**(b) Value.** A sim-free kinematic `FakeOps` implementing the FlightOps surface would let the full pilot loop (agent ↔ MCP tools ↔ ops ↔ contacts contract) run in pytest/CI without Gazebo/PX4 — today our fakes are per-test. The anti-silent-mock policy directly targets a documented pain of ours (launch scripts "succeeding" with N−1 drones / dead sub-services).

**(c) Fit.** Test-side only; no principle contact. Their teleport code itself is trivial — nothing worth porting literally.

**(d) Cost.** S. **M1**: a ~100-line kinematic fake behind the FlightOps interface, used by the T0 tool-surface test and the estop test; launch scripts adopt fail-loud-never-fake.

**(e) Verdict: ADAPT** — write our own fake + adopt their no-silent-mock policy; don't port their code.

### 6. Manual/AI switching + WASD control — REJECT

**(a) What it actually is.** `state.mode ∈ {manual, ai}` gates `execute_skill` vs `ai_task`, but the WASD path (`velocity_control`, `server.py:2526-2553`) **checks neither mode nor the busy lock** — the cockpit can stream body-velocity setpoints mid-autonomous-flight, racing the agent's offboard setpoints to the same adapter, last-setpoint-wins, no arbiter. Cockpit UX itself is polished (200 ms-throttled WS, key-up → zero-velocity stop).

**(b/c) Value/fit.** Manual nudge control is a nice debugging affordance, but an unarbitrated second writer into PX4 offboard is exactly the mixed-authority hazard our design excludes: one authoritative 10 Hz stream (track), one interruption path (estop). Their arbitration model is demo-grade and unsafe.

**(e) Verdict: REJECT** for v1 — the estop button (M4) is our operator override. If manual teleop ever matters, it needs a hard mode interlock designed from scratch, not this.

### 7. Multi-camera (5-cam) setup — REJECT

**(a) What it actually is.** Five identical 640×480@5 Hz fixed cameras on `x500_lidar_2d_cam/model.sdf:28-215` (front 15° down, rear, left, right, straight-down). In practice: **only `cam_front` is ever consumed automatically** (by the passive VLM loop at 0.125 Hz); the other four are UI tiles or on-demand skill grabs — 97 %+ of frames are UI-only. Two parallel camera abstractions (`GzCamera` per-call subscribe/unsubscribe vs `GzSensorBridge` streaming) do the same job.

**(b/c) Value/fit.** Our design answers off-axis sensing with `face`/`orbit` + one forward detector; the whole projection chain (§3.3) and the co-boresighted ToF beam (§3.10) assume a single forward camera. Five cameras = 5× detector load, per-camera extrinsics in projection, and a fatter perception surface — against KISS, for a capability their own code barely exercises.

**(e) Verdict: REJECT** for v1. A down-camera for landing/ground search is the only sliver worth a future note, post-M6.

### 8. 2D 360° LiDAR — REJECT

**(a) What it actually is.** The sensor SDF is **not in the repo** — `model.sdf:12-15` includes `model://lidar_2d_v2`, which doesn't exist in the tree (samples/range/rate undefined; only decoder fallbacks of 0.1–30 m exist). Consumption is two disconnected paths: a 3 m min-range **veto** on `fly_relative` only (not `fly_to`), wrapped in a bare `except` that silently skips on error (`motor_skills.py:589-638`); and an 8-sector nearest-obstacle **prose summary** ("障碍物: 正前方24.5m[视觉+雷达]") injected into the planner prompt every tick (`daemon.py:235-297` → `agent_loop.py:440`) — i.e. real-time avoidance delegated to a cloud LLM reading text. No reactive controller, nothing between skill calls.

**(b/c) Value/fit.** Unknown-environment obstacle sensing is a real gap-class, but not ours: our worlds carry known building footprints (principle 3 keeps the known map in the flight path — `scan`, obstacle refusal/clamp), and dense obstacle avoidance is an explicit §3.10 non-goal. Their LLM-reads-prose usage violates principle 1 outright; their veto-gate is a worse version of what our known-map clamping already does. And there's no sensor definition to borrow anyway.

**(e) Verdict: REJECT.** If unknown-world avoidance ever enters scope, it gets a classical reactive layer and an eval gate — a fresh design, not this.

### 9. Health-gated startup (doctor scripts, health gates) — ADOPT (the pattern)

**(a) What it actually is.** The strongest-engineered part of their repo. `doctor_gazebo.sh`: 7-section diagnostic (host tools, PX4 checkout + SITL binary, world/model SDF assets down to per-sensor greps of the model file, MicroXRCEAgent, `import mavsdk`, server presence, `--live` port/process/topic probes), PASS/WARN/FAIL with `exit 1` on FAIL. `sim_quickstart.sh:424-438` runs it as preflight and **aborts on failure**, then polls until the flight adapter is genuinely `px4 && connected` — explicitly refusing mock (`:179-202`). `/api/adapter/status` separates flight-control liveness from sensor liveness. (`/api/health` itself doesn't exist; the server gates nothing — only the shell script does.) `check_repository.py` adds repo-hygiene + stale-reference greps.

**(b) Value.** HIGH — this targets our single most documented pain source: PX4 instance-0 racing its own gz server, the sim-wait loop bailing under `set -e` and never starting agents, blank camera tiles from a missing `websockets` dep, DDS domain merges. Every one of those is a preflight-checkable condition.

**(c) Fit.** Pure ops tooling — orthogonal to all five principles, and it *serves* principle 4 (a run that starts half-broken poisons eval data).

**(d) Cost.** S–M. **M1**: `scripts/doctor_sim.sh` (gz topics present, PX4 ready + instance-0 alive, uXRCE agent, unique `ROS_DOMAIN_ID`, venv deps incl. `websockets`, model SDF present, camera topic publishing) gating `run_single_demo.sh`, every wait with a hard deadline; **M2** adds rangefinder-topic and `x500_depth_range`-model checks. Add a line to the M1 gate: demo script refuses to start on doctor FAIL.

**(e) Verdict: ADOPT** the pattern (checks written fresh for our stack — theirs are PX4-generic enough that a few sections port almost directly).

### 10. Universal device protocol (REST+WS SDK) — REJECT

**(a) What it actually is.** The REST/WS surface (register/token/state/sensor/action relay) is genuinely implemented and tested (`server.py:906-1104`, `test_device_protocol_api.py`) — but token auth is missing on `/action` and all WS state handlers; the documented 10 s heartbeat-timeout failsafe is fake (`last_heartbeat` written in 8 places, **read by nothing** — devices stay "online" forever); the architecture diagram's `DeviceManager/SafetyManager/FlightEnvelope/AuditLog` components don't exist (`server.py:25` notes they were removed); client SDKs are admitted unshipped.

**(b/c) Value/fit.** None for us: single drone, ROS topics as the hardware boundary (§1.3), swarm dropped. This is multi-device fleet plumbing for a fleet we deliberately don't have.

**(e) Verdict: REJECT.**

### 11. Identity documents (SOUL.md/BODY.md) — REJECT

**(a) What it actually is.** `SOUL.md`: static hand-written persona. `BODY.md`: auto-generated self-model — but only **half**-generated: adapter identity, sensor list, and hard-skill catalog come from live runtime objects (`body_generator.py:38-103`), while the motion-envelope and hardware-limit numbers are **hardcoded string literals** (`:51-57`, `:109-112`). The checked-in artifact has an empty sensors section (generated while the bridge was down) and a coordinate-frame statement ("AirSim world coords") that **contradicts** `WORLD_MAP.md`'s NED — a live inconsistency fed to the LLM every task. Injection is flat file concatenation into the system prompt (`agent_loop.py:392-398`).

**(b/c) Value/fit.** The generate-capabilities-from-source-of-truth kernel is right, but our design already implements the durable version: the §4.1 prompt is verbatim-spec'd with the rule "every behavioral claim must match the tool's real semantics," tool descriptions live next to the tool code, and §4.3 has a render test. Their BODY.md is a demonstration of the failure mode a separate identity file invites — half-generated, half-stale, self-contradicting. A persona layer adds tokens, not capability.

**(e) Verdict: REJECT** — our prompt-next-to-code + render-test approach is the correct form of this idea.

### 12. Skill evolution / statistics — REJECT

**(a) What it actually is.** `skill_evolution.py` accumulates per-skill time-series of the **LLM's self-reported** performance label (`good|acceptable|poor`) from reflection output (`:79`) — not measured success — into a JSON file. Analyses (degraded-skill detection, param drift) are reachable only via manual API endpoints; **nothing changes autonomously** — no prompt update, no selection change, no doc rewrite in the loop. A parallel `skill_memory.py` tracks *measured* booleans but is RAM-only and essentially unread. The two notions of "skill success" are inconsistent with each other.

**(b/c) Value/fit.** Our eval harness is the correct instantiation of this exact idea: oracle-graded, per-task/tier, Wilson CIs, sim-state ground truth. Their version is self-graded and inert — strictly dominated.

**(e) Verdict: REJECT.**

### 13. safety_config envelope — REJECT (as a pattern; enforcement folded into #4)

**(a) What it actually is.** A 125-line YAML (blacklist, whitelist, `confirm_required`, flight envelope with 120 m ceiling / geofence / battery-RTL thresholds, approval tiers, sandbox, audit) that **zero lines of Python load** — grep for every key across the repo hits only docs and the YAML itself. `SafetyViolationError` is defined and never raised. The YAML even claims the blacklist is "hardcoded-validated" — untrue. Real limits are the four adapter constants of candidate #4, which disagree with it.

**(b/c) Value/fit.** Pure negative lesson, and a sharp one: **a safety config that isn't enforced and tested is worse than no config**, because it manufactures false confidence and drifts from the constants that actually govern. Our rule follow-through: envelope values live in code at the enforcement point, with tests — no parallel declarative file.

**(e) Verdict: REJECT** the pattern; the enforcement idea ships via candidate #4's ADAPT.

---

## What we explicitly do NOT take — and our two standing rejections, re-examined against their code

**LLM-every-step loops — rejection CONFIRMED, now with evidence.** Their live path (`agent_loop.py:357-633`; the advertised two-stage planner in `planner_agent.py` is a **dead import**, never called) is one LLM call per tick, up to 50 ticks, plus side-calls (tactic planner on repetition, safe-return planning, reflection chain) — 50+ serial GPT-4o round-trips per task with the full system prompt + skill table + unbounded action history re-sent every tick (no caching, no truncation: O(N²) token growth). The tell is their own guardrail code: anti-repetition enforced by faking `SkillResult(success=False)` after 3 repeats, JSON-parse-failure counters, and a hardcoded list of Chinese "I am flying" phrases to detect **action hallucination** (`chat_mode.py:209-217`) — failure modes that only exist because the LLM sits inside the execution loop, free-forming skill names from a prose catalog with no real tool-calling. Our shape — one agent turn per command, MCP-typed tools, classical 10 Hz `track` — designs this entire bug class out. Keep it.

**Cloud VLM in the perception loop — rejection CONFIRMED.** Their *entire* semantic perception is a cloud VLM (default **OpenAI GPT-4o**, `vlm_analyzer.py:50-52`) returning JSON with natural-language directions and **LLM-guessed distances** — no boxes, no pixels, no measurement. There is **no local detector anywhere in the repo**: `detect_object` is an admitted mock ("暂未接入 YOLO", `perception_skills.py:94`); every `cv2` usage is JPEG encoding. A background loop calls the VLM on `cam_front` every 8 s while flying (`passive_perception.py`, started at `server.py:387-408`), and obstacle "safety" is the planner LLM reading prose sector summaries. This is precisely the architecture our G1/§5.4 forbids — and it validates the local-YOLO + ToF + EKF pipeline as the actual differentiator of our rebuild, not an implementation detail. Also confirms the Kimi-orchestration-only decision: their per-tick token burn is the cost curve we're opting out of.

**Also not taken:** the fictional safety/approval/heartbeat layer (candidates 3/10/13 — nothing exists); `SharedMemory` (never instantiated); `llm/base_client.py` (entire second LLM abstraction, dead); the `gazebo_direct` adapter's `ok or self.connected` success-laundering (`:131` — the exact opposite of our verifiable-tool-results principle); the meta-lesson being that accreted parallel subsystems (3 memory stacks, 2 camera abstractions, 2 LLM layers, 2 planners) are what KISS + eval gates exist to prevent.

---

## Integration order (ADOPT/ADAPT, ranked by value/cost)

| # | Item | Verdict | Cost | Slots into | Notes / milestone conflicts |
|---|------|---------|------|-----------|------------------------------|
| 1 | **Health-gated startup**: `doctor_sim.sh` + preflight-gated `run_single_demo.sh`, flight-vs-sensor liveness split | ADOPT (pattern) | S–M | **M1** (script + gate line), rangefinder/model checks added at **M2** | No conflict — strengthens the existing M1 sim-smoke gate; directly retires our documented launch gotchas (PX4 i0 race, `set -e` bail, missing `websockets`, DDS domain). All waits hard-deadlined. |
| 2 | **Fake FlightOps + anti-silent-mock policy**: kinematic sim-free fake behind the ops interface; launch scripts fail loud, never fall back to fake | ADAPT | S | **M1** | No conflict — serves the T0 exposed-tool-list test and the estop cancel test without booting Gazebo; policy line added to `run_single_demo.sh`. |
| 3 | **Consolidated command envelope**: one envelope dataclass at the FlightOps boundary (speed/alt/geofence/task ceiling), explicit clamp/reject text in tool results, unit-tested; constants in code, no YAML | ADAPT (concept) | S | **M1** or **M3a** (with O2/O6, where `clamp_ref_alt` routing already lands) | Mild overlap with existing §3.10 alt-bias routing — implement as the same mechanism, not a second one. Anti-drift rule: enforced-or-deleted. |
| 4 | **Eval-gated strategy snippets**: optional hand-written per-task-family strategy paragraph, A/B'd on the ladder, kept only on measured lift | ADAPT | S | **M5** | Adds one cell dimension to M5 sweeps — schedule inside the Kimi/OAuth quota discipline; never runtime-mutable (that half rejected). |

Everything else: **REJECT** (2, 3, 6, 7, 8, 10, 11, 12, 13 as detailed above). Nothing in the adopt list touches the M0→M6 dependency chain or any gate criterion; items 1–3 are gate *strengtheners* for M1/M2, item 4 is an M5 measurement add. Net code intake from AerialClaw itself: approximately zero lines — what we're taking is two ops patterns, one enforcement discipline, and a large, well-documented confirmation that the two loops we refused to build are the two loops that hurt them most.
