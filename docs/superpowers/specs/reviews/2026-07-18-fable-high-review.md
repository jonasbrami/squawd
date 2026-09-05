# Review: Single-Drone Rebuild Design Spec (2026-07-18)

**Reviewer basis:** every cited file read at the cited lines on the current branch (`feat/dynamic-scenarios` @ 7622618); SDK claims checked against installed `claude-agent-sdk` 0.2.87; Kimi/SDK-bug claims checked against the public web.

**Overall:** the spec is unusually accurate. Of ~40 file:line citations, all but one resolve exactly (store, bus QoS, camera topic, gzposes, world, perception, track constants, ops track lines 245-251/291-296/293-295, all 13 tool line refs, prompt 192-242, drone.py 22-74, run.py agent_env 30-44, VideoHub 150-176 with interval 0.05 = 20 Hz, TIERS 22-28, `_drive` 239-259, oracle CHECKS 474-498 with exactly 23 checks, sampler:22, pilot:229, the EVALS-TRACK quote at 107-112). The interface inventories match the code verbatim. The duck-type contact-source swap (O1) is real: `FlightOps.track` touches `gzposes` only via `poses()`/`sim_time()`. The architecture (extend "LLM plans, classical executes" to sensing) is sound. The findings below are where the spec's claims outrun the code or the sim.

---

## Blockers

**B1. Four of the five dynamic-world movers are airborne; flat-ground footpoint ranging cannot range them.**
Evidence: `sim/worlds/make_dynamic_world.py:34-53` — `mov_0` z=10.0, `mov_2` z=8.0, `mov_3` z=12.0, `mov_4` z=10.0 (flying boxes); only `mov_1` (z=1.2, the d2 rover) is a ground vehicle. Spec §3.3/§6.3 ranges a contact by intersecting the footpoint ray with the ground plane. For a target at z=10 seen from 25 m altitude at 40 m true range, the ground intersection lands at ~67 m — a +67 % *systematic* bias the α-β filter cannot remove; for a near-co-altitude target the depression is ≈0 and range is unobservable (`box_ground_range` returns None), so `VisionContacts` emits nothing and `track` has nothing to eat. R7 and R10 discuss horizon and terrain but never that the sim's own target ladder (d1, d3, d4, d5, w4-class intercepts) is airborne. G1's claim that vision-measured contacts replace ground truth for "the moving-target task class" holds only for d2 as designed.
Fix: amend the spec to scope vision-fed `track` v1 to ground movers explicitly, and pick one: (a) add a `target_alt` plane parameter to `box_ground_range` (LLM- or task-supplied, or estimated from box angular height at known size), (b) author ground-mover variants of d1/d3–d5 for the vision A/B, or (c) accept bearing-only contacts plus own-motion triangulation as a v1.1 item. Silence here would be discovered at M5 as a mystery regression.

**B2. Track-loss behavior is promised everywhere and implemented nowhere — and the spec forbids implementing it.**
Evidence: §3.5 O1 states "No other line of `track()` changes"; but the §4.1 prompt promises "if you lose it, the controller coasts ~1s then gives up," and the M3 gate requires "dropout >1 s ⇒ controller reports degraded, no flyaway." The actual loop (`agents/flight/ops.py:291-308`): when the name vanishes from `poses()` (the *common case* once contacts are vision-fed and `VisionContacts` drops a track after `COAST_S=1.0`), it silently `continue`s without streaming a setpoint until `duration_s` (up to 120 s) expires. Stopping offboard setpoints for >`COM_OF_LOSS_T` (~1 s default) triggers PX4's offboard-loss failsafe — behavior is then a PX4 param, not a tool contract, and the LLM gets no LOST status. Today this path never fires because `GzPoses` never forgets a name; O1 makes it routine.
Fix: O1 must include a second change to `track()`: keep streaming a hold/last-ref setpoint while the contact is missing, and after a loss budget (e.g. 3 s) break and return a legible `LOST target after Xs (last seen E.. N..)` result. Add a unit test with a fake contacts object that goes silent. Delete the "no other line changes" sentence.

---

## Major

**M1. The provenance claim "cited against branch `main`" is false; the rebuild's base branch is wrong or unstated.**
Evidence: `git ls-tree main` — `main` has no `agents/flight/track.py`, no `agents/flight/fleet.py`, no `agents/core/gzposes.py`, no `docs/benchmarks/EVALS-TRACK-2026-07-07.md`, no `evals/tasks/dynamic/`, no track-primitive spec; the current branch is 51 commits ahead. §7 says "Branch `rebuild-single-drone` created at M1 start" without a base. An implementing agent following the spec literally (fork `main`, "port unchanged `agents/flight/track.py`") fails at step one.
Fix: change the header to "against `feat/dynamic-scenarios` @ 7622618" and state the base branch in §7 (or merge `feat/dynamic-scenarios` to `main` first).

**M2. v1 "level flight" attitude assumption is wrong exactly when tracking is exercised.**
Evidence: §3.3 error model budgets ±1° attitude error (~0.5–1 m); the camera is body-fixed (`perception.py:11-13`), and a PX4 multirotor in transit or a 12 m/s intercept (`track.py:14` MAX_SPEED_MPS=12) pitches 10–20°+. That's not noise, it's a large systematic depression-angle error (tan law) plus targets exiting the frame vertically under pitch. The spec defers `/fmu/out/vehicle_attitude` to "v1.1 may" while gating M3 on camera-fed tracking.
Fix: promote the attitude subscription to v1 (`projection.py` already takes `camera_pitch_deg` — feed it pitch, add roll to the bearing rotation), or explicitly gate M3 at low-speed shadow only and add attitude in a named milestone.

**M3. No pose↔frame time alignment.**
Evidence: §3.4 projects with `World.drone_state` (wall-clock-latest PX4 sample) against the seq-latest frame; C1 adds a frame sim-stamp but nothing consumes it for pose lookup. At 12 m/s, 100–200 ms skew is 1.2–2.4 m; during `face`/`orbit` yaw rates, tens of ms of skew is several degrees of bearing → meters of lateral error at range, and the "2× max plausible per-frame displacement" association gate (§3.4) will reject valid measurements mid-turn, manufacturing dropouts that feed B2.
Fix: buffer recent timestamped `vehicle_local_position` samples and interpolate the pose at the frame stamp (the messages carry timestamps; ~15 lines), and state the gate must be tested under a yawing camera.

**M4. Model training is a load-bearing work item that appears in no milestone.**
Evidence: §6.1 decides "train YOLO11n on auto-labeled, domain-randomized Gazebo renders" — a real pipeline (render capture, GzPoses→bbox auto-labeling, randomization, training, ONNX export, accuracy iteration) — but M1–M6 never schedule it; M2/M3 lean on `ColorBlobBackend`. The blob detector is keyed to the target orange (`make_dynamic_world.py:55`) and is therefore single-class and blind to obstacle-class movers (blue-grey, `mov_2`): M5's decoy/`identified_target` tasks and any vision-fed obstacle-mover awareness exceed what the interim backend can express. R1's mitigation covers M2, not M5.
Fix: add the training pipeline as its own milestone (or an explicit M5 pre-task) with a dataset/accuracy gate, or de-scope M5 decoys to same-class-distinguished-by-position and say so.

**M5. Double filtering: `track()` re-differentiates already-filtered positions; the duck type discards the velocity the intercept law needs.**
Evidence: `VisionContacts` runs an α-β filter producing (e, n, ve, vn) (§3.4), but the `poses()`/`sim_time()` contract carries positions only, so `TargetEstimator` (`track.py:29-44`) finite-differences the *filtered* positions at an effective 5 Hz and EMA-smooths again — lag stacked on lag, feeding `intercept_t_go`'s lead solution with a stale, doubly smoothed velocity. At 5 Hz updates with 1–3 m position noise, raw finite differences are ±10–30 m/s before smoothing; convergence time will visibly cost intercept performance. The spec's own open question (b) circles this without naming the double-filter.
Fix: extend the contact contract with an optional `velocities()` (GzPoses returns `{}`, `track()` falls back to `TargetEstimator` — two lines of dispatch), and measure both paths in the M3 gate.

**M6. Mid-track contact renaming is acknowledged (R8) but the mitigation is eval-shaping, not a mechanism.**
Evidence: `vis_{cls}_{k}` with k = association index (§3.4); after a >1 s dropout the track drops and reacquisition mints a new k, so the `track()` loop's `poses().get(name)` misses forever even while the detector stares at the target. R8's "eval tasks sized so the tracked target is the nearest of its class" tunes the benchmark around the weakness.
Fix: on reacquisition, bind a new detection to a recently-dropped coasted track's name when it falls inside the coast gate (name reuse), and/or let `track()` re-key to the nearest same-class contact within a radius, reported in the summary. Cheap, and it converts R8 from a caveat into behavior.

---

## Minor

1. **"Numpy-free" is untenable with ONNX Runtime.** `onnxruntime` hard-depends on numpy (its I/O is numpy arrays), so §6.2's "keeps the project numpy-free" is false the moment §6.1 lands; and pure-Python letterbox + float32/255 over 640×360×3 at 5 Hz is ~1M Python-level ops/frame — likely slower than the 12 ms inference it feeds. Amend to "numpy confined to `agents/perceive`" and use it for preprocessing. (`requirements-swarm.txt` currently has neither — the delta must add both.)
2. **M1 sequencing contradicts §3.1's GzPoses demotion.** Until M3 the pilot still needs `GzPoses` wired into `FlightOps` for `scan`/`track` to function (M2's gate itself says "airborne `detect` lists the orange mover" while `track` is still ground-truth-fed). State that the demotion lands at M3, not M1.
3. **Kimi env recipe contains unverifiable vars — put them in the spike.** `ENABLE_TOOL_SEARCH` and `CLAUDE_CODE_AUTO_COMPACT_WINDOW` are not documented Claude Code env vars I can confirm anywhere; `"ANTHROPIC_API_KEY": ""` assumes empty-string reads as unset. Spike test 1 should assert all of these observable effects, not just the base URL. Also: `ResultMessage` cost for a non-Anthropic backend will be zero/meaningless — the evals `cost_usd` column needs a per-backend caveat, not "works regardless."
4. **`scan` via VisionContacts prints `alt 0m` for every contact** (`perception.py:101-102` prints `cz`; §3.4 fixes z=0.0) — actively misleading for the airborne movers of B1. Suppress or label the altitude field for vision contacts.
5. **Citation nits:** `types.py:91` is `AgentDefinition.model`, not `ClaudeAgentOptions.model` (~:1673); the functional claim is nonetheless true — `--model` is passed verbatim (`subprocess_cli.py:272`) and the env merge is exactly at `subprocess_cli.py:430-436`. Appendix says FlightOps has "12 async flight primitives" (it's 10 async + `scan`); M1 says "14-tool surface … minus `detect`" (that's 13). §4.2 lists `orbit` (:61) and `face` (:90) under "kept unchanged" but silently drops their drone-target clauses ("a drone," / "a drone ('drone_1'),") — those are edits; label them like `goto`'s. The "461 `look` calls" figure appears nowhere in the repo (transcript-derived; mark it as such). Camera hfov 1.204 rad lives in the PX4 model dir inside the container, not this repo — unverifiable here but consistent with `perception.py`'s docstring; 640×360@10 Hz confirmed (`run_swarm_demo.sh:28-29`, `sim/launch/swarm_sim.sh:127-132` patches the stock 1920×1080).

## Nits

- `agents/perception` vs `agents/perceive` is a package-naming landmine for humans and agents alike; consider `perception/` (pure math) and `vision/` (detector runtime).
- §4.1 prompt says "detect … ALWAYS prefer" twice (prompt + tool description) — fine, but the `look` T2 dedupe note ("use the previous image") assumes the previous image survived context compaction; harmless, occasionally confusing.
- Open question (c): keep `run_mission` — d2's own task header documents the trajectory-matching strategy class that `track` supersedes but geometry-heavy tasks (figure-8s, capstones) still need; it's verified working and free to keep.

---

## Externally verified (M6 inputs)

The Kimi Code base URL `https://api.kimi.com/coding/` for `ANTHROPIC_BASE_URL` and `kimi-for-coding` = K2.7 Code are confirmed by Moonshot's docs; SDK issue #677 is real and its confirmed workaround is exactly the spec's `cli_path=shutil.which("claude")`.
Sources: [Kimi Code Docs](https://www.kimi.com/code/docs/en/), [Kimi Code third-party tools](https://www.kimi.com/code/docs/en/third-party-tools/other-coding-agents.html), [claude-agent-sdk-python#677](https://github.com/anthropics/claude-agent-sdk-python/issues/677), [Kimi Code membership guide](https://www.kimi.com/help/kimi-code/membership-guide).

---

## Verdict per milestone

| Milestone | Verdict | Conditions |
|---|---|---|
| **M1** skeleton | **GO** | Fix the base-branch statement (Major M1); note GzPoses stays wired until M3 (Minor 2). Everything else cited exists and ports cleanly. |
| **M2** detector + detect/look | **GO** | Decide the numpy/preprocessing question first (Minor 1); scope the p50<5 m accuracy gate to the ground mover (`mov_1`) — it is unachievable for airborne movers under flat-ground ranging (B1). |
| **M3** vision-fed track | **NO-GO as written** | Becomes GO after: track-loss/abort semantics added to O1 (B2), attitude compensation in v1 or an explicit low-speed-shadow scope (M2), pose↔frame interpolation (M3), and a contacts-goes-silent unit test. The d2 gate itself is well chosen — `mov_1` is the one mover the geometry supports. |
| **M4** observatory | **GO** | Straightforward; VideoHub and server structure support it as claimed. |
| **M5** evals + perception grading | **CONDITIONAL GO** | Requires the training pipeline scheduled (M4-finding) or decoys de-scoped to the blob detector's single class; the GzPoses-vs-camera A/B must state it covers ground movers only until B1 is resolved. |
| **M6** Kimi switch | **GO** | Endpoint, model name, and the #677 workaround verified externally; the four spike tests are the right gate — add assertions for the undocumented env vars (Minor 3). |

**Bottom line:** approve the architecture (the duck-typed contact source and detector-as-text are the right moves, and the citation discipline is excellent); amend the spec for B1 and B2 before M3 work starts, restate the base branch, and schedule model training. Nothing here says rebuild the plan — it says the spec's realism claims currently exceed the sim's geometry in one specific, fixable way.
