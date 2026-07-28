All verification is in. Below is the review.

---

# Review — Single-Drone Rebuild Design v3 (rangefinder extension)

**Reviewer basis:** repo `feat/dynamic-scenarios` @ 7622618 (all cited lines re-verified); web sources current as of 2026-07-19.

**Verification summary (part 1 of the ask):** the v2 file:line inventory survives v3 essentially intact. Spot-checked and CONFIRMED: `store.py:14-52`, `bus.py:34-84`, `camera.py:41-93` (no stamp/snapshot today, as the spec implies), `gzposes.py:17-60`, `model.py:29-87` (no attitude storage, `resolve_xy` drones/buildings only), `perception.py:17-135`, `drone.py:22-74`, `run.py:30-44/47-67`, `commander.py:99-119`, `tools.py` line refs (off by ≤2), `ops.py:245-246/291-295/305/361-370/387-422`, `track.py:1-135` (no `feed_direct`, as O3 assumes), `runner.py` TIERS/Deps/client_for/GzPoses-injection/`_drive`/`:363-366`, `oracle.py:474-498` (23 checks), `pilot.py` (FlightOps-with-gzposes is at `:298-299`, not `:289` — nit), `sampler.py:22`, `video.py:113-176`, `make_dynamic_world.py:33-55` (5 movers, 4 airborne z=8–12, targets orange, obstacle blue-grey), `EVALS-TRACK-2026-07-07.md:108`, both §10 review files, all §3.9 test files, SDK 0.2.87 installed vs 0.2.107 lockfile. The "89 look calls" claim is **exactly right** (87 `mcp__d0__look` + 2 `mcp__d1__look` tool_use records in `evals/out/`). The sim spawns `gz_x500_depth` (OakD-Lite: IMX214 RGB, stock hfov 1.204 rad, patched to 640×360@10 Hz by `sim/launch/swarm_sim.sh:131-133`; plus a `StereoOV7251` depth camera, clip 0.2–19.1 m). No rangefinder/lidar code exists anywhere yet — §3.10 is genuinely NEW. The v3 problems below are therefore about the new architecture and the internal consistency of the v3 edits, not citation rot.

---

## Findings (ranked)

### BLOCKER-1 — "TF-Luna-class" cannot cover the spec's own operating envelope; the sim↔real parity hard requirement is self-violated

TF-Luna's datasheet: **0.2–8 m at 90% reflectivity, 2.5 m at 10% reflectivity**, ambient-light immunity 70 klux (direct sun ≈ 100 klux) ([Benewake datasheet](https://en.benewake.com/uploadfiles/2024/04/20240426135921367.pdf)). The spec's envelope: `VisionContacts(max_range_m=120.0)` (§3.4), M3 gate at ≤30 m slant, `detect` text "distances beyond ~60m are rough; the target you keep centered gets PRECISE distance from your forward rangefinder" (§4.2). That is 4–15× beyond the named sensor class — and on a dark target, 12–48×. §3.10 declares sim↔real parity a "hard requirement", then mandates simulating a sensor the named hardware cannot be: parity is broken by construction. The sensor class that actually covers 30–120 m outdoors is TF03/TF350 or LightWare SF20/SF30 (0.3–0.5° beam, 100–180 m at 90%, ~70 m at 10%, $220–300, 77–86 g — [TF03 datasheet](https://acroname.com/sites/default/files/assets/tf03_datasheet_v0.4_en.pdf), [SF20/C](https://lightwarelidar.com/shop/sf20-c-100-m/)); PX4's own stock 1D models (`x500_lidar_front`) simulate exactly the **LightWare LW20/C** ([PX4 vehicle list](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)).
**Fix:** name the sensor class honestly — LW20/TF03-class (there is even a stock `LW20` gz model to crib) — and set sim `min/max` + a reflectivity-scaled effective max to its datasheet; or re-scope ToF to a ≤8 m terminal-phase sensor and say so in G1/§4.2. Either way, delete "TF-Luna" or delete the 30–120 m numbers; they cannot coexist.

### BLOCKER-2 — Body-fixed forward beam + multicopter dynamics: the pointing problem is unsolved, and worst exactly where the spec wants ToF most (intercept)

The spec itself establishes (§3.2, W1) that the vehicle pitches 10–20°+ at 12 m/s; measured quadrotor data says −15° to −25° at 10–15 m/s, and pitch is the *speed actuator*, changing with every acceleration command ([flight-test data](https://journals.sagepub.com/doi/full/10.1177/1756829320923565)). W1 fixes this for the *camera* in software (attitude-corrected projection) — but a physical beam cannot be software-corrected. O6's only pointing authority is yaw (horizontal) and a ±3 m altitude bias (vertical); there is **no pitch-axis authority at all**. With a ~1° half-beam, 1° of pitch = 0.52 m of beam displacement at 30 m, against airborne targets that are **0.8×0.8×0.4 m** (`make_dynamic_world.py:33-54`). Nose-down cruise pitch of 15° at 30 m puts the beam ~8 m under the target. Consequence: ToF association (step 3's "within the beam half-width") holds only near hover at matched altitude — i.e., slow `shadow`; in `intercept` (where O6a promises "measured closing speed → sharper `intercept_t_go`") the beam points into the ground the entire pursuit. No fielded system does body-fixed 1D target ranging: Skydio/DJI are gimbaled-vision ([Skydio](https://medium.com/skydio/inside-the-mind-of-the-skydio-2-b1b78aa6dfa7), [DJI](https://support.dji.com/help/content?customId=en-us03400006832&spaceId=34&re=US&lang=en&documentType=artical&paperDocType=paper)); PX4's forward 1D use is collision prevention with a *rotating* SF45 ([PX4 rangefinders](https://docs.px4.io/main/en/sensor/rangefinders)).
**Fix (pick one, state it in §3.10/O6/R14):** (a) restrict the ToF-fusion envelope explicitly to shadow-mode, low speed, near co-altitude, and rewrite the M3 gate + §4.1/§4.2 "any altitude"/"PRECISE" claims to match; (b) put the sensor on a 1-axis pitch micro-gimbal (this is the field-standard answer); (c) demote ToF and promote the depth camera / radar (see JUDGMENT). Option (a) is the cheapest and keeps the architecture; the current text oversells it.

### BLOCKER-3 — v3 specifies beam-in-**mask** association, but the pipeline has no masks (v3 broke internal consistency with v2's §6.4/§3.4)

§2.4 ("beam footprint inside ONE **mask**?") and §3.10 steps 2–3 ("well inside exactly ONE detection **mask** (margin from **mask** boundaries)") require segmentation masks. But segmentation is explicitly deferred (§6.4), and `Detection` carries only `xyxy` (§3.4); the YOLO ONNX contract (1×84×8400) is boxes-only. A bbox is a bad mask proxy at this target scale: a 0.8 m mover's box at 30 m is ~17×8 px at 640×360, box-fill is partial, and a "footprint inside the box" beam can still be hitting background through non-target pixels — precisely the edge-mixing failure R13 warns about.
**Fix:** respecify step 3 against boxes with two guards: footprint ⊂ box shrunk by an inner margin (e.g. 20–25%), AND a range-consistency gate (accept the ToF sample only if within k·σ of the filter's predicted range, or of the support-plane prior for ground movers). Optionally note that `ColorBlobBackend` genuinely has a mask (HSV blob) and may use it, while `OnnxBackend` is box-only until the §6.4 revisit. As written, M3's "beam-in-mask association (§3.10)" task is unimplementable.

### MAJOR-1 — Airborne-target bootstrap is a chicken-and-egg deadlock; the M3 stretch gate is unreachable as written

§3.4 (v3): a contact enters `poses()` only with a range; "bearing-only detections … are not trackable". O4: `face`/`orbit`/`goto`/`track` resolve targets from `contacts.poses()`. O6: the beam is steered onto the target *inside* `track()`. Chain the three: an airborne detection is bearing-only (no geom range, §1.3) → not in `poses()` → `track(vis_x)`/`face(vis_x)` cannot resolve it → the beam is never steered onto it → it never acquires a ToF range → it never enters `poses()`. The M3 stretch gate ("track an AIRBORNE mover with ToF-fused metric range") cannot be reached by any tool sequence the spec defines.
**Fix:** define an acquisition path — e.g. `face` accepts bearing-only contacts (yaw needs only bearing), plus an explicit acquire behavior in `track` (accept a bearing-only designation, climb toward the target's estimated elevation angle, hold until first beam lock, then fuse), or a task-supplied initial `support_z` (§8 open question (c) — promote it into v1 for exactly this reason).

### MAJOR-2 — No bearing-only filter update during ToF dropouts → airborne tracks will LOST-cycle; §4.1 contradicts §3.4

For an airborne tracked target, a beam slip produces *no measurement at all* (geom unavailable), because §3.4's filter internals only consume full (e,n) position measurements (NN-gating "on projected ground points"). Given BLOCKER-2's pointing reality, slips are the norm, so the filter coasts → `lost_s=2` → LOST, repeatedly. Meanwhile the §4.1 prompt says "off-center or airborne-unranged targets **degrade to bearing-only**" — but §3.4 says bearing-only is *not trackable*. One of these is wrong; the LLM will be told behavior the controller doesn't have.
**Fix:** specify a bearing-only update mode (bearing innovation with held/predicted range, health e.g. `COASTING`), or align the prompt with reality ("loses range ⇒ coasts ~1s then LOST"). This interacts with O2's semantics and must be settled before M3.

### MAJOR-3 — `/range/front` transport is contradictory, and the PX4 auto-bridge side effect is unaddressed

§3.1's topic table lists `/range/front` as a ROS topic (`std_msgs/String` JSON, PX4_QOS) produced by "gz single-ray lidar → Rangefinder reader"; but §3.1's `Rangefinder` prose and the §2.3 diagram say it's a gz-transport Node subscribing the gz lidar topic directly (GzCameras pattern). If gz-direct, the ROS row is spurious (delete it); if ROS, you need a `ros_gz_bridge` — a *second* sim-stack change beyond "the one permitted addition", and `String`-JSON is the wrong shape when the bridge natively converts `gz.msgs.LaserScan → sensor_msgs/Range` with a min-reduce ([ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)). Separately: if the sensor follows PX4 naming (`lidar_sensor_link`/`lidar`), PX4's GZBridge **auto-ingests it as a `distance_sensor` uORB** (gated by `SIM_GZ_EN_LIDAR`, orientation inferred from the sensor pose — [GZBridge.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/GZBridge.cpp)); a forward beam intermittently returning target hits would then feed PX4 (and there is no stock `/fmu/out/distance_sensor` in `dds_topics.yaml` to read it back anyway).
**Fix:** decide gz-direct (recommended — Harmonic's gz-transport13 Python bindings subscribe `LaserScan` natively, matching GzCameras), fix the table, and name the link something non-`lidar_sensor_link` to opt out of PX4 ingestion — document that choice in §3.10.

### MAJOR-4 — A single-ray `gpu_lidar` cannot produce the imperfections §3.10 promises; the sim addition is under-specified

Native gz-sensors lidar `<noise>` supports only constant-σ Gaussian (no distance-scaled σ, no dropout, no latency, no quality — [SDF lidar spec](http://sdformat.org/spec?ver=1.11&elem=sensor)); ranges clamp to `[min,max]` and no-hit publishes `+inf` ([gz-sensors8 Lidar.cc](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/Lidar.cc)); rays are **infinitely thin**, so with 1 sample the "finite beam width / edge effects" requirement — which R13 calls out as something the "sim must model honestly (M2)" — is physically impossible. The established pattern is a small ray bundle over the divergence cone reduced in post (Webots cone-of-rays + per-distance noise lookup [DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor); Isaac's LightBeam array [docs](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/features/sensors_simulation/isaac_sim_sensors_physics_based_lightbeam.html)).
**Fix:** spec the sensor as e.g. a 3×3 `gpu_lidar` spanning the beam divergence, plus a Python shim (inside `Rangefinder` or between gz and it) that min-reduces, flags high intra-bundle spread as edge-invalid, and injects distance-scaled noise, dropouts, latency, and the `quality` field. This also makes `RangeSample.quality` honest — gz provides nothing to derive it from otherwise. Without this edit, the M2 deliverable "single-ray lidar … with the documented noise/dropout/edge imperfections" is self-contradictory.

### MAJOR-5 — `median(window_s=0.5)` breaks the M3 gate's own 0.5 m target during closure

At a 12 m/s closing rate, range sweeps ~6 m across a 0.5 s window; the median of that window lags truth by ~3 m — 6× the M3 gate's "<0.5 m p50" — and corrupts exactly the "measured closing speed" O6a advertises. Fine in shadow (near-zero closing), wrong in intercept.
**Fix:** shorten the default window to ~0.1–0.15 s (10–15 samples at 100 Hz — still plenty for outlier rejection), or use a slope-tolerant robust estimator (Hampel/median-of-residuals about a linear fit). State the closure-rate assumption next to the default.

### MINOR-1 — Forward beam vs ground movers at operating altitude: `detect`'s promise is overstated

From 20–40 m altitude (§3.3's stated regime) a ground rover 30 m out sits 30–50° below the horizon; a forward co-boresighted beam with ±3 m altitude authority can never touch it. §3.4 correctly gives ground movers `geom` ranging — but §4.2's `detect` text ("the target you keep centered gets PRECISE distance from your forward rangefinder") reads as class-independent. Scope the sentence to near-co-altitude targets.

### MINOR-2 — O6's altitude bias can violate task constraints

The ±3 m bias moves the vehicle off the commanded `alt`; oracle checks include `altitude`/`alt_ceiling` (`evals/oracle.py:474-498`), and `clamp_ref_alt` exists (`track.py:99`). Require the bias to route through `clamp_ref_alt` and respect task ceilings, else a ToF-centered track can fail a mission on altitude.

### MINOR-3 — GzPoses `velocities()` written in the present tense

§3.1 says GzPoses "returns `{}` for the contract's `velocities()` half" — no such method exists (`agents/core/gzposes.py` has only `poses/sim_time/anchor`). It's an O1-era addition; mark it as one so a port doesn't silently assume it.

### MINOR-4 — Missing simulated/real failure modes for the R-table

The imperfection list (§3.10) omits the ones the research says dominate outdoors: **sunlight noise floor** (TF-Luna specced to 70 klux; direct sun ~100 klux), **reflectivity-dependent max range** (8→2.5 m TF-Luna; 180→70 m TF03), **water no-return** (NIR absorbed/specularly deflected — Benewake's own datasheet admits it; directly relevant to the baylands world), **glass/specular multi-path** (why TF03 ships 4-echo), and **dust/prop-wash aerosols** during low-altitude rover chases ([905 nm degradation study](https://www.researchgate.net/publication/263472962_Comparison_of_905_nm_and_1550_nm_semiconductor_laser_rangefinders'_performance_deterioration_due_to_adverse_environmental_conditions)). At minimum add reflectivity-scaled max range and a water/no-return note to R13/R10; sunlight can stay a hardware-phase caveat.

### NIT-1 — File-name drift: M2 creates `evals/perceive_accuracy.py`; §2.1/§3.8/M5 call it `evals/perceive_eval.py`. Pick one.
### NIT-2 — `evals/pilot.py:289` → the FlightOps-with-gzposes construction is at `:298-299` (inside `pilot_client_builder`).
### NIT-3 — §3.8's Codex-B5 citation (`runner.py:399-403`) points at where `run_meta` is built; `identified_target` exists nowhere yet (it's the NEW M5 check) — fine, but phrase it as "insertion point", not as an existing path.
### NIT-4 — `/range/front` QoS labeled "PX4_QOS" — moot if MAJOR-3 resolves to gz-direct; otherwise mislabeled (not an `/fmu/out` topic).
### NIT-5 — `ranges()`'s z formula (`alt - range*sin(depression)`) should name the depression source (beam boresight elevation from `attitude_at(sample_stamp)`), and note `geom` z=0.0 vs mov_1's actual z=1.2 for oracle tolerance.

---

## JUDGMENT — is single-point ToF the right call? (part 3 of the ask)

**Qualified no as specced; yes under a narrowed envelope.** The honest state of the art for "cheap metric range to a moving target from a small drone" is: **vision-primary with gimbaled cameras** (Skydio/DJI — no rangefinder at the target at all), **mmWave radar** where range+Doppler matter (people-tracking at 6–14 m stock, ~100 m with beamforming; sunlight-immune; [TI IWR6843](https://www.ti.com/tool/TIDEP-01000)), and 1D lidar only for **altimetry and collision prevention**. Nobody ships body-fixed 1D target ranging, for exactly the pointing reasons in BLOCKER-2. Stereo doesn't rescue the long range: OAK-D-Lite error grows ~quadratically and is unusable past ~10 m real ([Luxonis depth accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy)) — and note the sim's own OakD-Lite depth camera clips at **19.1 m** ([OakD-Lite model.sdf](https://github.com/PX4/PX4-gazebo-models/blob/main/models/OakD-Lite/model.sdf)), so "free aligned depth in sim" also only covers the near field.

Given the project's actual v1 needs (shadow a mover, intercept, decoys, ranges mostly ≤30–60 m in sim), the defensible v3 shape is: **LW20/TF03-class sensor named honestly (B1), ray-bundle simulation (M4), box-margin + range-gate association (B3), an acquisition path (M1), and an explicitly narrowed fusion envelope — ToF fuses in shadow-mode at low speed near co-altitude; everywhere else it's opportunistic** (fuse when the sample passes the gates, never depended on). Vertical centering via altitude is workable *within that envelope* (calm, slow, ±3 m) and not outside it; a 1-axis micro-gimbal is the standard escape hatch if the envelope proves too tight, and mmWave radar is the right §6.3-named future primary for fast/any-attitude target ranging. The "one designated target, LLM plans / classical executes" philosophy fits a 1D sensor well — the spec's error is not the philosophy but the named sensor class and the claimed envelope.

---

## Go / No-Go per milestone

| Milestone | Verdict | Rationale |
|---|---|---|
| **M0** Kimi spike | **GO** | Untouched by v3; spike design still sound. |
| **M1** Skeleton | **GO** | Untouched by v3; §3.9 migration list verified complete against the repo; all named tests exist. |
| **M2** Frames/projection/detector + rangefinder reader + sim lidar | **CONDITIONAL GO** | Camera/projection/detector half is verified and ready. Rangefinder half needs four spec edits first — sensor class (B1), ray-bundle + shim (MAJOR-4), transport decision + PX4-ingestion opt-out (MAJOR-3), median window (MAJOR-5). All are paper fixes, none are research risks; make them, then go. |
| **M2.5** Training | **GO** | Unchanged by v3. |
| **M3** Vision-fed track + O6 ToF fusion | **NO-GO as written — split it** | O1–O5, VisionContacts, LOST semantics are ready (**GO** as "M3a"). The ToF-fusion half ("M3b") is blocked by B2 (pointing envelope), B3 (mask-less association), MAJOR-1 (airborne bootstrap deadlock), MAJOR-2 (dropout/bearing-only contradiction); the stretch gate is unreachable and the 0.5 m intercept-range gate collides with MAJOR-5. Re-spec M3b with the narrowed envelope (shadow, low speed, co-altitude) and an acquisition path, then it's viable. |
| **M4** Observatory | **GO** | Unchanged by v3. |
| **M5** Evals port | **GO** | With NIT-1 naming fix; `ranges()`/`range_src` fields flow through cleanly. |
| **M6** Kimi in-sim | **GO** | Unchanged by v3. |

**Bottom line:** the v3 edits are internally coherent everywhere *except* the rangefinder core, where three self-contradictions (mask-vs-bbox, trackability-vs-prompt, table-vs-diagram transport) and one physics gap (named sensor 8 m vs claimed 120 m; body-pointing a 1–2° beam from a pitching multicopter) need spec-level resolution before M2's sim work and M3's fusion work start. The fixes are all specifiable now — none require abandoning the architecture, only shrinking its claims to what a correctly-named sensor on a gimbal-less airframe can actually do.
