# Review verdict

**No-go for v3 as written.** The v2 repository audit remains largely sound, but the rangefinder extension introduces several architectural contradictions. Most importantly, a TF‑Luna-class sensor cannot support the spec’s stated distances or the M3 flight geometry, airborne contacts cannot bootstrap into tracking, and the proposed mask association has no mask producer.

The best corrected architecture is:

> RGB detection/tracking + bearing-only acquisition → support-plane/stereo coarse range → short-range single-point ToF for terminal precision, with a gimbal or a deliberately constrained flight envelope.

A single-point sensor remains a good cheap adjunct. It is not a general replacement for depth at the ranges and attitudes this spec claims.

## Repository verification

The high-level v2 basis is accurate:

- The checkout is `feat/dynamic-scenarios` at `7622618`.
- Interactive construction has no `GzPoses`: [DroneAgent](/home/quenouille/drone/agents/swarm/drone.py:25) and [swarm assembly](/home/quenouille/drone/agents/swarm/run.py:47). Evals construct it at [run_evals.py](/home/quenouille/drone/evals/run_evals.py:136).
- Current `track()` rejects a missing feed and silently stops setpoint streaming when a contact disappears, exactly as claimed: [ops.py](/home/quenouille/drone/agents/flight/ops.py:245) and [ops.py](/home/quenouille/drone/agents/flight/ops.py:291).
- Current interfaces and principal line references for `LatestStore`, `TopicLog`, `RosBridge`, `GzCameras`, `World`, `FlightOps`, `TargetEstimator`, VideoHub, and the 23 oracle checks are accurate.
- The camera is patched to 640×360@10 Hz by [swarm_sim.sh](/home/quenouille/drone/sim/launch/swarm_sim.sh:123).
- Four of five movers are airborne, as claimed: [make_dynamic_world.py](/home/quenouille/drone/sim/worlds/make_dynamic_world.py:33).

The important inconsistencies are included in the findings below.

# Findings

## Blockers

### B1. TF‑Luna cannot satisfy the stated range envelope or M3 geometry

**Evidence**

The spec calls the sensor “precise” for the centered target at any altitude, including in the prompt and detect description: [§3.10](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:749), [prompt](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:852), and [detect description](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:919).

TF‑Luna is specified for:

- 0.2–8 m on a 90%-reflective indoor target;
- ±6 cm to 3 m and ±2% from 3–8 m;
- 2° FOV;
- 1–250 Hz;
- only about 2.5 m on a 10%-reflective target.

See the official [TF‑Luna manual](https://en.benewake.com/uploadfiles/2024/04/20240426135946148.pdf).

The existing M3 control task flies at 12 m while `mov_1` is at z=1.2 m: [d2_shadow.yaml](/home/quenouille/drone/evals/tasks/dynamic/d2_shadow.yaml:33) and [mover definition](/home/quenouille/drone/sim/worlds/make_dynamic_world.py:38). Even after the proposed −3 m altitude bias, the vertical separation is 7.8 m. An 8 m slant limit leaves only about 1.8 m horizontally and requires approximately 77° depression—far outside the forward camera’s ~42° vertical FOV. At the nominal 12 m altitude, the vertical separation alone exceeds the sensor range.

**Fix**

Choose and name a concrete sensor and enforce its operating envelope:

- If TF‑Luna remains: define it as a **terminal sensor**, e.g. valid acquisition under 6 m slant, opaque/diffuse targets, and a compatible relative-altitude corridor. Remove 20–60 m and “any altitude” claims.
- For the current 30–60 m mission envelope, choose a long-range unit. The official TF03‑100 specification reaches 40 m on 10% reflectivity and 30 m at 100 klux, with ±10 cm below 10 m and 1% thereafter, but weighs about 86 g and has a still narrower beam. See [TF03‑100 datasheet](https://en.benewake.com/uploadfiles/2024/04/20240426134134211.pdf).
- Keep support-plane or stereo as the acquisition/coarse-range path.

---

### B2. Airborne tracking has a bootstrap deadlock

**Evidence**

The spec says a contact enters `poses()` only after it has a range: [§3.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:547). Airborne detections have no geometric range.

But ToF is associated only after the pilot designates a tracked/contact-of-interest ID, and O6 steers using the already tracked world contact: [§3.10 steps 1–3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:783). Current `track()` requires the ID already to exist in `poses()`: [ops.py](/home/quenouille/drone/agents/flight/ops.py:247).

Therefore:

1. Airborne detection is bearing-only.
2. Bearing-only detection is not in `poses()`.
3. `track(id)` refuses it.
4. The controller never steers the beam.
5. ToF never becomes associated.

No explicit designation API or acquisition state resolves that loop.

**Fix**

Introduce an explicit state machine:

```text
DETECTED_BEARING_ONLY
  → DESIGNATED
  → VISUAL_ACQUIRE (pixel-error yaw/vertical servo)
  → RANGE_LOCKED
  → WORLD_TRACKED
  → COASTING / LOST
```

`track(contact_id)` must accept bearing-only contacts and begin with image-plane visual servoing. Alternatively add `designate(contact_id)` plus `acquire_range(contact_id)`. Do not require a world position before acquisition.

---

### B3. “Beam inside mask” is unimplementable because v3 produces boxes, not masks

**Evidence**

`Detection` contains only `cls`, `conf`, and `xyxy`: [§3.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:474). Yet §3.10 requires the whole beam footprint to lie within exactly one detection mask: [association step 3](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:791).

Segmentation is explicitly deferred: [§6.4](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1117).

This is not merely terminology. An eroded box can still include background between limbs, around irregular shapes, or through an occlusion. Camera–LiDAR fusion practice uses segmentation or probabilistic/cluster association precisely to remove background points inside boxes; see the occlusion-aware rationale in [panoptic camera–LiDAR fusion](https://saemobilus.sae.org/papers/panoptic-based-camera-lidar-fusion-distance-estimation-in-autonomous-driving-vehicles-2022-28-0307).

**Fix**

Pick one:

- Add instance segmentation to M2.5 and change the model artifact to a small segmentation model; or
- Rename the rule to **eroded-box association**, document it as a weaker heuristic, and reject occluded/overlapping boxes. Require substantial inner-box margin and innovation consistency before fusing.

For flight control, a real instance mask is the sounder choice.

---

### B4. A one-sample Gazebo lidar cannot simulate the finite ToF beam or edge mixing

**Evidence**

Gazebo’s `<samples>` count is the number of simulated rays. One sample produces one zero-area ray and publishes a `gz.msgs.LaserScan`; it does not integrate a 2° optical footprint. See the official [Gazebo Harmonic sensor tutorial](https://gazebosim.org/docs/harmonic/sensors/).

The spec nevertheless requires a one-sample `gpu_lidar` plus finite-beam edge mixing: [§3.10](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:771).

Native SDF lidar noise supports Gaussian or Gaussian-quantized noise with fixed parameters. It does not natively provide range-dependent noise, sunlight saturation, signal amplitude, probabilistic dropout, transport latency, or mixed foreground/background optical returns: [SDFormat sensor specification](https://sdformat.org/spec/1.11/sensor/).

**Fix**

Simulate a single-output sensor, not literally a single ray:

- Cast a small 2-D cone, e.g. 5×5 or 7×7 rays over the hardware FOV.
- Convert those returns to one measurement using a documented sensor model: hit coverage, return strength, incidence, reflectivity, nearest/mixed-return behavior.
- Add a deterministic impairment adapter for delay, packet loss, signal saturation, quantization, stale packets, and randomized calibration/extrinsic error.
- Calibrate its distributions against logged hardware data.

Call the result “interface parity plus calibrated impairment modeling,” not “full sim↔real parity.” Physics-grounded noise and material randomization are established sim-to-real practices; see [DREDS](https://www.ecva.net/papers/eccv_2022/papers_ECCV/papers/136990369.pdf).

---

### B5. Altitude-only vertical centering is not generally workable

**Evidence**

O6 proposes at most ±3 m of altitude movement to center a 2° beam: [O6](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:607).

At 8 m, a 1° half-angle allows roughly ±14 cm of lateral centering error. In the 640-pixel image, 1° is only about eight pixels. The spec itself expects several pixels of detector jitter, while a multicopter accelerating at chase speeds pitches by many degrees. Altitude control cannot cancel those rapid body-pitch excursions, and ±3 m cannot bridge the 8–11 m vertical difference to the ground mover.

The current controller controls yaw toward a world point but does not control camera elevation: [ops.py](/home/quenouille/drone/agents/flight/ops.py:302).

**Fix**

Use one of:

- A two-axis gimbal or a small rangefinder-only servo.
- A much wider-beam/multizone sensor and zone association.
- A tightly constrained no-gimbal regime: low acceleration, target near co-altitude, explicit sensor tilt, bounded range, and an altitude corridor derived from target elevation.

Altitude bias may assist slow tracking, but it cannot be the primary boresight actuator for this mission set.

## Major

### Mj1. The 0.5 s median is robust to spikes but wrong for moving-target fusion

At 100 Hz it can contain 50 samples. A trailing median has approximately 0.25 s effective lag; at 12 m/s that is about 3 m. During edge transitions it can also select the dominant wrong surface with high apparent confidence.

TF‑Luna already averages five internal measurements at its default 100 Hz, according to its [manual](https://en.benewake.com/uploadfiles/2024/04/20240426135946148.pdf).

**Fix:** timestamp-join range to the image, reject samples outside a 20–50 ms synchronization gate, then use a short 3–7-sample Hampel/median filter on motion-compensated innovations. Preserve the selected sample time and forward-predict the filtered state to the control tick.

---

### Mj2. The rangefinder interface contradicts itself

**Evidence**

- The topic table says `/range/front` is ROS `std_msgs/String`: [§3.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:326).
- `Rangefinder` says it directly owns a Gazebo node and subscribes to the Gazebo lidar topic: [§3.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:370).
- §3.10 says the contract includes min/max/FOV, but `RangeSample` contains only stamp, range, and quality.
- `sensor_msgs/Range` does not contain a quality field.
- `sim_stamp` is not a hardware-neutral name or clock.
- `ranges()` permits `src="none"` while still requiring a float range.

**Fix**

Define one provider protocol and one canonical message:

```python
RangeSample(
    sample_time,
    receive_time,
    range_m: float | None,
    min_m,
    max_m,
    fov_rad,
    signal_raw: int | None,
    quality,
    status: VALID | LOW_SIGNAL | SATURATED | OUT_OF_RANGE | STALE | CRC_ERROR,
    seq,
)
```

Use `GazeboRangeProvider` and `TfLunaRangeProvider` behind it. Pick either direct Gazebo transport or a ROS bridge, not both.

---

### Mj3. `GzPoses` does not implement the stated control contract

The spec says `GzPoses.velocities()` returns `{}`: [§3.1](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:364). Actual `GzPoses` ends after `anchor()` and has no such method: [gzposes.py](/home/quenouille/drone/agents/core/gzposes.py:45).

O3 unconditionally calls `self.contacts.velocities()`, so the M3 ground-truth control lane would fail.

**Fix:** add `velocities() -> {}` explicitly and list `agents/core/gzposes.py` under M3 modifications, or make O3 use a defined protocol/capability check.

---

### Mj4. The filter lacks uncertainty and catastrophic-outlier protection

The range source switches between support-plane geometry, ToF, and coast predictions, but the same alpha-beta state consumes all of them without source covariance, innovation gating, or multi-hit confirmation. A single valid-looking edge/multipath association can jump the contact and therefore the aircraft.

**Fix:** use a small constant-velocity EKF with bearing/range measurement functions, source-specific covariance, normalized-innovation gating, acceleration limits, and two-hit confirmation after source changes. If retaining alpha-beta, at minimum add Mahalanobis-like residual bounds and never fuse a low-confidence ToF return directly into control.

---

### Mj5. The M2/M3 gates cannot demonstrate the claimed safety or parity

Problems include:

- M2 names `evals/perceive_accuracy.py`, while the module map, §3.8, and M5 name `evals/perceive_eval.py`: [M2](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:1178) versus [§3.8](/home/quenouille/drone/docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md:728).
- M3 requires camera-fed `d2_shadow` before the eval port in M5.
- The current task and pilot hardcode `mov_1`, not runtime `vis_*`: [d2_shadow.yaml](/home/quenouille/drone/evals/tasks/dynamic/d2_shadow.yaml:19).
- `<0.5 m p50` can hide false associations, long outages, and catastrophic tails.
- It is undefined whether errors are slant, horizontal, or 3-D and whether invalid samples are excluded.

**Fix**

Move a minimal single-drone oracle/control lane into M3 and gate all of:

- range availability/valid-return fraction;
- false-association rate;
- p50/p95/max error;
- latency and sample age;
- reacquisition time;
- ID-switch rate;
- sunlight/reflectivity/edge/occlusion cases;
- no-return behavior;
- slant and world-position error separately.

---

### Mj6. Failure coverage is materially incomplete

The current imperfection list covers generic noise, range limits, dropout, and edges, but misses:

- return-strength dependence on reflectivity, target size, and incidence;
- direct-sun saturation and ambient-light degradation;
- water, glass, mirrors, and wet/specular surfaces;
- cover-window crosstalk, dirt, scratches, condensation;
- multipath and ghost/near returns;
- vibration and boresight drift;
- rotor-blade occlusion;
- UART corruption, stale/repeated packets, checksum errors, sensor reset;
- interference from other active IR sensors.

The TF‑Luna manual marks low amplitude and sunlight-overexposed measurements unreliable. Cover glass and contamination are known crosstalk sources; see ST’s [cover-glass guidance](https://www.st.com/resource/en/application_note/an5231-cover-glass-guidelines-for-the-singlezone-timeofflight-sensor-stmicroelectronics.pdf). ToF edge mixing is also a documented optical effect, not ordinary Gaussian noise: [Mask‑ToF](https://openaccess.thecvf.com/content/CVPR2021/papers/Chugunov_Mask-ToF_Learning_Microlens_Masks_for_Flying_Pixel_Correction_in_Time-of-Flight_CVPR_2021_paper.pdf).

The repository’s launcher explicitly downloads Baylands and `Coast Water`: [swarm_sim.sh](/home/quenouille/drone/sim/launch/swarm_sim.sh:106). A generic Gazebo ray will not reproduce optical behavior over water or glass. Transparent/specular surfaces are problematic for both stereo and active depth; see [ClearGrasp](https://doi.org/10.1109/ICRA40945.2020.9197518).

Prop wash itself does not affect light in clean air, but the dust, grass, spray, condensation, and debris it entrains can create near returns or dropouts.

**Fix:** add a failure-mode test matrix with dedicated diffuse, dark, retroreflective, glass, mirror, wet, and water fixtures. Simulate window contamination, sunlight angle, boresight jitter, and particle/spray dropout in the impairment layer.

---

### Mj7. Prompt claims exceed actual control authority and sensor capability

“Keep the target centered” is addressed to the LLM even though `track()` is one blocking call and the controller—not the LLM—owns centering. “Precise at any altitude” and the centered-target claim after the “beyond 60 m” sentence are false for TF‑Luna.

**Fix:** state the real envelope:

> “The controller attempts to center the designated target. ToF range is available only while the target fully covers the beam and is inside the configured sensor range; otherwise tracking uses geometric/bearing-only estimates and may slow or return RANGE_UNAVAILABLE.”

Do not use `PRECISE` without a numeric bound and validity conditions.

## Minor / nit

- §5.1 says behavior was verified against installed SDK 0.2.87, while the actual lock contains 0.2.107: [uv.lock](/home/quenouille/drone/uv.lock:183). Re-run T0 on the locked version.
- `CMD_QOS` is scheduled in both M1 and M2. Assign it to one milestone.
- The new sensor model has no named repo-owned SDF/model files. The current launcher mutates upstream OakD SDF in place with `sed`: [swarm_sim.sh](/home/quenouille/drone/sim/launch/swarm_sim.sh:123). Create a reproducible `x500_depth_range` composite model instead. PX4 already documents front-facing 1-D lidar models and custom model attachment: [PX4 rangefinder simulation](https://docs.px4.io/main/en/sensor/rangefinders).
- The ToF world projection must explicitly use horizontal range `r*cos(elevation)` and preserve slant range; `contact_world(..., slant_range)` currently describes only horizontal bearing.

# Architecture judgment

For **cheap, single-target ranging under 6–8 m**, a single-point ToF is the right cost/weight/compute choice. It is especially attractive as a terminal stand-off or inspection sensor.

For **this specification’s general 20–60 m tracking, ground/airborne altitude spread, and aggressive fixed-camera flight**, it is not the right primary range source. The practical choices are:

- Coarse support-plane/stereo acquisition plus short-range ToF terminal fusion.
- A long-range single-point lidar plus a gimbal/servo.
- Stereo/depth when per-pixel semantic association and vertical freedom matter more than minimum weight.

The existing OAK‑D Lite-class hardware offers roughly 0.8–12 m ideal stereo range, with error growing from under 2% below 3 m to under 6% around 6–8 m, while consuming several watts: [Luxonis depth accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/) and [OAK‑D Lite specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Lite). It is costlier and heavier, but it directly solves association and provides simultaneous range over the image.

An 8×8 ToF array reduces pointing sensitivity but is only a roughly 4 m/60 Hz class solution: [VL53L8CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l8cx.html). Ultrasonic is wider, slower, short-range, and vulnerable to acoustic geometry/prop-wash effects. mmWave radar is the robust outdoor alternative for sunlight, dust, fog, range, velocity, and angle, but costs integration and angular-resolution complexity: [TI mmWave overview](https://www.ti.com/document-viewer/lit/html/SWRA819/GUID-8D12B5C9-CA25-4074-AEBE-733ADE6252E2).

# Milestone decisions

| Milestone | Decision | Reason |
|---|---|---|
| M0 | **GO** | Independent of ranging. Keep the fail-fast backend spike. |
| M1 | **GO, conditional** | Baseline skeleton is coherent after removing the duplicate CMD_QOS scheduling and testing SDK 0.2.107. |
| M2 | **NO-GO** | Range contract, transport, timestamping, concrete sensor, and simulation model are unresolved. |
| M2.5 | **NO-GO as written** | Training produces boxes while M3 requires masks. Decide detection versus instance segmentation first. |
| M3 | **NO-GO** | Bootstrap deadlock, impossible TF‑Luna/altitude geometry, weak estimator gating, and unavailable eval lane. |
| M4 | **GO after corrected M2** | UI architecture is sound and rangefinder-independent, provided the published detection schema is finalized. |
| M5 | **NO-GO** | Depends on corrected M3 and currently contains file-name/order/task-ID inconsistencies. |
| M6 | **NO-GO until M3/M5 pass** | Backend switch is independently reasonable, but its in-sim ladder currently depends on an invalid range/tracking design. |

Overall: preserve the single-point sensor as a **short-range precision aid**, but do not finalize v3 until the acquisition state machine, sensor envelope, mask source, simulator, and vertical pointing strategy are redesigned.