1. **Fix the three incontrovertible control bugs:** use `elevation_deg`; require fresh `foot_px`; carry the altitude state from acquisition into tracking. Add tests using the real `ContactView`, not a `SimpleNamespace`.

2. **Make the eval truth-bias genuinely continuous:** during acquisition and tracking, update  
   `z_sp = -(physical_target + EMA(EKF_alt − gz_alt))` at the controller rate. Keep this explicitly eval-only. Do not freeze the correction at handoff.

3. **Instrument one causal run before touching PX4 parameters:** record `vehicle_land_detected` flags, `actuator_armed`, `vehicle_status.nav_state`, Offboard-control flag, input `trajectory_setpoint.z/vz`, controller-local position setpoint, `vehicle_local_position.z/vz/z_deriv/dist_bottom_valid`, thrust setpoint, estimator height-source flags/resets, and `gz_z`. The first timestamp where each changes will settle the diagnosis.

4. **Remove the envelope-context race:** only the flight controller should set it, using measured vehicle speed. The scoring collector must be read-only.

5. **Replace −3.4° with vertical bbox-centering and target metadata:** height/base must be per designated object. Then rerun `mov_1` with physical center z=1.2 and preferably an inward/radial shadow path whose actual speed remains ≤3 m/s.

6. **Run one diagnostic A/B with `COM_DISARM_LAND=-1`:** not as the solution. If descent is unchanged but motor shutdown/recovery differs only after contact, that confirms land detection/auto-disarm is secondary.

7. **Use `mov_0` only as a diagnostic after fixing its 0.4 m/9.8 m geometry.** It is valuable for isolating low-altitude effects, but it is presently incompatible with the hard-coded range cue and awkward for the ≤3 m/s availability envelope.

I would not tune `LNDMC_*`, enable EKF range fusion, or declare M3b blocked on PX4 land detection until steps 1–4 are done.
tokens used
242,466
The v9.x diagnosis is partly reversed: the evidence strongly supports “a frozen EKF-relative altitude setpoint drives the physical vehicle into the ground”; it does not show that PX4 land detection initiates the descent. Land detection may become a secondary latch after contact, but `gz_z≈0 + COASTING + OUT_OF_RANGE` alone cannot establish that.

There are also several concrete code defects that invalidate parts of the stated v9.8 control story.

## 1. PX4 land detection and altitude control

This checkout uses PX4 v1.15.4. For a multicopter, the relevant detector parameters are `LNDMC_*`, not fixed-wing `LNDFW_*`:

- `LNDMC_TRIG_TIME`: total hysteresis time, default 1 s.
- `LNDMC_Z_VEL_MAX`: vertical-motion threshold, default 0.25 m/s.
- `LNDMC_XY_VEL_MAX`: horizontal-motion threshold, default 1.5 m/s.
- `LNDMC_ROT_MAX`: rotation threshold.
- `LNDMC_ALT_GND`: ground-effect classification height; it is not a “declare landed below this altitude” threshold.
- `MPC_THR_MIN`, `MPC_THR_HOVER`, `MPC_USE_HTE`: determine the detector’s low-thrust tests.
- `MPC_LAND_SPEED` and `MPC_LAND_CRWL`: constrain the permissible `LNDMC_Z_VEL_MAX`; `MPC_LAND_ALT1/2/3` mainly schedule autonomous landing speeds.
- `COM_DISARM_LAND`: auto-disarms after `landed`; default 2 s. Setting it negative disables auto-disarm, not land detection or the position controller’s ground-contact response. [PX4 land-detector documentation](https://docs.px4.io/v1.17/en/advanced_config/land_detector), [PX4 v1.15 parameter reference](https://docs.px4.io/v1.15/en/advanced_config/parameter_reference), [PX4 auto-disarming documentation](https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm).

Most importantly, PX4 does not declare a multicopter landed merely because physical height is approximately 0.5 m. In altitude-controlled flight it requires low thrust, low motion, and commanded descent. If a downward distance estimate is available it also requires distance-to-ground below a hard-coded 1 m threshold; otherwise that proximity check is skipped. See the exact [v1.15.4 detector source](https://github.com/PX4/PX4-Autopilot/blob/v1.15.4/src/modules/land_detector/MulticopterLandDetector.cpp).

Your Offboard packets contain a position setpoint plus a finite vertical velocity feed-forward of zero. MAVSDK/PX4 define that velocity as feed-forward; it does not prevent the position loop from climbing, but it also means the detector does not see an explicit downward velocity command in those packets. [MAVSDK `PositionNedYaw`](https://mavsdk.mavlink.io/v1.4/en/cpp/api_reference/structmavsdk_1_1_offboard_1_1_position_ned_yaw.html), [PX4 Offboard semantics](https://docs.px4.io/v1.15/en/flight_modes/offboard).

Therefore:

- There is no good reason to tune `LNDMC_*` to solve this gate.
- Increasing `LNDMC_TRIG_TIME` or making the velocity thresholds artificially strict can delay detection, but that masks the descent and weakens safety.
- `COM_DISARM_LAND=-1` is useful only as a one-run diagnostic: if the behavior changes only after actual contact, auto-disarm was a secondary latch.
- If PX4 actually changes `nav_state` to Land, that is not the land detector changing modes. Investigate Offboard-loss/failsafe or an explicit Land command.

The cleaner PX4 mechanism for true AGL hold would be a downward range sensor and terrain hold/following. `MPC_ALT_MODE` uses terrain estimates from a downward distance sensor and is documented for Position/Altitude modes, not as a replacement for a local-NED Offboard z controller. Feeding your horizontal target ToF into `EKF2_RNG_CTRL` or selecting it with `EKF2_HGT_REF=Range` would be physically wrong. [PX4 terrain hold/following](https://docs.px4.io/v1.16/en/flying/terrain_following_holding), [PX4 EKF range-height behavior](https://docs.px4.io/v1.15/en/advanced_config/tuning_the_ecl_ekf).

Also, “baro drift” is probably a misnomer here: the selected SITL airframe enables simulated GPS but explicitly disables simulated barometer input at [4001_gz_x500](/home/quenouille/drone/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500:16), and this PX4 version defaults `EKF2_HGT_REF` to GPS. Audit the live parameters and estimator control flags before changing any EKF parameters.

## 2. `mov_1` versus `mov_0`

Keep `mov_1` for the actual M3b gate after fixing the altitude controller. It is the easier and more faithful beam-association target:

- Its 1.2 m height gives three times the vertical angular target of `mov_0`.
- Physical altitude 1.2 m puts the horizontal beam through its center.
- It directly tests the specified low-speed/co-altitude envelope.

`mov_0` is useful as a diagnostic—“can acquisition work away from ground?”—but it is not a one-line `TARGET` substitution:

- Its center is z=10 m and height 0.4 m, so its base is z=9.8 m.
- The code hard-codes every `"target"` as height 1.2 m and base z=0.6 at [contacts.py](/home/quenouille/drone/agents/vision/contacts.py:345). The initial-lock bbox-height cross-check would therefore predict roughly three times the true `mov_0` range and likely reject the valid ToF return.
- `mov_0` travels at 4 m/s, above the contractual ≤3 m/s reliable envelope if the vehicle follows with a fixed offset. It can demonstrate a short stationary acquisition, but not honestly substitute for the 45 s in-envelope availability gate without a different pursuit geometry.
- Its eroded vertical acceptance band is only about 0.224 m tall, versus about 0.672 m for `mov_1`.

The elevation target should not be a fixed −3.4°. For a box of height \(h\), horizontal distance \(R\), and a footpoint/base measurement, the base target that centers the beam is:

\[
e_\text{base,target}=-\tan^{-1}\left(\frac{h/2}{R}\right)
\]

Examples:

| Range | `mov_1`, h=1.2 m | `mov_0`, h=0.4 m |
|---:|---:|---:|
| 5 m | −6.84° | −2.29° |
| 8 m | −4.29° | −1.43° |
| 10 m | −3.43° | −1.15° |
| 15 m | −2.29° | −0.76° |
| 30 m | −1.15° | −0.38° |

The best implementation is simpler: servo the detection’s vertical center to the principal point, rather than servoing its footpoint to a range-dependent angle. Add a fresh `aim_px`/bbox-center field to `ContactView`.

## 3. Control reasoning flaws

Several are material.

1. The elevation servo is currently dead.

   `_acquire()` reads `obs.elev_deg` at [ops.py](/home/quenouille/drone/agents/flight/ops.py:214), but the actual DTO field is `elevation_deg` at [contact.py](/home/quenouille/drone/agents/core/contact.py:62). The gate’s telemetry correctly uses `obs.elevation_deg`. The unit test misses this because its fake invents `elev_deg` at [test_track_ops.py](/home/quenouille/drone/tests/test_track_ops.py:208).

2. The “continuous drift EMA” stops at handoff.

   `sample_drift()` runs during rendezvous, but after the loop the gate evaluates `alt_ekf(PHYS_ALT)` once and passes that frozen scalar to `track()` at [tof_fusion_gate.py](/home/quenouille/drone/evals/tof_fusion_gate.py:302). The collector does not update drift. Thus growing estimator bias necessarily lowers physical altitude.

3. Recomputing `off_d` provides no drift correction.

   `World.drone_state()` defines `world_alt = -lp.z` at [world/model.py](/home/quenouille/drone/agents/world/model.py:70). Therefore:

   \[
   off_d=lp.z+world\_alt=lp.z-lp.z=0
   \]

   Recomputing it every cycle is algebraically redundant. `down = off_d-alt` is a correct local-NED setpoint, but `alt` remains an EKF-relative coordinate—not physical AGL.

4. Even a working acquisition elevation servo would be discarded after lock.

   `_acquire()` mutates its local `alt`, returns only a boolean, and `track()` then resumes using the original argument. Its acquired physical-altitude correction is not carried into the tracking controller.

5. Blind recovery is effectively unreachable after the first detection.

   `ContactView.foot_px` is explicitly the last accepted footpoint and remains populated in COASTING at [contacts.py](/home/quenouille/drone/agents/vision/contacts.py:885). `_acquire()` treats any non-`None` `foot_px` as fresh and resets `last_seen` every iteration, so the `elif b` blind-sweep branch never runs. Freshness must be gated by `age_s`, a frame sequence, or a “seen this frame” flag.

6. Acquisition success is too weak.

   `_acquire()` returns success whenever the name enters `contacts.poses()`, not specifically when the SM reaches `RANGE_LOCKED`/`WORLD_TRACKED` with `range_src=="tof"`. A geometry-born or predicted pose can satisfy it.

7. The fusion-envelope speed is wrong and raced.

   The track loop passes `hypot(est.ve, est.vn)` as `own_speed_mps` at [ops.py](/home/quenouille/drone/agents/flight/ops.py:699), but `est` is the target estimator, not vehicle velocity. Simultaneously the collector overwrites the same context with a fabricated 0.5 m/s at [tof_fusion_gate.py](/home/quenouille/drone/evals/tof_fusion_gate.py:266). The 5 Hz collector and 10 Hz controller race, so association depends on timing. Use actual `vehicle_local_position.vx/vy`, from one owner only.

The elevation sign itself is correct: if the observed base is above its desired elevation, increasing altitude moves the base downward in the image. The fixed target and DTO mismatch are the problems.

## Sanity-check of the v9.x tail

The tail does not, by itself, implicate PX4 land detection:

- `COASTING` is your perception SM and means range measurements are stale; it says nothing about PX4 flight state.
- `OUT_OF_RANGE` says the horizontal beam no longer hits the target.
- `gz_z→0` proves physical descent/contact.
- None proves `vehicle_land_detected.ground_contact`, `maybe_landed`, or `landed`.
- With a frozen EKF-relative altitude setpoint, growing positive `EKF_alt − gz_alt` directly commands progressively lower physical altitude. At the ground, PX4 may still believe it is above its requested estimator altitude and therefore is not being given a real climb request.
- The purported elevation correction cannot have helped because it reads the wrong DTO field.

My most likely causal order is:

1. Drift estimate is frozen at handoff.
2. EKF-relative position hold sinks the physical vehicle.
3. The target moves above the horizontal beam; ToF goes background/OUT_OF_RANGE.
4. Vision transitions to COASTING.
5. On actual contact, PX4 land detection may subsequently suppress thrust and/or auto-disarm—but that is secondary and not established by the present evidence.

## 4. Next steps, ranked by evidence × simplicity

1. **Fix the three incontrovertible control bugs:** use `elevation_deg`; require fresh `foot_px`; carry the altitude state from acquisition into tracking. Add tests using the real `ContactView`, not a `SimpleNamespace`.

2. **Make the eval truth-bias genuinely continuous:** during acquisition and tracking, update  
   `z_sp = -(physical_target + EMA(EKF_alt − gz_alt))` at the controller rate. Keep this explicitly eval-only. Do not freeze the correction at handoff.

3. **Instrument one causal run before touching PX4 parameters:** record `vehicle_land_detected` flags, `actuator_armed`, `vehicle_status.nav_state`, Offboard-control flag, input `trajectory_setpoint.z/vz`, controller-local position setpoint, `vehicle_local_position.z/vz/z_deriv/dist_bottom_valid`, thrust setpoint, estimator height-source flags/resets, and `gz_z`. The first timestamp where each changes will settle the diagnosis.

4. **Remove the envelope-context race:** only the flight controller should set it, using measured vehicle speed. The scoring collector must be read-only.

5. **Replace −3.4° with vertical bbox-centering and target metadata:** height/base must be per designated object. Then rerun `mov_1` with physical center z=1.2 and preferably an inward/radial shadow path whose actual speed remains ≤3 m/s.

6. **Run one diagnostic A/B with `COM_DISARM_LAND=-1`:** not as the solution. If descent is unchanged but motor shutdown/recovery differs only after contact, that confirms land detection/auto-disarm is secondary.

7. **Use `mov_0` only as a diagnostic after fixing its 0.4 m/9.8 m geometry.** It is valuable for isolating low-altitude effects, but it is presently incompatible with the hard-coded range cue and awkward for the ≤3 m/s availability envelope.

I would not tune `LNDMC_*`, enable EKF range fusion, or declare M3b blocked on PX4 land detection until steps 1–4 are done.
