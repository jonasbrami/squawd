## 1. Culprit ranking

**Verdict — The primary regression is the M3a/M3b trajectory shaper replacing the original direct moving-reference controller. The 23–34 m gap is lag, not an intentional standoff.**

1. **Trajectory shaper and 1 m/s² acceleration limit.** `control_ref()` still produces the correct shadow command—target plus standoff and target-velocity feedforward ([track.py:91](/home/quenouille/drone/agents/flight/track.py:91)). But `track()` discards that reference, initializes `_shp` at the drone, acceleration-limits it by only 0.1 m/s per tick, integrates it forward, and streams `_shp` instead ([ops.py:586](/home/quenouille/drone/agents/flight/ops.py:586), [ops.py:686](/home/quenouille/drone/agents/flight/ops.py:686), [ops.py:693](/home/quenouille/drone/agents/flight/ops.py:693), [ops.py:706](/home/quenouille/drone/agents/flight/ops.py:706)). That turns the baseline PX4 moving-reference controller into a lagging virtual-carrot controller. Both acceleration and integration assume exactly 0.1 s, making it load-sensitive.

2. **Truth-fed tracking is incorrectly given the beam-geometry altitude profile.** GzPoses has no `observation()`, so the ToF elevation servo itself cannot run. Instead, truth contacts always enter the fallback and descend from the requested 12 m toward `max(2.3, 0.18*gap+1.1)`—only 3.8 m at the 15 m gate ([ops.py:722](/home/quenouille/drone/agents/flight/ops.py:722), [ops.py:735](/home/quenouille/drone/agents/flight/ops.py:735), [ops.py:743](/home/quenouille/drone/agents/flight/ops.py:743)). That violates `alt=12` and introduces a roughly 8 m vertical maneuver while closing; shared thrust/tilt limits can reduce horizontal response. It does not create a horizontal standoff directly.

3. **Conditional amplifier: leaked PX4 pursuit parameters.** `tune_pursuit_params()` persistently sets `MPC_XY_VEL_MAX=6` and tilt to 12° ([ops.py:467](/home/quenouille/drone/agents/flight/ops.py:467)); reset restores only `MPC_XY_CRUISE` ([reset.py:134](/home/quenouille/drone/evals/reset.py:134)). `run_evals` does not call the tuner, so this is excluded in a genuinely fresh PX4 instance, but it could materially worsen a reused-container run.

`within_m` is only logging/intercept termination ([ops.py:677](/home/quenouille/drone/agents/flight/ops.py:677)); it never requests a 15 m standoff. The 7 m keep-out bubble cannot explain gaps above 15 m. Also, the shadow path ignores the soft speed ramp because `control_ref()` ignores `speed` in shadow mode.

Reconstruction anomaly: [ops.py:748](/home/quenouille/drone/agents/flight/ops.py:748) uses `min(ref_u, alt_ref)`, contradicting the claim that a raised building clamp “wins”; it should be `max` for obstacle clearance. That is serious but unrelated to this open-plaza regression.

## 2. Minimal fix

**Verdict — Restore the original direct controller only for observation-less truth shadowing; leave the M3b camera path byte-for-byte unchanged.**

Immediately after `control_ref()`, detect:

```python
beam_capable = callable(getattr(self.contacts, "observation", None))
if mode == "shadow" and not beam_capable:
    ref_u = trk.clamp_ref_alt(self.world, ref_e, ref_n, alt)
    # stream ref_e/ref_n and ff_ve/ff_vn directly, then continue
```

This skips `_shp`, the 7 m beam-oriented shaping, and gap-dependent descent for GzPoses, reproducing the July 6 law: `target+standoff`, velocity feedforward, commanded altitude. VisionContacts exposes `observation()`, so acquisition, shaping, co-altitude servo, WORLD_TRACKED behavior, and association performance remain untouched. Read-only review cannot guarantee the exact dwell, but this is the only minimal change that restores the proven baseline controller rather than retuning it.

## 3. The swing

**Verdict — CPU/real-time-factor and possibly persistent PX4 parameters explain the swing; EKF drift does not.**

`track()` expires after 75 wall-clock seconds ([ops.py:590](/home/quenouille/drone/agents/flight/ops.py:590)), and the oracle also measures dwell using wall-clock sampler timestamps ([sampler.py:38](/home/quenouille/drone/evals/sampler.py:38), [oracle.py:289](/home/quenouille/drone/evals/oracle.py:289)). PX4 and the rover, however, move in simulation time. Lower Gazebo real-time factor therefore provides fewer simulated seconds to cross the initial ~100 m gap before the wall-clock tool ends. The shaper’s hard-coded 0.1 s integration compounds missed or delayed ticks.

Both results report the correct 3.5 m/s estimate, so GzPoses timestamps and the finite-difference EMA were functioning; there is no CV-EKF in this lane. The available artifacts contain neither RTF nor controller-tick counts, so CPU starvation versus leaked `MPC_XY_VEL_MAX` is undecidable. It does not change the ranking: the fresh retry still regressed, proving the new controller remains the root cause.

## 4. Test gap

**Verdict — Existing tests verify dispatch and completion, not the streamed flight contract.**

Add `test_truth_shadow_preserves_direct_reference_and_altitude` beside [test_track_ops.py:162](/home/quenouille/drone/tests/test_track_ops.py:162):

- Fake GzPoses generates the 35 m, 3.5 m/s circle and exposes no `observation()`.
- Fake time advances with 0.1 s plus deterministic jitter.
- Fake offboard records setpoints; a small point-mass PX4 model applies `v_ff + Kp*(p_sp-p)`, capped at 12 m/s.
- Run 75 s from home and assert: position setpoint equals current target plus standoff, altitude stays 12 m, and contiguous gap ≤15 m is at least 45 s.
- A second beam-capable fake asserts the M3b shaped/elevation branch remains selected.

That fixture fails today on both direct-reference and altitude assertions without requiring SITL.