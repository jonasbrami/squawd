**Decision: FIX once; do not accept Run 7.** K1 may invalidate an attempt, but K2 and the sub-11 m gaps remain product failures. Given seven converging rounds, allow one bounded R8—not another open-ended cycle.

1. **K1 is environmental only when independently proven.** The PX4/GPS-height failure family is documented at [PROJECT-STATE.md:63](/home/quenouille/drone/docs/PROJECT-STATE.md:63), but Run 7 did not log drone Gazebo altitude, so it cannot be retroactively exempted on `/state` alone.

   Extend [w3_cornerwatch.py:22](/home/quenouille/drone/evals/out/w3_run7/w3_cornerwatch.py:22) to subscribe to `x500_depth_0` alongside `car_1`, logging PX4 altitude, Gazebo z, and sim time. Capture the initial offset `b=px4_alt−gz_z`; classify `INVALID_ENV` only when `|px4_alt−(gz_z+b)|>1.5 m` continuously for ≥0.5 s during a 4 m hold. Add that classification before altitude scoring at [w3_session_verdict.py:46](/home/quenouille/drone/evals/out/w3_run7/w3_session_verdict.py:46). Permit one fresh-container retry; a second invalidation stops the gate as unstable infrastructure. Do not add Gazebo-truth control or production z hardening.

2. **K2 is product-side, but not a yaw-law defect.** Immediately before loss, the box remains horizontally centered while its bottom reaches row 359 ([detwatch_retry.log:1398](/home/quenouille/drone/evals/out/w3_run7/retry/detwatch_retry.log:1398)); measured-bearing yaw already centers fresh detections at [ops.py:850](/home/quenouille/drone/agents/flight/ops.py:850). This is a pitch/depression transient at the level-camera floor, not SW-world geometry or insufficient yaw lead.

   Add a demo-shadow-only image-edge barrier at [ops.py:865](/home/quenouille/drone/agents/flight/ops.py:865):

   `q=clamp((bbox_y2−300)/40,0,1)` for `hold_altitude`, `mode=="shadow"`, and `obs.age_s<0.3`;  
   `R_vis=r_guard+4q`; project the radial reference to at least `R_vis`, and add outward feedforward `min(3, R_vis−gap)` before the existing 6 m/s cap. Thus the 4 m profile expands smoothly from 13 to 17 m only as the target approaches the bottom 60 pixels. Test beside [test_track_ops.py:1245](/home/quenouille/drone/tests/test_track_ops.py:1245): fresh `y2=280/320/345` produces monotonic expansion; stale observations and `hold_altitude=False` produce none.

3. **W3 ACCEPT becomes two explicit sub-gates**, replacing [w3-scope-codex-r4.md:9](/home/quenouille/drone/docs/benchmarks/w3-scope-codex-r4.md:9):

   - Endurance: one valid contiguous engagement ≥55 s, covering ≥4 truth-marked 90° corners/full lap, same ID, no LOST/manual re-lock, and MEASURED again ≤2 s after every corner.
   - Recovery/ops: three successful clicks total; dispatch ≤2 s; one controlled release/re-click ≤8 s from the first eligible box; Approach 14 m, Back-off 18 m, Orbit 15 m/8°·s⁻¹, Stop/Resume, and estop all acknowledged and observed.
   - During valid active tracking: altitude 3.7–4.5 m; gap 12–20 m for ≥80%; never below 11 m; zero ID churn.
   - An evidenced `INVALID_ENV` attempt contributes no pass or fail samples.

4. **Convergence rule:** if the image-edge barrier still loses a vertically clipped target in R8, stop W3 and document the operating-limit blocker—no R9 layering.

**Refuse:** accepting any product-caused LOST, identity swap, sub-11 m gap, or “environmental” exemption without synchronized PX4-versus-Gazebo evidence.