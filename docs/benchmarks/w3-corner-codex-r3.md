Run 3 proves the steady-state layers: 27.4 s, one ID, three recovered flickers, then a 15.1 m corner cut (`docs/benchmarks/w3-run3.md:41-76`).

1. **ACCEPT direct routing for `hold_altitude` shadow.** Change the branch at `agents/flight/ops.py:792-827` to:

   `direct = mode == "orbit" or (mode == "shadow" and (hold_altitude or not beam_capable))`

   Stream `control_ref`’s radial position and target-velocity feedforward directly. For camera-fed held shadow, retain measured-camera yaw from `observation(name).bearing_deg`; use the existing predicted-world yaw only as fallback (`ops.py:804-811`). Keep `set_beam_context` exactly as at `ops.py:813-825`. PX4 still smooths the 10 Hz reference under the existing 6 m/s velocity and 12° tilt limits (`ops.py:500-508`); the CV-EKF filters position/velocity before dispatch. Nothing in M3b changes because `hold_altitude=False` still enters the shaper. Pin: `test_hold_altitude_camera_shadow_uses_direct_reference_through_right_angle_corner`; update `test_coco_vehicle_shadow_holds_commanded_altitude` to expect the first pursuit reference on the ring, while `test_beam_capable_shadow_keeps_shaper_and_altitude_profile` remains unchanged.

2. **REJECT bearing-rate expansion initially.** Image bearing rate mixes target turns, drone translation, yaw lag, and noisy shallow-range corrections—the run recorded a 27.3→17.8 m measurement jump (`docs/benchmarks/w3-run3.md:74-76`). It would expand on ordinary transverse motion and could push detection toward the far-range recall limit. Direct routing removes the known lag without estimating another derivative.

3. **MODIFY the corner interlock into a scoped radial control barrier.** A reference clamp cannot literally constrain actual vehicle position, but held demo shadow can actively prioritize outward recovery. At `agents/flight/ops.py:804-812`, when actual gap `g < R_guard = R_min(H)+1 m`, add to feedforward:

   `v_escape = min(2.0, 0.8*(R_guard-g)) * (me-target)/g`

   Then cap the resulting vector at 6 m/s. This sacrifices tangential following while inside the guard—effectively “slowing angular advance”—until geometry recovers. Also change the implicit lock ring at `ops.py:634-636` from `R_min` to `R_min+2` (20 m at H=6); explicit standoff remains floored at `R_min`. Tests: `test_demo_radial_barrier_commands_outward_relative_velocity` and `test_demo_shadow_default_has_two_metre_transient_reserve`.

4. **REJECT a demo AMAX override.** Scoping it behind `hold_altitude` would avoid mover regression, but retaining the carrot preserves the exact lag architecture that failed. Raising AMAX also produces more pitch transient and still cannot represent an instantaneous 90° velocity change. The direct lane is already proven against a turning mover.

5. **Likely layer 4: adjacent-vehicle identity swap.** Two superclass-compatible cars inside the 5 m world gate can exchange associations without ID churn. Cheapest pre-emption: at `agents/vision/contacts.py:477-489`, prioritize the designated track’s last-bbox image-centre continuity before world-distance ordering; ambiguous crossings coast rather than swap. Pin: `test_designated_vehicle_does_not_swap_when_adjacent_tracks_cross`. Car_2’s 20 m circle needs only 0.8 m/s² at 4 m/s and should fit PX4’s tuned authority; pedestrians are dynamically easier.

6. **Revised W3 gate.** Hold altitude **5.5–6.7 m**; shadow **90 s without LOST**; gap **18–26 m for ≥80/90 s**, with **no measured sample below 17 m**. Default lock ring **20 m**. Orbit remains **20 m / 8°·s⁻¹ for 30 s without LOST**; Approach/Back-off remain **20–30 m**, 5 m steps.