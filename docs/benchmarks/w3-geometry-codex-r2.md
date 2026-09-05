The rerun isolates the controller-created blind cone: tracking dies after the car leaves frame, with no association churn (`docs/benchmarks/w3-rerun.md:32-50`, `docs/benchmarks/w3-rerun.md:54-73`).

1. **Geometry law.** The local SDF references OakD-Lite and fixes it level at +0.242 m (`sim/models/x500_depth/model.sdf:7-14`); HFOV 69° is the repository camera contract (`agents/perception/projection.py:4-19`), yielding VFOV **42.273°**, half-angle **21.136°**. Reserve a 3° bottom margin and conservatively aim at the centre of a 1 m target, \(z_a=0.5\) m:

   `R_min(H) = max(8, ceil((H + 0.242 − 0.5) / tan(21.136° − 3°)))`

   Thus R_min is 9, 12, 15, 18, 20, 24 m at H=3,4,5,6,6.5,8 m respectively. Add the pure helper beside `vfov_deg` at `agents/perception/projection.py:16-20`. Test exact vectors in `tests/test_projection.py`.

2. **Enforce authoritatively in FlightOps, not `control_ref`.** `control_ref` is generic and lacks altitude/profile context (`agents/flight/track.py:94-129`); changing it risks the mover lane. At `agents/flight/ops.py:612-621`, when `hold_altitude=True`:

   - shadow default: `range_m = R_min` instead of `None`;
   - standoff: `range_m = max(requested, R_min)`;
   - orbit: `radius_m = max(requested, R_min)`.

   Construct `OrbitPhase` only afterward. Pass `hold_altitude=True` for operator orbit at `agents/pilot/cmd.py:92-95`. Keep command validation’s absolute 8 m safety bound (`agents/pilot/cmd.py:29-33`); it cannot compute a dynamic floor without altitude. Tests: default-shadow, 12 m standoff, and 15 m orbit all clamp under hold-altitude, while `hold_altitude=False` remains unchanged.

3. **Do not lower the default to 3 m.** At H=3 m/R=8 m, depression to a 0.5 m aim point is **18.9°**: technically visible, but outside the chosen 18.136° safe envelope; R=9 is required. H=2.5/R=8 clears it. That gains only borderline ToF altitude—the horizontal beam can still pass above roofs—while worsening obstacle clearance, ground clutter, and demo quality. Six metres plus radial separation is preferable.

4. **Orbit defaults.** H=6/R=15 gives **21.8°** ignoring the mount and **22.6°** to the ground with it: outside. Even the 1 m target centre has virtually no margin. Safe 3°-margin pairs are approximately `(H,R)=(3,9),(4,12),(5,15),(6,18),(6.5,20),(8,24)`. Set ops-bar orbit default to **20 m / 8°·s⁻¹** at `agents/observatory/static/index.html:246,694,846-849`.

5. **Reject depression-driven descent for this gate.** Its equivalent law would be `H_ref ≤ z_a − 0.242 + R·tan(18.136°)`, but at R=8 it commands about 2.9 m—reintroducing altitude motion and low-clearance risk. Retain validated hard hold plus radial floor; leave the M3b law at `agents/flight/ops.py:867-879` untouched.

6. **W3 acceptance gate.** Command H=6 m; require **90 consecutive seconds without LOST**, altitude **5.5–6.7 m**, and horizontal gap **20–25 m for at least 80 of those 90 seconds**. Then orbit **20 m / 8°·s⁻¹ for 30 s without LOST**. Set Approach/Back-off to **20–30 m**, ±5 m steps, at `agents/observatory/static/index.html:850-858`; the flight floor remains the final authority.