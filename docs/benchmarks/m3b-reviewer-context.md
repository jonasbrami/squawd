# M3b co-altitude bind — reviewer context pack (2026-07-21)

You are advising on a UAV perception+fusion stack (custom ROS2-free bus,
PX4 SITL + Gazebo Harmonic, one x500 with a FIXED forward 640×360 camera
(hfov 69°, level, body-fixed) and a single-point forward ToF lidar (3-ray
bundle, 0.5° wide, level, co-boresighted with the camera). Worktree: this
repo, branch rebuild-single-drone. Design docs:
docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md (§3.10)
and docs/superpowers/specs/2026-07-19-interface-specification.md.

## The milestone (M3b)

Airborne ToF-locked acquisition on the ground mover `mov_1`: a 1.8×1.0×1.2 m
box FLOATING at z ∈ [0.6, 1.8] (centre 1.2), orbiting a 35 m circle at
3.5 m/s (angular rate 5.7°/s, ~63 s lap). Gate: in-envelope slant error
<0.5 m p50 / <1.5 m p95, ≥80% availability, 0 false associations, airborne
acquisition demo. Because the beam is horizontal and body-fixed, the drone
must hold PHYSICAL altitude ≈1.2 m (the box band 0.6–1.8) with the beam
footprint inside the (22%-eroded) mask — call this the co-altitude hold.

## What is PROVEN (438 unit/integration tests green)

- Full machinery: beam association (footprint→mask→consistency),
  acquisition SM (DESIGNATED→ACQUIRING→RANGE_LOCKED→WORLD_TRACKED),
  fusion into the CV-EKF, envelope gating, deterministic consumption.
- The LIVE LOCK once (v9.0): bearing-only → ACQUIRING → ASSOCIATED →
  WORLD_TRACKED; 2 fused samples, slant error p50 **0.43 m**, **0 false
  associations** in every run (association correctly rejects ground
  returns, background, edge-mix).
- The image-servo now works (v9.7: tracks the box's pixel angle to ≈0° and
  holds it; ContactView gained `foot_px` — before that the servo was dead
  code reading a nonexistent field).

## The blocker — the co-altitude hold collapses (v9.8 evidence)

Sequence every run: the chase-rendezvous delivers the drone to ~20–30 m at
physical ≈1.2 m with the box servo-centred; the acquisition starts
(offboard hold + yaw image-servo + segmented 2 m/s creep + elevation
servo); then the **baro/EKF altitude drift** (measured +0.7 m at priming,
growing to +2.3 m over ~3 min) sinks the PHYSICAL altitude toward ~0.5 m;
**PX4's land detection engages**; further offboard climb setpoints are
ignored; the drone settles on the ground (gz_z → −0.01), the box floats
above the camera's +21° FOV top, the track dies blind (lost_s = 2 s by
design). Every gate run's tail: gz_z decays to ≈0 while the SM sits in
COASTING, ToF reading OUT_OF_RANGE (sky) or ground returns.

## Constraints

- Never weaken/redefine the gate or the design's honesty rules (no faking
  ranges; background returns must never fuse; lost_s = 2 s is contractual).
- Product code must be honest (no gz truth in flight code — the EVAL may
  use truth for rendezvous/alt-bias choreography, documented).
- Sim model is fixed: x500_depth + forward ToF (no gimbal, no stereo).

## What we need from you (websearch PX4 docs/source as needed)

1. **PX4 land-detection in SITL**: which params govern it
   (LNDFW_* / MPC_LAND_* / EKF2_*?) — is there a legitimate way to stop
   land-detection hijacking offboard z at ~0.5 m physical, or a cleaner
   mechanism (e.g. altitude source, terrain estimator) to hold ~1.2 m?
2. **Is the co-altitude-at-1.2 m acquisition even the right geometry**, or
   should the demo target mov_0 (airborne 0.8×0.8×0.4 box at z≈10 m on a
   4 m/s line — no land detection, no ground effect, but a much narrower
   vertical beam target)? Trade-offs, and what the elevation-servo target
   should be per box height.
3. **Any flaw in our control reasoning**: offboard PositionNedYaw z =
   off_d − alt (off_d recomputed each cycle from lp.z + world_alt), the
   elevation servo (el_deg up-positive; drive box base to −3.4° ≈ camera
   mid-box at ~10 m; rate ±0.2·dz per 10 Hz cycle), and the drift-EMA
   re-biasing. Spot anything wrong or fragile.
4. Concrete next steps, ranked by (evidence × simplicity).
