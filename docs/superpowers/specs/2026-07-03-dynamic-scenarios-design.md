# Dynamic scenarios: moving targets & moving obstacles — design

**Date:** 2026-07-03 · **Status:** approved ("go")
**Goal:** add a dynamic difficulty axis to the `evals/` benchmark — tasks whose
world state evolves during the run — to discriminate model tiers above the
now-mapped static ceiling (flat saturated for opus/sonnet; static obstacles:
opus 6/8, sonnet 1/8, haiku 2/7).

Grounded in two investigations (2026-07-03): a live probe of Gazebo Harmonic
mechanics in the `evals-obst` container, and a literature survey of dynamic
UAV benchmarks (AT-Drone, DeTrack, BEDI, AvoidBench, Robotouille, Gaia2,
survivability-calibration arXiv:2404.14848, MTS search literature).

## The core constraint

The agent decides every ~4–10 s (LLM latency + blocking movement tools), so
**tasks must be winnable by planning + periodic re-observation, never by
high-rate feedback**. The screening number for every task:

```
k = v_mover × T_cycle / R_tolerance      (T_cycle ≈ 8–15 s for our agents)
```

k < 0.5 → naive chase suffices (only acceptable on the entry rung);
k ≈ 1–2 → prediction/lead required (the discrimination band);
k > ~3 → requires feedback control → unfair harness artifact, excluded.

## Mechanism (verified live in-container)

- **Movers are kinematic `<model>`s** (box/sphere collision+visual,
  `static=false`, gravity off) driven by a **Python server-side system**
  (`gz-sim-python-system-loader-system`; `gz.sim8` bindings verified in the
  image). The plugin computes `pose(sim_time)` analytically each physics step
  (4 ms) → bit-repeatable motion, zero wall-clock jitter, real collisions.
  Force-based `trajectory-follower` rejected (path depends on inertia);
  SDF `<actor>` rejected (no collisions — drone flies through);
  external 5 Hz `set_pose` driver kept only as documented fallback.
- **Trajectories are data**: the world generator writes each mover's
  parametric trajectory into the model's plugin XML *and* a sidecar JSON
  (like `obstacles_boxes.json`):
  `{"name","kind":"obstacle|target","shape","z","traj":{"type":
  "line|waypoint_loop|circle", ...pts/center/radius, "speed_mps"}}`.
  One pure module `agents/world/trajectory.py` evaluates `pos(t)`/`vel(t)`
  for generator, plugin, scan, and oracle cross-checks — single source of truth.
- **Phase anchoring:** the plugin re-zeroes its `t0` to current sim time when
  a message arrives on `/movers/anchor` (gz-transport). The eval runner
  publishes it during reset, so every repeat starts at trajectory phase 0 —
  without this, K-repeats sample random phases and pass rates are confounded.
- **Pose readback:** `/world/<w>/dynamic_pose/info` (`gz.msgs.Pose_V`,
  ~49 Hz, sim-time-stamped) via a new `agents/core/gzposes.py`, the exact
  subscription pattern of `GzCameras`. Must run under the container's
  `GZ_PARTITION`.

## Grading (no time-base drift by construction)

`Snapshot` gains `movers: dict[name, (e,n,alt)]`, captured by `snapshot_now`
in the **same tick** as drone poses. All dynamic checks are pure geometry
within snapshots — sim-time vs wall-clock drift cancels out:

| check | criterion |
|---|---|
| `intercept` | min over t of drone↔mover distance < R (optional deadline predicate: mover still outside zone Z at contact) |
| `dwell_moving` | within R of mover for ≥ T contiguous seconds (1-sample dropout tolerated) |
| `avoid_moving` | ∀t: distance > R (moving keep-out bubble) |
| `escort` | fraction-of-run within R ≥ X% AND max continuous gap ≤ G s |

Discretization guard: require `R > v_rel_max × sample_interval` in task
design so a 2 Hz track cannot miss a graze. The analytic trajectory is also
compared to sampled poses each run (error < 0.5 m) to detect anchor bugs.

## Sensing

`scan` reports movers as **neutrally-named contacts** ("contact_0 62m NE
(E45 N120)") with distance + bearing + position, **no velocity** — deriving
course by differencing two scans over a known interval IS the capability
probe (L4/L5). Names must never leak trajectory semantics. Movers have no
ROS/MAVLink presence, so `run_mission`-authored code cannot close a feedback
loop on them (preserves the planning construct). A second-PX4-drone
"realistic evader" tier is deferred (meter-level nondeterminism, double
reset cost, catastrophic drone-drone collisions).

## Calibration: the dual-baseline gate

Extends the existing `--pilot` gate. Every dynamic task ships TWO scripted
no-LLM baselines:
1. **naive chaser** — repeatedly `goto(mover's current position)`. MUST FAIL
   every rung above L1 (else the task grades tool semantics, not prediction).
2. **lead predictor** — computes the analytic intercept/lead point. MUST PASS
   (else the task is a harness bug).

Plus null-policy checks at authoring time: no static hover point may satisfy
the criterion (loop radius > R; routes offset from spawn). Rungs are placed
by measured baseline behaviour, not intuition (survivability lesson,
arXiv:2404.14848).

## The ladder

| rung | task | capability | oracle |
|---|---|---|---|
| L1 | scheduled rendezvous (route+timetable in prompt) | space-time arithmetic, patience | intercept |
| L2 | shadow a 1.5 m/s loop, 30 s contiguous | short-horizon extrapolation (k≈1: naive chase *just* fails) | dwell_moving |
| L3 | timing gate past a patrolling keep-out sweep | phase reading, wait-then-dash | reached + avoid_moving |
| L4 | estimate-and-intercept (unknown course, constant velocity) | two scans → velocity → lead point | intercept |
| L5 | perimeter defense (intercept intruder before it reaches tower) | intercept under deadline; deliberation efficiency | intercept w/ deadline |
| L6 | search-then-tag in 300×300 m (capped scan radius, alt band) | coverage planning for moving target | intercept + budgets |
| L7 | convoy escort through the obstacle field | station-keeping cadence over ~4 min | escort + clearance |
| L8 | racer intercept (8 m/s loop; chase can never close) | period/phase math, chord ambush | intercept |

Phase 2 builds L1–L5; L6–L8 follow once the knee is located. Optional L8b:
abort-judgment seeds (some provably unwinnable; grade correct refusal).

## Build plan

1. `agents/world/trajectory.py` (pure, unit-tested) + `sim/plugins/mover_system.py`
2. `sim/worlds/make_dynamic_world.py` (+ `dynamic` branch in `swarm_sim.sh`,
   `GZ_SIM_SYSTEM_PLUGIN_PATH`)
3. `agents/core/gzposes.py`; `Snapshot.movers`; sampler capture
4. oracle predicates + tests
5. `World.movers` + scan contact section + tests
6. live gate in a fresh `evals-dyn` container (unique ROS_DOMAIN_ID +
   GZ_PARTITION per the parallel-sim rule)
7. Phase 2: pilot behaviours (naive_chaser / lead_intercept) + L1–L5 YAMLs +
   dual-baseline gate → then tier sweep.
