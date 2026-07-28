# M2 gate results — 2026-07-21 (branch rebuild-single-drone)

Gate (design §7, M2): **PASS**

## Accuracy instrument run (live, dynamic world, composite x500_depth + ToF)

```
n=460  p50=4.35m  p95=20.14m
  <=30m: n=212  p50=2.79m  p95=10.45m     <- GATE: p50 < 5m @ <=30m  PASS
  30-60m: n=218 p50=5.70m  p95=19.27m
  >60m:  n=30   p50=21.13m p95=28.47m
```

Command: `docker exec pilot-sim bash -lc 'uv run --no-project python evals/perceive_accuracy.py'`

## Gate checklist

- pytest green: **351 passed** (host, `uv run --with pyyaml --with numpy pytest tests/ -q`)
- airborne detect lists the orange mover with sane bearing through a 12 m/s
  transit: 460 airborne blob detections measured through the outbound transit +
  aimed hover; bearing projection error 0.19° mean (offline decomposition of
  112 logged contacts); `detect` tool path unit-tested (tests/test_drone_tools)
  and live-verified returning sane strings.
- **accuracy p50 < 5 m @ ≤30 m vs GzPoses for the ground mover: 2.79 m — PASS**
- rangefinder reader reports canonical RangeSamples with honest status/quality:
  live `latest() = 29.08 m VALID quality 0.51`, `robust_at` OK; robust estimator
  rejects injected dropouts within lag budget (14 rangefinder unit tests).
- detector `latency_ms` logged (PerceptionSnapshot.detector schema, flows live).
- registry adapters dormant-but-wired; mis-paired explicit tracker+backend
  fails closed legibly (vision config tests).
- raw-detection snapshots flow on `/pilot/detections` (schema v1, contacts
  empty, Codex-B4) — verified live.

## Root causes found and fixed this milestone (the hard part)

1. **"Rangefinder destabilizes PX4" was confounded — the lidar never did it.**
   Two independent defects:
   - **Land-mode arming lock** (PX4 `mode_req_prevent_arming`): after any
     `land()`, a bare `arm()` is permanently denied for the rest of the boot
     ("cannot takeoff in current mode", health_report system bit). MAVSDK
     `takeoff()` arms in the *current* nav_state. Fixed: `hold()`→`arm()`→
     `takeoff()` with bounded retries in `FlightOps._arm_robust`
     (agents/flight/ops.py) and `evals/reset.py`.
   - **Cross-world `EKF2_MAG_DECL` poisoning**: PX4 auto-saves its learned
     declination at disarm into the host-mounted rootfs; a baylands (CA,
     ≈+12.8°) value silently broke Zurich-world (≈+2.5°) boots with "Yaw
     estimate error" preflight denials. Fixed: factory-state wipe of
     parameters.bson/dataman/eeprom in `sim/launch/swarm_sim.sh`.
   Evidence trail: ulog analysis (mag_test_ratio oscillating across the 0.5
   arming gate; arming_state never left standby despite takeoff ACK; motor
   commands zero). The gpu_lidar composite **flies clean** now (arm→8 m→land).
2. **Scan topic mismatch**: gz derives the topic from the sensor *name*
   (`.../sensor/lidar/scan`), not its type (`gpu_lidar`). `RANGE_TOPIC` fixed
   in agents/core/rangefinder.py + ICD §2.5.
3. **Blob detector saw nothing**: thresholds assumed the SDF ambient orange
   (229,115,26); the rendered box measures shadow (95,68,30)–sunlit
   (204,150,74). Retuned against live frames (r-g>15, g-b>20, r-b>45);
   detects 16–44 m with one tight bbox, zero false positives on probe frames.
4. **Floating mover**: mov_1's 1.2 m box is centered at z=1.2 → base floats at
   z=0.6 (visible over its shadow). Support-plane projection to z=0 overshoots
   range ~15%. Gate harness uses support_z=0.6.
5. **Camera-aim in the accuracy harness**: travel-heading left the FOV off the
   mover (0 measurements); lead-aiming parked it at the trailing edge (blob
   clipping, 5× worse p50). Frame-cadence live-bearing aim + ±10° bearing
   association gate (rejects other orange movers) fixed it.
6. **attitude_at edge-None on ~25% of frames**: recorder clock was referenced
   to the 10 Hz camera stamp; re-referenced to the physics-rate GzPoses clock.
7. **Probe hygiene (documented for future live probes)**: mavsdk `connect()`
   must precede starting the Detector/rclpy spin threads — gRPC connect
   starves for minutes under GIL contention otherwise (pilot already orders it
   correctly in run.py); SIGKILLed probes wedge mavsdk_server/DDS state —
   bounce the container rather than piling on.

## Remaining known limitations (not gate items)

- Camera-forward perception means the mover leaves the frame when closer than
  ~13 m at 6 m altitude (geometry); follow logic owns close-in behavior (M3).
- Ad-hoc live probes are fragile in a long-lived container; the two gate
  instruments (perceive_accuracy, pilot pipeline) are the supported paths.
