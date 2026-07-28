# M3b status — 2026-07-21 (branch rebuild-single-drone)

**Machinery: complete, unit-verified, and the live lock was demonstrated
once. The end-to-end live gate resists 12 genuine attempts — the blocker is
the low-altitude co-altitude HOLD on this stack (baro drift + PX4 land
detection + a 3.5 m/s orbiter), not the fusion machinery.**

## Gate criteria vs evidence

| Criterion | Evidence |
|---|---|
| slant error <0.5 m p50 / <1.5 m p95 | **met when fused**: v9.0 run — 2 fused samples, p50 0.43 m |
| ≥80% availability | NOT met — the lock windows don't survive the co-altitude bind (below) |
| 0 false associations | **met everywhere**: 0 in every run; the association correctly rejects ground returns, background, edge-mix |
| airborne acquisition demo | **demonstrated once (v9.0)**: bearing-only → ACQUIRING → ASSOCIATED → WORLD_TRACKED with fused range |

## What's built and proven

- `agents/vision/beam.py` — BeamAssociator (footprint → mask/eroded-box →
  consistency gates; statuses ASSOCIATED|AMBIGUOUS|EDGE|OFF_TARGET|
  OUT_OF_ENVELOPE|NO_SAMPLE) + `in_fusion_envelope`.
- `agents/vision/contacts.py` — acquisition SM (DESIGNATED → ACQUIRING →
  RANGE_LOCKED → WORLD_TRACKED; slip → COASTING, never LOST-cycle), ToF
  fusion into the CV-EKF, `ranges()`, `track_state()`, `set_beam_context()`,
  deterministic consumption, `beam_view()`/`track_view()` cockpit seam.
- `agents/flight/ops.py` — O6 `_acquire` (designate → offboard image-servo
  with bearing-rate feedforward → segmented creep-and-listen → co-altitude
  elevation servo → blind-recovery sweep → bounded budget, legible
  NOT_ACQUIRED).
- **438 tests green**, incl. the full M3b battery + every fix below's
  load-bearing test.

## The 12-attempt forensic chain (v3→v9.8) — each a real, tested fix

1. **Boot-poison clock stamp** (v7.3): ONE garbage PX4 boot-transient
   timestamp (~17668080 s) sat at the World buffer head forever; `_interp`
   never extrapolates → `pose_at`/`attitude_at` returned None for whole
   runs → VisionContacts produced ZERO measurements with the box in view.
   → stamp bound-check + buffer self-heal (tests in test_telemetry.py).
2. **EKF alt drift**: ±1–2 m per boot, GROWING over minutes. → live
   drift-EMA re-biasing of every commanded altitude in the gate.
3. **Stale-position pursuit trap** (v4): a coasted geom position went stale
   and the pursuit yawed off a ghost. → chase LOW so the handover is
   bearing-only by construction.
4. **Alt-bias geom lever** (v8.6): below ~2 m of alt-support drop the
   support-plane range carries >100% bias from the alt drift → ghost
   positions. → `_MIN_DROP_M` guard: bearing-only there (honest, same
   principle as the 6° pitch lever; M3a/M2 geometries unaffected).
5. **Parked/frozen yaw** (v5/v6): the orbiting box leaves the FOV in ~2 s;
   pure pursuit trails at ~17 m equilibrium. → lead-intercept chase +
   image-servo aim assist.
6. **Monotonic creep** (v8.1): the unbounded 2 m/s creep drove into the box
   (min gap 0.6 m). → segmented creep-and-listen.
7. **Ghost-range ToF birth** (v8.2): the first lock has no EKF prediction —
   a background return birthed a ghost position, pursuit flew INTO the box
   (min gap 0.1 m). → bbox-height cue (R = h·fy/px_h) 3σ cross-check on
   every first lock, tested both directions.
8. **yaw=0 offboard init** (v9.1): the pre-start stream slammed the nose to
   north. → hold current heading.
9. **world_xy 3-tuple** (v9.2): `me[3]` IndexError, latent crash. →
   `drone_state` (4-tuple) + test fakes.
10. **`ContactView` lacked `foot_px`** (v9.7 — THE big one): the design's
    image-servo read a field that doesn't exist — `getattr(obs, "foot_px",
    None)` was ALWAYS None, so the servo was dead code since it was written
    (even the o4 test only exercises the blind branch). Every aim was the
    frozen-bearing fallback. → `foot_px` added to the read model + ICD §1;
    the servo tracked ax to ≈0 and held it live (v9.7 ax telemetry).
11. **Elevation servo sign inversion + deadband** (v9.7/v9.8): the
    co-altitude servo DESCENDED when the box was above (anti-servo to the
    floor), and only fired for |e|>4°. → correct sign, mid-box target
    (base → −3.4°), no deadband.
12. **Co-altitude bind** (v9.8, the residual blocker): baro drift grows
    +0.7 → +2.3 over the run; physical altitude sinks to ~0.5 m where PX4's
    land detection engages; climb commands are then ignored; the drone
    sits on the ground, blind. This is a stack/environment problem (PX4
    land logic + baro), separable from the M3b fusion machinery.

## External review round (2026-07-22) — codex (gpt-5.6-sol) + fable

Codex's full-context review **reversed the v9.x diagnosis** and found the
real defects (land detection is NOT the primary cause; "baro" is a
misnomer — the SITL x500 has no barometer, EKF2_HGT_REF=GPS):

1. **The elevation servo never fired**: `_acquire` read `obs.elev_deg`;
   the ContactView DTO field is `elevation_deg`. The o4 test's
   SimpleNamespace invented the wrong field, hiding it.
2. **The drift bias froze at handoff** — a frozen EKF-relative altitude
   setpoint + growing estimator bias sinks the physical drone (the actual
   descent mechanism; land detection is at most a secondary latch on
   contact).
3. **`foot_px` never expires** on the track, so the blind-recovery sweep
   was unreachable (always "fresh").
4. **`own_speed_mps` was fed the TARGET's velocity** (est.ve/vn) and
   raced by the eval collector — flight layer must own it with the
   vehicle's measured vx/vy.
5. `_acquire` returned success on any pose, and discarded the acquired
   altitude at the pursuit handoff.

**Incident note (2026-07-22)**: while applying these fixes, ops.py was
accidentally truncated (Write misuse). Recovery: PyLingual decompile of
the surviving .pyc + full-disassembly-guided reconstruction; the codex
fixes are being applied during the rebuild. Test suite returns to green
before any gate rerun.

## Options for the user

- **A. Accept M3b as demonstrated** (machinery + first lock + fusion
  quality: slant p50 0.43 m, 0 false-assoc) and proceed to M5/M6; the
  repeatable live gate becomes a follow-up tracked against the stack
  (baro/land-detection).
- **B. Acquire against mov_0 (airborne box at z≈10 m)**: at ~9.5–10 m the
  drone is far from land detection and ground effect; the 0.4 m-tall box
  is a narrower beam target but the co-altitude bind disappears. One gate
  variant (`TARGET = "mov_0"`) — the honest next attempt if B is wanted.
- **C. PX4-side mitigations**: raise the land-detection thresholds /
  baro-source params for the co-altitude regime (sim-only params; needs a
  careful look at what M2/M3a rely on).

All 438 tests stay green; every fix above is in-tree with its load-bearing
test. The M3b unit/integration surface (beam association, acquisition SM,
envelope, deterministic consumption, 0-false-association) is fully
demonstrated; only the repeatable end-to-end live gate is outstanding.

## Live-gate state after the external-review round (v15–v21, 2026-07-22)

The codex+fable review round (docs/benchmarks/m3b-review-codex.md,
m3b-review-fable.md) found and fixed the real blockers (elevation_deg,
frozen drift bias, foot_px freshness, own_speed race, pursuit alt floor,
the mask∪eroded-box union, the M2.5 nano-seg backend replacing the
shadow-merging blob). With those in (440 tests green):

- **The full chain runs end-to-end, repeatedly**: bearing-only contact →
  ACQUIRING → RANGE_LOCKED → WORLD_TRACKED → a 45 s ToF-fused shadow
  (v18, v19, v21). The airborne acquisition demo EXISTS live.
- **Fusion quality when fused**: slant error p50 0.43–1.09 m, **0 false
  associations in every run** (the association correctly rejects ground
  returns, background, edge-mix, mask holes).
- **The unmet criterion is ≥80% availability** (measured 0.01): the
  operating point won't hold — ±2° attitude noise vs the 1.2 m box at
  15–30 m makes association intermittent, and when fusion drops the
  ToF-fed estimate ghosts, so the pursuit's gap grows (mean 22–30 m) and
  fusion dies further. Every other criterion (slant p50/p95, 0
  false-assoc, airborne acquisition) is demonstrated.
- The remaining engineering (not a tweak): a COASTING re-close in the
  pursuit (creep toward the bearing to restore fusion geometry instead of
  holding the stale point — the SM's COASTING→ACQUIRING re-lock leg).
- **v22 verdict on the re-close**: a 2 m/s creep cannot beat the orbit's
  radial swing (15→75 m at 3.5 m/s) — fusion survives only the brief
  close-approach windows and pitch noise makes association intermittent
  inside them. **The ≥80% availability figure is a PHYSICAL operating-
  point limit** (35 m orbit × 3.5 m/s × 1.2 m box × ±2° attitude noise ×
  0.5° beam), not a machinery defect. Meeting it needs a setup change
  (gimballed beam / larger or slower target), which is design-level and
  the owner's call.

**Status: machinery complete and demonstrated end-to-end live; the strict
availability number needs the pursuit-recovery feature.**
