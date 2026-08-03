# PROJECT STATE — single-drone rebuild (living document)

> **GOAL SWITCH 2026-07-28 (owner):** the M0→M6 rebuild goal is PARKED at
> "M6 rungs remaining" (S6 spike PASSED first attempt; d2_shadow + perceive
> + obstacle rungs + quota-metrics report outstanding — see §5 item 2).
> The ACTIVE goal is the **Demo Cockpit Prototype** per
> `docs/superpowers/specs/2026-07-28-demo-prototype-design.md` (v0.2,
> codex-confirmed). Resume the M6 rungs as a separate goal afterward.
> Demo containers killed at switch.

> **Purpose.** This is the "state of mind" file: where the project is, what
> we're doing next, and what we're stuck on. It exists so that when we
> PAUSE, an **independent agent** (fresh context — e.g. `claude -p --model
> fable` or `codex exec -m gpt-5.6-sol`) can read it cold, look at the repo
> and the evidence, and advise a way forward. Keep it honest and current;
> update it at every pause. Last updated: **2026-07-22**.

---

## 1. The project in one paragraph

A single UAV agent: PX4 SITL + Gazebo Harmonic (single x500 with a fixed
forward 640×360 camera + a single-point forward ToF lidar), a custom
ROS2-free bus, an **LLM pilot** (Kimi via claude-agent-sdk behind a backend
seam) that flies the drone in plain language through **classical
perception+flight primitives** (YOLO-seg detection → CV-EKF contact
tracking → ToF beam fusion → offboard pursuit), plus a **cockpit web UI**
(POV video + sensor-fusion HUD) and an **eval harness**. The rebuild is
specified in `docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md`
(v4.2) + `2026-07-19-interface-specification.md` (v3.1), milestone-gated
M0→M6. Branch: `rebuild-single-drone` (uncommitted working tree by policy —
no commits without the owner).

## 2. Milestone map (status + evidence)

| Milestone | Status | Evidence |
|---|---|---|
| M0 Kimi spike | **DONE** | `spikes/M0-RESULTS.md` |
| M1 skeleton + estop/envelope | **DONE** | pytest, 12 tools, estop arbiter |
| M2 perception (contact error) | **DONE** | `docs/benchmarks/m2-gate-results.md` (p50 0.50 m) |
| M2.5 nano-seg artifact | **DONE** | `models/mover-nano-seg-v1.onnx` + manifest SHA-256 (production detector) |
| M3a vision-fed CV-EKF shadow | **DONE** | `docs/benchmarks/m3a-gate-results.md` (2/2 camera + 2/2 truth) |
| M3b ToF fusion | **ACCEPTED (deviation)** | `docs/benchmarks/m3b-status.md` — everything proven EXCEPT ≥80% availability |
| M4 cockpit observatory | **DONE (data plane)** | `/state` + cam + `/ws_detections` live; beam/track SM transitions live; estop holds mid-tool; 3 ICD tests |
| M5 evals + perception grading | **DONE** | 509-unit green; perceive gates + accuracy JSONs + A/B infra; d2_shadow regression found→codex-reviewed→fixed→validated (dwell 69.1 s vs 45 gate); d1–d5 truth-fed re-confirmation **9/9 cells match pre-M5 behavior** (`evals/out/m5_d1d5_confirm_20260728/`, `docs/benchmarks/m5-gate-results.md`) |
| M6 Kimi switch + backend seam | **CODE DONE** | `agents/flight/backend.py` seam, `cli_path` enforced, ToS documented, ICD tests ①②. **Outstanding: in-sim spike + kimi mini-ladder** |

**Unit suite: 490 green** (`uv run --with pyyaml --with numpy pytest tests/ -q`).

## 3. What we're struggling with (the three walls)

### 3.1 The physics wall — ToF availability (M3b)
The M3b gate wants ≥80% ToF-fusion availability shadowing a 1.2 m box
orbiting a 35 m circle at 3.5 m/s, with a **0.5° fixed beam** and **±2°
attitude noise**. At 15–30 m the box is <4° tall, so association is
intermittent; when fusion drops the position estimate ghosts, the pursuit's
gap grows, and fusion dies further. **This is a physical operating-point
limit, not a code defect** — the owner accepted M3b with availability as a
documented follow-up. Everything else (slant p50 0.43–1.09 m, 0 false
associations, airborne acquisition → WORLD_TRACKED → 45 s shadow ×3) is
proven live.

### 3.2 The stack wall — EKF drift + land detection
PX4 SITL's EKF global position drifts **~85 m** (navsat) and altitude
**±1–2 m** (GPS-height; the x500 SITL has no baro reference issue — it has
a gz air-pressure sensor, `EKF2_HGT_REF=GPS`). A frozen EKF-relative
altitude setpoint sinks the physical drone to the ground, where PX4's land
detector + auto-disarm (`COM_DISARM_LAND` 2 s) makes it an absorbing
failure. Mitigations in place: live drift-EMA re-biasing, co-altitude
elevation servo, min-drop geom guard. Navigation must use **relative `fly`
moves** (never `goto_location` for precision).

### 3.3 The LLM-pilot wall — Kimi choreography (M4/M6)
The Kimi LLM pilot is erratic on navigation (once climbed to 100 m
uncommanded; repositions instead of committing to a co-altitude track).
Bounded-altitude commands help. This is M6 territory (backend behavior),
not a perception/cockpit defect.

## 4. The M3b saga — the cautionary tale (why this document exists)

The M3b live gate **resisted 22 iterations** across two days. Each
iteration root-caused and fixed a *real* bug (12+ genuine defects: the
boot-poison clock stamp that starved all vision, `elevation_deg` vs
`elev_deg` (a servo that never fired), the `foot_px`-less ContactView that
made the design's image-servo dead code, the mask-hole association union,
the pursuit's 2.3 m altitude floor parking the beam over the box, the blob
merging box+shadow (→ switched to the M2.5 seg model), a track()
success-path `else:` bug, and more). The machinery is now production-grade
and the full chain is proven live. **But the availability criterion never
converged** — and grinding for it consumed the weekly token quota. There
was also one real process incident: `ops.py` was accidentally truncated
mid-work (Write misuse) and had to be reconstructed from `.pyc` (28/31
code objects byte-identical, suite green). **Lesson: a hard gate on a
noisy physical setup is not a linear bug-fix ladder — know when to stop.**

## 5. Next steps (ordered, with blockers)

1. **M5 d1–d5 regression — d2_shadow FIXED 2026-07-28 (fix iteration 1,
   first attempt).** Codex review (`docs/benchmarks/d2-regression-review-codex.md`)
   named the culprit: the `_shp` trajectory shaper (ops.py) discards
   `control_ref()`'s direct shadow reference and chases a 1 m/s² accel-limited
   carrot initialized at the drone (~7–10 s lag on a 3.5 m/s target), plus the
   beam-geometry altitude profile wrongly applied to truth-fed contacts (no
   `observation()` → descends to ~3.8 m despite alt=12). Fix: `beam_capable`
   gate — truth-fed shadow streams the direct reference at commanded alt;
   camera-fed M3b path byte-identical. ICD tests added (suite 509 green).
   **In-sim validation: dwell 69.1 s vs 45 gate (baseline 68.1 s), null lane
   still FAILs correctly** (`evals/out/m5_d2_fix1_20260728/`). Full-ladder
   re-confirmation 2026-07-28: **9/9 cells match pre-M5 behavior**
   (`evals/out/m5_d1d5_confirm_20260728/`) — **M5 CLOSED**.
   Deferred follow-ups codex
   flagged: `ops.py` `min(ref_u, alt_ref)` defeats the building clamp;
   `evals/reset.py` restores only `MPC_XY_CRUISE` (not `MPC_XY_VEL_MAX`/
   `MPC_TILTMAX_AIR`); `track()` expiry + oracle dwell run on wall-clock
   while PX4/rover run on sim time — RTF<1 shrinks effective windows
   (explains the 10.5 s outlier run).
2. **M6 in-sim spike + kimi mini-ladder** — take_off → scan → detect →
   report text-only on the Kimi tier (S6), then the §5.6 mini-ladder:
   d2_shadow + one perceive task + one obstacle task (three rungs, K=1
   each, owner pause between) → `docs/benchmarks/` with §5.5 quota
   metrics. No-LLM prep DONE (eval detect wiring, backend quota metrics,
   eval Envelope, track safeguards, s6_kimi_spike.yaml). Budget: ~25
   requests / ~90k input tokens, hard ceiling 200k input.
   *Blocker RESOLVED 2026-07-28: S0 probe passes (3 turns, 558 in/153 out
   tokens, $0.0115, no errors) — Kimi quota is LIVE again.*
   *S6 launch recipe (ready): fresh container `-e RENDER_BACKEND=cpu
   -e PX4_MODEL=gz_x500_depth -e SWARM_N=1 -e PX4_GZ_WORLD=perceive
   -e GZ_WORLD=perceive -e KIMI_API_KEY` (host: `set -a; . ./.env; set +a`
   first — key never printed); in-container `GZ_WORLD=perceive uv run
   --no-project --with onnxruntime python -m evals.run_evals --tasks
   evals/tasks/perceive/s6_kimi_spike.yaml --assignments drones=kimi
   --feed vision --backend onnx --k 1 --out evals/out/m6_s6_kimi`. Fresh
   container mandatory — the ~85 m EKF drift class appears with container
   age, not at boot (m5-gate-results forensics).*
3. **Follow-ups (documented, not blockers):** ToF availability (§3.1);
   LLM-pilot co-altitude commitment (§3.3); pilot-lane dwell ≥30 s in
   perceive (chase-controller survival — M3a physics, not grading);
   live ID-switch numbers (needs a track-id backend e.g. ultralytics).
4. **Camera-fed d2 from home spawn — broken in the M5-era harness (found
   via cockpit demo 2026-07-28).** Scripted `track(mov_1)` infra-fails at
   step 0: `unknown moving contact 'mov_1' (visible: none seen yet)` —
   the M5 contact-resolution layer requires a camera-visible contact, and
   the rover orbits ~125 m from spawn (detector trained ≤30 m). M3a's
   camera-fed 2/2 (2026-07-06) predates that layer; the M5 re-confirmation
   was truth-fed, so the gap was never exercised. Null lane also failed on
   GzPoses warm-up (`not visible on gz poses` at t≈0). **Threatens the M6
   d2 rung if it goes camera-fed from home.** Needs codex review + a
   decision (acquisition path / pre-visibility rule / rung feed) at
   resume. Evidence: `evals/out/demo_d2_vision/results.jsonl`.

## 6. Working agreement (owner-set, 2026-07-22)

- **Control:** no open-ended goal-mode grinding. Work in bounded steps;
  report at each milestone or blocker; the owner decides each advance.
- **Circle-pause rule:** if one gate/criterion takes **>3 genuine fix
  iterations without measurable convergence** (the metric doesn't move), we
  **STOP iterating**, mark it here as a *circle*, and hand it to an
  independent reviewer (§7) instead of trying a 4th/5th/…/22nd time.
- **Quota is a first-class budget:** long sim-gate series and LLM-cell
  evals are expensive — prefer unit/fixture tests and short sim runs; batch
  live-gate attempts; never re-run a whole gate to change one probe.

## 7. Independent-agent investigation protocol

When we pause on a circle or a hard blocker, run a fresh reviewer with
this file as the entry point. Give it: (a) this file, (b) the specific
status doc (`docs/benchmarks/m3b-status.md` for perception/fusion,
`m5-gate-results.md` for evals), (c) the freedom to read the repo and to
**websearch** (PX4/Gazebo/ROS specifics). Ask it a **narrow question**
(e.g. "given ±2° attitude noise, a 0.5° fixed beam and a 1.2 m orbiting
box, is ≥80% fixed-beam ToF availability achievable at 8–12 m, or is the
setup physically capped — and what is the cheapest setup change that
makes it achievable?"). Two independent reviewers (`fable` and
`codex/gpt-5.6-sol`) have already produced high-value findings in this
project (see `docs/benchmarks/m3b-review-fable.md` /
`m3b-review-codex.md`) — prefer them over grinding.

## 8. Open questions for the next reviewer

1. Is ≥80% fixed-beam ToF availability *physically* achievable on this
   setup (§3.1), and if not, what is the smallest setup change (gimbal?
   taller/slower target? narrower orbit?) that makes it achievable without
   redefining the gate?
2. The EKF alt drift grows +0.7→+2.3 m over minutes *with* GPS-referenced
   height and truth-fed GPSSIM — baro-aiding artifact or datum resets?
   (One A/B with `EKF2_BARO_CTRL 0` + `z_reset_counter` logging settles it.)
3. What makes the Kimi LLM commit to a co-altitude track instead of
   repositioning/climbing (§3.3) — prompt, tool-shape, or tier?

---

## 9. Resume consultation 2026-07-28 (codex gpt-5.6-sol, high, 2 rounds)

Full transcripts: `docs/benchmarks/resume-review-codex-r1.md` + `-r2.md`.

**Recommended order** (each → owner pause):
1. CHEAP: unit suite, `git diff --check` (currently flags a trailing blank
   line at `tests/evals/test_runner.py:211`), model-manifest sha256 →
   owner checkpoint-commit decision.
2. SIM: M5 d1–d5 truth-fed regression — LLM-free (scripted pilot), **no
   quota reason to wait**. Fresh dynamic container (CPU + `gz_x500`;
   doctor_sim's camera requirement irrelevant truth-fed). Baseline =
   composite `evals/out/pilot_dynamic/dyn_pilot_gate2` (d1/d3/d5) +
   `evals/out/pilot_track/dyn` (d2/d4); compare pass/fail semantics, tool
   order, step counts — not float telemetry. d2 truth-fed failure = real
   regression by construction (bypasses camera/CV-EKF/ToF; pre-M5 passed
   2/2) — M3a dwell physics is NOT a valid excuse for this lane. Decision
   tree: 1 fresh-container diagnostic repeat → max 3 tested fixes → circle
   → owner-accepted deviation (never a pass).
3. CHEAP: M6 no-LLM prep — wire `detect_text` into the eval client
   (`Deps.pipeline` + `make_detect_text` in `FleetHarness.client_for`;
   production pilot discards Result/usage so S6 runs through run_evals),
   quota-error classification in `backend.py`, `Envelope` into eval
   `FlightOps` (missing at `evals/runner.py:234`), altitude/commitment
   safeguards (default `track.alt` to current alt, mission ceiling), new
   `evals/tasks/perceive/s6_kimi_spike.yaml`.
4. LLM after quota reset: S6 → d2 → perceive → obstacle, K=1 each, owner
   pause between rungs. **Budget committed: ~25 Kimi requests, ~90k
   input / ~5k output tokens; hard ceiling 200k input.**
5. CHEAP: evidence → `docs/benchmarks/`.

**Corrections to this file:** the M6 mini-ladder is THREE rungs per spec
§5.6 (d2_shadow + one perceive + one obstacle) — §5 item 2 understated it;
the reconstructed file is `agents/flight/ops.py` (not evals/).

**Open:** which perceive task for the ladder (scripted pilot still fails
`dwell_moving` 30 s — resolve that baseline BEFORE spending Kimi, else
controller physics conflates with model quality); obstacle rung needs o1
`--pilot` re-verified on the flat `obstacles` world first (historical 4/4
in `evals/out/pilot_obstacle/`).

**Owner resolved 2026-07-28:** checkpoint commit YES (`484eee8`, local,
unpushed) · M5-now YES (bounded manual step — done, d2_shadow regression
found) · M6-prep YES (all) · ladder scope = spec §5.6 three rungs.
