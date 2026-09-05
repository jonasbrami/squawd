# PROJECT STATE — single-drone system (living document)

> **CURRENT CHECKPOINT — 2026-09-05.** The supported product is the single-drone
> pilot/cockpit stack. The demo prototype W0–W5 is implemented, and the optional
> deep-perception follow-on M1–M4 is documented as shipped with quantified
> limits (`docs/benchmarks/deep-perception-m1.md` through `-m4.md`). The
> checkpoint includes that follow-on, the Codex/Kimi/Claude provider switch,
> and a reviewed COASTING position-latch correction. The rejected 5.5 m
> pursuit-altitude/range experiment was discarded; its review and telemetry
> remain as dated evidence. A whole-repo ponytail audit then removed generated
> eval outputs, historical swarm/benchmark/spike code, and single-implementation
> abstractions; the active single-drone path and dated documentation remain.
> Treat the verification note below as authoritative for the current tree.

> **ACTIVE GOAL — 2026-09-05:** whole-repo simplification is implemented and
> host-verified, pending owner review. The remaining M6 live model ladder is a
> separate, explicitly budgeted campaign; do not spend simulator or LLM quota
> as part of this cleanup.

> **Purpose.** This is the "state of mind" file: where the project is, what
> we're doing next, and what we're stuck on. It exists so that when we
> PAUSE, an **independent agent** (fresh context — e.g. `claude -p --model
> fable` or `codex exec -m gpt-5.6-sol`) can read it cold, look at the repo
> and the evidence, and advise a way forward. Keep it honest and current;
> update it at every pause. Last updated: **2026-09-05**.

---

## 1. The project in one paragraph

A single UAV agent: PX4 SITL + Gazebo Harmonic (single x500 with a fixed
forward 640×360 camera + a single-point forward ToF lidar), a custom
ROS 2 application bus, an **LLM pilot** (Codex via `openai-codex`, or Claude /
Kimi via `claude-agent-sdk`, behind one backend seam) that flies the drone in
plain language through **classical perception+flight primitives** (YOLO-seg
detection → CV-EKF contact
tracking → ToF beam fusion → offboard pursuit), plus a **cockpit web UI**
(POV video + sensor-fusion HUD) and an **eval harness**. The rebuild is
specified in `docs/superpowers/specs/2026-07-18-single-drone-rebuild-design.md`
(v4.2) + `2026-07-19-interface-specification.md` (v3.1), milestone-gated
M0→M6. Branch: `rebuild-single-drone`.

## 2. Milestone map (dated status + evidence)

| Milestone | Status | Evidence |
|---|---|---|
| M0 Kimi spike | **DONE (historical)** | Git history; superseded by the backend seam |
| M1 skeleton + estop/envelope | **DONE** | pytest, 12 tools, estop arbiter |
| M2 perception (contact error) | **DONE** | `docs/benchmarks/m2-gate-results.md` (p50 0.50 m) |
| M2.5 nano-seg artifact | **DONE** | `models/mover-nano-seg-v1.onnx` + manifest SHA-256 (production detector) |
| M3a vision-fed CV-EKF shadow | **DONE** | `docs/benchmarks/m3a-gate-results.md` (2/2 camera + 2/2 truth) |
| M3b ToF fusion | **ACCEPTED (deviation)** | `docs/benchmarks/m3b-status.md` — everything proven EXCEPT ≥80% availability |
| M4 cockpit observatory | **DONE (data plane)** | `/state` + cam + `/ws_detections` live; beam/track SM transitions live; estop holds mid-tool; 3 ICD tests |
| M5 evals + perception grading | **DONE** | 509-unit milestone; perceive gates + accuracy/A-B infra; d2_shadow regression fixed and validated; dated summary in `docs/benchmarks/m5-gate-results.md` (raw generated output removed from HEAD) |
| M6 Kimi switch + backend seam | **PARTIAL** | backend seam and S6 smoke done; the three-rung Kimi ladder (`d2_shadow` + perceive + obstacle) and quota report remain outstanding |
| Codex/Kimi/Claude provider switch | **IMPLEMENTED; KIMI LIVE GATE QUOTA-BLOCKED (2026-08-08)** | neutral tool catalog; authenticated Codex/Terra two-turn MCP spike; required-server fail-closed check; fresh image; scripted and Codex flight smoke pass. Kimi returned a classified billing-cycle quota error before tool use; see `docs/benchmarks/backend-switch-smoke-2026-08-08.md` |
| Demo cockpit W0–W5 | **DONE (2026-08-02 evidence)** | `docs/superpowers/specs/2026-07-28-demo-prototype-design.md`, `docs/benchmarks/w5-golden-path.md` |
| Deep perception M1–M4 | **SHIPPED WITH QUANTIFIED LIMITS (2026-08-03 evidence)** | sidecar, `look`/`pinpoint`, slow lane, cockpit layer, and metrics in `docs/benchmarks/deep-perception-m1.md` … `-m4.md` |

### Current verification (2026-09-05 ponytail-audit checkpoint)

The worktree contains the uncommitted audit cleanup. Verification on
2026-09-05:

- `git diff --check` passes.
- `bash -n scripts/*.sh sim/launch/*.sh evals/scripts/*.sh` passes.
- Full host unit lane (`uv run --extra dev --with pyyaml --with numpy pytest
  tests/ --ignore=tests/integration -q`): **717 passed in 156.33 s**. Two
  non-failing warnings remain: Starlette's `httpx` TestClient deprecation and
  the existing frame-store hammer test's background-writer integer overflow.
- No Docker image rebuild, Gazebo/PX4 run, GPU campaign, or LLM request was
  spent for this structural cleanup. The last live provider evidence remains
  the dated `docs/benchmarks/backend-switch-smoke-2026-08-08.md` report.

Next bounded step: review the deletion set, then rebuild `squawd:dev` and run
the scripted four-step backend smoke in a fresh container if live assurance is
required before committing.

Historical claims such as “490 green”, “509 green”, “638+ green”, or “753
green” identify the milestone snapshot in which they were recorded. Do not use
them as a current-HEAD claim without rerunning the relevant lane.

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

## 5. Next steps (ordered, bounded)

1. **Close the Kimi half of the provider-switch live gate after quota reset.**
   Re-run only `evals/tasks/smoke/backend_switch.yaml`, K=1, in a fresh
   container. Do not repeat the already-green scripted or Codex cells. Append
   the result to `docs/benchmarks/backend-switch-smoke-2026-08-08.md`.
2. **Redesign pursuit altitude as a separate bounded change.** The 5.5 m
   command/range patch failed review and was discarded. If revisited, address
   staging, orbit/standoff semantics, finite bounds, and the qualified detector
   band together; see `docs/benchmarks/pursuit-alt-fix-codex-r1.md`.
3. **Resume the remaining M6 ladder only as a separate bounded campaign.**
   Preserve the original three-rung plan: `d2_shadow`, one perceive task, and
   one obstacle task, K=1 with an owner pause between rungs and explicit quota
   accounting. Re-prove each scripted pilot baseline before spending model
   quota.
4. **Tracked follow-ups, not hidden blockers:** fixed-beam ToF availability;
   camera-fed d2 acquisition from home; co-altitude pilot commitment; sim-time
   versus wall-time deadlines; incomplete movement-envelope wiring; clean-clone
   PX4/model provisioning; cockpit authentication/network binding.
5. **Agent-queryable contact memory is proposed, not implemented.** A durable
   contact ledger, bounded read-only SQL tool, and guarded stationary-yaw
   reacquisition path are specified in
   `docs/superpowers/specs/2026-08-09-agent-queryable-contact-memory.md` for a
   later, separately budgeted milestone.
6. **Body-fixed-camera lock retention is a recorded circle.** Default shadow,
   20 m standoff, 20 m slow orbit, image-bbox association, and a bounded
   settle/ramp were tested without full-lap convergence. The settle removed the
   initial attitude spike but lock still died after 18.5 s; experimental code
   was discarded. See
   `docs/benchmarks/lock-camera-motion-experiments-2026-08-09.md`. Next work
   should evaluate a stabilized gimbal or replay-tested visibility-aware
   classical controller, not a fourth live tweak. The ranked low-effort path
   and its acceptance ladder are recorded in
   `docs/superpowers/plans/2026-09-05-lock-retention-low-hanging-fruits.md`;
   none of those changes is implemented yet.

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

> Historical consultation snapshot. “Current” statements and test counts in
> this section refer to 2026-07-28 and are superseded by §2's 2026-08-08
> verification note.

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
