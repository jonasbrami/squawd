# Deep Perception M3 — gated slow-lane annotator + cockpit deep layer + A/B coexistence gate (2026-08-03)

Milestone M3 of the deep-perception plan (`black-hawk-taskmaster-groot`,
codex r1 binding — findings 3, 4 and CR4): the **gated slow lane**
(`agents/vision/slowlane.py`) samples the live frame at ~0.3 Hz through the
M1 sidecar with a static vocabulary and publishes frame-keyed annotations +
an `fp_suspect` false-positive advisory; the cockpit serves and renders
them (plus the M2 pinpoint mask) with real ≤0.5 s frame-age expiry; and the
**GPU-coexistence A/B gate** decided the armed-default for intel render.

**Verdict: SHIPPED.** Suite 753 green (+45 vs the 708 M2 baseline). The A/B
is **GREEN** (RTF, PX4 time-sync, fast-lane latency/cadence, VRAM all flat
across ≥5 min arms with the drone armed in HOLD) → per the plan the armed
gate was lifted for intel (`ARMED_GATE_EXEMPT = ("intel",)`); nvidia stays
gated. Annotations live-populated with building/house entries; no
`fp_suspect` event occurred naturally (recorded honestly below).

## Shipped

- NEW `agents/vision/slowlane.py` — `SlowLane` daemon THREAD (never the
  pilot's asyncio loop): samples `frame_source()` at DEEP_SLOWLANE_HZ
  (default 0.3, clamped ≤2 Hz), `DeepClient.detect` with DEEP_SLOWLANE_VOCAB
  (default `building,house,tree,pole,tower`) at DEEP_SLOWLANE_CONF (default
  0.05, the M1b operating point). **Skip-if-busy, zero queue**: one call at
  a time, an overrun eats the missed ticks (no catch-up burst), sidecar
  BUSY/UNAVAILABLE/ERROR drops the tick — on-demand look/pinpoint keep
  priority via the sidecar's own one-inference lock. Every payload carries
  `frame_seq` + `sim_stamp` + captured monotonic + the exact dets +
  `fast_dets` audit trail. **FP advisory**: overlap =
  intersection/fast_box_area vs the fast dets of the **exact submitted
  InferenceResult** (polled by seq from the Detector, bounded 1.5 s; a
  wrong/absent frame ⇒ `fp_checked: false`, never a substitute frame —
  codex F3); ≥0.6 under a building/house annotation ⇒ `fp_suspects` entry.
  Pure advisory: the module never touches VisionContacts/tracker and the
  payload has no contact-shaped fields (test-pinned).
- TOUCHED `agents/pilot/run.py` — `build_deep_client()` extracted (M2 boot
  logs verbatim), shared by the tools and the lane; `build_slowlane()` wires
  the gate (DEEP_SLOWLANE/RENDER_BACKEND env + live armed state from a
  vehicle_status subscription, the recorder's callback pattern) and
  publishes every tick on **`/pilot/slowlane`** (String JSON, STATE_QOS —
  the /pilot/deep precedent; observatory stays topic-only, ICD §0.1).
- TOUCHED `agents/observatory/overlay.py` — pure, tested join helpers:
  `annotations_for` (0.5 s frame-age gate, age_ms), `pinpoint_mask_for`
  (box-local passthrough), `mark_fp_suspects` (contact flag via same-cls +
  IoU≥0.5, double freshness gate, never mutates input).
- TOUCHED `agents/observatory/metrics.py` + `server.py` — `/state` gains
  `annotations` (cls/conf/xyxy/frame_seq/sim_stamp/age_ms), `pinpoint_mask`,
  `slowlane` health, and `fp_suspect` on contacts; main() subscribes
  `/pilot/slowlane` + `/pilot/deep`.
- TOUCHED `agents/observatory/static/index.html` — annotation boxes/labels
  in magenta (distinct from det boxes), the pinpoint mask as a translucent
  silhouette (JS `rleDecode` mirrors `types.py rle_decode`, box-local at
  xyxy), `FP?` on flagged det labels + contact pills, a legend line. NO
  identify-click UI (deferred, codex CR4).
- TOUCHED `scripts/run_single_demo.sh` — passthrough for DEEP_SLOWLANE /
  _HZ / _VOCAB / _CONF. TOUCHED `docs/RUN-DEMO.md` — deep-layer runbook.
- NEW `tests/test_deep_slowlane.py` (39) + `tests/test_cockpit_server.py`
  additions (6) — counts below.

## The gate + the A/B decision (codex F4)

`gate_decision(force, render_backend, armed)`: `DEEP_SLOWLANE=off` always
off, `=on` always on; unset ⇒ off when `RENDER_BACKEND=nvidia` (gz shares
the GPU), off while armed unless the backend is A/B-exempt. The A/B measured
the shipped stack (demo world, drone ARMED in HOLD at 16 m, intel render,
slowlane default 0.3 Hz through the :8101 tap proxy) — each arm ≥5 min of
steady state, `evals/out/deep_m3/ab_{off,on}.{jsonl,summary.json}`:

| metric | OFF (5.52 min) | ON (5.53 min) | verdict |
|---|---|---|---|
| gz sim RTF mean (min) | 0.921 (0.262) | 0.970 (0.277) | no degradation |
| PX4 `time jump detected` | 18 (3.26/min) | 19 (3.43/min) | same PRE-EXISTING churn (present with the sidecar fully idle) |
| fast-lane `detector.latency_ms` mean / p95 | 40.5 / 44.3 ms | 42.6 / 44.7 ms | within normal jitter |
| cam cadence (/state cam_seq) | 9.76 Hz | 9.70 Hz | unchanged |
| sidecar detect p50 / p95 (tap, wire) | — (0 calls; gate held, 263 ticks all `skipped_gate`) | 22 / 31 ms (n=100) | static vocab rides the registry's vocab cache |
| VRAM (nvidia-smi, whole GPU) | 2636 → 2631 MiB | 2599 → 2592 MiB | flat (sidecar torch stays 898 MiB) |
| slowlane counters | 263 ticks / 0 calls | 116 ticks / 116 calls / 116 ok / **116 fp_checked** | every tick served, exact-frame advisory every time |

**Decision: GREEN** — no meaningful fast-lane/RTF/PX4 degradation ⇒ the
armed/OFFBOARD default flips to ON for intel (`ARMED_GATE_EXEMPT =
("intel",)` in `slowlane.py`, pinned by
`test_shipped_default_gate_matches_the_ab_decision`); **nvidia stays
gated** (gz would share this GPU — untested here, conservative stands).
Live proof of the flipped default: pilot restarted with `env -u
DEEP_SLOWLANE` while armed ⇒ `/state.slowlane =
{active: true, note: "default (non-nvidia)"}` with annotations flowing.

## Live acceptance (world=demo, armed HOLD 16 m facing the houses, intel)

Evidence under `evals/out/deep_m3/`:

- **`/state.annotations` populated** — `state_annotations_live.json`:
  `building 0.197` + `house 0.171` (frame_seq 4538, age_ms 100) + `tree
  0.17`, joined by frame_seq with the 0.5 s expiry (entries vanish between
  polls — at 0.3 Hz with 0.5 s sim-time expiry the visible duty cycle is
  ~16% at RTF ~1, by design per codex F3: stale boxes are worse than none;
  5/31 ten-second collector samples caught fresh annotations).
- **Screenshot** — `m3_annotations.png` (the w3_capture render path, i.e.
  exactly what the cockpit draws): magenta `building 0.27`/`house 0.21`
  boxes on the house + `tree 0.33` on the pine, slowlane state in the
  header.
- **Pinpoint mask live through /state** — `state_pinpoint_mask_live.json`
  (box-local RLE 109×63, xyxy [119,240,228,303], cls "house" from the
  look() seed, age_ms 200) and `m3_mask.png`: the translucent silhouette
  sits exactly on the house (area 6074 px, score 0.94). The k3 drove
  look→pinpoint itself from chat (`chat_transcript.json`).
- **`fp_suspect`: NO natural occurrence** — 116 exact-frame advisory
  computations ran (path exercised continuously, `fp_checked: 116`), zero
  fast dets sat ≥60% inside a building/house annotation: the demo cars
  loop far from the staged houses, so no mover ever overlapped. NOT
  forced; absence recorded honestly. Unit tests pin the math incl. the
  0.6 boundary and the exact-frame rule.
- **A/B numbers** — the table above (`ab_collect.py` on the host).

## Gates

- `uv run --with pyyaml --with numpy pytest tests/ -q` → **753 passed,
  1 skipped** (708 M2 baseline + 45 new: cadence gating, skip-if-busy
  no-queue, ≤0.5 s expiry, overlap math + 0.6 boundary + exact-frame rule,
  gate matrix incl. the exempt flip, /state schema + expiry; the skip is
  the container-only resolution test). Re-run green AFTER the gate flip.
- Import-rules gate green: slowlane is stdlib+perception only (publisher
  injected); observatory stays topic-only.
- JS syntax-checked (`node --check` on the extracted page script).

## Deviations / ops notes

1. **Takeoff preflight stalled ~15 min post-boot** — `Preflight Fail: Yaw
   estimate error` (+ one `GPS Vertical Pos Drift`) while the EKF converged
   under host load ~19–20 (my full-suite run overlapped the container
   boot). Cleared on its own; `stage.py` then took off normally. Same
   family as the RUN-DEMO load warning; not M3-related.
2. **Pre-existing pilot CPU burn observed** — the pilot process holds
   ~13.5 cores from boot (native threads; a faulthandler dump shows ALL
   Python threads idle: detector wait_next, rclpy spin, slowlane Event
   wait, to_thread workers). Points at the onnxruntime thread pools, not
   M3 code; it was identical in both A/B arms (and in M1/M2, plausibly the
   old container's 0.41 RTF), so the comparison stands. Worth its own
   investigation outside this milestone.
3. **The M2 tap proxy (:8101) was kept in path** — the container's
   DEEP_PERCEPTION_URL still points at it; the A/B sidecar p50/p95 are
   wire times through the tap (adds ~1–2 ms vs direct, per M2's note). The
   staging `probe.py` hit :8100 directly to keep the tap log A/B-clean.
4. **Duplicate pilots briefly during post-A/B restarts** — my pilot-only
   restarts raced the singleton lock/rm and two wrappers ran concurrently
   for ~2 min BEFORE the flipped-default check; all A/B collection windows
   were single-pilot. Cleaned (pkill + lock rm); the final pilot runs with
   `env -u DEEP_SLOWLANE` (the flipped default).
5. **The pilot wrapper** — `/tmp/pilot_wrap.py` (faulthandler-on-SIGUSR1)
   was used for the CPU investigation and kept for the restarts; it execs
   the unmodified `agents.pilot.run`.

## Left for M4

- Per-concept recall/FP numbers + mask IoU on the recorded pursuit-aspect
  set; the slowlane operating point re-checked against them.
- nvidia-render coexistence stays unmeasured — the gate holds there.
- The native-thread CPU burn (deviation 2) and the RTF headroom question.
- The sidecar and tap proxy are LEFT RUNNING
  (`pgrep -f agents.vision.deep.service`, `:8101` tap); the demo stack is
  up with the slowlane at its flipped default.
