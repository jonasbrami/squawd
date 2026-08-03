# W3-integration — live validation verdict (demo world, 2026-08-01)

**Scope.** Headless golden path per design §5-§6: takeoff → headless UI click
(`POST /api/lock`) → pursuit with beam LOCKED → estop mid-track, plus orbit
smoke. All LLM-free (click path rides `/api/lock` + `/pilot/cmd`, never the
LLM). Suite: **582 → 585 green** (3 new pinning tests).

**Verdict in one line:** the deterministic chain (click → arbiter →
`ops.track` → offboard pursuit → estop) WORKS end-to-end; the pursuit itself
does NOT survive the demo world's COCO-perception noise — every engagement
ended `LOST` in ~2-20 s and the beam never reached LOCKED. **W3: PARTIAL —
click/dispatch/estop PASS, sustained pursuit + LOCKED FAIL** (root cause in
perception tracking, not in the W3 wiring; two small W3 wiring bugs were
found and fixed, below).

## 1. PASSED (with evidence)

- **Takeoff + positioning, LLM-free.** A small in-container script
  (`evals/out/w3_integration/w3_position.py`) reusing the repo's FlightOps
  over a second mavsdk `System` at 127.0.0.1:50051 (`take_off` / `goto` /
  `face`). Repeatedly took off and positioned to within 0.1-3.9 m of target
  (e.g. `drone_0 -> E50 N52 alt 3; arrived (0.1m off target)`). No
  `/pilot/user_input` was ever published.
- **Click chain.** 9 successful headless clicks: `POST /api/lock {x,y}` with
  the bbox center → **200 + contact name**, e.g.
  `{"locked": "vis_car_51", "x": 262.5, "y": 267.9, "attempt": 3}`; one
  overlapped click correctly returned **409 `{"reason":"ambiguous"}`**
  (attempt 6, first session). Each lock was ack'd on `/pilot/chat`
  (`cmd lock: ok`) — i.e. `/pilot/cmd` → CommandArbiter lease →
  `ops.track(name, mode="shadow")` dispatched.
- **Ops bar dispatch.** `POST /api/cmd` standoff → `200 {"ok":true,"op":"standoff"}`;
  orbit → `200 {"ok":true,"op":"orbit"}`; stop → `200 {"ok":true,"op":"stop"}`
  with the drone holding after (`mode=HOLD spd 0.0`).
- **Pursuit ENGAGES (dispatch + offboard streaming are live).** On the
  vis_car_2 engagement the drone physically pursued ~34 m from (50.0,52.1) to
  (26.9,19.3) under OFFBOARD; on vis_car_56 the sampler caught
  `mode=OFFBOARD alt=2.1 spd=1.8 track=COASTING target=vis_car_56
  gap_m=68.5 beam=SEARCHING`.
- **Estop mid-track — PASS.** Estop fired while the standoff op was OFFBOARD
  → `/pilot/chat`: **`estop: drone_0 HOLDING (estop) (tool cancelled: True)`**
  — the operator lease was cancelled through THE shared ActiveToolRegistry
  and the hold landed via the same FlightOps (ICD §7.1 / W0.4 semantics);
  post-estop samples: `mode=HOLD spd=0.0-0.1 armed=True alt=3.3`.
- **LLM-free — CONFIRMED.** `/tmp/pilot.log` contained only the two boot
  lines (`drone_0 connected` / `pilot online: drone_0 — waiting for commands
  on /pilot/user_input.`) across the entire session — no backend requests,
  no query traces. KIMI_API_KEY was present for backend init only; the pilot
  never received user_input. (The final `grep -ci` was not re-run — the box
  was frozen by stop order; the log was read in full repeatedly.)
- **Suite.** Baseline 564 passed + 18 skipped (=582); after fixes below:
  **567 passed + 18 skipped (=585)**, zero regressions.

## 2. FAILED (timeline evidence)

Sustained pursuit and any LOCKED sample. Eleven engagement attempts
(pursuit_echo*.log) all ended the same way; representative lines from
`evals/out/w3_integration/timeline.log` / pursuit echoes:

```
[pursuit   0.0s] mode=HOLD alt=2.4 track=LOST gap=None beam=SEARCHING range=None contacts=1
[pursuit   0.0s] mode=HOLD alt=2.2 track=LOST gap=None beam=EDGE-MIX  range=None contacts=1
mode=OFFBOARD alt=2.1 E=33.7 N=23.1 spd=1.8 track=COASTING/vis_car_56 gap=68.5 beam=SEARCHING
```

- track went DESIGNATED → (brief OFFBOARD) → **LOST** in ~2-20 s on every
  run; the drone held safely each time (structured LOST, no flyaway).
- beam statuses seen: SEARCHING, NO-RETURN, EDGE-MIX, OUT-OF-ENVELOPE —
  **zero LOCKED**, zero range_m.
- Contact IDs churned continuously even under continuous visibility:
  `vis_car_19→20→21→…→29` across 90 s in one window; locked contacts died
  and rebirthed (`vis_car_51` → `vis_car_56` → `vis_car_59`-era) within
  seconds of each lock. Double-births of one physical car were observed
  (`vis_car_19` + `vis_car_20` simultaneously).
- The M3b altitude profile visibly sagged the drone: `alt=5.5→4.9→3.5→2.3`
  across failed runs.
- Orbit smoke: op accepted (200) but the contact churned before the orbit
  engaged (`track=LOST beam=OUT-OF-ENVELOPE` for the full 32 s window);
  stop → 200 + HOLD verified. Live orbit-radius stats: not measurable.

## 3. Diagnosis — two failure mechanisms

**(a) Contact-ID churn kills the track.** COCO-nano@640 on the Fuel car
meshes is noisy at 20-60 m: class flapping (car↔truck — association gates
strictly on class, `agents/vision/contacts.py:464`), double-births, and
constant false positives (`airplane` 0.26-0.56, `potted plant`). Track coasts
on rejected/absent measurements, and after `TrackerConfig.lost_s = 2.0`
(`contacts.py:52`) `_drop_stale` (`contacts.py:886`) deletes it to the
graveyard; re-detection births a NEW id. The pursuit's adoption
(`agents/flight/ops.py:134` `_readopt_contact`, called at `ops.py:642`)
refuses ambiguity (`len(matches) != 1` — the double-birth case) or misses the
5 m gate, so the op exits via the LOST break (`agents/flight/ops.py:661-667`).

**(b) The M3b shadow-mode altitude profile descends the drone out of the
detection envelope.** `agents/flight/ops.py:833`:
`alt_ref = min(alt, max(2.3, 0.18 * gap + 1.1))` sags the pursuit toward
2.3-3 m. The body-fixed camera has **±21° vfov** (hfov 69°, 640×360): below
~4-5 m the car at 8-15 m sits at/below the frame floor or in horizon ground
clutter (the captured `probe_view.png` frame shows exactly this clutter with
`airplane` FPs). At ≥5 m start alt the horizontal closure (≤6 m/s) outpaces
the descent, so the drone arrives over the car still high → car exits the
vfov floor → det stops → mechanism (a). M3b was proven against the
mover-nano red-box detector, which has none of this noise.

Design-geometry note: the spec's "~14 m alt, 30-40 m" engagement is not
viable in this world — at 14 m the car is at 19-27° depression (outside the
±21° vfov for much of the loop), `house_1` (16.5×12.9×7.7 m at (50,-48))
occludes the whole southern vantage (see `car_centered.png`: frame full of
roof), and recall is effectively dead beyond ~40 m slant.

## 4. Integration bugs FIXED (minimal, test-pinned, suite green)

1. **`ops.track` never designated an already-positioned (geom) contact**
   (`agents/flight/ops.py`, designate block after contact resolution +
   re-designate on name-churn adoption). Before: `_feed_tof` idled on
   `designated is None`, so the ToF beam never fused on the click path and
   the cockpit track banner/beam chip stayed IDLE for whole pursuits —
   `set_beam_context` was being fed every tick with no consumer (design §5
   says the click path runs through `VisionContacts.designate()`).
   Tests: `tests/test_track_ops.py::test_track_designates_positioned_contact`,
   `tests/test_track_ops.py::test_track_readoption_redesignates`.
2. **The live path never applied the pursuit tuning.** Only eval harnesses
   called `tune_pursuit_params` (MPC_TILTMAX_AIR=12°, MPC_XY_VEL_MAX=6);
   `ops.track` now applies it itself (best-effort, idempotent).
   Test: `tests/test_track_ops.py::test_track_applies_pursuit_tuning`.

Suite: 582 → 585 (3 new tests), green in-container. Honesty note: fix #2 is
correct hygiene but did NOT change the LOST outcome — the churn (3a) is the
dominant mechanism.

## 5. Tried and did NOT work (feeds the codex review)

- Six engagement vantages × altitudes 14 / 11 / 9 / 6.5 / 5 / 4 / 3.2 m —
  LOST every time (11 pursuit timelines).
- Click gates: box-size, EKF range, edge margin, boresight-phase (cx bands)
  — better lock quality, no survival gain.
- Radial stand-off (`standoff R=15`) instead of plain shadow — longest
  OFFBOARD survival (~15-20 s), still LOST.
- Orbit chained 0.8-2 s after lock — contact churned before engagement.
- Environment incidents (not code): a stale `w0-assets` container cross-
  talked on the shared docker bridge ROS graph (2 publishers on
  `vehicle_local_position` — poisoned session 1; container stopped, pilot+
  cockpit restarted); a z-fix drift (+13 m) made one `goto` compute a
  below-ground absolute-altitude target and PX4 descended to the ground
  (armed, no failsafe) → full sim restart; prefer relative moves or
  takeoff+`face` for demo scripting.

## 6. W4 focus (handoff)

- Fix contact survival for COCO-class noise in VisionContacts: soften the
  strict class gate for vehicle classes (or class-agnostic merge), lengthen
  `lost_s` under continuous detections, make adoption ambiguity-tolerant —
  a design change for review, not a W3 patch.
- Re-spec the golden-path engagement envelope to dep ~8-18°, slant ~12-25 m,
  alt ≤6 m; revisit the altitude-profile sag for the demo world.
- Then re-run the golden path: click → pursuit → LOCKED → estop → orbit
  R15 → stop/resume.

**Evidence.** `evals/out/w3_integration/`: `timeline.log` (all phases),
`click.log` (9 locks + 409 + standoff/orbit/stop posts), `pursuit_echo*.log`
(11 timelines), `car_centered.png` (occlusion), `probe_view.png` (ground
clutter/FPs), `shot_pursuit.png`, and the tooling (`w3_position.py`,
`w3_click.py`, `w3_timeline.py`, `w3_capture.py`, `w3_autopsy.py`,
`w3_probe.py`).
