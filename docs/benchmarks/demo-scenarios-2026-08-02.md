# Demo scenarios — real-life campaign verdict (2026-08-02)

**Stack under test.** Fresh container `pilot-sim`, world `demo` (3 cars + 2
walkers on loops), detector `coco-nano-seg-v2-640.onnx` via explicit
`VISION_MODEL` override (env-verified in container + pilot process),
`SQUAWD_BACKEND=kimi` with the **Kimi K3-256k pilot LLM** (owner question:
is the chat path faster/better vs the old mandatory-thinking tier — the
`CLAUDE_CODE_EFFORT_LEVEL` var was NOT visible in the pilot process env,
noted honestly). Cockpit http://localhost:8000. No product code changed
anywhere in the campaign; bugs and oddities are recorded, not fixed.
Evidence root: `evals/out/demo_scenarios/<slug>/` (timeline.log = /state at
2 s, scenario logs, overlay frames).

**Headline.** Both control paths deliver the demo's core loop — click or
chat → lock → pursuit with ops and a proven estop — and the K3 chat path is
genuinely better at improvisation (self-initiated repositioning, goto-circle
orbits, blind-cone-aware climbs) with honest limits (it narrated a
"couldn't" when it truly couldn't). The engagement layer underneath remains
the prototype's weak muscle: typical visual pursuits die in 4–20 s, the
mover's corners kill them, and at the z-safe 10 m staging altitude the
v2 qualified band (10–22 m slant) is geometrically unreachable
(R_min(10) = 30 m), so car work below ~6 m is where the prototype lives.

---

## S1 — "tail the suspect" (chat: *take off to 10 meters and follow the red car*) — **PASS**

- TTFA (chat → first physical effect, armed/TAKEOFF): **23.6 s**. Chat →
  pursuit engaged (OFFBOARD, ACQUIRING): **50.6 s** (inside the 90 s gate).
- Engagement: ~40 s contiguous (ACQUIRING→COASTING→LOST; gap 84→31 m
  closing, then the car outran/ghosted to 101 m and dropped). The LLM lane
  sagged alt 11→6.5 m (M3b profile, no hold-altitude opt-out — expected).
- K3 behavior note: its **first tool guess was wrong** (`track vis_car_0` —
  contact didn't exist; the tool wrapper returned the error cleanly), it
  recovered with the right contact on its own. Its chat narrative
  ("Took off to 10m, located the car (vis_car_8)… shadowed it ~45 s,
  following it ~130 m NE (min gap 30 m)") matches the flight data.
- pilot.log carries **no per-run usage/ttfa lines** (boot lines + tool
  tracebacks only) — latency was measured via /state timestamps as planned.
- Evidence: `s1_tail_suspect/` (timeline.log, chat.log, chat_reply.txt,
  shot_final.png).

## S2 — "inspect the delivery truck" (chat: *orbit the truck at 20 meter radius*) — **DEGRADED**

- First attempt: k3 honestly reported **"Could not orbit: no truck is
  present anymore"** — the drone had drifted almost directly above the
  truck's route (E90,−20 at 9.5 m; the truck 11 m away = 40° depression,
  inside the camera's blind cone). Honest failure, geometrically correct.
- After ONE nudge (route + position): the k3 **climbed to 15 m on its own
  initiative** (out of the blind cone — smart) and **flew a self-composed
  goto-circle**: 118 s of continuous circling at **radius mean 21.7 m**
  (min 14.6, arc-fit center ~(90,62)) vs the 20 m request.
- **But the center was a phantom**: the real car_3 never leaves its
  E85–120/N−40..0 route; the k3 "detected" the truck at E91 N67 — a
  truck-class false contact in the gas-station area (the same FP family
  seen live as triple `vis_truck_*` births in `s2_orbit_truck/shot_orbit_zone.png`).
  It orbited the phantom perfectly and never engaged the real truck.
- Side finding: during the goto-circle the cockpit `mode` read a raw
  PX4 nav_state enum (`#21`) the server's mode map doesn't name — cosmetic.
- Evidence: `s2_orbit_truck/` (timeline.log, chat.log, shot_orbit_zone.png).

## S3 — "patrol then shadow" (chat: *take off to 10m, fly to east 50 north -30, scan, then shadow the truck for 30 seconds*) — **PASS**

- All four legs executed **in order, zero stalls >120 s**: descend to
  ~9.7–10 m ✓ → arrive (50.1,−30.0) ✓ → scan hold ~25–30 s ✓ → shadow
  engaged for **~32 s** (ACQUIRING/COASTING with a LOST→readoption flicker
  in the middle — the R1/R7 machinery recovering mid-leg).
- Notable: when the truck wasn't visible from the scan point, the k3
  **repositioned itself to (80,35)** near the route's north leg before
  engaging — initiative the §3.3 weak-link note didn't predict. Target
  identity (`vis_truck_51/66`, gap 44–49 m) is consistent with the real
  car_3 on the route (40–50 m) but carries the FP caveat from S2.
- Evidence: `s3_patrol_shadow/` (timeline.log, chat.log, shot_shadow.png).

## S4 — "click-lock pursuit" (click path, stage → lock → shadow through ≥2 corners) — **FAIL (gate)**

- Staging truth-in-advertising: the prescribed 10 m (50,−60)/N put the car
  behind house_1's roof with bottom-clipped bearing-only contacts; (50,−64)
  same. Restaged to 10 m (50,60)/S — leg at 30 m, in-frame, but slant
  **35–40 m, outside v2's qualified 10–22 m band**.
- Three click engagements (vis_car_47/63/69): all died in **8–13 s**, zero
  corners survived. Re-lock latencies 27–110 s (lap-paced, not ≤8 s).
  Beam statuses honestly: NO-RETURN / OUT-OF-ENVELOPE / SEARCHING, never
  LOCKED (expected on ground cars).
- Read: consistent with the documented baseline (typical 4–20 s
  engagements) — the ≥20 s bar is a strong-window event (run-8's 64.8 s),
  and 10 m staging pushes every engagement out of the recall band.
- Evidence: `s4_click_pursuit/` (stage.log, timeline.log ×5, shot_final.png).

## S5 — "VIP stand-off" (standoff 15 then 25) — **DEGRADED**

- At 10 m both POSTs returned 200 but engagements died in 5–8 s (out of
  band); declared **environmentally invalid** and restaged to 6 m (50,48)/S
  (R_min(6) = 18 m, inside the band).
- **standoff 15 → gap 22.7→**18.9** settled in ~10 s** — the R2 clamp
  (15→18 m at 6 m) working live and exactly, ring held ~40 s.
- **standoff 25 → gap 18.9→34.8→**29.7** settled** (band edge, +4.7 of
  setpoint) at ~37 s post-POST (~7 s over the 30 s criterion), then LOST.
- The stand-off **control** converged on both setpoints; the second
  convergence was late and the engagement died. Ghost caveat applies to
  all EKF gap numbers.
- Evidence: `s5_vip_standoff/` (click.log, timeline.log ×2, shot_final.png).

## S6 — "lost & found" (release, lap, re-acquire) — **PASS**

- Locked vis_car_107, deliberately stopped at 18:08:02 (stop 200, op
  canceled, drone held). The car lapped away; re-acquired with a fresh
  click (200, vis_car_109) at **+52 s** — inside one lap, just over the
  ~35–45 s typical band (contacts appeared at +34 s but only became
  gate-clickable at +51 s — edge/range/cx gate timing, recorded honestly).
- Evidence: `s6_lost_found/` (click.log, timeline.log, shot_final.png).

## S7 — "person of interest" (click a walker, follow) — **PASS (best-effort)**

- First-attempt lock (0.0 s) on walker_1 (`vis_person_30`, box 21×50 px at
  18 m). Pursuit engaged OFFBOARD at gap 18.3 m with a **dwell of ~9 s**
  before LOST — exactly the documented best-effort envelope (v2 person
  recall 86.3 %; persons are display/best-effort by design).
- Evidence: `s7_person_interest/` (click.log, timeline.log, shot_final.png
  showing live `vis_person_*` dets).

## S8 — "emergency stop" (estop mid-pursuit, then recover) — **PASS**

- Fresh lock vis_car_124 (200), estop fired at OFFBOARD/ACQUIRING (gap
  22.0): **HOLD spd 0.0 + chat `estop: drone_0 HOLDING (estop) (tool
  cancelled: True)`** — the mid-track cancel landed through the shared
  registry.
- Recovery: `resume` returned **`cmd resume: ok`** — the Resume op releases
  the estop latch by design (design-doc semantics confirmed live; the op
  itself then found the contact had churned, honest). Second estop with
  `land` → **`estop: drone_0 LANDING (estop)`** → mode LAND → **alt 2.0 m,
  disarmed**. Stack left usable (container + pilot + cockpit all alive).
- Evidence: `s8_estop/` (click.log incl. chat lines, timeline.log,
  shot_landed.png).

---

## WHAT WE ARE GOOD AT

- **Click → lock → engaged pursuit** with VISION LOCK semantics and honest
  ToF status (RANGE LOCKED never faked; beam NO-RETURN/OUT-OF-ENVELOPE
  shown raw).
- **Estop safety**: mid-track cancel through the shared registry
  (`tool cancelled: True`), HOLD every time, resume-releases-latch
  semantics, and a land recovery that disarms.
- **K3-256k chat path (the new tier)**: TTFA ~24 s to first physical
  effect, ~51 s to a pursuit; recovers from its own bad tool calls; smart
  improvisation — self-initiated repositioning when targets aren't visible
  (S3), a blind-cone-aware climb (S2), a self-composed 20 m goto-circle
  flown for 118 s at r_mean 21.7 m; and **honest** when it can't
  ("Could not orbit…"), with flight data matching its narratives.
- **Stand-off control**: radial ring converges exactly (18.9 m on a
  clamped-18 request in ~10 s), with the R2 altitude clamp doing its job.
- **Re-acquisition**: windowed re-locks work every time a window exists
  (52 s from release; 27–45 s after natural LOSTs).
- **Person best-effort**: walker lock + short dwell delivered as designed.

## WHAT WE ARE NOT GOOD AT (yet)

- **Pursuit endurance**: typical engagement life is 4–20 s; ≥20 s is a
  strong-window event (run-8's 64.8 s). S4: three windows, three sub-15 s
  deaths. Corner handling specifically: the mover's 90° corners are still
  where engagements go to die (see S4).
- **Orbit/target accuracy**: orbit radius tracks the EKF reference, not
  truth (ghost-offset caveat — W4's ~7 m); and the chat path can be lured
  by truck-class false positives on buildings (S2's phantom orbit around
  the gas-station area — the detector's residual FP mode the v2 0/1100
  scoreboard doesn't cover in pursuit context).
- **Geometry-window interplay nobody owns**: the z-safe staging altitude
  (10 m) and the v2 qualified band (10–22 m) don't overlap (R_min(10) = 30 m),
  so 10 m car work is out-of-band by construction (S4/S5 first attempts);
  re-lock availability is lap-dictated (27–110 s), never the ≤8 s ideal.
- **Choreography reliability caveat for the chat path**: S3 passed cleanly,
  but it needed no nudges only because the k3 repositioned itself; S2
  needed one. Multi-step tasks ride on the LLM's own target-hunting, and
  the stack has no answer when perception disagrees with the LLM's map.

### Top-3 weaknesses, ranked by demo impact

1. **Engagement fragility (4–20 s typical)**, corners as the kill locus —
   it's the difference between "follow the car" and "follow the car for
   ten seconds", and it gates every other op's effective dwell.
2. **Ghost/false-target accuracy** — the ~7 m EKF-vs-truth offset that
   widens orbits, plus building-class truck FPs that can send the chat
   path orbiting a phantom; the demo can look convincing while shadowing
   the wrong object.
3. **Altitude/band geometry gap** — the z-safety rule (≥10 m staging) and
   the v2 recall band (10–22 m) are mutually exclusive for car pursuit;
   today's demo silently lives at 4–6 m, and a user staging higher will
   see instant LOSTs with no hint why (worth an explicit cockpit hint).

*Config footnote: detector `coco-nano-seg-v2-640.onnx` (sha f7007721…),
override not default; pilot.log showed zero LLM-usage/ttfa telemetry —
chat latency numbers here are wall-clock measurements via /state.*
