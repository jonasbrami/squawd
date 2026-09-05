# W3-rerun — validation verdict (demo world, codex re-staging, 2026-08-02)

**Scope.** ONE reviewed-design validation of the W3 pursuit fix set
(`w3-pursuit-fix-codex.md`: vehicle superclass association keys, COCO 5 s
lost/rebind grace, designated readoption relaxation, `hold_altitude` opt-out,
HUD honesty), per the codex staging prescription (6 m alt, 18–24 m horizontal
stand-off, scripted takeoff + relative fly, LLM-free click path). Protocol:
one run + exactly ONE fresh-container diagnostic repeat, then stop. **No
production code was changed** (no small integration bug surfaced — the
failure is a design-geometry gap, below); the suite was not touched (595
green as given at handoff).

**Verdict in one line:** the click→arbiter→track chain and the altitude fix
work, but sustained pursuit FAILS with the same <10 s signature as the
pre-fix run — **W3: FAIL (circle → codex)**. The fix set's grace/readoption
machinery never engages: the pursuit's own close-in flies the drone into the
±21.1° vfov blind cone, the car leaves the frame 2–3 s after every lock, and
no re-detection (hence no readoption) ever occurs inside the 5 s grace.

## Attempts and per-step verdicts

| Step | Attempt 1 (fresh container) | Attempt 2 (diagnostic repeat, fresh container) |
|---|---|---|
| Staging | (E10,N0) @6.2 m facing east — 20–22 m from car_1's west leg | (E46,N0) @6.1 m facing west — 20 m from the west leg |
| 1. Click | **PASS** — 200 `vis_car_18` (attempt 29, 42.1 s; box fully in-frame, ranged ≤40 m) | **PASS** — one 409 `ambiguous` (double-birth correctly refused), then 200 `vis_car_9` (attempt 25, 38.6 s, boresight) |
| 2. Pursuit ≥90 s no-LOST | **FAIL** — LOST ≤ ~8 s | **FAIL** — LOST ≤ ~9 s |
| 3. Standoff 12 m | not run (blocked) | not run (blocked) |
| 4. Orbit 15 m + stop | not run (blocked) | not run (blocked) |
| 5. Estop | not run (blocked) | not run (blocked) |

Honest numbers (both attempts):

- The op **did engage**: OFFBOARD chase of ~32 m (10,0)→(35.6,−19.1) in
  attempt 1 and ~29 m (46,0)→(48.6,−29.1) in attempt 2 (timeline E/N,
  mode=OFFBOARD rows), then the structured LOST break and a safe HOLD.
- Contact life post-lock: ≤ ~7–8 s both times (car exits the frame at
  ~2–3 s post-lock — late-transit/edge geometry at the lock instant — then
  `_drop_stale` at the 5 s grace, op LOST-breaks, PX4 HOLD).
- **Flicker/readoption: zero adoptions in both attempts** (track targets
  stayed `vis_car_18` / `vis_car_9` until LOST; no candidate births while
  the ops lived — the 10 Hz autopsy shows dets 0–1 and `contacts=[]` for
  the full 100 s post-drop window, i.e. the car sat in the drone's blind
  cone, not in association gates).
- Gap stats: not measurable (no `gap_m` sample while alive — samplers
  started as the ops died; op life ≈ 7–9 s).
- **Altitude: held 6.2–6.5 m throughout (attempt-2 mean 6.34)** — the
  `hold_altitude` opt-out visibly works; the pre-fix M3b sag (5.5→2.3 m) is
  gone. Beam: SEARCHING only (zero LOCKED, zero fused range; ToF statuses
  seen this session: SEARCHING, plus OUT-OF-ENVELOPE pre-lock) — expected
  per the codex verdict (beam LOCKED not required at 6 m over flat roofs).
- LLM-free: CONFIRMED both attempts — `pilot.log` boot lines only (7–8
  lines), zero backend requests; all ops via `/api/lock` + `/pilot/cmd`.

## Mechanism (why the fix set never gets its chance)

The codex §4 staging fixes the *click* geometry (6 m, 18–24 m) and removes
the altitude sag, but the **lock op's shadow default** (`range_m=None` →
close to the 7 m keep-out bubble, `agents/flight/track.py:113-120`) at a
6 m hold flies the drone into the frame floor: VFOV ±21.1° ⇒ the car is
invisible inside ~13.7 m horizontal at 6 m (`slant ≥17.3 m` from the codex
doc's own math). The chase closes 20 m → <13.7 m in the first seconds, the
car drops below the frame, dets stop, and the 5 s grace cannot bridge a
blind cone the drone itself is parked in — no re-birth, so neither the
superclass association nor readoption relaxation is ever exercised. Same
terminal signature as pre-fix mechanism (b) (`w3-integration.md` §3), minus
the sag.

**For the next design round (codex):** the operator shadow at
`hold_altitude` needs a radial floor ≳14 m at 6 m hold (or a
depression-aware close-in law, or a ≤2.5–3 m hold altitude). Note the
validation's own step-3 value (standoff 12 m at 6 m ⇒ 23.9° depression) is
also inside the blind cone — the next spec should floor standoff ranges by
hold altitude (`range ≥ (alt−0.8)/tan(21.1°)` ≈ 13.5 m at 6 m).

## Evidence

`evals/out/w3_rerun/`: `click.log` (attempt 1: 200), `pursuit_echo.log` +
`timeline.log` (attempt-1 95 s: LOST×48, HOLD, E35.5-35.7/N−19), `shot_lock.png`
(LOST header), `pilot_attempt1.log` (LLM-free), `retry/` — `click.log`
(409→200), `timeline.log` + `pursuit_echo.log` (OFFBOARD×2 then LOST×46,
E48.0-48.6/N−26.8..−29.2), `autopsy_vis_car_9.log` (10 Hz, 982 rows: zero
births post-drop), `shot_lock.png`, `pilot_retry.log` (LLM-free). Tooling:
`w3_click.py`, `w3_position.py`, `w3_timeline.py`, `w3_capture.py`,
`w3_autopsy.py`, `w3_verdict.py` (copied from `w3_integration/`, verdict
math added).
