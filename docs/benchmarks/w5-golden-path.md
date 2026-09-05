# W5 — golden path, recorded (Demo Cockpit Prototype, 2026-08-02)

The recorded end-to-end golden path, driven headless but exactly along the
user's UI flow (`POST /api/lock` with the bbox center = the canvas click;
`POST /api/cmd` = the ops bar; `/pilot/estop` = the estop button). Fresh
container, v2 detector via explicit `VISION_MODEL=coco-nano-seg-v2-640.onnx`
override (env-verified in container + pilot process). LLM: **zero requests**
(pilot.log boot-lines only — all motion via FlightOps scripts + the two HTTP
ops endpoints + estop). Evidence: `evals/out/w5_golden/`.

## Beat-by-beat (boot 4, 14:58–15:12 UTC; times from `ops.log` + `timeline.log`)

| # | Beat | Outcome | Evidence |
|---|---|---|---|
| 1 | Takeoff to 4 m, position (50,−14) facing S, 16 m from car_1's leg | **OK** — arrived (50.0,−14.0, 4.3 m), HOLD | `ops.log` beat 1, `timeline.log` |
| 2 | Click: `POST /api/lock` {bbox center} | **200** `vis_car_6` (attempt 8, 10.5 s in) | `ops.log` beat 2 |
| 3 | Pursuit engaged — VISION LOCK pill (`track.target` set) | **OK** — OFFBOARD, COASTING, gap 26.2, beam NO-RETURN (pill semantics: VISION LOCK + honest ToF status; died ~18 s — run-8 caveat) | pill read in `ops.log`, `beat_lock.png`, `beat_pursuit.png` |
| 4 | Re-lock (R4 convention) | **200** `vis_car_13` (lap-dictated ~40 s) | `ops.log` beat 3b |
| 5 | Orbit 15 m @ 8 dps | **200 engaged, died <30 s** (engagement fragility — see caveats) | `ops.log` beat 4, `beat_orbit.png` (honest LOST frame) |
| 6 | Back-off 18 (`standoff 18`) | **200 engaged** (`vis_car_23`, gap 24.0→; died ~9 s) | `ops.log` beat 5, `beat_standoff.png` |
| 7 | Approach 14 (`standoff 14`) | **200 engaged** (`vis_car_26`; died ~7 s) | `ops.log` beats 6–9 |
| 8 | Stop → hold | **OK** — 200, HOLD spd 0.0 | `ops.log`, post-stop read |
| 9 | Resume → re-engage | **not re-testable at this beat** (no live contact in window; the resume path itself proven earlier — run 8 `w3-run8.md`, and the estop chain below used a live re-lock) | `ops.log` |
| 10 | Estop mid-track | **OK** — PRE OFFBOARD/ACQUIRING `vis_car_30` gap 21.7 → chat `estop: drone_0 HOLDING (estop) (tool cancelled: True)` → HOLD | `ops.log`, `beat_estop.png` |
| 11 | Recovery: estop `land` | **OK** — LAND, alt 0.3 m (estop release is not a wired op; a second estop with `land` is the supported recovery) | `ops.log` beat 10 |

LOST/re-lock cycle ran all run (vis_car_6→13→15→23→26→30) — the honest
state of the engagement layer (see caveats).

## The video artifact

`evals/out/w5_golden/golden_path.mp4` — the full-POV recording of the whole
path (two contiguous segments, concatenated losslessly): **h264/yuv420p,
640×360, 10 fps, 11 min 46 s, 7,068 frames, 7.9 MB**. The cockpit-overlay
content is burned in (COCO boxes + masks + track/beam/pill header,
freshness-gated at 0.5 s; raw frames when no fresh snapshot). Verified:
ffprobe stream OK + probe frames at t=330 s (car painted, VISION LOCK
header) and t=600 s (post-estop) read back — real frames, no staging
(`probe_frame_t330.png`, `probe_frame_t600.png`).

## Caveats carried (honest)

- **Engagement fragility**: most shadow/standoff engagements died in 4–20 s
  (run-8's 64.8 s endurance was the strong window, not the norm). LOST →
  re-lock works but is lap-window-limited (35–45 s typical, not the ≤8 s
  ideal). The corner gap-band swings (72 % in-band, run 8) apply.
- **Radius accuracy (W4)**: EKF-vs-truth ghost offset ~7 m on orbit; the
  lane tracks its EKF reference (p50 14.7 m on a 15 m command).
- **Persons are display/best-effort only** — engage CARS.
- **v2 qualified band**: 10–22 m slant; engagement geometry should stay in
  it. v2 is NOT the demo default yet — promotion is gated on a quiet-host
  interleaved latency re-bench (p50 ≤50 ms / p95 ≤70 ms / ≤10 % vs v1).
- **Environment (this run's boots 1–3)**: PX4 EKF yaw failed to converge
  under host load ≥~30 (time-sync storms → "blind land" failsafes; one boot
  had no compass data at all). Boot 4 on a quiet host was healthy
  (0 compass fails, detector 40 ms). If you hit arming denial: check
  `Ready for takeoff!` in the PX4 log and the host load first.

## HOW TO RUN THE DEMO

Host prerequisites: docker, the repo at `/home/quenouille/drone`, `.env`
with `KIMI_API_KEY`, a quiet-ish host (load < ~15 on 20 cores matters at
boot time).

```bash
cd /home/quenouille/drone
set -a; . ./.env; set +a
VISION_MODEL=coco-nano-seg-v2-640.onnx SQUAWD_BACKEND=kimi \
  ./scripts/run_single_demo.sh demo        # fresh container + doctor gate + pilot
# cockpit server (once per container):
docker exec -d pilot-sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
  source /opt/px4_ws/install/setup.bash; cd /workspace && \
  PYTHONPATH=/workspace:$PYTHONPATH PYTHONUNBUFFERED=1 \
  uv run --no-project python -m agents.observatory.server \
  > /tmp/cockpit.log 2>&1'
```

- **Browser**: http://localhost:8000 — live POV + boxes; click a contact to
  lock; the ops bar runs Orbit / Approach / Back-off / Stop / Resume;
  ESTOP button top right.
- **Headless click**: `POST /api/lock {"x": <bbox center x>, "y": <y>}`
  (409 = stale/ambiguous/miss — retry on the next frame).
- **Ops**: `POST /api/cmd {"op":"orbit","contact":name,"radius_m":15,"rate_dps":8}`
  · `{"op":"standoff","contact":name,"range_m":14..30}`
  · `{"op":"stop"}` · `{"op":"resume","contact":name}`.
- **Estop**: `docker exec pilot-sim bash -lc "ros2 topic pub --once \
  /pilot/estop std_msgs/String \"{data: 'hold'}\""` (`'land'` to land).
- **Logs**: `docker exec pilot-sim tail -f /tmp/pilot.log` (should stay
  boot-lines only in headless use — LLM is idle).
- **Teardown**: `docker rm -f pilot-sim`.

FlightGeometry for the demo world: take off to **4 m**, put the car's leg
**14–18 m** away (inside v2's 10–22 m qualified band), and prefer locking
mid-leg (the mover's 90° corners are the engagement-killers).
