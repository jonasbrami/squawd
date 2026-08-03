# RUN-DEMO — Demo Cockpit Prototype runbook

One drone, one pilot agent, the `demo` world (3 cars + 2 walkers on loops),
COCO-v2 detector at 640×360@10, the cockpit web UI on :8000. Headless
(LLM-free) by default here.

## Prereqs

- Repo at `/home/quenouille/drone`, `.env` containing `KIMI_API_KEY`.
- Docker. A reasonably quiet host at boot time (load < ~15 on 20 cores;
  PX4's EKF fails to converge yaw under load ≥ ~30 — "blind land"
  failsafes; check `Ready for takeoff!` in the PX4 log if arming is denied).

## Launch

```bash
cd /home/quenouille/drone
set -a; . ./.env; set +a
VISION_MODEL=coco-nano-seg-v2-640.onnx SQUAWD_BACKEND=kimi \
  ./scripts/run_single_demo.sh demo
```

The script builds/starts the container, gates on `scripts/doctor_sim.sh`,
and starts the pilot agent. Then start the cockpit server (once per
container):

```bash
docker exec -d pilot-sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
  source /opt/px4_ws/install/setup.bash; cd /workspace && \
  PYTHONPATH=/workspace:$PYTHONPATH PYTHONUNBUFFERED=1 \
  uv run --no-project python -m agents.observatory.server \
  > /tmp/cockpit.log 2>&1'
```

## Fly it

- **UI**: http://localhost:8000 — live POV with COCO boxes/masks; click a
  contact to lock (the pill reads **VISION LOCK** + the ToF's honest raw
  status; RANGE LOCKED only when the beam truly fuses). Ops bar:
  Orbit (default 15 m @ 8 dps), Approach/Back-off (stand-off, clamped
  14–30 m), Stop, Resume, ESTOP.
- **Headless click** (what the canvas sends):
  `POST http://localhost:8000/api/lock {"x": <bbox center x>, "y": <y>}`
  → 200 + contact name, or 409 stale/ambiguous/miss (retry next frame).
- **Ops API**: `POST /api/cmd` with
  `{"op":"orbit","contact":name,"radius_m":15,"rate_dps":8}` ·
  `{"op":"standoff","contact":name,"range_m":R}` · `{"op":"stop"}` ·
  `{"op":"resume","contact":name}`.
- **Estop**: `docker exec pilot-sim bash -lc "ros2 topic pub --once \
  /pilot/estop std_msgs/String \"{data: 'hold'}\""` — HOLD + cancels the op
  (`'land'` to land). Note: estop *release* is not a wired op; re-lock
  after an estop needs a fresh container if the latch persists.
- **Logs**: `docker exec pilot-sim tail -f /tmp/pilot.log` — in headless
  use it should stay boot-lines only (LLM idle, zero requests).
- **Teardown**: `docker rm -f pilot-sim`.

## Demo-world geometry (learned the hard way)

- Take off to **4 m**, park the car's leg **14–18 m** out (v2's qualified
  band is 10–22 m slant).
- Lock **mid-leg**, not near a corner — the mover's 90° corners are where
  engagements die (even with the maneuver-mode tracker + edge barrier).
- Expect LOST↔re-lock cycling on some windows; the re-lock works whenever
  the car is clickable (typ. within a lap, ~35–45 s).
- Persons render and get boxed, but pursuit of persons is best-effort —
  the gates are validated on cars.

## Evidence / how this was validated

`docs/benchmarks/w5-golden-path.md` (recorded run + beat table +
`evals/out/w5_golden/golden_path.mp4`), `w3-run8.md` (W3 gate),
`w4-orbit.md` (W4 orbit), design doc
`docs/superpowers/specs/2026-07-28-demo-prototype-design.md` §6/§10.
