# RUN-DEMO — Demo Cockpit Prototype runbook

One drone, one pilot agent, the `demo` world (3 cars + 2 walkers on loops),
COCO-v2 detector at 640×360@10, and the cockpit web UI on :8000. Structured
cockpit operations are LLM-free; natural-language `/command` requests invoke
the configured Codex, Claude, or Kimi backend.

## Prereqs

- Repo at `/home/quenouille/drone`.
- A local, built `PX4-Autopilot/` checkout. It is git-ignored and is not built
  by the Dockerfile; `build/px4_sitl_default/bin/px4` must already exist.
- A prebuilt `squawd:dev` image:

  ```bash
  docker build -f docker/Dockerfile.swarm -t squawd:dev .
  ```

- `models/coco-nano-seg-v2-640.onnx` plus its JSON manifest. V2 remains an
  explicit demo override rather than the launcher's default.
- For Codex, run `codex login` on the host and confirm `~/.codex/auth.json`
  exists. This route uses the logged-in ChatGPT/Codex subscription, not
  `OPENAI_API_KEY`.
- For Kimi, use `.env` containing `KIMI_API_KEY`; for Claude, use a logged-in
  host CLI with `~/.claude/.credentials.json`.
- Docker. A reasonably quiet host at boot time (load < ~15 on 20 cores;
  PX4's EKF fails to converge yaw under load ≥ ~30 — "blind land"
  failsafes; check `Ready for takeoff!` in the PX4 log if arming is denied).
  M4 note (2026-08-03): with ~5 GB swap pressure the yaw/height/GPS-drift
  preflight flap already appeared at load ~20 and lasted ~90 min — if
  `takeoff` keeps getting silently refused while MAVLink
  `telemetry.health_all_ok` reads true, send **`arm()` first, then
  `takeoff()`** (arm catches a clean EKF window; the takeoff command
  re-runs the failing check). Details: docs/benchmarks/deep-perception-m4.md §8.

- Intel or NVIDIA rendering. `RENDER_BACKEND=cpu` currently selects the
  camera-less `gz_x500` model and cannot pass the demo's camera preflight.

> **Network boundary:** the cockpit has no authentication and is published on
> host port 8000. Use it only on a trusted local simulation workstation; do not
> expose it to an untrusted network or real vehicle.

## Launch the simulator and pilot

Kimi (existing Claude SDK → Kimi coding endpoint route):

```bash
cd /home/quenouille/drone
set -a; . ./.env; set +a
VISION_MODEL=coco-nano-seg-v2-640.onnx SQUAWD_BACKEND=kimi \
  ./scripts/run_single_demo.sh demo
```

Codex (defaults to `gpt-5.6-terra`, low reasoning effort):

```bash
cd /home/quenouille/drone
VISION_MODEL=coco-nano-seg-v2-640.onnx SQUAWD_BACKEND=codex \
  ./scripts/run_single_demo.sh demo
```

Override with `SQUAWD_MODEL=<model>` and, for Codex,
`SQUAWD_CODEX_EFFORT=low|medium|high|xhigh`. Direct Claude remains available
with `SQUAWD_BACKEND=claude`. The default backend remains Claude when
`SQUAWD_BACKEND` is unset.

The script starts a fresh container from the existing image, gates on
`scripts/doctor_sim.sh`, and starts the pilot agent. It does not build the image
and it does not start the cockpit. Start the cockpit once per container:

```bash
docker exec -d pilot-sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
  source /opt/px4_ws/install/setup.bash; cd /workspace && \
  PYTHONPATH=/workspace:$PYTHONPATH PYTHONUNBUFFERED=1 \
  uv run --no-project python -m agents.observatory.server \
  > /tmp/cockpit.log 2>&1'
```

Readiness checks:

```bash
curl -fsS http://localhost:8000/state >/dev/null
docker exec pilot-sim tail -n 30 /tmp/pilot.log
docker exec pilot-sim tail -n 30 /tmp/cockpit.log
```

`doctor_sim.sh` reports the selected backend, model/effort where applicable,
runtime import readiness, and credential presence without printing credential
contents. On Codex, the launcher creates a fresh `/tmp/pilot-codex`, copies only
the host's `auth.json`, and mounts it as writable `/root/.codex`; it does not
copy the host's `config.toml`, MCP servers, plugins, skills, or workspace state.
The pilot then starts a required, bearer-authenticated MCP endpoint bound only
to container loopback. If that endpoint cannot initialize, pilot startup fails
instead of falling back to another API or backend.

## Optional deep-perception sidecar

The fast COCO lane and cockpit work without the sidecar. To enable
open-vocabulary `look`, prompted `pinpoint`, and slow-lane annotations, provision
the pinned weights once and start the host service before or after the demo:

```bash
./scripts/provision_deep_models.sh
uv venv .venv-train-gpu
uv pip install -p .venv-train-gpu -e '.[deep]'
./scripts/deep_perception.sh
```

In another shell:

```bash
./scripts/deep_perception.sh --selftest
```

If `.deep_token` exists before `run_single_demo.sh` starts, the launcher passes
the token and `http://host.docker.internal:8100` endpoint into the container and
prints a reachability result. If the token is created afterward, restart the
demo container so the pilot receives those environment variables.

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
- **Deep layer (M3)**: the host-GPU sidecar (bearer in `.deep_token`) serves the
  `look`/`pinpoint` LLM tools and the gated slow-lane annotator
  (`agents/vision/slowlane.py`). Slow-lane annotations (magenta boxes) and
  the pinpoint mask (translucent silhouette) render in the cockpit,
  frame-age-gated ≤0.5 s; `fp_suspect` advisories flag contacts in /state.
  Gate: default OFF only when `RENDER_BACKEND=nvidia` (the armed gate was
  lifted for intel after the M3 A/B — docs/benchmarks/deep-perception-m3.md);
  `DEEP_SLOWLANE=on|off` forces; `DEEP_SLOWLANE_HZ/VOCAB/CONF` tune
  (defaults 0.3 Hz, `building,house,tree,pole,tower`, conf 0.05).
- **Teardown**: `docker rm -f pilot-sim`.

Stop the sidecar with `Ctrl-C` in its host terminal. `.deep_token` and model
weights are local, git-ignored artifacts.

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
