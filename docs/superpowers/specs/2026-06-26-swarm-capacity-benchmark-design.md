# Swarm capacity benchmark — design

**Date:** 2026-06-26
**Status:** approved design, pre-implementation
**Goal:** Measure how many drones the swarm can sustain, and at what camera
resolution, on this host — separately for each of the three render backends
(CPU/llvmpipe, Intel iGPU, NVIDIA dGPU) — and produce a clear, decision-useful
capacity frontier with the limiting resource annotated at each knee.

---

## 1. Question being answered

> "Using CPU, NVIDIA GPU, and Intel GPU, how many drones and what camera
> resolution can this computer sustain, and how good is each configuration?"

The deliverable is a **capacity frontier**: a `render-backend × camera-resolution
→ max sustainable drone count` table/heatmap, with a per-cell note on *what
saturated first* (CPU, iGPU render, dGPU, VRAM, or RAM).

Host under test: i9-12900H (6 P-core + 8 E-core, 20 threads), 31 GB RAM,
Intel Iris Xe iGPU + NVIDIA RTX 3070 Ti Laptop (8 GB). `nvidia-container-toolkit`
v1.19.1 installed; Docker `nvidia` runtime registered.

---

## 2. Metric definition

A `(backend, resolution, N)` configuration **PASSES** iff, over the measurement
window, all of the following hold:

1. **Delivered camera FPS (primary):** the minimum per-drone delivered frame rate
   `min_i fps_i ≥ 0.90 × CAM_FPS`, where `fps_i` = frames received from drone `i`'s
   gz camera topic ÷ window seconds, and `CAM_FPS` is the configured sensor rate
   (10 Hz default). Also report mean and p10 across drones.
2. **Real-time factor (co-gate):** Gazebo `real_time_factor ≥ 0.90` over the
   window. Guards the "slow-motion false pass" where gz holds the sensor rate by
   slowing sim-time — cameras look healthy while flight physics lags.
3. **Flight liveness:** all `N` drones are still armed/alive (telemetry flowing,
   no PX4 dropout/crash) at window end.

**Headroom snapshot** (descriptive, sampled ~1 Hz, reported per run; not a gate):
total CPU% + loadavg, RAM used, dGPU util/VRAM/power (`nvidia-smi`), iGPU
render/video-engine busy% (`intel_gpu_top -J`, needs sudo), container CPU/mem
(`docker stats`). Used to label the **limiting resource** at each knee.

The **knee** for a `(backend, resolution)` cell = the largest `N` that PASSES.

---

## 3. Architecture

A host-side Python orchestrator drives the existing Docker swarm stack once per
grid cell, with two small in-container instrumentation additions, and writes
machine- and human-readable results.

```
scripts/bench/
  run_bench.py      # orchestrator: sweep cells, ramp N to the knee, record
  sample_host.py    # ~1 Hz host sampler: nvidia-smi + intel_gpu_top + cpu/ram + docker stats
  frontier.py       # post-process per-run JSON -> frontier.md + frontier.png
docs/benchmarks/<timestamp>/
  runs.csv          # one row per run (backend,res,N,fps_min/mean/p10,rtf,headroom,verdict)
  run-*.json        # full per-run detail incl. time series
  frontier.md       # backend × resolution -> max-N table, limiting resource per cell
  frontier.png      # heatmap
```

Everything the orchestrator drives already exists (`run_swarm_demo.sh` /
`swarm_sim.sh` bring-up, observatory HTTP, ROS `/swarm/user_input`); the harness
adds a thin measurement + sweep layer on top, plus four small enabling changes.

---

## 4. Components & required changes

### 4.1 Parametrize the sim launch (enabling change)

- **`sim/launch/swarm_sim.sh`** — replace the binary `GPU_RENDER` block and the
  hardcoded `640×360@10` OakD sed patch with:
  - `RENDER_BACKEND={cpu|intel|nvidia}` selector (diffs already drafted in
    `docs/nvidia-render-investigation.md`; `cpu`→`LIBGL_ALWAYS_SOFTWARE=1`,
    `intel`→iris/renderD128, `nvidia`→force NVIDIA EGL ICD, no Mesa override).
    Back-compat: `GPU_RENDER=1` with no `RENDER_BACKEND` ⇒ `intel`.
  - `CAM_W` / `CAM_H` / `CAM_FPS` driving the sed patch instead of constants
    (defaults 640/360/10 preserve current behaviour).
- **`scripts/run_swarm_demo.sh`** — wire `RENDER_BACKEND` to the right docker
  device/runtime args (drafted diffs); `nvidia` uses the clean
  `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display
  -e __EGL_VENDOR_LIBRARY_FILENAMES=…/10_nvidia.json` path now that the toolkit
  is installed. Forward `CAM_W/CAM_H/CAM_FPS` into the container.

The default behaviour of both scripts (the interactive demo) must be unchanged.

### 4.2 Delivered-FPS counter (enabling change)

- **`agents/core/camera.py` (`GzCameras`)** — add a per-drone monotonic frame
  counter + last-frame timestamp incremented in the existing camera callback
  (zero new ROS subscriptions; it already consumes every camera topic).
- **`agents/observatory/server.py`** — surface those counters on the HTTP `/state`
  JSON the orchestrator polls (e.g. `cam_frames[i]`, `cam_last_ts[i]`). The
  orchestrator computes FPS as a delta between two polls over the window — no
  reliance on a server-side rate calc.

### 4.3 Host sampler — `scripts/bench/sample_host.py`

Runs on the host (GPUs are host devices). Every ~1 s appends a sample:
`{t, cpu_pct, load1, ram_used_gb, nvidia:{util,mem_used_mb,power_w},
intel:{render_pct,video_pct}, container:{cpu_pct,mem_mb}}`. Sources:
`nvidia-smi --query-gpu=… --format=csv,noheader,nounits`; `intel_gpu_top -J`
(sudo — prompt once at sweep start, or pre-authorize); `/proc/stat` + `/proc/meminfo`
(or `psutil`); `docker stats --no-stream swarm-multi`. Writes a JSONL time series
the orchestrator slices per run.

### 4.4 Orchestrator — `scripts/bench/run_bench.py`

Per run `(backend, resolution, N)`:
1. `docker rm -f swarm-multi` (clean slate).
2. Bring up via `run_swarm_demo.sh` semantics with `RENDER_BACKEND`, `CAM_W/H/FPS`,
   `SWARM_N=N`, `WORLD=baylands`.
3. Poll sim-ready (`ros2 topic list | grep -c vehicle_local_position ≥ N`),
   hard-capped (~5 min for baylands).
4. Start observatory + agents (existing exec commands).
5. Send one **fixed warm-up command** to `/swarm/user_input`:
   `"everyone take off, climb to 20 m, and orbit"` — consistent flight load,
   bounded token cost (one command per run).
6. **Settle 30 s**, then **measure 60 s**: snapshot `/state` cam counters at
   window start/end → per-drone FPS; read RTF from `gz topic … /stats`; slice the
   host sampler series; check telemetry liveness for all N.
7. Evaluate PASS/FAIL; write `run-*.json` + append `runs.csv` row; `docker rm -f`.

Each run is **time-boxed** (hard cap ~10 min incl. bring-up); a hang aborts the
run as FAIL-INFRA (distinct from a capacity FAIL) and the sweep continues.

### 4.5 Sweep algorithm (capacity-frontier, efficient)

For each `(backend, resolution)` cell, find the knee with an
**exponential-probe-then-bisect**, seeded by neighbours (capacity is monotone
non-increasing as resolution rises and is comparable across backends):

```
seed N0 = knee of the already-measured neighbour cell (next-lower resolution,
          same backend), else 1.
probe up: N0, 2·N0, 4·N0 … until first FAIL or N_cap (=32) or a RAM/VRAM guard trips.
bisect between last PASS and first FAIL to the exact knee.
record knee N*, plus the headroom snapshot + limiting resource at N*.
```

Keeps each cell to ~4–6 runs instead of one-per-N.

### 4.6 Post-process — `scripts/bench/frontier.py`

Reads `runs.csv`/`run-*.json` → emits `frontier.md` (backend × resolution table
of max-N, each cell annotated with limiting resource) and `frontier.png` heatmap.

---

## 5. Scope / defaults

- **Backends:** `cpu`, `intel`, `nvidia` (all viable; verify each run's actual
  backend via `nvidia-smi` attribution / iris presence, not just env).
- **Resolutions:** 320×180, 640×360, 960×540, 1280×720, 1920×1080 — all @ 10 Hz.
- **N ramp:** 1 → cap 32, RAM/VRAM guard aborts earlier.
- **Per run:** 30 s settle + 60 s measure; ~10 min hard cap incl. baylands bring-up.
- **World:** baylands (realistic load).
- **`--smoke` mode:** Approach C — fixed N (=8), resolution sweep per backend,
  ~15 short runs (~1 h) to validate the harness before the full overnight sweep.
- **Full-sweep runtime estimate:** ~3 backends × 5 resolutions × ~5 runs × ~8 min
  ≈ **~10 h wall** — an overnight, time-boxed run.
- **Token cost:** full-stack live agents, but exactly one scripted command per run
  ⇒ bounded.

---

## 6. Risks & mitigations

- **Baylands bring-up is slow (~5 min/run).** → neighbour-seeded probe+bisect
  keeps run count minimal; every run hard-time-boxed; `--smoke` validates first.
- **`intel_gpu_top` needs sudo.** → prompt/authorize once at sweep start; if
  unavailable, record iGPU headroom as N/A (FPS gate still works).
- **dGPU render attribution.** → confirm each `nvidia` run with `nvidia-smi`
  showing the gz process on GPU 0; otherwise mark the run backend-misconfigured.
- **Non-determinism from live LLM agents.** → fixed warm-up command + settle
  window; camera render cost is per-frame regardless of motion, so flight
  variance has little effect on the measured render/encode load.
- **FAIL-INFRA vs capacity FAIL conflation.** → distinct verdict; infra failures
  retried once before being recorded, and excluded from knee determination.

---

## 7. Out of scope

- Tuning the swarm to go faster (this measures the current stack, doesn't optimize
  it). Offloading H.264 encode to NVENC or perception to CUDA is a separate effort.
- Multi-host / distributed drones.
- Changing the demo's default behaviour.
