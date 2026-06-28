# Swarm camera-capacity benchmark — results & how to resume

Single source of truth for "how many camera-carrying drones at what resolution
can this box sustain, per render backend." If you're about to re-benchmark,
**read this first** — most of the search space is already mapped. Raw per-run
JSON/JSONL artifacts were intentionally discarded; everything load-bearing is here.

## The box these numbers are for

i9-12900H (6P + 8E = 20 threads) · 31 GB RAM · Intel Iris Xe iGPU + NVIDIA
RTX 3070 Ti Laptop (8 GB, driver 580.159.03). Numbers are **not** portable to
other hardware — re-run there.

## Method & pass gate

World **baylands**, **full stack with live LLM agents**, camera **10 Hz**.
A config **PASSES** iff, over a 20 s window after a 30 s settle:
min per-drone delivered cam FPS ≥ **9.0** (0.9×10) **AND** Gazebo **RTF ≥ 0.90**
**AND** all N drones **armed/alive**. Headroom (host/container CPU%, both GPUs,
RAM) sampled ~1 Hz. Harness climbs a demand ladder (drones × resolution) from a
seed, up on PASS / down on FAIL, until the first FAIL = the ceiling.

## Headline (2026-06-28, confirmed)

| Backend | Comfortable cruise | **Ceiling (max sustained)** | First FAIL | Behaviour at the cliff |
|---|---|---|---|---|
| **Intel iGPU** (Iris Xe, EGL) | 6 @ 1280×720 | **10 @ 1280×720** | 12 @ 1280×720 | **collapse** — RTF 0.10, 0/12 armed |
| **NVIDIA dGPU** (RTX 3070 Ti, EGL) | 6 @ 1280×720 | **10 @ 1280×720** | 12 @ 1280×720 | **graceful** — RTF 0.81, `limiting=dgpu` |
| **CPU** (llvmpipe software GL) | — | **not viable** | every rung | sim never reaches "ready" (render too slow) |

**Recommended operating point: 10 drones @ 720p @ 10 Hz** on either GPU; prefer
the **dGPU** when pushing near the limit (real headroom left), the iGPU below it
(frees the dGPU for compute).

## Rung detail (the runs that pinned the above)

PUSH ladder — both GPUs identical up to the wall:

| backend | drones | res | min FPS | RTF | alive | verdict |
|---|---|---|---|---|---|---|
| intel | 6 | 960×540 | 9.9 | 1.00 | 6/6 | PASS |
| intel | 8 | 960×540 | 9.9 | 1.00 | 8/8 | PASS |
| intel | 8 | 1280×720 | 9.9 | 1.00 | 8/8 | PASS |
| intel | 10 | 1280×720 | 9.8 | 1.00 | 10/10 | **PASS ← ceiling** |
| intel | 12 | 1280×720 | 8.3 | 0.10 | 0/12 | FAIL (collapse) |
| nvidia | 6 | 960×540 | 9.9 | 1.00 | 6/6 | PASS |
| nvidia | 8 | 960×540 | 9.95 | 1.00 | 8/8 | PASS |
| nvidia | 8 | 1280×720 | 9.8 | 1.00 | 8/8 | PASS |
| nvidia | 10 | 1280×720 | 9.8 | 1.00 | 10/10 | **PASS ← ceiling** |
| nvidia | 12 | 1280×720 | 8.8 | 0.81 | 0/12 | FAIL (`limiting=dgpu`) |

Headroom at the comfortable cruise (6 @ 720p, peak sample):

| | Intel render | NVIDIA render |
|---|---|---|
| host CPU | 26% | 35% |
| container CPU | ~5.1 cores | ~5.8 cores |
| RAM used | 14.1 GB | 13.4 GB |
| dGPU VRAM | 0.84 GB¹ | 1.87 GB |
| dGPU power | 20 W | 30 W |

¹ iGPU run's dGPU usage is incidental (desktop/compositor); the iGPU's own
busy-% was **not** captured (`intel_gpu_top` needs sudo and the sampler ran
without it).

## What a future runner must know (so you don't redo this)

1. **The high-end limiter is CPU, not GPU.** Both GPUs cap at the *same* drone
   count → the 12-drone wall is single-thread Gazebo physics + N agents arming
   at once. A faster GPU will **not** raise the drone count; it only buys
   graceful degradation. To go past 10 drones, attack CPU/physics (or stagger
   takeoffs), not the renderer.
2. **CPU/llvmpipe is a dead end** for camera sims — fails at *bring-up*, not at
   the FPS gate. Don't spend a budget probing it; a GPU render path is required.
3. **NVIDIA render needs the glvnd EGL ICD fix.** `--gpus all` injects
   `libEGL_nvidia.so.0` but not `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`,
   so ogre2 segfaults in `CreateRenderSystem`. `sim/launch/swarm_sim.sh` now
   creates that ICD in-container (idempotent). Full story:
   `docs/nvidia-render-investigation.md`.
4. **`limiting=dgpu` is a false positive at low load** — a single peak
   nvidia-smi sample spikes to 100%. Trust it only when RTF is also dropping.
5. **Untested corners** (climb stops at first FAIL): **1080p** rungs and the
   **low-drones / high-resolution** corner (e.g. 6 @ 1920×1080). Add via
   `--ladder` if resolution matters more than count.

## Reproduce / extend

Run from the **main repo dir** (`/home/quenouille/drone`), NOT the worktree —
`PX4-Autopilot` is untracked, so the worktree can't bring the sim up.

```bash
# Re-confirm the ceiling (push both GPUs to first FAIL):
uv run --extra dev python -m bench.run_bench --quick \
  --backends intel,nvidia --world baylands \
  --ladder "6x960x540,8x960x540,8x1280x720,10x1280x720,12x1280x720" \
  --seed-idx 0 --run-cap 5 --settle 30 --measure 20 \
  --out docs/benchmarks/<label>-<ts>

# Map the untested 1080p / low-N corner:
uv run --extra dev python -m bench.run_bench --quick \
  --backends nvidia --world baylands \
  --ladder "6x1280x720,6x1920x1080,4x1920x1080" \
  --seed-idx 0 --run-cap 3 --settle 30 --measure 20 \
  --out docs/benchmarks/<label>-<ts>
```

Flags: `--ladder "NxWxH,…"` custom rungs · `--run-cap K` max rungs/backend ·
`--seed-idx I` where to start the climb. After a run, fold the headline + any
new knowledge **back into this file** and delete the raw run dir — keep this
folder to one or two files.

Harness + unit tests: top-level `bench/` package, `tests/bench/` (30 tests).
Branch `bench/capacity-benchmark` (not merged to main by choice).
