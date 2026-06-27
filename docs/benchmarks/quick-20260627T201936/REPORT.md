# Swarm capacity — best per platform (quick test)

_World `baylands` · full-stack live agents · 2026-06-27 · wall-clock ~32 min (1600 s budget; cpu's slow bring-up failures pushed it over)._

Host: i9-12900H (6P+8E, 20 threads), 31 GB RAM, Intel Iris Xe iGPU + NVIDIA RTX 3070 Ti Laptop (8 GB).

## TL;DR — best sustained config per platform

| Platform | Best sustained (this test) | min FPS | RTF | bottleneck |
|---|---|---|---|---|
| **Intel iGPU** (Iris Xe, EGL) | **≥ 6 drones @ 960×540** | 9.9 | 1.00 | not reached |
| **NVIDIA dGPU** (RTX 3070 Ti, EGL) | **≥ 6 drones @ 960×540** | 9.95 | 1.00 | not reached |
| **CPU** (llvmpipe software GL) | **none** — sim never readied | — | — | software render too slow |

> The intel/nvidia headlines are **lower bounds**: both were *still passing* (RTF 1.00, full 10 Hz delivery) at the quick test's 4-rung exploration cap, so neither was pushed to failure. Their true ceilings are higher — the full sweep (`--out … python -m bench.run_bench`) would pin them.

## Pass gate

A config PASSES iff, over a 20 s measurement window (after 30 s settle): min per-drone delivered camera FPS ≥ **9.0** (0.9 × 10 Hz) **AND** Gazebo RTF ≥ **0.90** **AND** all N drones **armed/alive**. Each platform climbs a demand ladder (drones × resolution increasing) from a seed of 2 @ 640×360, climbing on PASS / descending on FAIL, capped at 4 runs and a per-platform slice of the time budget.

## All rungs tested

### Intel iGPU (Iris Xe, EGL)
| drones | resolution | min FPS | RTF | alive | verdict |
|---|---|---|---|---|---|
| 2 | 640×360 | 10.0 | 1.00 | 2/2 | PASS |
| 4 | 640×360 | 10.0 | 1.00 | 4/4 | PASS |
| 4 | 960×540 | 10.0 | 1.00 | 4/4 | PASS |
| 6 | 960×540 | 9.9 | 1.00 | 6/6 | PASS ← headline (cap hit, still passing) |

### NVIDIA dGPU (RTX 3070 Ti, EGL)
| drones | resolution | min FPS | RTF | alive | verdict |
|---|---|---|---|---|---|
| 2 | 640×360 | 10.0 | 1.00 | 2/2 | PASS |
| 4 | 640×360 | 10.0 | 1.00 | 4/4 | PASS |
| 4 | 960×540 | 10.0 | 1.00 | 4/4 | PASS |
| 6 | 960×540 | 9.95 | 1.00 | 6/6 | PASS ← headline (cap hit, still passing) |

### CPU (llvmpipe software GL)
| drones | resolution | min FPS | RTF | alive | verdict | reason |
|---|---|---|---|---|---|---|
| 2 | 640×360 | 0.0 | 0.00 | 0/2 | FAIL | sim never reached ready (infra) |
| 2 | 320×180 | 0.0 | 0.00 | 0/2 | FAIL | sim never reached ready (infra) |

## Findings

- **Intel iGPU and NVIDIA dGPU are indistinguishable in this range.** At every rung up to 6 drones @ 960×540 on baylands, both hold **real-time (RTF 1.00)** and deliver the **full 10 Hz** camera rate. The workload never stressed either GPU, so this quick test can't separate them — the differentiating point is at higher drone counts / resolutions (e.g. 8–16 drones, 1280×720+), which is exactly what the unbounded overnight sweep maps.
- **CPU / llvmpipe is not viable for camera-carrying baylands sim.** Software rendering couldn't bring the camera pipeline up fast enough for the sim to reach "ready," so every cpu rung failed at *bring-up* (not at the FPS gate). For a swarm with onboard cameras, a GPU render path (intel or nvidia) is required.
- **NVIDIA needed a fix this session and now works.** The dGPU render path was segfaulting in `ogre2 CreateRenderSystem` during multi-drone bring-up: the `nvidia-container-toolkit` (`--gpus all`) injects `libEGL_nvidia.so.0` but does **not** create the glvnd vendor ICD `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`, so `__EGL_VENDOR_LIBRARY_FILENAMES` pointed at a missing file → no NVIDIA EGL loaded → null deref. Fix: `sim/launch/swarm_sim.sh` now creates that ICD in-container (idempotently) when `libEGL_nvidia.so.0` is present. Verified on baylands (gz attributed to GPU 0, 891 MiB, no crash). See `docs/nvidia-render-investigation.md`.

## Caveats & how to go deeper

- **Headlines are lower bounds** — the 4-rung-per-platform cap stopped intel/nvidia while still passing. Raise the cap / extend the ladder (or run the full sweep) to find the true edge.
- **`limiting=none`** throughout because nothing saturated — at these loads there is genuine headroom; the column only becomes meaningful near the edge. Raw per-run headroom (host CPU%, container CPU%, both GPUs, RAM) is in `samples-*.jsonl`.
- **Wall-clock ran ~32 min vs the 1600 s budget**: cpu's two bring-up failures each waited out the readiness timeout (and the once-retry on infra-fail doubled them). Excluding the doomed cpu attempts, intel + nvidia completed in ~20 min.
- To pin exact ceilings and separate the two GPUs: `uv run --extra dev python -m bench.run_bench --backends intel,nvidia --world baylands` (full knee search, no rung cap).

## Artifacts (this directory)
- `runs.csv` — one row per run.
- `run-<backend>-<res>-<n>.json` — full per-run detail (fps summary, rtf, alive, peak headroom).
- `samples-<backend>-<res>-<n>.jsonl` — ~1 Hz host headroom time series per run.
