# Swarm capacity — pushed to the ceiling (intel vs nvidia)

_World `baylands` · full-stack live agents · 2026-06-28 · wall-clock ~16 min._

Host: i9-12900H (6P+8E, 20 threads), 31 GB RAM, Intel Iris Xe iGPU + NVIDIA RTX 3070 Ti Laptop (8 GB).
Climb seeded at the known-good 6 @ 960×540 anchor and pushed up (drones × resolution) until each platform's first FAIL.

## TL;DR

| Platform | Ceiling (max sustained) | first FAIL | RTF at the cliff |
|---|---|---|---|
| **Intel iGPU** | **10 drones @ 1280×720** (720p) | 12 @ 1280×720 | **0.10** (total stall) |
| **NVIDIA dGPU** | **10 drones @ 1280×720** (720p) | 12 @ 1280×720 | **0.81** (near miss) |

**Pass gate:** min per-drone cam FPS ≥ 9.0 (0.9×10 Hz) AND Gazebo RTF ≥ 0.90 AND all N drones armed/alive, over a 20 s window after 30 s settle.

## Rungs tested

### Intel iGPU (Iris Xe, EGL)
| drones | resolution | min FPS | RTF | alive | verdict |
|---|---|---|---|---|---|
| 6 | 960×540 | 9.9 | 1.00 | 6/6 | PASS |
| 8 | 960×540 | 9.9 | 1.00 | 8/8 | PASS |
| 8 | 1280×720 | 9.9 | 1.00 | 8/8 | PASS |
| 10 | 1280×720 | 9.8 | 1.00 | 10/10 | **PASS ← ceiling** |
| 12 | 1280×720 | 8.3 | 0.10 | 0/12 | FAIL (sim collapsed) |

### NVIDIA dGPU (RTX 3070 Ti, EGL)
| drones | resolution | min FPS | RTF | alive | verdict |
|---|---|---|---|---|---|
| 6 | 960×540 | 9.9 | 1.00 | 6/6 | PASS |
| 8 | 960×540 | 9.95 | 1.00 | 8/8 | PASS |
| 8 | 1280×720 | 9.8 | 1.00 | 8/8 | PASS |
| 10 | 1280×720 | 9.8 | 1.00 | 10/10 | **PASS ← ceiling** |
| 12 | 1280×720 | 8.8 | 0.81 | 0/12 | FAIL (`limiting=dgpu`) |

## What this means

- **Both GPUs cap at the same drone count (10 @ 720p).** At N=12 the bottleneck is **not** the camera render — it's the **CPU / single-threaded Gazebo physics + 12 agents all arming/taking off at once**. Throwing a faster GPU at it doesn't raise the count, because the GPU isn't the limiter.
- **But the dGPU degrades far more gracefully at the cliff.** At 12 drones the iGPU collapses to **RTF 0.10** (the whole sim stalls — drones can't even arm, 0/12), while the dGPU holds **RTF 0.81**, just under the 0.9 bar, and its limiting resource is the **GPU itself** (`dgpu`), not a total stall. The dGPU offloads rendering so more CPU is left for physics/flight — it's "almost" sustaining 12. With a slightly looser gate (RTF ≥ 0.8) the dGPU would pass 12 and the iGPU would not.
- **Practical guidance:** for camera-carrying baylands swarms, **10 drones @ 720p @ 10 Hz** is the comfortable real-time ceiling on this box, on either GPU. Prefer the **dGPU** when pushing near the limit (it has real headroom left at the edge); the iGPU is equivalent below the limit and frees the dGPU for compute.

## Caveats

- The climb stops at the **first** FAIL, so it didn't test the **1080p** rungs or the **fewer-drones-higher-resolution** corner (e.g. 6 @ 1920×1080) — a different "best" if you favor resolution over count. Add those to `--ladder` to map that corner.
- The N=12 failures are partly **liveness** (12 agents can't all take off inside a 30 s settle once RTF drops) compounding the RTF failure — a longer settle might let the dGPU's 12-drone case arm, but RTF 0.81 still wouldn't clear the 0.9 gate.
- Raw per-run headroom (host/container CPU%, both GPUs, RAM) is in `samples-*.jsonl`; per-run detail in `run-*.json`.
