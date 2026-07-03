# Rendering & GPU

Camera rendering is the expensive part of the swarm, so the launcher lets you choose
**which GPU (or no GPU) renders the cameras** via `RENDER_BACKEND`. For everyday use
the default (Intel iGPU) just works — see the [README quickstart](../README.md#quickstart).

## The three backends

| | **`intel`** (default) | **`nvidia`** | **`cpu`** |
|---|---|---|---|
| Renderer | Intel iGPU (Iris Xe, `i915`) via **EGL headless** | NVIDIA dGPU via **EGL headless** (`--gpus all`) | Software GL (**llvmpipe**) |
| Camera POV tiles | ✅ real-time | ✅ real-time, most headroom | ❌ disabled (sim can't even ready) |
| Drone model | `gz_x500_depth` (camera) | `gz_x500_depth` (camera) | `gz_x500` (no camera) |
| Needs | `/dev/dri` (out of the box) | `nvidia-container-toolkit` + driver | nothing |
| Use it when | the everyday default | pushing near the capacity ceiling | flight + chat only, no cameras |

```bash
RENDER_BACKEND=intel  ./scripts/run_swarm_demo.sh 3   # default; GPU=1 is an alias
RENDER_BACKEND=nvidia ./scripts/run_swarm_demo.sh 3   # dGPU (auto-adds --gpus all)
RENDER_BACKEND=cpu    ./scripts/run_swarm_demo.sh 3   # software GL; GPU=0 is an alias
```

## Resolution & rate knobs

| Var | Values | Default | What it does |
|-----|--------|---------|--------------|
| `RENDER_BACKEND` | `intel` · `nvidia` · `cpu` | `intel` | Which GPU (or none) renders cameras |
| `CAM_W` × `CAM_H` | px | `640` × `360` | Per-drone camera resolution |
| `CAM_FPS` | Hz | `10` | Camera update rate |

## How to choose (measured)

- **Both GPUs cap at the same drone count** on a given box — the high-end limiter is
  single-thread Gazebo physics, not the renderer. The dGPU's win is *graceful
  degradation* near the limit, not a higher count. Measured ceilings + how to
  re-benchmark your own hardware: **[benchmarks/RESULTS.md](benchmarks/RESULTS.md)**.
- **`cpu`/llvmpipe is flight-only** — software camera render is too slow for the sim
  to reach "ready," so cameras are disabled (`gz_x500`, no camera model).

## Why it's wired this way

- Under `intel`, only **`renderD128`** is exposed to the container on purpose: if
  Mesa can see an NVIDIA node with no Mesa driver, `ogre2` segfaults.
- **NVIDIA** needs the glvnd EGL vendor ICD, which `--gpus all` does *not* create;
  `swarm_sim.sh` writes it in-container (idempotent). Full root-cause story:
  [nvidia-render-investigation.md](nvidia-render-investigation.md).
- Gazebo always runs **server-only** (`HEADLESS=1`) — no Qt GUI. The GUI aborts
  under offscreen Qt and would take down the gz-launching PX4 instance.
